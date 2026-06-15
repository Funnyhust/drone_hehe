#include "board_pinmap.h"
#include "config.h"
#include "driver/battery_adc.h"
#include "driver/motor_pwm.h"
#include "driver/mpu6050.h"
#include "driver/rc_crsf.h"
#include "driver/soft_i2c.h"
#include "driver/soft_uart.h"
#include "driver/eeprom_24lc256.h"
#include "driver/flash_backup.h"
#include "middleware/blackbox.h"
#include "middleware/imu_estimator.h"
#include "middleware/motor_mixer.h"
#include "middleware/pid_controller.h"
#include "middleware/safety.h"
#include "middleware/test.h"
#include <Arduino.h>

#if ENABLE_DEBUG
HardwareSerial SerialDebug(PIN_DEBUG_RX, PIN_DEBUG_TX);
#endif

// Biến lưu trữ dữ liệu thô cảm biến và tư thế góc
static MpuData imu_raw;
static Attitude attitude_angles;

// Quản lý chu kỳ vòng lặp chính
static uint32_t last_loop_time_us = 0;
static uint32_t target_loop_period_us = CONTROL_LOOP_PERIOD_US; // Mặc định 500Hz (2000us)
static bool is_i2c_fast_mode = true;

// Quản lý thời gian chạy các tác vụ nền
static uint32_t last_battery_time_ms = 0;

// Biến lưu trữ thời gian thực thi tác vụ trong loop để giám sát budget
static uint16_t budget_overrun_counter = 0;

// Các biến lưu trữ tạm đầu ra PID
static float out_roll = 0.0f;
static float out_pitch = 0.0f;
static float out_yaw = 0.0f;

// Cấu hình cấu trúc dữ liệu config toàn cục
static DroneConfig global_config;

// =============================================================================
// Hàm Quét I2C Scanner (Cho chế độ Bring-up)
// =============================================================================
void runI2cScanner() {
  softUartPrintln("--- Starting SoftI2C Scan ---");
  uint8_t count = 0;
  for (uint8_t address = 1; address < 127; address++) {
    uint8_t dummy = 0;
    if (softI2cReadReg(address, 0x00, &dummy) == I2C_OK ||
        softI2cReadReg(address, 0x75, &dummy) == I2C_OK) { // WHO_AM_I MPU
      softUartPrint("I2C device found at address 0x");
      if (address < 16)
        softUartPrint("0");
      softUartPrintln(address, HEX);
      count++;
    }
  }
  if (count == 0) {
    softUartPrintln("No I2C devices found.");
  }
  softUartPrintln("-----------------------------");
}

// =============================================================================
// Hàm nháy LED báo lỗi khởi động (chạy phi chặn)
// =============================================================================
void handleStartupError(StartupError err) {
  static uint32_t last_blink_time = 0;
  static uint8_t blink_cnt = 0;
  static bool led_state = false;
  
  motorStopAll(); // Khóa cứng động cơ để đảm bảo an toàn tuyệt đối
  
  uint32_t now = millis();
  uint8_t target_blinks = (uint8_t)err; // Số nhịp chớp LED tương đương với mã lỗi
  
  if (target_blinks == 0) {
    digitalWrite(PB4, HIGH); // Sáng liên tục nếu lỗi không xác định
    return;
  }
  
  // Chu kỳ nháy: nhấp nháy target_blinks lần, mỗi lần 200ms sáng, 200ms tắt.
  // Sau đó tắt hẳn 1.5 giây để phân biệt chu kỳ rồi lặp lại.
  if (blink_cnt < target_blinks * 2) {
    if (now - last_blink_time >= 200) {
      led_state = !led_state;
      digitalWrite(PB4, led_state ? HIGH : LOW); // LED đỏ PB4
      blink_cnt++;
      last_blink_time = now;
    }
  } else {
    digitalWrite(PB4, LOW);
    if (now - last_blink_time >= 1500) {
      blink_cnt = 0;
      last_blink_time = now;
    }
  }
}

// =============================================================================
// Máy trạng thái Khởi động phi chặn (Non-blocking Startup FSM)
// =============================================================================
void updateStartupFsm() {
  static uint32_t fsm_timer = 0;
  static uint16_t gyro_sample_cnt = 0;
  static int32_t gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;
  static int64_t gyro_sum_sq_x = 0, gyro_sum_sq_y = 0, gyro_sum_sq_z = 0;
  static uint8_t gyro_retry_cnt = 0;
  
  static float acc_sum_x = 0, acc_sum_y = 0, acc_sum_z = 0;
  static uint8_t acc_sample_cnt = 0;
  
  StartupState state = safetyGetStartupState();
  
  switch (state) {
    case STARTUP_BOOT: {
      // 1. Tải cấu hình từ EEPROM/Flash
      uint8_t load_src = configLoad(&global_config);
      
      if (load_src <= 1) {
        // Áp dụng PID parameters vào controller (vòng Rate)
        pidSetGains(AXIS_ROLL, global_config.kp_roll, global_config.ki_roll, global_config.kd_roll, true);
        pidSetGains(AXIS_PITCH, global_config.kp_pitch, global_config.ki_pitch, global_config.kd_pitch, true);
        pidSetGains(AXIS_YAW, global_config.kp_yaw, global_config.ki_yaw, global_config.kd_yaw, true);
                    
        // Áp dụng offsets cảm biến đã calib
        mpu6050SetOffsets(global_config.accel_offset_x, global_config.accel_offset_y, global_config.accel_offset_z,
                          global_config.gyro_offset_x, global_config.gyro_offset_y, global_config.gyro_offset_z);
                          
        softUartPrintf("[BOOT] Config loaded from %s. PID/Offsets applied.\r\n", 
                       (load_src == 0) ? "EEPROM" : "Flash Backup");
      } else {
        softUartPrintln("[BOOT] Using Default Config (No profile saved).");
      }
      
      fsm_timer = millis();
      safetySetStartupState(STARTUP_IMU_INIT);
      break;
    }
    
    case STARTUP_IMU_INIT: {
      // 2. Khởi tạo MPU6050 ở tốc độ cao (400kHz)
      softI2cSetSpeed(400);
      uint8_t status = mpu6050Init();
      
      if (status != 0) {
        softUartPrintln("[WARNING] IMU MPU6050 failed at 400kHz. Attempting Fallback to 100kHz...");
        softI2cSetSpeed(100);
        is_i2c_fast_mode = false;
        status = mpu6050Init();
      }
      
      if (status == 0) {
        softUartPrintln("[IMU] MPU6050 Init OK. Ready for Gyro Calib.");
        gyro_sample_cnt = 0;
        gyro_sum_x = gyro_sum_y = gyro_sum_z = 0;
        gyro_sum_sq_x = gyro_sum_sq_y = gyro_sum_sq_z = 0;
        gyro_retry_cnt = 0;
        fsm_timer = millis();
        safetySetStartupState(STARTUP_GYRO_CALIB);
      } else {
        softUartPrintln("[FATAL] MPU6050 Init Failed completely!");
        safetySetStartupError(ERR_IMU_INIT_FAIL);
        safetySetStartupState(STARTUP_ERROR);
      }
      break;
    }
    
    case STARTUP_GYRO_CALIB: {
      // 3. Hiệu chuẩn con quay hồi chuyển phi chặn
      MpuData temp_imu;
      // Tạm thời xóa offset Gyro nội bộ để lấy mẫu thô
      int16_t ax_o, ay_o, az_o, gx_o, gy_o, gz_o;
      mpu6050GetOffsets(&ax_o, &ay_o, &az_o, &gx_o, &gy_o, &gz_o);
      mpu6050SetOffsets(ax_o, ay_o, az_o, 0, 0, 0);
      
      bool read_ok = (mpu6050Read(&temp_imu) == 0);
      
      // Khôi phục lại offset để bảo toàn trạng thái
      mpu6050SetOffsets(ax_o, ay_o, az_o, gx_o, gy_o, gz_o);
      
      if (read_ok) {
        gyro_sum_x += temp_imu.gx_raw;
        gyro_sum_sq_x += (int32_t)temp_imu.gx_raw * temp_imu.gx_raw;
        
        gyro_sum_y += temp_imu.gy_raw;
        gyro_sum_sq_y += (int32_t)temp_imu.gy_raw * temp_imu.gy_raw;
        
        gyro_sum_z += temp_imu.gz_raw;
        gyro_sum_sq_z += (int32_t)temp_imu.gz_raw * temp_imu.gz_raw;
        
        gyro_sample_cnt++;
      }
      
      // Nhấp nháy LED đỏ nhanh trong khi đang calib
      if (millis() - fsm_timer >= 100) {
        digitalWrite(PB4, !digitalRead(PB4));
        fsm_timer = millis();
      }
      
      if (gyro_sample_cnt >= 1000) {
        // Đủ 1000 mẫu, tính toán Mean và StdDev
        float mean_gx = (float)gyro_sum_x / gyro_sample_cnt;
        float mean_gy = (float)gyro_sum_y / gyro_sample_cnt;
        float mean_gz = (float)gyro_sum_z / gyro_sample_cnt;
        
        float var_gx = ((float)gyro_sum_sq_x / gyro_sample_cnt) - (mean_gx * mean_gx);
        float var_gy = ((float)gyro_sum_sq_y / gyro_sample_cnt) - (mean_gy * mean_gy);
        float var_gz = ((float)gyro_sum_sq_z / gyro_sample_cnt) - (mean_gz * mean_gz);
        
        float stddev_gx = sqrt(var_gx >= 0.0f ? var_gx : 0.0f);
        float stddev_gy = sqrt(var_gy >= 0.0f ? var_gy : 0.0f);
        float stddev_gz = sqrt(var_gz >= 0.0f ? var_gz : 0.0f);
        
        float max_stddev = stddev_gx;
        if (stddev_gy > max_stddev) max_stddev = stddev_gy;
        if (stddev_gz > max_stddev) max_stddev = stddev_gz;
        
        if (max_stddev <= GYRO_CALIB_STDDEV_THRESHOLD) {
          // Calib thành công! Lưu offset Gyro
          mpu6050SetOffsets(ax_o, ay_o, az_o, (int16_t)mean_gx, (int16_t)mean_gy, (int16_t)mean_gz);
          softUartPrintf("[CALIB] Gyro Calib Success. Offsets: %d, %d, %d. Max StdDev: %d/100 deg/s\r\n", 
                         (int16_t)mean_gx, (int16_t)mean_gy, (int16_t)mean_gz, (int16_t)(max_stddev * 100));
          digitalWrite(PB4, LOW); // Tắt LED đỏ
          fsm_timer = millis();
          safetySetStartupState(STARTUP_RC_CHECK);
        } else {
          // Thất bại do drone bị rung/lắc
          gyro_retry_cnt++;
          softUartPrintf("[WARNING] Gyro Calib failed due to motion (StdDev: %d/100). Retry %d/5...\r\n", 
                         (int16_t)(max_stddev * 100), gyro_retry_cnt);
          
          if (gyro_retry_cnt >= 5) {
            safetySetStartupError(ERR_GYRO_CALIB_MOVING);
            safetySetStartupState(STARTUP_ERROR);
          } else {
            // Reset tích lũy thử lại sau 500ms
            gyro_sample_cnt = 0;
            gyro_sum_x = gyro_sum_y = gyro_sum_z = 0;
            gyro_sum_sq_x = gyro_sum_sq_y = gyro_sum_sq_z = 0;
            delay(500);
          }
        }
      }
      break;
    }
    
    case STARTUP_RC_CHECK: {
      // 4. Kiểm tra tay điều khiển ELRS
      crsfUpdate();
      if (!crsfIsLinkActive()) {
        if (millis() - fsm_timer > 5000) { // Quá 5 giây không có sóng tay điều khiển
          softUartPrintln("[FATAL] ELRS Receiver link not active!");
          safetySetStartupError(ERR_RC_LINK_LOST);
          safetySetStartupState(STARTUP_ERROR);
        }
        break;
      }
      
      uint16_t ch_roll = crsfGetChannel(0);
      uint16_t ch_pitch = crsfGetChannel(1);
      uint16_t ch_throttle = crsfGetChannel(2);
      uint16_t ch_yaw = crsfGetChannel(3);
      
      // Kiểm tra ga phải ở mức thấp nhất (< 1050us)
      if (ch_throttle > 1050) {
        softUartPrintf("[FATAL] Safety Block: Throttle is high (%dus)!\r\n", ch_throttle);
        safetySetStartupError(ERR_THROTTLE_NOT_MIN);
        safetySetStartupState(STARTUP_ERROR);
        break;
      }
      
      // Kiểm tra Roll/Pitch/Yaw nằm quanh dải trung tâm (1500us +- 30us)
      if (abs((int16_t)ch_roll - 1500) > 30 ||
          abs((int16_t)ch_pitch - 1500) > 30 ||
          abs((int16_t)ch_yaw - 1500) > 30) {
        softUartPrintf("[FATAL] Safety Block: Sticks not centered! R:%d P:%d Y:%d\r\n", 
                       ch_roll, ch_pitch, ch_yaw);
        safetySetStartupError(ERR_RC_CENTER_OUT);
        safetySetStartupState(STARTUP_ERROR);
        break;
      }
      
      softUartPrintln("[RC] Receiver validate OK. Starting Accel Validation...");
      acc_sample_cnt = 0;
      acc_sum_x = acc_sum_y = acc_sum_z = 0;
      safetySetStartupState(STARTUP_ACC_CHECK);
      break;
    }
    
    case STARTUP_ACC_CHECK: {
      // 5. Kiểm tra tính toàn vẹn vật lý Accel tĩnh
      MpuData temp_imu;
      if (mpu6050Read(&temp_imu) == 0) {
        acc_sum_x += temp_imu.ax;
        acc_sum_y += temp_imu.ay;
        acc_sum_z += temp_imu.az;
        acc_sample_cnt++;
      }
      
      if (acc_sample_cnt >= 10) {
        float ax_mean = acc_sum_x / acc_sample_cnt;
        float ay_mean = acc_sum_y / acc_sample_cnt;
        float az_mean = acc_sum_z / acc_sample_cnt;
        
        float g_total = sqrt(ax_mean * ax_mean + ay_mean * ay_mean + az_mean * az_mean);
        if (g_total >= 0.95f && g_total <= 1.05f) {
          softUartPrintf("[IMU] Accel Static Validation OK (g = %d/100).\r\n", (int16_t)(g_total * 100));
          safetySetStartupState(STARTUP_ESC_CHECK);
        } else {
          softUartPrintf("[FATAL] Accel Static Check Failed! total g = %d/100 (Expected 0.95g-1.05g)\r\n", 
                         (int16_t)(g_total * 100));
          safetySetStartupError(ERR_ACC_VALIDATION_FAIL);
          safetySetStartupState(STARTUP_ERROR);
        }
      }
      break;
    }
    
    case STARTUP_ESC_CHECK: {
      // 6. Kiểm tra trạng thái hiệu chuẩn ESC
      if (global_config.esc_calibrated == 1) {
        softUartPrintln("[BOOT] ESC Calibration status check: OK.");
      } else {
        softUartPrintln("[WARNING] ESC is NOT calibrated! Disarming safety will reject arming.");
        // Ghi nhận cờ lỗi nhưng cho phép boot qua để người dùng biết lý do chốt safety
      }
      
      softUartPrintln("=== STARTUP CALIBRATION COMPLETE! SYSTEM READY TO BAY ===");
      safetySetStartupState(STARTUP_READY);
      break;
    }
    
    case STARTUP_READY:
      // Bật LED xanh sáng liên tục báo hiệu hệ thống đã hoàn toàn sẵn sàng bay
      digitalWrite(PB3, HIGH);
      digitalWrite(PB4, LOW);
      break;
      
    case STARTUP_ERROR:
      handleStartupError(safetyGetStartupError());
      break;
  }
}

// =============================================================================
// Quy trình hiệu chuẩn một lần duy nhất qua công tắc CH5
// =============================================================================
void runEscCalibration() {
  crsfUpdate();
  uint16_t ch5 = crsfGetChannel(4); // Đọc CH5 (AUX 1)
  
  if (ch5 > 1750) {
    motorWriteAllUs(2000, 2000, 2000, 2000); // Xuất xung tối đa
    digitalWrite(PB3, HIGH); // Bật LED xanh báo hiệu đang phát xung ga tối đa
  } else {
    motorWriteAllUs(1000, 1000, 1000, 1000); // Xuất xung tối thiểu
    digitalWrite(PB3, LOW);
  }
  
  safetyFeedWatchdog(); // Tránh Watchdog reset
  delay(10);
}

void runAccelCalibration() {
  static enum { CAL_IDLE, CAL_RUNNING, CAL_DONE, CAL_ERROR } cal_state = CAL_IDLE;
  static uint32_t led_timer = 0;
  
  crsfUpdate();
  uint16_t ch5 = crsfGetChannel(4);
  safetyFeedWatchdog();
  
  switch (cal_state) {
    case CAL_IDLE:
      // Nháy LED đỏ chậm báo hiệu đang ở chế độ chờ kích hoạt
      if (millis() - led_timer > 500) {
        digitalWrite(PB4, !digitalRead(PB4));
        led_timer = millis();
      }
      
      // Kích hoạt khi người dùng gạt CH5 lên mức cao (> 1750us)
      if (ch5 > 1750) {
        digitalWrite(PB4, HIGH); // Sáng LED đỏ báo hiệu đang calib
        cal_state = CAL_RUNNING;
      }
      break;
      
    case CAL_RUNNING:
      softUartPrintln("Starting Accel Calibration... Keep drone completely level and still.");
      delay(1000); // Chờ 1 giây để ổn định
      
      if (mpu6050CalibrateAccel() == 0) {
        // Lấy offset mới đo được
        int16_t ax_o, ay_o, az_o;
        int16_t gx_o, gy_o, gz_o;
        mpu6050GetOffsets(&ax_o, &ay_o, &az_o, &gx_o, &gy_o, &gz_o);
        
        // Tải cấu hình hiện tại để giữ lại các thông số PID
        DroneConfig temp_config;
        configLoad(&temp_config);
        
        temp_config.accel_offset_x = ax_o;
        temp_config.accel_offset_y = ay_o;
        temp_config.accel_offset_z = az_o;
        temp_config.esc_calibrated = 1; // Đồng thời thiết lập cờ ESC đã calib
        
        // Lưu trữ kép (EEPROM + Flash backup)
        if (configSave(&temp_config) == 0) {
          softUartPrintln("Accel Calibration SUCCESS! Saved to EEPROM & Flash.");
          cal_state = CAL_DONE;
        } else {
          softUartPrintln("Accel Calibration SUCCESS but failed to save config!");
          cal_state = CAL_ERROR;
        }
      } else {
        softUartPrintln("Accel Calibration FAILED due to sensor read error!");
        cal_state = CAL_ERROR;
      }
      led_timer = millis();
      break;
      
    case CAL_DONE:
      // Nháy nhanh LED xanh báo thành công
      digitalWrite(PB4, LOW);
      if (millis() - led_timer > 100) {
        digitalWrite(PB3, !digitalRead(PB3));
        led_timer = millis();
      }
      break;
      
    case CAL_ERROR:
      // Nháy nhanh LED đỏ báo lỗi
      digitalWrite(PB3, LOW);
      if (millis() - led_timer > 100) {
        digitalWrite(PB4, !digitalRead(PB4));
        led_timer = millis();
      }
      break;
  }
  delay(10);
}

// =============================================================================
// Hàm Khởi động Setup
// =============================================================================
void setup() {
  // 1. Khởi tạo Software UART 19200 baud để phát log
  softUartInit(19200);

  // 2. Khởi tạo cổng truyền Serial phần cứng (CLI)
  Serial.begin(115200);
  delay(1000);
  softUartPrintln("=== DRONE FIRMWARE BOOTING ===");

  // 3. Khởi tạo bus Software I2C
  softI2cInit();
  
  // 4. Khởi tạo các Driver & Middleware khác
  eepromInit();
  crsfInit();
  motorInit(false);
  batteryInit();
  safetyInit();
  blackboxInit();
  imuEstimatorInit();
  pidInit();

  last_loop_time_us = micros();
  last_battery_time_ms = millis();
  
  // Thiết lập trạng thái khởi động ban đầu
  safetySetStartupState(STARTUP_BOOT);

  softUartPrintf("Boot completed. Calibration Mode: %d\r\n", CALIBRATION_MODE);
}

// =============================================================================
// Vòng lặp Loop chính
// =============================================================================
void loop() {
  // ---------------------------------------------------------------------------
  // Xử lý Chế độ Hiệu chuẩn Đặc biệt (Calibration Modes)
  // ---------------------------------------------------------------------------
#if defined(CALIBRATION_MODE) && (CALIBRATION_MODE == 1)
  runEscCalibration();
  return;
#elif defined(CALIBRATION_MODE) && (CALIBRATION_MODE == 2)
  runAccelCalibration();
  return;
#endif

  // ---------------------------------------------------------------------------
  // A. Hỗ trợ giao diện CLI Test/Debug qua Serial
  // ---------------------------------------------------------------------------
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == 's') {
      runI2cScanner();
    } else if (cmd == 'i') {
      softUartPrintln("Printing Raw IMU (Send any key to stop)...");
      while (!Serial.available()) {
        safetyFeedWatchdog();
        MpuData temp_imu;
        if (mpu6050Read(&temp_imu) == 0) {
          char ax_str[10], ay_str[10], az_str[10];
          char gx_str[10], gy_str[10], gz_str[10];
          dtostrf(temp_imu.ax, 6, 2, ax_str);
          dtostrf(temp_imu.ay, 6, 2, ay_str);
          dtostrf(temp_imu.az, 6, 2, az_str);
          dtostrf(temp_imu.gx, 6, 2, gx_str);
          dtostrf(temp_imu.gy, 6, 2, gy_str);
          dtostrf(temp_imu.gz, 6, 2, gz_str);
          softUartPrintf("ACC: %s, %s, %s | GYRO: %s, %s, %s\r\n", ax_str,
                         ay_str, az_str, gx_str, gy_str, gz_str);
        }
        delay(50);
      }
      Serial.read();
    } else if (cmd == 'r') {
      softUartPrintln("Printing CRSF Kênh RC (Send any key to stop)...");
      while (!Serial.available()) {
        safetyFeedWatchdog();
        crsfUpdate();
        softUartPrintf(
            "RC -> Roll(CH1): %d | Pitch(CH2): %d | Throttle(CH3): %d | "
            "Yaw(CH4): %d | Arm(CH5): %d | LQ: %d%% | RSSI: %d dBm\r\n",
            crsfGetChannel(0), crsfGetChannel(1), crsfGetChannel(2),
            crsfGetChannel(3), crsfGetChannel(4), crsfGetLq(), crsfGetRssi());
        delay(100);
      }
      Serial.read();
    } else if (cmd == 'b') {
      if (safetyGetState() == STATE_DISARMED) {
        blackboxDumpSerial();
      } else {
        softUartPrintln("Dump blackbox rejected. Disarm the drone first.");
      }
    } else if (cmd == 'c') {
      if (safetyGetState() == STATE_DISARMED) {
        softUartPrintln("Re-running Gyroscope startup calibration...");
        safetySetStartupState(STARTUP_BOOT); // Đưa về boot để calib lại
      } else {
        softUartPrintln("Calibration rejected. Disarm first.");
      }
    }
  }

  // ---------------------------------------------------------------------------
  // B. Vòng lặp điều khiển chính (Chạy non-blocking)
  // ---------------------------------------------------------------------------
  uint32_t start_time_us = micros();
  uint32_t dt_us = start_time_us - last_loop_time_us;

  // Chạy các tác vụ FSM khởi động (bao gồm calib gyro phi chặn)
  if (safetyGetStartupState() != STARTUP_READY) {
    updateStartupFsm();
  }

  // Kiểm tra chu kỳ lặp
  if (dt_us >= target_loop_period_us) {
    last_loop_time_us = start_time_us;
    float dt = (float)dt_us / 1000000.0f;

    // 1. Đọc gói dữ liệu từ bộ thu RC CRSF
    crsfUpdate();

    // 2. Đọc cảm biến IMU bằng Burst Read (Sau khi khởi tạo OK)
    bool imu_ok = false;
    if (safetyGetStartupState() == STARTUP_READY) {
      imu_ok = (mpu6050Read(&imu_raw) == 0);
      
      // 3. Ước lượng tư thế góc nghiêng
      if (imu_ok) {
        imuEstimatorUpdate(&imu_raw, dt);
        const Attitude *p_att = imuEstimatorGetAttitude();
        attitude_angles.roll = p_att->roll;
        attitude_angles.pitch = p_att->pitch;
        attitude_angles.yaw = p_att->yaw;
      }
    }

    // 4. Cập nhật Watchdog và State Machine an toàn
    safetyUpdate(imu_ok);

    // 5. Kiểm tra và thực thi các trạng thái điều khiển bay
    FlightState current_fstate = safetyGetState();

    if (current_fstate == STATE_ARMED) {
      // Đọc các kênh điều khiển thô và scale sang đơn vị góc/tốc độ góc
      uint16_t ch_roll = crsfGetChannel(0);
      uint16_t ch_pitch = crsfGetChannel(1);
      uint16_t ch_throttle = crsfGetChannel(2);
      uint16_t ch_yaw = crsfGetChannel(3);

      float target_roll = (float)((int16_t)ch_roll - 1500) * MAX_ROLL_ANGLE_DEG / 500.0f;
      float target_pitch = (float)((int16_t)ch_pitch - 1500) * MAX_PITCH_ANGLE_DEG / 500.0f;
      float target_yaw_rate = (float)((int16_t)ch_yaw - 1500) * MAX_YAW_RATE_DEGS / 500.0f;

      // Xử lý giới hạn ga theo trạng thái nguồn Pin
      BatteryState bat = batteryGetState();
      uint16_t throttle_limit = ch_throttle;

      if (bat == BATTERY_LOW) {
        if (throttle_limit > THROTTLE_LIMIT_LOW_BAT) {
          throttle_limit = THROTTLE_LIMIT_LOW_BAT;
        }
      } else if (bat == BATTERY_CRITICAL) {
        if (throttle_limit > THROTTLE_LIMIT_CRIT_BAT) {
          throttle_limit = THROTTLE_LIMIT_CRIT_BAT;
        }
      }

      // Tính toán thuật toán PID vòng kép
      pidCompute(&attitude_angles, &imu_raw, target_roll, target_pitch,
                 target_yaw_rate, dt, &out_roll, &out_pitch, &out_yaw);

      // Trộn công suất Mixer và xuất ra driver PWM
      uint16_t m1, m2, m3, m4;
      motorMixerCompute(throttle_limit, out_roll, out_pitch, out_yaw, &m1, &m2,
                        &m3, &m4);
      
      // Khóa an toàn bổ sung nếu cấu hình báo ESC chưa được calib
      if (global_config.esc_calibrated == 1) {
        motorWriteAllUs(m1, m2, m3, m4);
      } else {
        motorStopAll(); // Từ chối cấp công suất motor nếu ESC chưa calib
      }
    } else {
      // Nếu không ARMED: khóa an toàn động cơ và reset bộ PID
      motorStopAll();
      pidReset();
      out_roll = 0.0f;
      out_pitch = 0.0f;
      out_yaw = 0.0f;
    }

    // 6. Ghi dữ liệu vào debug Blackbox (chỉ chạy khi READY)
    if (safetyGetStartupState() == STARTUP_READY) {
      blackboxLog(dt_us, out_roll, out_pitch, out_yaw, batteryGetVoltage(),
                  crsfGetLq(), crsfGetRssi());
    }

    // 7. Thực hiện giám sát loop budget và tự động hạ tần số loop nếu I2C quá tải
    uint32_t elapsed_us = micros() - start_time_us;

    if (!is_i2c_fast_mode && target_loop_period_us == CONTROL_LOOP_PERIOD_US) {
      if (elapsed_us > CONTROL_LOOP_PERIOD_US) {
        budget_overrun_counter++;
        if (budget_overrun_counter > 50) {
          target_loop_period_us = FALLBACK_LOOP_PERIOD_US; // Hạ tần số xuống 250Hz
          softUartPrintln("[WARNING] Loop budget overrun! Falling back to 250Hz loop rate.");
        }
      } else {
        if (budget_overrun_counter > 0)
          budget_overrun_counter--;
      }
    }

    // ---------------------------------------------------------------------------
    // C. Tác vụ nền (Background Tasks - Chạy tần số thấp)
    // ---------------------------------------------------------------------------
    uint32_t current_time_ms = millis();
    if (current_time_ms - last_battery_time_ms >= 100) {
      last_battery_time_ms = current_time_ms;
      batteryUpdate();
    }

    // Gửi Telemetry Pin định kỳ mỗi 200ms
    static uint32_t last_telemetry_time_ms = 0;
    if (current_time_ms - last_telemetry_time_ms >= 200) {
      last_telemetry_time_ms = current_time_ms;
      
      float vbat = batteryGetVoltage();
      uint16_t real_volts = (uint16_t)(vbat * 100);
      
      uint8_t percent = 0;
      if (vbat >= 12.6f) {
        percent = 100;
      } else if (vbat <= 10.5f) {
        percent = 0;
      } else {
        percent = (uint8_t)((vbat - 10.5f) / (12.6f - 10.5f) * 100.0f);
      }
      
      crsfSendTelemetryBattery(real_volts, 0, 0, percent);
    }
  }
}