#include "board_pinmap.h"
#include "config.h"
#include "driver/battery_adc.h"
#include "driver/motor_pwm.h"
#include "driver/mpu6050.h"
#include "driver/rc_crsf.h"
#include "driver/soft_i2c.h"
#include "driver/soft_uart.h"
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
static uint32_t target_loop_period_us =
    CONTROL_LOOP_PERIOD_US; // Mặc định 500Hz (2000us)
static bool is_i2c_fast_mode = true;

// Quản lý thời gian chạy các tác vụ nền
static uint32_t last_battery_time_ms = 0;

// Biến lưu trữ thời gian thực thi tác vụ trong loop để giám sát budget
static uint16_t budget_overrun_counter = 0;

// Các biến lưu trữ tạm đầu ra PID
static float out_roll = 0.0f;
static float out_pitch = 0.0f;
static float out_yaw = 0.0f;

// =============================================================================
// Hàm Quét I2C Scanner (Cho chế độ Bring-up)
// =============================================================================
void runI2cScanner() {
  softUartPrintln("--- Starting SoftI2C Scan ---");
  uint8_t count = 0;
  for (uint8_t address = 1; address < 127; address++) {
    uint8_t dummy = 0;
    // Gửi byte kiểm tra
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
// Hàm Khởi động Setup
// =============================================================================
void setup() {
  // Khởi tạo Software UART 19200 baud để phát log
  softUartInit(19200);

  // Khởi tạo cổng truyền Serial phần cứng (cho việc nhận CLI từ RX)
  Serial.begin(115200);
  delay(1000);
  softUartPrintln("=== DRONE FIRMWARE BOOTING ===");

  // 2. Khởi tạo bus Software I2C trên PA9/PA10
  softI2cInit();

  // 3. Khởi tạo cảm biến MPU6050
  // Cố gắng chạy ở 400kHz trước để đáp ứng loop rate 500Hz
  softI2cSetSpeed(400);
  uint8_t imu_status = mpu6050Init();

  if (imu_status != 0) {
    softUartPrintln("[WARNING] IMU MPU6050 failed at 400kHz. Attempting "
                    "Fallback to 100kHz...");

    // Fallback tốc độ I2C xuống 100kHz
    softI2cSetSpeed(100);
    is_i2c_fast_mode = false;
    imu_status = mpu6050Init();

    if (imu_status == 0) {
      softUartPrintln(
          "[WARNING] IMU MPU6050 initialized successfully at 100kHz.");
    } else {
      softUartPrintln(
          "[FATAL] MPU6050 initialization failed completely! Loop will lock.");
    }
  } else {
    softUartPrintln("MPU6050 initialized successfully at 400kHz.");
  }

  // 4. Khởi động hiệu chuẩn MPU6050 (2000 mẫu ~ 2 giây)
  if (imu_status == 0) {
    softUartPrintln(
        "Calibrating MPU6050 (Please keep the drone flat and still)...");
    mpu6050Calibrate();
    softUartPrintln("Calibration completed.");
  }

  // 5. Khởi tạo các Driver & Middleware khác
  crsfInit();

  // Khởi tạo PWM 50Hz (phù hợp với ESC SimonK/ESC vàng của động cơ A2212)
  motorInit(true);

  batteryInit();
  safetyInit();
  blackboxInit();
  imuEstimatorInit();
  pidInit();

  last_loop_time_us = micros();
  last_battery_time_ms = millis();

#if (defined(PRE_FLIGHT_TEST) && (PRE_FLIGHT_TEST == 1))
  softUartPrintln("=================================================");
  softUartPrintln("[WARNING] PRE-FLIGHT TEST MODE IS ACTIVE!");
  softUartPrintln("[WARNING] Motors will spin but throttle is capped.");
  softUartPrintln("[WARNING] DO NOT TRY TO FLY IN THIS MODE!");
  softUartPrintln("=================================================");
#endif

  softUartPrintln("=== DRONE READY ===");
  runI2cScanner();
}

// =============================================================================
// Vòng lặp Loop chính
// =============================================================================
void loop() {
#if defined(TEST_MOTOR_MODE) && (TEST_MOTOR_MODE > 0)
  checkEmergencyStop();
  runTestMode();
  return;
#endif

  // ---------------------------------------------------------------------------
  // A. Hỗ trợ giao diện CLI Bring-up / Test phần cứng qua Serial
  // ---------------------------------------------------------------------------
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == 's') {
      runI2cScanner();
    } else if (cmd == 'i') {
      softUartPrintln("Printing Raw IMU (Send any key to stop)...");
      while (!Serial.available()) {
        safetyFeedWatchdog(); // Tránh watchdog reset trong loop con
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
      Serial.read(); // Đọc ký tự xóa đệm
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
      // Chỉ cho phép dump blackbox khi đã ngắt động cơ an toàn
      if (safetyGetState() == STATE_DISARMED) {
        blackboxDumpSerial();
      } else {
        softUartPrintln("Dump blackbox rejected. Disarm the drone first.");
      }
    } else if (cmd == 'c') {
      if (safetyGetState() == STATE_DISARMED) {
        softUartPrintln("Re-calibrating MPU6050...");
        mpu6050Calibrate();
        softUartPrintln("Re-calibration completed.");
      } else {
        softUartPrintln("Calibration rejected. Disarm first.");
      }
    }
#ifdef TEST_MOTOR_OUTPUT
    else if (cmd == 't') {
      if (safetyGetState() == STATE_DISARMED) {
        softUartPrintln(
            "=== MOTOR TEST MODE ACTIVE (WARNING: REMOVE PROPELLERS) ===");
        softUartPrintln("Send: '1' -> 1000us, '2' -> 1200us, '3' -> 1500us, "
                        "'4' -> 1800us, '0' -> Stop");
        while (true) {
          safetyFeedWatchdog();
          if (Serial.available() > 0) {
            char m_cmd = Serial.read();
            if (m_cmd == '0') {
              motorStopAll();
              softUartPrintln("Motors Stopped");
              break;
            } else if (m_cmd == '1') {
              motorWriteAllUs(1000, 1000, 1000, 1000);
              softUartPrintln("1000us");
            } else if (m_cmd == '2') {
              motorWriteAllUs(1200, 1200, 1200, 1200);
              softUartPrintln("1200us");
            } else if (m_cmd == '3') {
              motorWriteAllUs(1500, 1500, 1500, 1500);
              softUartPrintln("1500us");
            } else if (m_cmd == '4') {
              motorWriteAllUs(1800, 1800, 1800, 1800);
              softUartPrintln("1800us (Be Careful!)");
            }
          }
        }
      }
    }
#endif
  }

  // ---------------------------------------------------------------------------
  // B. Vòng lặp điều khiển chính (Chạy non-blocking)
  // ---------------------------------------------------------------------------
  uint32_t start_time_us = micros();
  uint32_t dt_us = start_time_us - last_loop_time_us;

  // Kiểm tra chu kỳ lặp (Target 500Hz tương đương 2000us, hoặc 250Hz tương
  // đương 4000us)
  if (dt_us >= target_loop_period_us) {
    last_loop_time_us = start_time_us;
    float dt = (float)dt_us / 1000000.0f;

    // 1. Đọc gói dữ liệu từ bộ thu RC CRSF (nhanh, phi chặn)
    crsfUpdate();

    // 2. Đọc cảm biến IMU bằng Burst Read 14 bytes
    bool imu_ok = (mpu6050Read(&imu_raw) == 0);

    // 3. Ước lượng tư thế góc nghiêng
    if (imu_ok) {
      imuEstimatorUpdate(&imu_raw, dt);
      const Attitude *p_att = imuEstimatorGetAttitude();
      attitude_angles.roll = p_att->roll;
      attitude_angles.pitch = p_att->pitch;
      attitude_angles.yaw = p_att->yaw;
    }

    // 4. Cập nhật Watchdog và State Machine an toàn
    safetyUpdate(imu_ok);

    // 5. Kiểm tra và thực thi các trạng thái điều khiển bay
    FlightState current_fstate = safetyGetState();

    if (current_fstate == STATE_ARMED) {
      // Đọc các kênh điều khiển thô và scale sang đơn vị góc/tốc độ góc
      // CH1 (Roll), CH2 (Pitch), CH3 (Throttle), CH4 (Yaw)
      uint16_t ch_roll = crsfGetChannel(0);
      uint16_t ch_pitch = crsfGetChannel(1);
      uint16_t ch_throttle = crsfGetChannel(2);
      uint16_t ch_yaw = crsfGetChannel(3);

      // Chuyển đổi góc: 1500us tương đương 0 độ, dải [1000 - 2000] tương đương
      // [-30, 30] độ
      float target_roll =
          (float)((int16_t)ch_roll - 1500) * MAX_ROLL_ANGLE_DEG / 500.0f;
      float target_pitch =
          (float)((int16_t)ch_pitch - 1500) * MAX_PITCH_ANGLE_DEG / 500.0f;

      // Chuyển đổi Yaw: dải [1000 - 2000] tương đương tốc độ xoay [-150, 150]
      // deg/s
      float target_yaw_rate =
          (float)((int16_t)ch_yaw - 1500) * MAX_YAW_RATE_DEGS / 500.0f;

      // Xử lý giới hạn ga theo trạng thái nguồn Pin
      BatteryState bat = batteryGetState();
      uint16_t throttle_limit = ch_throttle;

      if (bat == BATTERY_LOW) {
        if (throttle_limit > THROTTLE_LIMIT_LOW_BAT) {
          throttle_limit = THROTTLE_LIMIT_LOW_BAT;
        }
      } else if (bat == BATTERY_CRITICAL) {
        // Prototype mode: Giới hạn cần ga cực thấp khi pin nguy kịch, không
        // disarm giữa trời
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
      motorWriteAllUs(m1, m2, m3, m4);
    } else {
      // Nếu không ARMED (DISARMED, PRE_ARM, FAILSAFE): khóa an toàn động cơ và
      // reset bộ PID
      motorStopAll();
      pidReset();
      out_roll = 0.0f;
      out_pitch = 0.0f;
      out_yaw = 0.0f;
    }

    // 6. Ghi dữ liệu vào debug Blackbox
    blackboxLog(dt_us, out_roll, out_pitch, out_yaw, batteryGetVoltage(),
                crsfGetLq(), crsfGetRssi());

    // 7. Thực hiện giám sát loop budget và tự động hạ tần số loop nếu I2C quá
    // tải
    uint32_t elapsed_us = micros() - start_time_us;

    if (!is_i2c_fast_mode && target_loop_period_us == CONTROL_LOOP_PERIOD_US) {
      if (elapsed_us > CONTROL_LOOP_PERIOD_US) {
        budget_overrun_counter++;
        // Nếu bị vượt quá budget 50 chu kỳ liên tiếp
        if (budget_overrun_counter > 50) {
          target_loop_period_us =
              FALLBACK_LOOP_PERIOD_US; // Hạ tần số chính thức xuống 250Hz
          softUartPrintln("[WARNING] Loop budget overrun! Falling back to "
                          "250Hz loop rate.");
        }
      } else {
        if (budget_overrun_counter > 0)
          budget_overrun_counter--;
      }
    }

    // ---------------------------------------------------------------------------
    // C. Tác vụ nền (Background Tasks - Chạy tần số thấp để tránh chiếm dụng
    // loop)
    // ---------------------------------------------------------------------------
    // Định kỳ 100ms cập nhật điện áp pin
    uint32_t current_time_ms = millis();
    if (current_time_ms - last_battery_time_ms >= 100) {
      last_battery_time_ms = current_time_ms;
      batteryUpdate();
    }

    // In log định kỳ ra soft_uart mỗi 1000ms
    static uint32_t last_soft_uart_print_ms = 0;
    if (current_time_ms - last_soft_uart_print_ms >= 1000) {
      last_soft_uart_print_ms = current_time_ms;
      char vbat_str[10];
      dtostrf(batteryGetVoltage(), 4, 2, vbat_str);
      softUartPrintf("System -> State: %s | Vbat: %s V | ELRS: LQ %d%%, RSSI %d dBm\r\n",
                     safetyGetStateStr(safetyGetState()),
                     vbat_str,
                     crsfGetLq(), crsfGetRssi());
    }

    // Gửi Telemetry Pin định kỳ mỗi 200ms về tay điều khiển
    static uint32_t last_telemetry_time_ms = 0;
    if (current_time_ms - last_telemetry_time_ms >= 200) {
      last_telemetry_time_ms = current_time_ms;
      
      // Fake dung lượng pin theo chu kỳ 15 giây (15000ms)
      // 0s - 5s: HIGH (12.60V, 100%)
      // 5s - 10s: MEDIUM (11.40V, 50%)
      // 10s - 15s: LOW (10.20V, 10%)
      uint32_t cycle_ms = current_time_ms % 15000;
      uint16_t fake_volts = 1260; // 12.60V (đơn vị 0.01V)
      uint8_t fake_percent = 100;
      
      if (cycle_ms < 5000) {
        fake_volts = 1260; // HIGH
        fake_percent = 100;
      } else if (cycle_ms < 10000) {
        fake_volts = 1140; // MEDIUM
        fake_percent = 50;
      } else {
        fake_volts = 1020; // LOW
        fake_percent = 10;
      }
      
      // Gửi gói CRSF telemetry
      crsfSendTelemetryBattery(fake_volts, 0, 0, fake_percent);
    }
  }
}