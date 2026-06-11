#include "middleware/test.h"
#include "config.h"
#include "driver/motor_pwm.h"
#include "driver/mpu6050.h"
#include "driver/rc_crsf.h"
#include "driver/soft_uart.h"
#include "middleware/safety.h"

// Biến lưu trữ phần trăm ga hiện tại (0.0f - 100.0f)
static float current_throttle_percent = 0.0f;

// Cờ báo hiệu trạng thái dừng khẩn cấp (Emergency Stop)
static bool is_emergency_stopped = false;

void setThrottlePercent(float percent) {
  // Giới hạn đầu vào phần trăm ga từ 0% đến 100%
  percent = constrain(percent, 0.0f, 100.0f);

  // Tuyến tính hóa từ % sang giá trị xung PWM từ 1000us đến 2000us
  uint16_t pulse = 1000 + (uint16_t)(percent * 10.0f);

  // Output Clamp: Giới hạn cứng đầu ra PWM để đảm bảo an toàn tuyệt đối
  pulse = constrain(pulse, 1000, 2000);

  // Nếu bị dừng khẩn cấp, khóa cứng ga ở mức 1000us (Stop)
  if (is_emergency_stopped) {
    motorWriteAllUs(1000, 1000, 1000, 1000);
  } else {
    motorWriteAllUs(pulse, pulse, pulse, pulse);
  }

  current_throttle_percent = percent;
}

void rampTo(float targetPercent) {
  targetPercent = constrain(targetPercent, 0.0f, 100.0f);

  static uint32_t last_ramp_time = 0;
  uint32_t now = millis();

  // Tăng/giảm tốc độ từ từ với bước thay đổi 1% sau mỗi 50ms (Smooth Ramp)
  if (now - last_ramp_time >= 50) {
    last_ramp_time = now;

    if (current_throttle_percent < targetPercent) {
      current_throttle_percent += 1.0f;
      if (current_throttle_percent > targetPercent) {
        current_throttle_percent = targetPercent;
      }
    } else if (current_throttle_percent > targetPercent) {
      current_throttle_percent -= 1.0f;
      if (current_throttle_percent < targetPercent) {
        current_throttle_percent = targetPercent;
      }
    }

    setThrottlePercent(current_throttle_percent);
  }
}

void checkEmergencyStop() {
  // Đọc cổng Serial phần cứng để nhận dạng lệnh STOP (non-blocking)
  static char stop_buffer[16];
  static uint8_t stop_idx = 0;

  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      stop_buffer[stop_idx] = '\0';
      if (stop_idx > 0) {
        String cmd = String(stop_buffer);
        cmd.trim();
        if (cmd.equalsIgnoreCase("STOP")) {
          is_emergency_stopped = true;
          motorStopAll();
          softUartPrintln("\r\n!!! EMERGENCY STOP TRIGGERED !!!");
          softUartPrintln("Motors forced to 1000us. System locked.");
        }
      }
      stop_idx = 0;
    } else if (stop_idx < sizeof(stop_buffer) - 1) {
      stop_buffer[stop_idx++] = c;
    } else {
      stop_idx = 0;
    }
  }
}

void runTestMode() {
  // Liên tục kiểm tra lệnh dừng khẩn cấp từ cổng Serial
  checkEmergencyStop();

  // Nếu hệ thống đã kích hoạt Dừng khẩn cấp, ép động cơ về mức an toàn và khóa
  // hệ thống
  if (is_emergency_stopped) {
    motorWriteAllUs(1000, 1000, 1000, 1000);
    safetyFeedWatchdog();
    delay(10);
    return;
  }

#if defined(TEST_MOTOR_MODE) && (TEST_MOTOR_MODE > 0)

// =============================================================================
// CHẾ ĐỘ 1: RAMP TEST (TĂNG/GIẢM GA THEO BƯỚC 10%, MỖI MỨC GIỮ 5 GIÂY)
// =============================================================================
#if (TEST_MOTOR_MODE == 1)
  static uint32_t boot_start_time = 0;
  static bool esc_armed = false;

  // Startup Safety: Khởi động ở 1000us trong 3 giây để ESC hoàn tất nhận diện
  if (boot_start_time == 0) {
    boot_start_time = millis();
    softUartPrintln(
        "=== Startup Safety: Keep throttle at 1000us for 3 seconds ===");
  }

  if (!esc_armed) {
    motorWriteAllUs(1000, 1000, 1000, 1000);
    if (millis() - boot_start_time >= 3000) {
      esc_armed = true;
      softUartPrintln("ESC Armed");
      softUartPrintln("Manual Speed Control Ready");
    }
    safetyFeedWatchdog();
    delay(10);
    return;
  }

  // Khai báo các mức phần trăm ga kiểm thử (0% -> 100% -> 0%)
  static const float test_steps[] = {0.0f,  10.0f, 20.0f, 30.0f, 40.0f,  50.0f,
                                     60.0f, 70.0f, 80.0f, 90.0f, 100.0f, 90.0f,
                                     80.0f, 70.0f, 60.0f, 50.0f, 40.0f,  30.0f,
                                     20.0f, 10.0f, 0.0f};
  static const int total_steps = sizeof(test_steps) / sizeof(test_steps[0]);

  static uint32_t step_start_time = 0;
  static int current_step_index = 0;

  if (step_start_time == 0) {
    step_start_time = millis();
    softUartPrintf("Throttle=%d%%\r\n", (int)test_steps[current_step_index]);
  }

  // Cấu hình ga theo bước hiện tại
  setThrottlePercent(test_steps[current_step_index]);

  // Kiểm tra thời gian giữ 5 giây mỗi mức
  if (millis() - step_start_time >= 5000) {
    step_start_time = millis();
    current_step_index++;

    if (current_step_index >= total_steps) {
      current_step_index = 0; // Lặp lại vô hạn
      softUartPrintln("Ramp Test Cycle Completed. Restarting...");
    }

    softUartPrintf("Throttle=%d%%\r\n", (int)test_steps[current_step_index]);
  }

  safetyFeedWatchdog();
  delay(10);
#endif

// =============================================================================
// CHẾ ĐỘ 2: SMOOTH RAMP TEST (TĂNG TỐC VÀ GIẢM TỐC TỪ TỪ SỬ DỤNG HÀM rampTo)
// =============================================================================
#if (TEST_MOTOR_MODE == 2)
  static uint32_t boot_start_time = 0;
  static bool esc_armed = false;

  if (boot_start_time == 0) {
    boot_start_time = millis();
  }

  if (!esc_armed) {
    motorWriteAllUs(1000, 1000, 1000, 1000);
    if (millis() - boot_start_time >= 3000) {
      esc_armed = true;
      softUartPrintln("ESC Armed");
      softUartPrintln("Smooth Ramp Test Mode Active");
    }
    safetyFeedWatchdog();
    delay(10);
    return;
  }

  static float target_percent = 100.0f;
  static uint32_t last_target_change = 0;

  // Thực hiện tăng/giảm ga mượt mà hướng tới target_percent
  rampTo(target_percent);

  // Sau khi đạt được ga đích, giữ tĩnh 3 giây rồi đổi chiều (100% -> 0% hoặc 0%
  // -> 100%)
  if (current_throttle_percent == target_percent) {
    if (last_target_change == 0) {
      last_target_change = millis();
    }
    if (millis() - last_target_change >= 3000) {
      last_target_change = 0;
      target_percent = (target_percent == 100.0f) ? 0.0f : 100.0f;
      softUartPrintf("Smooth Ramp Target Changed -> %d%%\r\n",
                     (int)target_percent);
    }
  } else {
    last_target_change = 0;
  }

  safetyFeedWatchdog();
  delay(10);
#endif

// =============================================================================
// CHẾ ĐỘ 3: ESC CALIBRATION MODE (HIỆU CHUẨN DẢI GA ESC: 2000us -> 3s ->
// 1000us)
// =============================================================================
#if (TEST_MOTOR_MODE == 3)
  static uint32_t calibration_start = 0;
  static bool step1_done = false;
  static bool step2_done = false;

  if (calibration_start == 0) {
    calibration_start = millis();
    softUartPrintln("=== ESC Calibration Mode ===");
    softUartPrintln(
        "Step 1: Sending Max Throttle (2000us). Connect battery now!");
  }

  if (!step1_done) {
    // Không bao giờ phát >1000us lúc boot bình thường, ngoại trừ lúc
    // Calibration được thiết lập rõ ràng
    motorWriteAllUs(2000, 2000, 2000, 2000);
    if (millis() - calibration_start >= 3000) {
      step1_done = true;
      calibration_start = millis();
      softUartPrintln(
          "Step 2: Sending Min Throttle (1000us). Wait for confirmation...");
    }
  } else if (!step2_done) {
    motorWriteAllUs(1000, 1000, 1000, 1000);
    if (millis() - calibration_start >= 3000) {
      step2_done = true;
      softUartPrintln("ESC Calibration Completed!");
      softUartPrintln("You can power cycle the drone now.");
    }
  } else {
    motorWriteAllUs(1000, 1000, 1000, 1000);
  }

  safetyFeedWatchdog();
  delay(10);
#endif

// =============================================================================
// CHẾ ĐỘ 4: STRESS TEST (TĂNG 0 -> 100% RỒI GIẢM VỀ 0%, LẶP LẠI 100 LẦN)
// =============================================================================
#if (TEST_MOTOR_MODE == 4)
  static uint32_t boot_start_time = 0;
  static bool esc_armed = false;

  if (boot_start_time == 0) {
    boot_start_time = millis();
  }

  if (!esc_armed) {
    motorWriteAllUs(1000, 1000, 1000, 1000);
    if (millis() - boot_start_time >= 3000) {
      esc_armed = true;
      softUartPrintln("ESC Armed");
      softUartPrintln("Stress Test Mode: Running 100 cycles...");
    }
    safetyFeedWatchdog();
    delay(10);
    return;
  }

  static uint32_t cycle_start_time = 0;
  static int current_cycle = 0;
  static bool is_at_high = false;

  if (current_cycle < 100) {
    if (cycle_start_time == 0) {
      cycle_start_time = millis();
      softUartPrintf("Stress Cycle %d/100 -> %s\r\n", current_cycle + 1,
                     is_at_high ? "100%" : "0%");
    }

    if (is_at_high) {
      setThrottlePercent(100.0f);
    } else {
      setThrottlePercent(0.0f);
    }

    // Giữ mỗi trạng thái (0% hoặc 100%) trong 1 giây để kiểm tra sốc dòng,
    // nhiệt, rung động
    if (millis() - cycle_start_time >= 1000) {
      cycle_start_time = millis();
      if (is_at_high) {
        is_at_high = false;
        current_cycle++; // Tăng đếm chu kỳ
      } else {
        is_at_high = true;
      }
    }
  } else {
    // Kết thúc stress test
    motorWriteAllUs(1000, 1000, 1000, 1000);
    static bool printed_done = false;
    if (!printed_done) {
      printed_done = true;
      softUartPrintln("=== Stress Test Completed 100 cycles successfully ===");
    }
  }

  safetyFeedWatchdog();
  delay(10);
#endif

// =============================================================================
// CHẾ ĐỘ 5: MOTOR DIRECTION TEST (QUAY 10% GA VÀ IN HƯỚNG DẪN ĐẢO CHIỀU)
// =============================================================================
#if (TEST_MOTOR_MODE == 5)
  static uint32_t boot_start_time = 0;
  static bool esc_armed = false;

  if (boot_start_time == 0) {
    boot_start_time = millis();
  }

  if (!esc_armed) {
    motorWriteAllUs(1000, 1000, 1000, 1000);
    if (millis() - boot_start_time >= 3000) {
      esc_armed = true;
      softUartPrintln("ESC Armed");
      softUartPrintln("Motor Direction Check");
      softUartPrintln("Note: Swap any 2 of the 3 phase wires to change motor "
                      "rotation direction.");
    }
    safetyFeedWatchdog();
    delay(10);
    return;
  }

  // Quay cả 4 motor ở 10% để kiểm tra chiều quay trực quan
  setThrottlePercent(10.0f);

  safetyFeedWatchdog();
  delay(10);
#endif

// =============================================================================
// CHẾ ĐỘ 6: TEST CẢM BIẾN MPU6050 (IN LIÊN TỤC GIA TỐC VÀ GÓC QUAY)
// =============================================================================
#if (TEST_MOTOR_MODE == 6)
  static uint32_t last_print_time = 0;
  MpuData temp_imu;

  if (mpu6050Read(&temp_imu) == 0) {
    uint32_t now = millis();
    if (now - last_print_time >= 100) {
      last_print_time = now;
      char ax_str[10], ay_str[10], az_str[10];
      char gx_str[10], gy_str[10], gz_str[10];
      dtostrf(temp_imu.ax, 6, 2, ax_str);
      dtostrf(temp_imu.ay, 6, 2, ay_str);
      dtostrf(temp_imu.az, 6, 2, az_str);
      dtostrf(temp_imu.gx, 6, 2, gx_str);
      dtostrf(temp_imu.gy, 6, 2, gy_str);
      dtostrf(temp_imu.gz, 6, 2, gz_str);
      softUartPrintf("MPU Test -> ACC: %s, %s, %s | GYRO: %s, %s, %s\r\n",
                     ax_str, ay_str, az_str, gx_str, gy_str, gz_str);
    }
  }

  safetyFeedWatchdog();
  delay(10);
#endif

#if (TEST_MOTOR_MODE == 7)
  static uint32_t last_print_time = 0;

  // Cập nhật gói dữ liệu CRSF
  crsfUpdate();

  uint32_t now = millis();
  if (now - last_print_time >= 500) { // In định kỳ mỗi 500ms
    last_print_time = now;

    uint16_t ch1 = crsfGetChannel(0); // Roll
    uint16_t ch2 = crsfGetChannel(1); // Pitch
    uint16_t ch3 = crsfGetChannel(2); // Throttle
    uint16_t ch4 = crsfGetChannel(3); // Yaw
    uint16_t ch5 = crsfGetChannel(4); // Arm (AUX1)
    uint8_t lq = crsfGetLq();
    int8_t rssi = crsfGetRssi();
    uint32_t rx_cnt = crsfGetRxCnt();

    softUartPrintf("ELRS -> Roll(CH1): %d | Pitch(CH2): %d | Throttle(CH3): %d "
                   "| Yaw(CH4): %d | Arm(CH5): %d | LQ: %d%% | RSSI: %d dBm | "
                   "RX_Bytes: %lu\r\n",
                   ch1, ch2, ch3, ch4, ch5, lq, rssi, (unsigned long)rx_cnt);
  }

  safetyFeedWatchdog();
  delay(5);
#endif

#if (TEST_MOTOR_MODE == 8)
  static uint32_t boot_start_time = 0;
  static bool esc_armed = false;

  // Startup Safety: Giữ ga 1000us trong 3 giây để arm ESC
  if (boot_start_time == 0) {
    boot_start_time = millis();
    softUartPrintln("=== RC Direct Control Mode ===");
    softUartPrintln("=== Startup Safety: Arming ESC at 1000us for 3 seconds ===");
  }

  if (!esc_armed) {
    motorWriteAllUs(1000, 1000, 1000, 1000);
    if (millis() - boot_start_time >= 3000) {
      esc_armed = true;
      softUartPrintln("ESC Armed! Ready for RC Throttle Control.");
    }
    safetyFeedWatchdog();
    delay(10);
    return;
  }

  // Cập nhật dữ liệu ELRS
  crsfUpdate();

  uint16_t ch3 = crsfGetChannel(2); // Kênh 3: Throttle

  // Giới hạn an toàn trước khi bay (để không bay mất khi test)
  uint16_t pulse = ch3;
#if (defined(PRE_FLIGHT_TEST) && (PRE_FLIGHT_TEST == 1))
  if (pulse > PRE_FLIGHT_MAX_PULSE) {
    pulse = PRE_FLIGHT_MAX_PULSE;
  }
#endif
  pulse = constrain(pulse, 1000, 2000);

  // Xuất xung cho cả 4 động cơ
  motorWriteAllUs(pulse, pulse, pulse, pulse);

  // In log định kỳ mỗi 200ms
  static uint32_t last_print_time = 0;
  uint32_t now = millis();
  if (now - last_print_time >= 200) {
    last_print_time = now;
    softUartPrintf("RC Throttle: Input=%d us | Output PWM=%d us\r\n", ch3, pulse);
  }

  safetyFeedWatchdog();
  delay(5);
#endif

#endif
}
