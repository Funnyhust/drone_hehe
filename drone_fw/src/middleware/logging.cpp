#include "middleware/logging.h"
#include "board_pinmap.h"
#include "driver/battery_adc.h"
#include "driver/motor_pwm.h"
#include "driver/mpu6050.h"
#include "driver/rc_crsf.h"
#include "driver/soft_i2c.h"
#include "driver/soft_uart.h"
#include <math.h>

enum LoggingMode {
  LOG_MODE_IDLE = 0,
  LOG_MODE_RC_CHANNELS,
  LOG_MODE_I2C_SCAN,
  LOG_MODE_RAW_GYRO,
  LOG_MODE_RAW_ACCEL,
  LOG_MODE_ATTITUDE,
  LOG_MODE_LED_TEST,
  LOG_MODE_BATTERY,
  LOG_MODE_OFFSETS,
  LOG_MODE_VIBRATION,
  LOG_MODE_MAX
};

static LoggingMode active_mode = LOG_MODE_IDLE;
static int last_ch7_state = -1;
static int last_ch6_state = -1;
static int selected_motor_index = 0; // 0: M1, 1: M2, 2: M3, 3: M4, 4: ALL
static uint32_t last_print_ms = 0;
static uint32_t mode_entered_ms = 0;
static bool first_run_in_mode = false;

// Các biến phục vụ bài test rung động động cơ
static bool throttle_init_ok = false;
static uint32_t wait_timer = 0;
static bool waiting_for_throttle = false;
static int32_t vibration_array[17] = {0};
static int32_t vibration_total_result = 0;
static uint8_t vibration_counter = 0;

// Trạng thái start giả lập YMFC-32
static uint8_t start_status = 0;

static void printMenu() {
  softUartPrintln(
      "\r\n================ YMFC-32 SETUP LOGGING ================");
  softUartPrintln("Gat CH7 (AUX3) de chuyen giua cac che do test.");
  softUartPrintln("Danh sach Mode:");
  softUartPrintln(" 0. IDLE / Main Menu");
  softUartPrintln(" 1. In 6 kenh tay dieu khien (YMFC-32 'a')");
  softUartPrintln(" 2. Quet dia chi I2C Bus (YMFC-32 'b')");
  softUartPrintln(" 3. In gia tri thô Gyroscope (YMFC-32 'c')");
  softUartPrintln(" 4. In gia tri thô Accelerometer (YMFC-32 'd')");
  softUartPrintln(" 5. In goc nghiêng IMU (YMFC-32 'e')");
  softUartPrintln(" 6. Test LED canh bao (YMFC-32 'f')");
  softUartPrintln(" 7. In dien ap nguon Pin (YMFC-32 'g')");
  softUartPrintln(" 8. In gia tri Offsets da calib (YMFC-32 'h')");
  softUartPrintln(
      " 9. Test rung dong dong co (YMFC-32 '1'-'5') - Gat CH6 chon motor");
  softUartPrintln("======================================================");
}

static void printModeIntro(LoggingMode mode) {
  softUartPrintf("\r\n[MODE CHANGE] Entering Mode %d: ", (int)mode);
  switch (mode) {
  case LOG_MODE_IDLE:
    softUartPrintln("IDLE");
    printMenu();
    break;
  case LOG_MODE_RC_CHANNELS:
    softUartPrintln("RC Channels. Dang in tin hieu tay dieu khien...");
    break;
  case LOG_MODE_I2C_SCAN:
    softUartPrintln("I2C Scanner. Dang quet thiet bi...");
    break;
  case LOG_MODE_RAW_GYRO:
    softUartPrintln("Raw Gyro. Giu drone dung yen...");
    break;
  case LOG_MODE_RAW_ACCEL:
    softUartPrintln("Raw Accelerometer.");
    break;
  case LOG_MODE_ATTITUDE:
    softUartPrintln("IMU Angles (Roll, Pitch, Yaw).");
    break;
  case LOG_MODE_LED_TEST:
    softUartPrintln("LED Test. Dang chay chu ky LED...");
    break;
  case LOG_MODE_BATTERY:
    softUartPrintln("Battery Voltage.");
    break;
  case LOG_MODE_OFFSETS:
    softUartPrintln("Calibrated Offsets.");
    break;
  case LOG_MODE_VIBRATION:
    softUartPrintln("Motor Vibration Test. Gat ga ve min de bat dau. Gat CH6 "
                    "de doi motor.");
    softUartPrintln("  M1 (Truoc Phai) -> M2 (Sau Phai) -> M3 (Sau Trai) -> M4 "
                    "(Truoc Trai) -> ALL");
    break;
  default:
    break;
  }
}

void loggingInit() {
  active_mode = LOG_MODE_IDLE;
  last_ch7_state = -1;
  last_ch6_state = -1;
  selected_motor_index = 0;
  last_print_ms = 0;
  throttle_init_ok = false;
  waiting_for_throttle = false;
  vibration_total_result = 0;
  vibration_counter = 0;
  start_status = 0;
  printMenu();
}

bool loggingGetMotorCommand(uint16_t *m1, uint16_t *m2, uint16_t *m3,
                            uint16_t *m4) {
  if (active_mode != LOG_MODE_VIBRATION || !throttle_init_ok) {
    return false;
  }

  uint16_t ch_throttle = crsfGetChannel(2);

  // Gioi han ga an toan khi test tren ban (toi da 1500us ga de tránh bay mat)
  uint16_t test_throttle = ch_throttle;
  if (test_throttle > 1500) {
    test_throttle = 1500;
  }

  *m1 = 1000;
  *m2 = 1000;
  *m3 = 1000;
  *m4 = 1000;

  switch (selected_motor_index) {
  case 0:
    *m1 = test_throttle; // M1: Truoc Phai (Front Right)
    break;
  case 1:
    *m2 = test_throttle; // M2: Sau Phai (Rear Right)
    break;
  case 2:
    *m3 = test_throttle; // M3: Sau Trai (Rear Left)
    break;
  case 3:
    *m4 = test_throttle; // M4: Truoc Trai (Front Left)
    break;
  case 4:
    *m1 = test_throttle;
    *m2 = test_throttle;
    *m3 = test_throttle;
    *m4 = test_throttle;
    break;
  }
  return true;
}

void loggingUpdate(uint16_t ch_roll, uint16_t ch_pitch, uint16_t ch_throttle,
                   uint16_t ch_yaw, uint16_t ch5, uint16_t ch6, uint16_t ch7,
                   const MpuData *p_imu, const Attitude *p_att) {
  uint32_t now = millis();

  // Chia gia tri CH7 thanh 3 phan de phat hien chuyen trang thai
  int current_ch7_state = 0;
  if (ch7 < 1300)
    current_ch7_state = 0;
  else if (ch7 < 1700)
    current_ch7_state = 1;
  else
    current_ch7_state = 2;

  // Chi phat hien va chuyen che do khi tay dieu khien da ket noi song va nhan
  // duoc goi tin RC Channels dau tien
  if (crsfIsLinkActive() && crsfHasRcChannels()) {
    if (last_ch7_state == -1) {
      last_ch7_state = current_ch7_state;
    } else if (current_ch7_state != last_ch7_state) {
      last_ch7_state = current_ch7_state;
      active_mode = (LoggingMode)((active_mode + 1) % LOG_MODE_MAX);
      mode_entered_ms = now;
      first_run_in_mode = true;
      throttle_init_ok = false;
      waiting_for_throttle = false;
      vibration_total_result = 0;
      vibration_counter = 0;
      last_ch6_state = -1;
      selected_motor_index = 0;
      printModeIntro(active_mode);
    }
  } else {
    // Reset khi mat ket noi de cho phien ket noi tiep theo, tranh lay sai moc
    // cu
    last_ch7_state = -1;
  }

  switch (active_mode) {
  case LOG_MODE_IDLE:
    break;

  case LOG_MODE_RC_CHANNELS:
    if (now - last_print_ms >= 250) {
      last_print_ms = now;

      // Gia lap logic khoi dong cua YMFC-32
      if (ch_throttle < 1100 && ch_yaw < 1100)
        start_status = 1;
      if (start_status == 1 && ch_throttle < 1100 && ch_yaw > 1450)
        start_status = 2;
      if (start_status == 2 && ch_throttle < 1100 && ch_yaw > 1900)
        start_status = 0;

      softUartPrintf("Start:%d  Roll:", start_status);
      if ((int16_t)ch_roll - 1480 < 0)
        softUartPrint("<<<");
      else if ((int16_t)ch_roll - 1520 > 0)
        softUartPrint(">>>");
      else
        softUartPrint("-+-");
      softUartPrintf("%d  Pitch:", ch_roll);

      if ((int16_t)ch_pitch - 1480 < 0)
        softUartPrint("^^^");
      else if ((int16_t)ch_pitch - 1520 > 0)
        softUartPrint("vvv");
      else
        softUartPrint("-+-");
      softUartPrintf("%d  Throttle:", ch_pitch);

      if ((int16_t)ch_throttle - 1480 < 0)
        softUartPrint("vvv");
      else if ((int16_t)ch_throttle - 1520 > 0)
        softUartPrint("^^^");
      else
        softUartPrint("-+-");
      softUartPrintf("%d  Yaw:", ch_throttle);

      if ((int16_t)ch_yaw - 1480 < 0)
        softUartPrint("<<<");
      else if ((int16_t)ch_yaw - 1520 > 0)
        softUartPrint(">>>");
      else
        softUartPrint("-+-");
      softUartPrintf("%d  CH5:%d  CH6:%d  CH7:%d\r\n", ch_yaw, ch5, ch6, ch7);
    }
    break;

  case LOG_MODE_I2C_SCAN:
    if (first_run_in_mode) {
      first_run_in_mode = false;
      softUartPrintln("Scanning address 1 till 127...");
      uint16_t nDevices = 0;
      for (uint8_t address = 1; address < 127; address++) {
        // Giao tiep I2C tim thiet bi phan hoi
        uint8_t err = softI2cScanAddress(address);
        if (err == I2C_OK) {
          softUartPrintf("I2C device found at address 0x");
          if (address < 16) {
            softUartPrint("0");
          }
          softUartPrintln(address, HEX);
          nDevices++;
        }
      }
      if (nDevices == 0) {
        softUartPrintln("No I2C devices found");
      } else {
        softUartPrintln("done");
      }
    }
    break;

  case LOG_MODE_RAW_GYRO:
    if (now - last_print_ms >= 250) {
      last_print_ms = now;
      softUartPrintf("Gyro_x = %d  Gyro_y = %d  Gyro_z = %d\r\n", p_imu->gx_raw,
                     p_imu->gy_raw, p_imu->gz_raw);
    }
    break;

  case LOG_MODE_RAW_ACCEL:
    if (now - last_print_ms >= 250) {
      last_print_ms = now;
      softUartPrintf("ACC_x = %d  ACC_y = %d  ACC_z = %d\r\n", p_imu->ax_raw,
                     p_imu->ay_raw, p_imu->az_raw);
    }
    break;

  case LOG_MODE_ATTITUDE:
    if (now - last_print_ms >= 250) {
      last_print_ms = now;
      char p_str[10], r_str[10], y_str[10], t_str[10];
      dtostrf(p_att->pitch, 5, 1, p_str);
      dtostrf(p_att->roll, 5, 1, r_str);
      dtostrf(p_att->yaw, 5, 1, y_str);
      dtostrf(p_imu->temp, 5, 1, t_str);
      // In Pitch, Roll, Yaw va nhiet do cam bien duoi dang chuoi %s
      softUartPrintf("Pitch: %s  Roll: %s  Yaw: %s  Temp: %s\r\n", p_str, r_str,
                     y_str, t_str);
    }
    break;

  case LOG_MODE_LED_TEST:
    if (first_run_in_mode) {
      first_run_in_mode = false;
      softUartPrintln("The red LED is now ON for 3 seconds");
    } else if (now - mode_entered_ms >= 3000 && now - mode_entered_ms < 6000) {
      static bool green_notified = false;
      if (!green_notified) {
        softUartPrintln("The green LED is now ON for 3 seconds");
        green_notified = true;
      }
    } else if (now - mode_entered_ms >= 6000) {
      active_mode = LOG_MODE_IDLE;
      printModeIntro(active_mode);
    }
    break;

  case LOG_MODE_BATTERY:
    if (now - last_print_ms >= 1000) {
      last_print_ms = now;
      float vbat = batteryGetVoltage();
      char v_str[10];
      dtostrf(vbat, 4, 1, v_str);
      softUartPrintf("Voltage = %sV\r\n", v_str);
    }
    break;

  case LOG_MODE_OFFSETS:
    if (first_run_in_mode) {
      first_run_in_mode = false;
      int16_t ax_o, ay_o, az_o, gx_o, gy_o, gz_o;
      mpu6050GetOffsets(&ax_o, &ay_o, &az_o, &gx_o, &gy_o, &gz_o);
      softUartPrintln("Offsets hien tai:");
      softUartPrintf("  Accel Offsets: X=%d, Y=%d, Z=%d\r\n", ax_o, ay_o, az_o);
      softUartPrintf("  Gyro Offsets:  X=%d, Y=%d, Z=%d\r\n", gx_o, gy_o, gz_o);
    }
    break;

  case LOG_MODE_VIBRATION:
    // Doc trang thai CH6 de xoay vong motor ngay ca khi chua mo throttle
    {
      int current_ch6_state = 0;
      if (ch6 < 1300)
        current_ch6_state = 0;
      else if (ch6 < 1700)
        current_ch6_state = 1;
      else
        current_ch6_state = 2;

      if (last_ch6_state == -1) {
        last_ch6_state = current_ch6_state;
      } else if (current_ch6_state != last_ch6_state) {
        last_ch6_state = current_ch6_state;
        selected_motor_index = (selected_motor_index + 1) % 5;
        softUartPrint("\r\n[MOTOR SELECT] Selected: ");
        switch (selected_motor_index) {
        case 0:
          softUartPrintln("M1 (Truoc Phai)");
          break;
        case 1:
          softUartPrintln("M2 (Sau Phai)");
          break;
        case 2:
          softUartPrintln("M3 (Sau Trai)");
          break;
        case 3:
          softUartPrintln("M4 (Truoc Trai)");
          break;
        case 4:
          softUartPrintln("ALL (Tat ca)");
          break;
        }
      }
    }

    if (!throttle_init_ok) {
      if (!waiting_for_throttle) {
        waiting_for_throttle = true;
        wait_timer = now + 10000;
        softUartPrintln("Throttle is not in the lowest position.");
        softUartPrintf("Throttle value is: %d\r\n", ch_throttle);
        softUartPrint("Waiting for 10 seconds:");
      }

      if (ch_throttle < 1050) {
        throttle_init_ok = true;
        waiting_for_throttle = false;
        softUartPrintln(" OK!");
      } else if (now >= wait_timer) {
        softUartPrintln(" Timeout! Returning to idle.");
        active_mode = LOG_MODE_IDLE;
        printModeIntro(active_mode);
      } else {
        static uint32_t last_dot_ms = 0;
        if (now - last_dot_ms >= 500) {
          last_dot_ms = now;
          softUartPrint(".");
        }
      }
    } else {
      // Chi tinh toan va in do rung khi ga dang mo (throttle >= 1050)
      if (ch_throttle >= 1050) {
        // Tinh toan do rung
        float total_acc = sqrt((float)p_imu->ax_raw * p_imu->ax_raw +
                               (float)p_imu->ay_raw * p_imu->ay_raw +
                               (float)p_imu->az_raw * p_imu->az_raw);

        for (int i = 16; i > 0; i--) {
          vibration_array[i] = vibration_array[i - 1];
        }
        vibration_array[0] = (int32_t)total_acc;

        int32_t average_vibration_level = 0;
        for (int i = 0; i < 17; i++) {
          average_vibration_level += vibration_array[i];
        }
        average_vibration_level /= 17;

        vibration_total_result +=
            abs(vibration_array[0] - average_vibration_level);
        vibration_counter++;

        if (vibration_counter >= 20) {
          vibration_counter = 0;
          /*
          const char *m_name = "";
          switch (selected_motor_index) {
          case 0:
            m_name = "M1 (Truoc Phai)";
            break;
          case 1:
            m_name = "M2 (Sau Phai)";
            break;
          case 2:
            m_name = "M3 (Sau Trai)";
            break;
          case 3:
            m_name = "M4 (Truoc Trai)";
            break;
          case 4:
            m_name = "ALL";
            break;
          }
          // softUartPrintf("[%s] Vibration level: %d\r\n", m_name,
          // vibration_total_result / 50);
          */
          vibration_total_result = 0;
        }
      } else {
        // Reset data khi ga ve min
        vibration_counter = 0;
        vibration_total_result = 0;
      }
    }
    break;

  default:
    break;
  }
}
