#include "middleware/safety.h"
#include "board_pinmap.h"
#include "driver/motor_pwm.h"
#include "driver/rc_crsf.h"
#include "driver/battery_adc.h"
#include "driver/soft_uart.h"

// Trạng thái bay hiện tại của hệ thống
static FlightState current_state = STATE_DISARMED;

// Trạng thái khởi động hiện tại
static StartupState current_startup_state = STARTUP_BOOT;
static StartupError current_startup_error = ERR_NONE;

// Bộ đếm số lần lỗi đọc IMU liên tiếp
static uint16_t imu_error_counter = 0;

void safetyInit() {
  current_state = STATE_DISARMED;
  current_startup_state = STARTUP_BOOT;
  current_startup_error = ERR_NONE;
  imu_error_counter = 0;

  // Đảm bảo động cơ dừng an toàn khi mới bật nguồn
  motorStopAll();

  // Cấu hình Independent Watchdog (IWDG) trên STM32F103
  // LSI clock ~40kHz. Đặt Prescaler = 64 (IWDG_PR = 0x04) -> Tần số đếm counter = 625Hz
  // Đặt reload counter = 625 -> timeout = 625 * (1/625) = 1.0s = 1000ms
  IWDG->KR = 0x5555; // Cho phép ghi vào PR và RLR
  IWDG->PR = 0x04;   // Prescaler = 64
  IWDG->RLR = 625;   // Reload = 625 (tương ứng ~1000ms)
  IWDG->KR = 0xAAAA; // Reload counter ban đầu
  IWDG->KR = 0xCCCC; // Bắt đầu chạy Watchdog
}

void safetyUpdate(bool imu_ok) {
  // 1. Feed Watchdog liên tục ở mỗi vòng lặp để giữ chip không bị reset
  safetyFeedWatchdog();

  // 2. Theo dõi số lần lỗi IMU liên tiếp
  if (imu_ok) {
    imu_error_counter = 0;
  } else {
    imu_error_counter++;
  }

  // 3. Đọc dữ liệu trạng thái điều khiển
  bool link_active = crsfIsLinkActive();
  BatteryState bat_state = batteryGetState();
  uint16_t throttle = crsfGetChannel(2);   // CH3 là ga (chỉ số 2)
  uint16_t arm_switch = crsfGetChannel(4); // CH5 là công tắc Arm (chỉ số 4)

  FlightState previous_state = current_state;

  // 4. Mô hình Trạng thái (ARM/DISARM State Machine)
  switch (current_state) {
    case STATE_DISARMED:
      // Chi tat dong co mot lan khi vua chuyen sang DISARMED de tranh xung dot voi che do test motor
      if (previous_state != STATE_DISARMED) {
        motorStopAll();
      }

      // Chi cho phep kich hoat Arm khi cong tac chuyen trang thai tu Low (< 1300us) sang High (> 1500us)
      static bool arm_switch_was_low = false; // Tránh tự động Arm khi bật nguồn với switch ở mức cao
      if (arm_switch < 1300) {
        arm_switch_was_low = true;
      }

      // Kiểm tra công tắc Arm gạt lên mức cao (> 1500us)
      if (arm_switch > 1500 && arm_switch_was_low) {
        arm_switch_was_low = false;
        softUartPrintf("[ARM REQUEST] CH5=%d throttle=%d startup=%s link=%d imu_err=%d bat=%d\r\n",
                       arm_switch, throttle,
                       safetyGetStartupStateStr(current_startup_state),
                       link_active ? 1 : 0, imu_error_counter, bat_state);
        current_state = STATE_PRE_ARM;
      }
      break;

    case STATE_PRE_ARM:
      // Thực hiện các kiểm tra điều kiện an toàn nghiêm ngặt (Pre-arm Checks)
      if (current_startup_state == STARTUP_READY &&   // 1. Startup Calibration phải hoàn thành và READY
          link_active &&                              // 2. Sóng tốt
          throttle < 1050 &&                          // 3. Ga ở mức tối thiểu
          bat_state != BATTERY_CRITICAL &&            // 4. Pin không nguy kịch
          imu_error_counter == 0)                     // 5. IMU bình thường không lỗi đọc
      {
        softUartPrintf("[ARM OK] throttle=%d CH5=%d LQ=%d RSSI=%d\r\n",
                       throttle, arm_switch, crsfGetLq(), crsfGetRssi());
        current_state = STATE_ARMED;
      } else {
        // Nếu không đạt, in lý do từ chối Arm (chỉ in 1 lần)
        softUartPrint("[ARM REJECTED] Reasons: ");
        if (current_startup_state != STARTUP_READY) {
          softUartPrintf("Startup NOT READY (%s, Err: %s); ", 
                         safetyGetStartupStateStr(current_startup_state),
                         safetyGetStartupErrorStr(current_startup_error));
        }
        if (!link_active) softUartPrint("RC Link Inactive; ");
        if (throttle >= 1050) { softUartPrintf("Throttle high (%dus); ", throttle); }
        if (bat_state == BATTERY_CRITICAL) softUartPrint("Battery Critical; ");
        if (imu_error_counter > 0) softUartPrint("IMU Error; ");
        softUartPrintf("raw: CH5=%d T=%d LQ=%d RSSI=%d bat=%d imu_err=%d",
                       arm_switch, throttle, crsfGetLq(), crsfGetRssi(),
                       bat_state, imu_error_counter);
        softUartPrintln("");
        // Quay về DISARMED
        current_state = STATE_DISARMED;
      }
      break;

    case STATE_ARMED:
      // Động cơ được điều khiển bình thường qua Mixer (xử lý ở loop chính)

      // Kiểm tra Failsafe (sự cố đột ngột khi đang bay)
      // 1. Mất sóng cứng (quá 200ms)
      // 2. Lỗi đọc IMU liên tục quá 5 lần
      if (!link_active || imu_error_counter > 5) {
        softUartPrintf("[FAILSAFE] link=%d imu_err=%d CH5=%d throttle=%d LQ=%d RSSI=%d\r\n",
                       link_active ? 1 : 0, imu_error_counter, arm_switch,
                       throttle, crsfGetLq(), crsfGetRssi());
        current_state = STATE_FAILSAFE;
      }
      // Người dùng gạt công tắc Disarm chủ động (< 1300us)
      else if (arm_switch < 1300) {
        softUartPrintf("[DISARM] CH5 low (%d). Motors locked.\r\n", arm_switch);
        current_state = STATE_DISARMED;
      }
      break;

    case STATE_FAILSAFE:
      // Lập tức dừng khẩn cấp toàn bộ động cơ
      motorStopAll();

      // Chỉ cho phép thoát Failsafe về DISARMED nếu gạt switch Arm về mức thấp,
      // đồng thời sóng và cảm biến đã hồi phục ổn định.
      if (arm_switch < 1300 && link_active && imu_error_counter == 0) {
        softUartPrintln("[FAILSAFE CLEAR] Back to DISARMED.");
        current_state = STATE_DISARMED;
      }
      break;

    default:
      current_state = STATE_DISARMED;
      break;
  }

  // In log debug khi trạng thái thay đổi
  if (current_state != previous_state) {
    softUartPrintf("[STATE CHANGE] %s -> %s\r\n", 
                   safetyGetStateStr(previous_state), 
                   safetyGetStateStr(current_state));
  }
}

void safetyFeedWatchdog() {
  IWDG->KR = 0xAAAA; // Ghi 0xAAAA để reload lại thanh ghi counter IWDG
}

FlightState safetyGetState() {
  return current_state;
}

const char* safetyGetStateStr(FlightState state) {
  switch (state) {
    case STATE_DISARMED: return "DISARMED";
    case STATE_PRE_ARM:  return "PRE_ARM";
    case STATE_ARMED:     return "ARMED";
    case STATE_FAILSAFE:  return "FAILSAFE";
    default:              return "UNKNOWN";
  }
}

bool safetyRequestArm() {
  if (current_state == STATE_DISARMED) {
    current_state = STATE_PRE_ARM;
    return true;
  }
  return false;
}

void safetyRequestDisarm() {
  current_state = STATE_DISARMED;
  motorStopAll();
}

// =============================================================================
// Các API quản lý Máy trạng thái Khởi động (Startup State Machine)
// =============================================================================

StartupState safetyGetStartupState() {
  return current_startup_state;
}

void safetySetStartupState(StartupState state) {
  current_startup_state = state;
}

StartupError safetyGetStartupError() {
  return current_startup_error;
}

void safetySetStartupError(StartupError error) {
  current_startup_error = error;
}

const char* safetyGetStartupStateStr(StartupState state) {
  switch (state) {
    case STARTUP_BOOT:       return "BOOT";
    case STARTUP_IMU_INIT:   return "IMU_INIT";
    case STARTUP_GYRO_CALIB: return "GYRO_CALIB";
    case STARTUP_RC_CHECK:   return "RC_CHECK";
    case STARTUP_ACC_CHECK:  return "ACC_CHECK";
    case STARTUP_ESC_CHECK:  return "ESC_CHECK";
    case STARTUP_READY:      return "READY";
    case STARTUP_ERROR:      return "ERROR";
    default:                 return "UNKNOWN";
  }
}

const char* safetyGetStartupErrorStr(StartupError error) {
  switch (error) {
    case ERR_NONE:                 return "NONE";
    case ERR_IMU_INIT_FAIL:        return "IMU_INIT_FAIL";
    case ERR_GYRO_CALIB_MOVING:    // 2 nhịp chớp
      return "GYRO_CALIB_MOVING";
    case ERR_RC_LINK_LOST:         // 3 nhịp chớp
      return "RC_LINK_LOST";
    case ERR_RC_CENTER_OUT:        // 4 nhịp chớp
      return "RC_CENTER_OUT";
    case ERR_THROTTLE_NOT_MIN:     // 5 nhịp chớp
      return "THROTTLE_NOT_MIN";
    case ERR_ACC_VALIDATION_FAIL:  // 6 nhịp chớp
      return "ACC_VALIDATION_FAIL";
    case ERR_EEPROM_FAIL:          // 7 nhịp chớp
      return "EEPROM_FAIL";
    default:                       return "UNKNOWN_ERROR";
  }
}
