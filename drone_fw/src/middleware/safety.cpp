#include "middleware/safety.h"
#include "board_pinmap.h"
#include "driver/motor_pwm.h"
#include "driver/rc_crsf.h"
#include "driver/battery_adc.h"
#include "driver/soft_uart.h"

// Trạng thái hiện tại của hệ thống
static FlightState current_state = STATE_DISARMED;

// Bộ đếm số lần lỗi đọc IMU liên tiếp
static uint16_t imu_error_counter = 0;

void safetyInit() {
  current_state = STATE_DISARMED;
  imu_error_counter = 0;

  // Đảm bảo động cơ dừng an toàn khi mới bật nguồn
  motorStopAll();

  // Cấu hình Independent Watchdog (IWDG) trên STM32F103
  // LSI clock ~40kHz. Đặt Prescaler = 64 (IWDG_PR = 0x04) -> Tần số đếm counter = 625Hz
  // Đặt reload counter = 625 -> timeout = 625 * (1/625) = 1.0s = 1000ms (an sau và tránh reset khi debug)
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
      // Tắt động cơ hoàn toàn
      motorStopAll();

      // Kiểm tra công tắc Arm gạt lên mức cao (> 1500us)
      if (arm_switch > 1500) {
        current_state = STATE_PRE_ARM;
      }
      break;

    case STATE_PRE_ARM:
      // Thực hiện các kiểm tra điều kiện an toàn (Pre-arm Checks)
      if (link_active &&                      // 1. Sóng tốt
          throttle < 1050 &&                  // 2. Ga ở mức tối thiểu
          bat_state != BATTERY_CRITICAL &&    // 3. Pin không nguy kịch
          imu_error_counter == 0)             // 4. IMU bình thường
      {
        current_state = STATE_ARMED;
      } else {
        // Nếu không đạt, in lý do từ chối Arm (chỉ in 1 lần)
#if ENABLE_DEBUG
        softUartPrint("[ARM REJECTED] Reasons: ");
        if (!link_active) softUartPrint("RC Link Inactive; ");
        if (throttle >= 1050) { softUartPrintf("Throttle high (%dus); ", throttle); }
        if (bat_state == BATTERY_CRITICAL) softUartPrint("Battery Critical; ");
        if (imu_error_counter > 0) softUartPrint("IMU Error; ");
        softUartPrintln("");
#endif
        // Quay về DISARMED
        current_state = STATE_DISARMED;
      }
      break;

    case STATE_ARMED:
      // Động cơ được điều khiển bình thường qua Mixer (sẽ xử lý ở loop chính)

      // Kiểm tra Failsafe (sự cố đột ngột khi đang bay)
      // 1. Mất sóng cứng (quá 200ms)
      // 2. Lỗi đọc IMU liên tục quá 5 lần
      if (!link_active || imu_error_counter > 5) {
        current_state = STATE_FAILSAFE;
      }
      // Người dùng gạt công tắc Disarm chủ động (< 1300us)
      else if (arm_switch < 1300) {
        current_state = STATE_DISARMED;
      }
      break;

    case STATE_FAILSAFE:
      // Lập tức dừng khẩn cấp toàn bộ động cơ
      motorStopAll();

      // Chỉ cho phép thoát Failsafe về DISARMED nếu gạt switch Arm về mức thấp,
      // đồng thời sóng và cảm biến đã hồi phục ổn định.
      if (arm_switch < 1300 && link_active && imu_error_counter == 0) {
        current_state = STATE_DISARMED;
      }
      break;

    default:
      current_state = STATE_DISARMED;
      break;
  }

  // In log debug khi trạng thái thay đổi
  if (current_state != previous_state) {
#if ENABLE_DEBUG
    softUartPrintf("[STATE CHANGE] %s -> %s\r\n", 
                   safetyGetStateStr(previous_state), 
                   safetyGetStateStr(current_state));
#endif
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
