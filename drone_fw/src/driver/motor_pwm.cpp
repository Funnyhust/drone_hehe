#include "driver/motor_pwm.h"
#include "board_pinmap.h"
#include "driver/soft_uart.h"
#include <HardwareTimer.h>

// Con trỏ đối tượng Hardware Timer điều khiển
static HardwareTimer *timer3 = nullptr;
static HardwareTimer *timer4 = nullptr;

// =============================================================================
// Định nghĩa các hàm API
// =============================================================================

void motorInit(bool use_50hz) {
  // 1. Cấu hình AFIO để giải phóng JTAG (dành PB4 cho TIM3_CH1)
  // và bật Partial Remap cho TIM3 (PB4=CH1, PB5=CH2)
  __HAL_RCC_AFIO_CLK_ENABLE();

  // Tắt JTAG, giữ lại SWD
  __HAL_AFIO_REMAP_SWJ_NOJTAG();

  // Bật Partial Remap cho TIM3 -> CH1 xuất ra PB4, CH2 xuất ra PB5
  __HAL_AFIO_REMAP_TIM3_PARTIAL();

  // Khởi tạo Timer 3 và Timer 4
  if (timer3 == nullptr) {
    timer3 = new HardwareTimer(TIM3);
  }
  if (timer4 == nullptr) {
    timer4 = new HardwareTimer(TIM4);
  }

  // 2. Cấu hình chu kỳ (Period / Overflow): 200Hz hoặc 50Hz (Để 200Hz để an
  // toàn tương thích mọi loại ESC)
  uint32_t freq = use_50hz ? 50 : 200;

  timer3->setOverflow(freq, HERTZ_FORMAT);
  timer4->setOverflow(freq, HERTZ_FORMAT);

  // Timer 4 (Mặc định không remap)
  // PB6: TIM4_CH1 (Motor 2)
  timer4->setPWM(1, PIN_MOTOR_2, freq, 0);
  // PB7: TIM4_CH2 (Motor 3)
  timer4->setPWM(2, PIN_MOTOR_3, freq, 0);

  // Timer 3 (Partial Remap)
  // PB4: TIM3_CH1 (Motor 4)
  timer3->setPWM(1, PIN_MOTOR_4, freq, 0);
  // PB5: TIM3_CH2 (Motor 1)
  timer3->setPWM(2, PIN_MOTOR_1, freq, 0);

  // Xuất mức thấp ban đầu để đảm bảo an toàn cho ESC
  motorWriteAllUs(PWM_PULSE_SAFE, PWM_PULSE_SAFE, PWM_PULSE_SAFE,
                  PWM_PULSE_SAFE);

  // Khởi chạy Timer
  timer3->resume();
  timer4->resume();
}

void motorWriteUs(uint8_t motor, uint16_t us) {
  // Giới hạn an toàn của tín hiệu xung
  if (us < PWM_PULSE_MIN)
    us = PWM_PULSE_MIN;
  if (us > PWM_PULSE_MAX)
    us = PWM_PULSE_MAX;

  switch (motor) {
  case 0: // Motor 1 - PB5 (TIM3 CH2)
    if (timer3)
      timer3->setCaptureCompare(2, us, MICROSEC_COMPARE_FORMAT);
    break;
  case 1: // Motor 2 - PB6 (TIM4 CH1)
    if (timer4)
      timer4->setCaptureCompare(1, us, MICROSEC_COMPARE_FORMAT);
    break;
  case 2: // Motor 3 - PB7 (TIM4 CH2)
    if (timer4)
      timer4->setCaptureCompare(2, us, MICROSEC_COMPARE_FORMAT);
    break;
  case 3: // Motor 4 - PB4 (TIM3 CH1)
    if (timer3)
      timer3->setCaptureCompare(1, us, MICROSEC_COMPARE_FORMAT);
    break;
  default:
    break;
  }
}

void motorWriteAllUs(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
  motorWriteUs(0, m1);
  motorWriteUs(1, m2);
  motorWriteUs(2, m3);
  motorWriteUs(3, m4);
}

void motorStopAll() {
  motorWriteAllUs(PWM_PULSE_SAFE, PWM_PULSE_SAFE, PWM_PULSE_SAFE,
                  PWM_PULSE_SAFE);
}
