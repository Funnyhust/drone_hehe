#include "driver/motor_pwm.h"
#include "board_pinmap.h"
#include <HardwareTimer.h>

// Biến lưu trữ độ rộng xung hiện tại cho 4 động cơ (mặc định 1000us)
volatile uint16_t m1_pulse_width = PWM_PULSE_SAFE;
volatile uint16_t m2_pulse_width = PWM_PULSE_SAFE;
volatile uint16_t m3_pulse_width = PWM_PULSE_SAFE;
volatile uint16_t m4_pulse_width = PWM_PULSE_SAFE;

// Con trỏ đối tượng Hardware Timer điều khiển
static HardwareTimer *pMotorTimer = nullptr;

// =============================================================================
// Các hàm Callback ngắt (ISR) để tạo xung PWM
// =============================================================================

// Ngắt Overflow (Bắt đầu chu kỳ mới): Bật cả 4 chân động cơ lên HIGH
void motor_overflow_callback() {
  // Ghi trực tiếp thanh ghi BSRR của GPIOB để kéo PB3, PB4, PB5, PB6 lên HIGH cùng lúc (cực nhanh)
  GPIOB->BSRR = (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6);
}

// Ngắt Compare Kênh 1: Hạ PB3 (Motor 1) xuống LOW
void motor_compare1_callback() {
  GPIOB->BRR = (1 << 3);
}

// Ngắt Compare Kênh 2: Hạ PB4 (Motor 2) xuống LOW
void motor_compare2_callback() {
  GPIOB->BRR = (1 << 4);
}

// Ngắt Compare Kênh 3: Hạ PB5 (Motor 3) xuống LOW
void motor_compare3_callback() {
  GPIOB->BRR = (1 << 5);
}

// Ngắt Compare Kênh 4: Hạ PB6 (Motor 4) xuống LOW
void motor_compare4_callback() {
  GPIOB->BRR = (1 << 6);
}

// =============================================================================
// Định nghĩa các hàm API
// =============================================================================

void motorInit(bool use_50hz) {
  // 1. Giải phóng JTAG trên PB3/PB4 nhưng giữ lại SWD bằng cách can thiệp thanh ghi AFIO
  // Giúp sử dụng được PB3, PB4 làm GPIO thông thường
  __HAL_RCC_AFIO_CLK_ENABLE();
  #ifndef AFIO_MAPR_SWJ_CFG
    #define AFIO_MAPR_SWJ_CFG (0x7U << 24)
  #endif
  #ifndef AFIO_MAPR_SWJ_CFG_JTAGDISABLE
    #define AFIO_MAPR_SWJ_CFG_JTAGDISABLE (0x2U << 24)
  #endif
  MODIFY_REG(AFIO->MAPR, AFIO_MAPR_SWJ_CFG, AFIO_MAPR_SWJ_CFG_JTAGDISABLE);

  // 2. Cấu hình chân GPIO của 4 động cơ làm OUTPUT
  pinMode(PIN_MOTOR_1, OUTPUT);
  pinMode(PIN_MOTOR_2, OUTPUT);
  pinMode(PIN_MOTOR_3, OUTPUT);
  pinMode(PIN_MOTOR_4, OUTPUT);

  // Xuất mức thấp ban đầu để đảm bảo an toàn cho ESC
  digitalWrite(PIN_MOTOR_1, LOW);
  digitalWrite(PIN_MOTOR_2, LOW);
  digitalWrite(PIN_MOTOR_3, LOW);
  digitalWrite(PIN_MOTOR_4, LOW);

  // 3. Khởi tạo và cấu hình Hardware Timer (sử dụng TIM2)
  if (pMotorTimer == nullptr) {
    pMotorTimer = new HardwareTimer(TIM2);
  }

  // Cấu hình Prescaler để tần số đếm counter là 1MHz (tức là 1 tick = 1us)
  uint32_t timer_clock = pMotorTimer->getTimerClkFreq();
  pMotorTimer->setPrescaleFactor(timer_clock / 1000000);

  // Cấu hình chu kỳ (Period / Overflow): 400Hz (2500us) hoặc 50Hz (20000us)
  uint32_t period_us = use_50hz ? 20000 : 2500;
  pMotorTimer->setOverflow(period_us, TICK_FORMAT);

  // Đăng ký ngắt Overflow (bắt đầu chu kỳ)
  pMotorTimer->attachInterrupt(motor_overflow_callback);

  // Cấu hình chế độ Output Compare cho 4 kênh nhưng ở dạng ngắt callback
  // Kênh 1: PB3 (TIM2_CH2 - sử dụng ngắt Compare 1 của timer ảo này)
  pMotorTimer->setMode(1, TIMER_OUTPUT_COMPARE);
  pMotorTimer->setCaptureCompare(1, m1_pulse_width, TICK_COMPARE_FORMAT);
  pMotorTimer->attachInterrupt(1, motor_compare1_callback);

  // Kênh 2: PB4 (TIM2_CH3 - ngắt Compare 2)
  pMotorTimer->setMode(2, TIMER_OUTPUT_COMPARE);
  pMotorTimer->setCaptureCompare(2, m2_pulse_width, TICK_COMPARE_FORMAT);
  pMotorTimer->attachInterrupt(2, motor_compare2_callback);

  // Kênh 3: PB5 (TIM2_CH4 - ngắt Compare 3)
  pMotorTimer->setMode(3, TIMER_OUTPUT_COMPARE);
  pMotorTimer->setCaptureCompare(3, m3_pulse_width, TICK_COMPARE_FORMAT);
  pMotorTimer->attachInterrupt(3, motor_compare3_callback);

  // Kênh 4: PB6 (sử dụng Kênh 4 ảo trên TIM2 - ngắt Compare 4)
  pMotorTimer->setMode(4, TIMER_OUTPUT_COMPARE);
  pMotorTimer->setCaptureCompare(4, m4_pulse_width, TICK_COMPARE_FORMAT);
  pMotorTimer->attachInterrupt(4, motor_compare4_callback);

  // Khởi chạy Timer
  pMotorTimer->resume();
}

void motorWriteUs(uint8_t motor, uint16_t us) {
  // Giới hạn an toàn của tín hiệu xung
  if (us < PWM_PULSE_MIN) us = PWM_PULSE_MIN;
  if (us > PWM_PULSE_MAX) us = PWM_PULSE_MAX;

  switch (motor) {
    case 0:
      m1_pulse_width = us;
      if (pMotorTimer) pMotorTimer->setCaptureCompare(1, us, TICK_COMPARE_FORMAT);
      break;
    case 1:
      m2_pulse_width = us;
      if (pMotorTimer) pMotorTimer->setCaptureCompare(2, us, TICK_COMPARE_FORMAT);
      break;
    case 2:
      m3_pulse_width = us;
      if (pMotorTimer) pMotorTimer->setCaptureCompare(3, us, TICK_COMPARE_FORMAT);
      break;
    case 3:
      m4_pulse_width = us;
      if (pMotorTimer) pMotorTimer->setCaptureCompare(4, us, TICK_COMPARE_FORMAT);
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
  motorWriteAllUs(PWM_PULSE_SAFE, PWM_PULSE_SAFE, PWM_PULSE_SAFE, PWM_PULSE_SAFE);
}
