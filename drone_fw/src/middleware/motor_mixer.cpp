#include "middleware/motor_mixer.h"
#include "driver/motor_pwm.h" // Sử dụng các hằng số PWM_PULSE_MIN/MAX

void motorMixerCompute(uint16_t throttle, float roll, float pitch, float yaw,
                       uint16_t *m1, uint16_t *m2, uint16_t *m3, uint16_t *m4) {
  
  // 1. Áp dụng sơ đồ trộn động cơ Quad-X của Betaflight
  // Động cơ 1: Trước Phải (quay ngược chiều kim đồng hồ CCW)
  // Động cơ 2: Sau Phải (quay thuận chiều kim đồng hồ CW)
  // Động cơ 3: Sau Trái (quay ngược chiều kim đồng hồ CCW)
  // Động cơ 4: Trước Trái (quay thuận chiều kim đồng hồ CW)
  float m1_raw = (float)throttle - roll + pitch + yaw;
  float m2_raw = (float)throttle - roll - pitch - yaw;
  float m3_raw = (float)throttle + roll - pitch + yaw;
  float m4_raw = (float)throttle + roll + pitch - yaw;

  // 2. Giới hạn xung đầu ra nghiêm ngặt trong khoảng [1000us, 2000us] để bảo vệ ESC
  if (m1_raw < PWM_PULSE_MIN) m1_raw = PWM_PULSE_MIN;
  if (m1_raw > PWM_PULSE_MAX) m1_raw = PWM_PULSE_MAX;

  if (m2_raw < PWM_PULSE_MIN) m2_raw = PWM_PULSE_MIN;
  if (m2_raw > PWM_PULSE_MAX) m2_raw = PWM_PULSE_MAX;

  if (m3_raw < PWM_PULSE_MIN) m3_raw = PWM_PULSE_MIN;
  if (m3_raw > PWM_PULSE_MAX) m3_raw = PWM_PULSE_MAX;

  if (m4_raw < PWM_PULSE_MIN) m4_raw = PWM_PULSE_MIN;
  if (m4_raw > PWM_PULSE_MAX) m4_raw = PWM_PULSE_MAX;

  // 3. Trả về kết quả
  *m1 = (uint16_t)m1_raw;
  *m2 = (uint16_t)m2_raw;
  *m3 = (uint16_t)m3_raw;
  *m4 = (uint16_t)m4_raw;
}
