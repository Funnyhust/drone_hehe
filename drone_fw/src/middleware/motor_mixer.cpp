#include "middleware/motor_mixer.h"
#include "config.h"
#include "driver/motor_pwm.h"

// ===========================================================================
// MOTOR MIXER - GIỐNG HỆT BROKKING YMFC-32
// ===========================================================================
// Sơ đồ motor (nhìn từ trên xuống, mũi drone hướng lên):
//
//        MŨI (TRƯỚC)
//    M4(CW)     M1(CCW)
//      \\         /
//       \\       /
//        X-----X
//       /       \\
//      /         \\
//    M3(CCW)    M2(CW)
//        ĐUÔI (SAU)
//
// Brokking YMFC-32:
//   esc_1 = throttle - pitch + roll - yaw   (front-right, CCW)
//   esc_2 = throttle + pitch + roll + yaw   (rear-right,  CW)
//   esc_3 = throttle + pitch - roll - yaw   (rear-left,   CCW)
//   esc_4 = throttle - pitch - roll + yaw   (front-left,  CW)

void motorMixerCompute(uint16_t throttle, float roll, float pitch, float yaw,
                       uint16_t *m1, uint16_t *m2, uint16_t *m3, uint16_t *m4) {

  // Giới hạn ga tối đa 1800us giống Brokking - giữ headroom cho PID điều khiển
  if (throttle > 1800)
    throttle = 1800;

  // Mixer đồng bộ 100% với chiều cảm biến thực tế của bạn:
  // - Nghiêng Phải (Roll dương) -> Tăng lực motor Phải (M1, M2) để đẩy thăng
  // bằng
  // - Chúi Trước (Pitch âm) -> Tăng lực motor Trước (M1, M4) để ngẩng mũi
  // - Xoay Trái CCW (Yaw dương) -> Tăng lực motor CCW (M1, M3) để tạo mô-men
  // phản lực xoay CW
  float m1_raw = (float)throttle - pitch + roll - yaw; // Front-Right CCW (M1)
  float m2_raw = (float)throttle + pitch + roll + yaw; // Rear-Right  CW  (M2)
  float m3_raw = (float)throttle + pitch - roll - yaw; // Rear-Left   CCW (M3)
  float m4_raw = (float)throttle - pitch - roll + yaw; // Front-Left  CW  (M4)

  // Xác định xung tối đa cho phép
  uint16_t max_pulse = PWM_PULSE_MAX;
#if (defined(PRE_FLIGHT_TEST) && (PRE_FLIGHT_TEST == 1))
  max_pulse = PRE_FLIGHT_MAX_PULSE;
#endif

  // Giới hạn xung: Brokking dùng 1100-2000
  // 1100 thay vì 1000 để giữ motor luôn quay (idle) khi armed
  uint16_t min_pulse = 1100;

  if (m1_raw < min_pulse)
    m1_raw = min_pulse;
  if (m1_raw > max_pulse)
    m1_raw = max_pulse;

  if (m2_raw < min_pulse)
    m2_raw = min_pulse;
  if (m2_raw > max_pulse)
    m2_raw = max_pulse;

  if (m3_raw < min_pulse)
    m3_raw = min_pulse;
  if (m3_raw > max_pulse)
    m3_raw = max_pulse;

  if (m4_raw < min_pulse)
    m4_raw = min_pulse;
  if (m4_raw > max_pulse)
    m4_raw = max_pulse;

  *m1 = (uint16_t)m1_raw;
  *m2 = (uint16_t)m2_raw;
  *m3 = (uint16_t)m3_raw;
  *m4 = (uint16_t)m4_raw;
}
