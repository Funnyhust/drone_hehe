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

  // Ánh xạ tuyến tính ga thô (1000 - 2000us) sang (1100 - 1600us)
  // Chừa lại 100us (lên tới 1700us) làm headroom để PID bù lực giữ thăng bằng
  uint16_t scaled_throttle = map(throttle, 1000, 2000, 1100, 1600);

  // Mixer gốc của Brokking YMFC-32:
  // esc_1 = throttle - pitch + roll - yaw   (Trước Phải - CCW)
  // esc_2 = throttle + pitch + roll + yaw   (Sau Phải - CW)
  // esc_3 = throttle + pitch - roll - yaw   (Sau Trái - CCW)
  // esc_4 = throttle - pitch - roll + yaw   (Trước Trái - CW)
  float m1_raw =
      (float)scaled_throttle - pitch + roll - yaw; // Front-Right (M1)
  float m2_raw = (float)scaled_throttle + pitch + roll + yaw; // Rear-Right (M2)
  float m3_raw = (float)scaled_throttle + pitch - roll - yaw; // Rear-Left (M3)
  float m4_raw = (float)scaled_throttle - pitch - roll + yaw; // Front-Left (M4)

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
