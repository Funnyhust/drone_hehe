#ifndef MOTOR_MIXER_H
#define MOTOR_MIXER_H

#include <Arduino.h>

/**
 * @file motor_mixer.h
 * @brief Bộ trộn động cơ (Motor Mixer) cho Quadcopter cấu hình Quad-X.
 * @note Trích xuất thuật toán ma trận trộn từ Betaflight, phân bổ lực đẩy Roll, Pitch, Yaw,
 *       và giới hạn dải xung ra an toàn [1000us, 2000us] cho ESC.
 */

/**
 * @brief Tính toán tốc độ / độ rộng xung PWM cho 4 động cơ.
 * @param throttle Giá trị ga hiện tại từ tay phát (1000 - 2000us)
 * @param roll Lực hiệu chỉnh Roll từ bộ PID
 * @param pitch Lực hiệu chỉnh Pitch từ bộ PID
 * @param yaw Lực hiệu chỉnh Yaw từ bộ PID
 * @param m1 Con trỏ lưu xung đầu ra Motor 1 (Trước Phải)
 * @param m2 Con trỏ lưu xung đầu ra Motor 2 (Sau Phải)
 * @param m3 Con trỏ lưu xung đầu ra Motor 3 (Sau Trái)
 * @param m4 Con trỏ lưu xung đầu ra Motor 4 (Trước Trái)
 */
void motorMixerCompute(uint16_t throttle, float roll, float pitch, float yaw,
                       uint16_t *m1, uint16_t *m2, uint16_t *m3, uint16_t *m4);

#endif // MOTOR_MIXER_H
