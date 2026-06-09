#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "driver/mpu6050.h"
#include "middleware/imu_estimator.h"

/**
 * @file pid_controller.h
 * @brief Bộ điều khiển PID vòng kép (Angle Loop + Rate Loop) cho Roll, Pitch và Yaw.
 * @note Tham khảo thuật toán từ Betaflight, tích hợp bộ lọc thông thấp IIR cho D-term
 *       và cơ chế Anti-windup cho thành phần tích phân.
 */

// Định nghĩa chỉ số các trục
#define AXIS_ROLL   0
#define AXIS_PITCH  1
#define AXIS_YAW    2

// Cấu trúc bộ thông số PID
struct PidGains {
  float kp;
  float ki;
  float kd;
};

/**
 * @brief Khởi tạo các tham số PID mặc định và reset các bộ đệm tích phân.
 */
void pidInit();

/**
 * @brief Thiết lập thông số PID cho một trục cụ thể.
 * @param axis Trục cần đặt (AXIS_ROLL, AXIS_PITCH, AXIS_YAW)
 * @param kp Hệ số tỉ lệ
 * @param ki Hệ số tích phân
 * @param kd Hệ số vi phân
 * @param is_rate_loop true nếu cấu hình vòng Rate, false nếu cấu hình vòng Angle
 */
void pidSetGains(uint8_t axis, float kp, float ki, float kd, bool is_rate_loop);

/**
 * @brief Lấy thông số PID hiện tại.
 */
void pidGetGains(uint8_t axis, bool is_rate_loop, float *kp, float *ki, float *kd);

/**
 * @brief Reset toàn bộ bộ tích phân và sai số cũ (thường gọi khi DISARMED để tránh tích lũy sai số tĩnh).
 */
void pidReset();

/**
 * @brief Tính toán đầu ra điều khiển PID vòng lặp kép cho các trục.
 * @param current_att Tư thế góc nghiêng hiện tại từ IMU estimator (độ)
 * @param current_imu Dữ liệu cảm biến thô hiện tại (tốc độ góc deg/s)
 * @param target_roll_deg Góc Roll đích mong muốn từ tay phát (độ)
 * @param target_pitch_deg Góc Pitch đích mong muốn từ tay phát (độ)
 * @param target_yaw_rate Tốc độ góc Yaw đích mong muốn từ tay phát (deg/s)
 * @param dt Chu kỳ vòng lặp tính toán (giây, ví dụ 0.002s ở 500Hz)
 * @param out_roll Con trỏ lưu đầu ra hiệu chỉnh lực Roll
 * @param out_pitch Con trỏ lưu đầu ra hiệu chỉnh lực Pitch
 * @param out_yaw Con trỏ lưu đầu ra hiệu chỉnh lực Yaw
 */
void pidCompute(const Attitude *current_att, const MpuData *current_imu,
                float target_roll_deg, float target_pitch_deg, float target_yaw_rate,
                float dt, float *out_roll, float *out_pitch, float *out_yaw);

#endif // PID_CONTROLLER_H
