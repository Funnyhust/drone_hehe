#include "middleware/pid_controller.h"

// Thông số PID mặc định cho vòng Angle (vòng ngoài)
static PidGains angle_gains[3] = {
  {4.5f, 0.0f, 0.0f}, // Roll
  {4.5f, 0.0f, 0.0f}, // Pitch
  {0.0f, 0.0f, 0.0f}  // Yaw (không dùng vòng Angle cho Yaw)
};

// Thông số PID mặc định cho vòng Rate (vòng trong)
static PidGains rate_gains[3] = {
  {3.5f, 2.5f, 0.05f}, // Roll (P tăng để phản ứng mạnh, I tăng mạnh để giữ tư thế)
  {3.5f, 2.5f, 0.05f}, // Pitch
  {4.0f, 1.5f, 0.02f}  // Yaw
};

// Các biến lưu trữ thành phần tích phân (I-term memory)
static float i_mem_roll = 0.0f;
static float i_mem_pitch = 0.0f;
static float i_mem_yaw = 0.0f;

// Các biến lưu sai số của lần tính trước (cho phép tính vi phân D-term)
static float last_error_roll = 0.0f;
static float last_error_pitch = 0.0f;
static float last_error_yaw = 0.0f;

// Bộ đệm lưu giá trị D-term đã lọc (IIR lowpass filter)
static float d_filtered_roll = 0.0f;
static float d_filtered_pitch = 0.0f;
static float d_filtered_yaw = 0.0f;

// Hệ số lọc thông thấp D-term IIR (Alpha = 0.2f tương ứng với tần số cắt ~15-20Hz)
static const float Alpha_D = 0.2f;

void pidInit() {
  pidReset();
}

void pidSetGains(uint8_t axis, float kp, float ki, float kd, bool is_rate_loop) {
  if (axis >= 3) return;

  if (is_rate_loop) {
    rate_gains[axis].kp = kp;
    rate_gains[axis].ki = ki;
    rate_gains[axis].kd = kd;
  } else {
    angle_gains[axis].kp = kp;
    angle_gains[axis].ki = ki;
    angle_gains[axis].kd = kd;
  }
}

void pidGetGains(uint8_t axis, bool is_rate_loop, float *kp, float *ki, float *kd) {
  if (axis >= 3) return;

  if (is_rate_loop) {
    *kp = rate_gains[axis].kp;
    *ki = rate_gains[axis].ki;
    *kd = rate_gains[axis].kd;
  } else {
    *kp = angle_gains[axis].kp;
    *ki = angle_gains[axis].ki;
    *kd = angle_gains[axis].kd;
  }
}

void pidReset() {
  i_mem_roll = 0.0f;
  i_mem_pitch = 0.0f;
  i_mem_yaw = 0.0f;

  last_error_roll = 0.0f;
  last_error_pitch = 0.0f;
  last_error_yaw = 0.0f;

  d_filtered_roll = 0.0f;
  d_filtered_pitch = 0.0f;
  d_filtered_yaw = 0.0f;
}

void pidCompute(const Attitude *current_att, const MpuData *current_imu,
                float target_roll_deg, float target_pitch_deg, float target_yaw_rate,
                float dt, float *out_roll, float *out_pitch, float *out_yaw) {
  
  // Tránh chia cho 0 trong tính toán vi phân
  if (dt <= 0.0f) dt = 0.002f; 

  // ===========================================================================
  // 1. TRỤC ROLL (Cánh bên / ngang)
  // ===========================================================================
  // Vòng ngoài: Góc (Angle) -> Tốc độ góc (Rate)
  float error_angle_roll = target_roll_deg - current_att->roll;
  float target_rate_roll = angle_gains[AXIS_ROLL].kp * error_angle_roll;
  
  // Giới hạn tốc độ góc đặt tối đa để an toàn (deg/s)
  if (target_rate_roll > 250.0f) target_rate_roll = 250.0f;
  else if (target_rate_roll < -250.0f) target_rate_roll = -250.0f;

  // Vòng trong: Tốc độ góc (Rate) -> Lực động cơ
  float error_rate_roll = target_rate_roll - current_imu->gx;
  float p_term_roll = rate_gains[AXIS_ROLL].kp * error_rate_roll;
  
  // Tích phân I-term & bộ khử bão hòa (Anti-windup)
  i_mem_roll += rate_gains[AXIS_ROLL].ki * error_rate_roll * dt;
  if (i_mem_roll > 250.0f) i_mem_roll = 250.0f;
  else if (i_mem_roll < -250.0f) i_mem_roll = -250.0f;

  // Vi phân D-term thô
  float d_raw_roll = rate_gains[AXIS_ROLL].kd * (error_rate_roll - last_error_roll) / dt;
  
  // Áp dụng lọc thông thấp IIR cho D-term
  d_filtered_roll = d_filtered_roll + Alpha_D * (d_raw_roll - d_filtered_roll);
  
  *out_roll = p_term_roll + i_mem_roll + d_filtered_roll;
  last_error_roll = error_rate_roll;

  // ===========================================================================
  // 2. TRỤC PITCH (Mũi lên xuống / dọc)
  // ===========================================================================
  // Vòng ngoài: Góc (Angle) -> Tốc độ góc (Rate)
  float error_angle_pitch = target_pitch_deg - current_att->pitch;
  float target_rate_pitch = angle_gains[AXIS_PITCH].kp * error_angle_pitch;
  
  if (target_rate_pitch > 250.0f) target_rate_pitch = 250.0f;
  else if (target_rate_pitch < -250.0f) target_rate_pitch = -250.0f;

  // Vòng trong: Tốc độ góc (Rate) -> Lực động cơ
  float error_rate_pitch = target_rate_pitch - current_imu->gy;
  float p_term_pitch = rate_gains[AXIS_PITCH].kp * error_rate_pitch;
  
  i_mem_pitch += rate_gains[AXIS_PITCH].ki * error_rate_pitch * dt;
  if (i_mem_pitch > 250.0f) i_mem_pitch = 250.0f;
  else if (i_mem_pitch < -250.0f) i_mem_pitch = -250.0f;

  float d_raw_pitch = rate_gains[AXIS_PITCH].kd * (error_rate_pitch - last_error_pitch) / dt;
  d_filtered_pitch = d_filtered_pitch + Alpha_D * (d_raw_pitch - d_filtered_pitch);
  
  *out_pitch = p_term_pitch + i_mem_pitch + d_filtered_pitch;
  last_error_pitch = error_rate_pitch;

  // ===========================================================================
  // 3. TRỤC YAW (Quay đầu)
  // ===========================================================================
  // Yaw chỉ chạy trực tiếp vòng Rate dựa vào cần điều khiển (Yaw Rate Loop)
  float error_rate_yaw = target_yaw_rate - current_imu->gz;
  float p_term_yaw = rate_gains[AXIS_YAW].kp * error_rate_yaw;
  
  i_mem_yaw += rate_gains[AXIS_YAW].ki * error_rate_yaw * dt;
  if (i_mem_yaw > 150.0f) i_mem_yaw = 150.0f;
  else if (i_mem_yaw < -150.0f) i_mem_yaw = -150.0f;

  float d_raw_yaw = rate_gains[AXIS_YAW].kd * (error_rate_yaw - last_error_yaw) / dt;
  d_filtered_yaw = d_filtered_yaw + Alpha_D * (d_raw_yaw - d_filtered_yaw);
  
  *out_yaw = p_term_yaw + i_mem_yaw + d_filtered_yaw;
  last_error_yaw = error_rate_yaw;
}
