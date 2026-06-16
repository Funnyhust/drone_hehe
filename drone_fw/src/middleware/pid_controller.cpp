#include "middleware/pid_controller.h"

// ===========================================================================
// PID GIỐNG HỆT BROKKING YMFC-32
// KHÔNG dùng dt trong công thức - phụ thuộc vào vòng lặp cố định
// I = I + Ki * error          (KHÔNG nhân dt)
// D = Kd * (error - last_err) (KHÔNG chia dt)
// ===========================================================================

// Thông số PID cho vòng Angle (vòng ngoài) - chỉ dùng P
// Brokking: roll_level_adjust = angle_roll * 15 / 3.0 = angle_roll * 5.0
static PidGains angle_gains[3] = {
    {5.0f, 0.0f, 0.0f}, // Roll (giá trị hiệu dụng là 5.0 sau khi chia 3 ở Brokking)
    {5.0f, 0.0f, 0.0f}, // Pitch
    {0.0f, 0.0f, 0.0f}  // Yaw (không dùng vòng Angle)
};

// Thông số PID cho vòng Rate (vòng trong) - GIỐNG HỆT BROKKING GỐC (250Hz)
static PidGains rate_gains[3] = {
    {1.0f, 0.012f, 4.0f}, // Roll  (Brokking: P=1.0, I=0.012, D=4.0)
    {1.0f, 0.012f, 4.0f}, // Pitch (giống Roll)
    {3.0f, 0.01f, 0.0f}   // Yaw   (Brokking: P=3.0, I=0.01, D=0.0)
};

// Giới hạn đầu ra PID (giống Brokking: pid_max_roll = 400)
static const float PID_MAX_ROLL_PITCH = 400.0f;
static const float PID_MAX_YAW = 400.0f;

// Các biến lưu trữ thành phần tích phân (I-term memory)
static float i_mem_roll = 0.0f;
static float i_mem_pitch = 0.0f;
static float i_mem_yaw = 0.0f;

// Các biến lưu sai số của lần tính trước (cho phép tính vi phân D-term)
static float last_error_roll = 0.0f;
static float last_error_pitch = 0.0f;
static float last_error_yaw = 0.0f;

// Bộ lọc gyro 70/30 giống Brokking (làm mượt tín hiệu trước khi vào PID)
// gyro_input = gyro_input * 0.7 + new_reading * 0.3
static float gyro_roll_input = 0.0f;
static float gyro_pitch_input = 0.0f;
static float gyro_yaw_input = 0.0f;

void pidInit() { pidReset(); }

void pidSetGains(uint8_t axis, float kp, float ki, float kd,
                 bool is_rate_loop) {
  if (axis >= 3)
    return;

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

void pidGetGains(uint8_t axis, bool is_rate_loop, float *kp, float *ki,
                 float *kd) {
  if (axis >= 3)
    return;

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

  gyro_roll_input = 0.0f;
  gyro_pitch_input = 0.0f;
  gyro_yaw_input = 0.0f;
}

void pidCompute(const Attitude *current_att, const MpuData *current_imu,
                float target_roll_deg, float target_pitch_deg,
                float target_yaw_rate, float *out_roll, float *out_pitch,
                float *out_yaw) {

  // ===========================================================================
  // BỘ LỌC GYRO 70/30 - GIỐNG HỆT BROKKING (dòng 210-212)
  // Mục đích: Làm mượt tín hiệu gyro trước khi đưa vào PID, chống nhiễu khâu D
  // gyro_roll_input = (gyro_roll_input * 0.7) + (gyro_roll / 65.5 * 0.3)
  // ===========================================================================
  gyro_roll_input = (gyro_roll_input * 0.7f) + (current_imu->gx * 0.3f);
  gyro_pitch_input = (gyro_pitch_input * 0.7f) + (current_imu->gy * 0.3f);
  gyro_yaw_input = (gyro_yaw_input * 0.7f) + (current_imu->gz * 0.3f);

  // ===========================================================================
  // 1. TRỤC ROLL - GIỐNG HỆT BROKKING
  // ===========================================================================
  // Vòng ngoài: Angle -> Rate setpoint
  float error_angle_roll = target_roll_deg - current_att->roll;
  float target_rate_roll = angle_gains[AXIS_ROLL].kp * error_angle_roll;

  // Giới hạn rate setpoint (Brokking: chia 3 -> max ~164 deg/s)
  if (target_rate_roll > 164.0f)
    target_rate_roll = 164.0f;
  else if (target_rate_roll < -164.0f)
    target_rate_roll = -164.0f;

  // Vòng trong: Rate PID - GIỐNG HỆT BROKKING
  // pid_error_temp = gyro_roll_input - pid_roll_setpoint
  float error_rate_roll = gyro_roll_input - target_rate_roll;

  // I = I + Ki * error (KHÔNG nhân dt)
  i_mem_roll += rate_gains[AXIS_ROLL].ki * error_rate_roll;
  if (i_mem_roll > PID_MAX_ROLL_PITCH)
    i_mem_roll = PID_MAX_ROLL_PITCH;
  else if (i_mem_roll < -PID_MAX_ROLL_PITCH)
    i_mem_roll = -PID_MAX_ROLL_PITCH;

  // Output = P*error + I_mem + D*(error - last_error) (KHÔNG chia dt)
  float pid_output_roll =
      rate_gains[AXIS_ROLL].kp * error_rate_roll + i_mem_roll +
      rate_gains[AXIS_ROLL].kd * (error_rate_roll - last_error_roll);

  if (pid_output_roll > PID_MAX_ROLL_PITCH)
    pid_output_roll = PID_MAX_ROLL_PITCH;
  else if (pid_output_roll < -PID_MAX_ROLL_PITCH)
    pid_output_roll = -PID_MAX_ROLL_PITCH;

  last_error_roll = error_rate_roll;
  *out_roll = pid_output_roll;

  // ===========================================================================
  // 2. TRỤC PITCH - GIỐNG HỆT BROKKING
  // ===========================================================================
  float error_angle_pitch = target_pitch_deg - current_att->pitch;
  float target_rate_pitch = angle_gains[AXIS_PITCH].kp * error_angle_pitch;

  if (target_rate_pitch > 164.0f)
    target_rate_pitch = 164.0f;
  else if (target_rate_pitch < -164.0f)
    target_rate_pitch = -164.0f;

  float error_rate_pitch = gyro_pitch_input - target_rate_pitch;

  i_mem_pitch += rate_gains[AXIS_PITCH].ki * error_rate_pitch;
  if (i_mem_pitch > PID_MAX_ROLL_PITCH)
    i_mem_pitch = PID_MAX_ROLL_PITCH;
  else if (i_mem_pitch < -PID_MAX_ROLL_PITCH)
    i_mem_pitch = -PID_MAX_ROLL_PITCH;

  float pid_output_pitch =
      rate_gains[AXIS_PITCH].kp * error_rate_pitch + i_mem_pitch +
      rate_gains[AXIS_PITCH].kd * (error_rate_pitch - last_error_pitch);

  if (pid_output_pitch > PID_MAX_ROLL_PITCH)
    pid_output_pitch = PID_MAX_ROLL_PITCH;
  else if (pid_output_pitch < -PID_MAX_ROLL_PITCH)
    pid_output_pitch = -PID_MAX_ROLL_PITCH;

  last_error_pitch = error_rate_pitch;
  *out_pitch = pid_output_pitch;

  // ===========================================================================
  // 3. TRỤC YAW - GIỐNG HỆT BROKKING
  // ===========================================================================
  float error_rate_yaw = gyro_yaw_input - target_yaw_rate;

  i_mem_yaw += rate_gains[AXIS_YAW].ki * error_rate_yaw;
  if (i_mem_yaw > PID_MAX_YAW)
    i_mem_yaw = PID_MAX_YAW;
  else if (i_mem_yaw < -PID_MAX_YAW)
    i_mem_yaw = -PID_MAX_YAW;

  float pid_output_yaw =
      rate_gains[AXIS_YAW].kp * error_rate_yaw + i_mem_yaw +
      rate_gains[AXIS_YAW].kd * (error_rate_yaw - last_error_yaw);

  if (pid_output_yaw > PID_MAX_YAW)
    pid_output_yaw = PID_MAX_YAW;
  else if (pid_output_yaw < -PID_MAX_YAW)
    pid_output_yaw = -PID_MAX_YAW;

  last_error_yaw = error_rate_yaw;
  *out_yaw = pid_output_yaw;
}

void pidGetIMem(float *i_roll, float *i_pitch, float *i_yaw) {
  *i_roll = i_mem_roll;
  *i_pitch = i_mem_pitch;
  *i_yaw = i_mem_yaw;
}
