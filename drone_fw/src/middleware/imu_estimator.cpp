#include "middleware/imu_estimator.h"
#include <cmath>

static Attitude current_att;
static bool is_first_run = true;

// Hằng số bộ lọc bù (Complementary Filter) - GIỐNG BROKKING (0.9996/0.0004)
// Alpha rất cao → tin tưởng Gyro gần như tuyệt đối, chỉ bù drift rất chậm từ Accel
// Giúp chống nhiễu rung động cơ ảnh hưởng qua Accel
static const float Alpha = 0.9996f;

void imuEstimatorInit() {
  current_att.roll = 0.0f;
  current_att.pitch = 0.0f;
  current_att.yaw = 0.0f;
  is_first_run = true;
}

void imuEstimatorUpdate(const MpuData *p_imu, float dt) {
  // 1. Tính toán góc tư thế nghiêng tĩnh từ Gia tốc kế (Accel)
  // Đơn vị đầu ra: độ (degrees)
  float roll_acc = atan2f(p_imu->ay, p_imu->az) * 57.29578f; // 180 / PI
  
  // Tính pitch dựa trên ax và độ lớn tổng hợp của ay, az
  float pitch_acc = atan2f(-p_imu->ax, sqrtf(p_imu->ay * p_imu->ay + p_imu->az * p_imu->az)) * 57.29578f;

  if (is_first_run) {
    // Lần chạy đầu tiên, đặt góc trực tiếp bằng góc gia tốc kế để tránh thời gian hội tụ trễ
    current_att.roll = roll_acc;
    current_att.pitch = pitch_acc;
    current_att.yaw = 0.0f;
    is_first_run = false;
    return;
  }

  // 2. Tích phân Gyro vào góc hiện tại (đơn vị: độ)
  current_att.roll  += p_imu->gx * dt;
  current_att.pitch += p_imu->gy * dt;
  current_att.yaw   += p_imu->gz * dt;

  // 3. Bù chéo góc khi xoay Yaw (Yaw coupling compensation)
  // Khi drone xoay Yaw, góc Roll và Pitch sẽ tự động hoán đổi một phần cho nhau theo hệ trục tọa độ drone
  // yaw_rad = (tốc độ yaw độ/s) * dt * (PI / 180) -> góc xoay yaw tính bằng radian trong chu kỳ dt
  float yaw_rad = p_imu->gz * dt * 0.0174532925f;
  float sin_yaw = sinf(yaw_rad);
  
  float prev_roll = current_att.roll;
  current_att.roll  += current_att.pitch * sin_yaw;
  current_att.pitch -= prev_roll * sin_yaw;

  // 4. Thuật toán hòa trộn cảm biến bộ lọc bù (Complementary Filter)
  // Góc mới = Alpha * Góc tích phân gyro (đã bù yaw) + (1 - Alpha) * Góc Accel
  current_att.roll  = Alpha * current_att.roll  + (1.0f - Alpha) * roll_acc;
  current_att.pitch = Alpha * current_att.pitch + (1.0f - Alpha) * pitch_acc;

  // 5. Giới hạn góc Yaw trong khoảng [-180, 180] độ (Wrap-around)
  if (current_att.yaw > 180.0f) {
    current_att.yaw -= 360.0f;
  } else if (current_att.yaw < -180.0f) {
    current_att.yaw += 360.0f;
  }
}

const Attitude* imuEstimatorGetAttitude() {
  return &current_att;
}
