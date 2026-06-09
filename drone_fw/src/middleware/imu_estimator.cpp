#include "middleware/imu_estimator.h"
#include <cmath>

static Attitude current_att;
static bool is_first_run = true;

// Hằng số bộ lọc bù (Complementary Filter Coefficient)
// Alpha lớn -> tin tưởng tích phân Gyro nhiều hơn (lọc nhiễu tốt), phản ứng chậm với Accel
// Alpha nhỏ -> bám theo Accel nhanh hơn nhưng nhạy nhiễu rung động cơ
static const float Alpha = 0.98f;

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

  // 2. Thuật toán hòa trộn cảm biến bộ lọc bù (Complementary Filter)
  // Góc mới = Alpha * (Góc cũ + Tích phân Gyro) + (1 - Alpha) * Góc Accel
  current_att.roll  = Alpha * (current_att.roll  + p_imu->gx * dt) + (1.0f - Alpha) * roll_acc;
  current_att.pitch = Alpha * (current_att.pitch + p_imu->gy * dt) + (1.0f - Alpha) * pitch_acc;
  
  // Trục Yaw chỉ có thể tích phân từ Gyro Z (vì phần cứng không có cảm biến địa từ/compass)
  current_att.yaw   = current_att.yaw + p_imu->gz * dt;

  // 3. Giới hạn góc Yaw trong khoảng [-180, 180] độ (Wrap-around)
  if (current_att.yaw > 180.0f) {
    current_att.yaw -= 360.0f;
  } else if (current_att.yaw < -180.0f) {
    current_att.yaw += 360.0f;
  }
}

const Attitude* imuEstimatorGetAttitude() {
  return &current_att;
}
