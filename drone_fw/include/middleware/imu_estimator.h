#ifndef IMU_ESTIMATOR_H
#define IMU_ESTIMATOR_H

#include "driver/mpu6050.h"

/**
 * @file imu_estimator.h
 * @brief Bộ ước lượng tư thế góc nghiêng (Attitude Estimator) cho Drone.
 * @note Sử dụng thuật toán hòa trộn cảm biến (gyro attitude estimator)
 *       để tính toán các góc Roll, Pitch, Yaw chính xác ở tần số 500Hz.
 */

// Cấu trúc lưu trữ tư thế góc nghiêng hiện tại (đơn vị: độ - degrees)
struct Attitude {
  float roll;
  float pitch;
  float yaw;
};

/**
 * @brief Khởi tạo bộ ước lượng tư thế.
 */
void imuEstimatorInit();

/**
 * @brief Cập nhật góc tư thế từ dữ liệu thô cảm biến.
 * @param p_imu Dữ liệu IMU đọc được từ cảm biến
 * @param dt Khoảng thời gian trích mẫu (giây, ví dụ ở 500Hz là 0.002s)
 */
void imuEstimatorUpdate(const MpuData *p_imu, float dt);

/**
 * @brief Lấy góc tư thế hiện tại.
 * @return const Attitude* Con trỏ tới struct chứa các góc Roll, Pitch, Yaw
 */
const Attitude* imuEstimatorGetAttitude();

#endif // IMU_ESTIMATOR_H
