#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>

/**
 * @file mpu6050.h
 * @brief Driver cảm biến IMU MPU6050 thông qua SoftI2C.
 */

// Địa chỉ I2C mặc định của MPU6050
#define MPU6050_ADDR        0x68

// Độ nhạy cảm biến (dựa trên cấu hình đo)
#define MPU6050_ACCEL_SO_8G  4096.0f   // Độ nhạy Accel cho dải +-8g (LSB/g)
#define MPU6050_GYRO_SO_2000 16.4f     // Độ nhạy Gyro cho dải +-2000 deg/s (LSB/(deg/s))

// Ngưỡng độ lệch chuẩn tối đa cho phép lúc Calib Gyro (đơn vị LSB thô)
// Nếu stddev lớn hơn ngưỡng này tức là drone đang bị di chuyển hoặc rung lắc
#define GYRO_CALIB_STDDEV_THRESHOLD  15.0f

// Cấu trúc chứa dữ liệu thô và giá trị vật lý của IMU
struct MpuData {
  // Dữ liệu thô sau khi trừ offset
  int16_t ax_raw, ay_raw, az_raw;
  int16_t gx_raw, gy_raw, gz_raw;
  int16_t temp_raw;

  // Giá trị vật lý đã quy đổi (g đối với Accel, deg/s đối với Gyro, oC đối với Temp)
  float ax, ay, az;
  float gx, gy, gz; // Tốc độ góc (deg/s)
  float temp;       // Nhiệt độ (oC)
};

/**
 * @brief Khởi tạo và thiết lập cảm biến MPU6050.
 * @return uint8_t 0 nếu thành công, 1 nếu không kết nối được MPU6050.
 */
uint8_t mpu6050Init();

/**
 * @brief Đọc liên tiếp 14 byte dữ liệu từ MPU6050 bằng Burst Read.
 * @param p_data Con trỏ tới cấu trúc MpuData để lưu dữ liệu đọc được
 * @return uint8_t 0 nếu thành công, 1 nếu lỗi đường truyền I2C
 */
uint8_t mpu6050Read(MpuData *p_data);

/**
 * @brief Hiệu chuẩn Gyroscope bắt buộc lúc khởi động (Zero Bias).
 * @details Thu thập 1000 mẫu ở tần số 1kHz. Tính toán bias trung bình và kiểm tra rung động (StdDev).
 * @param p_stddev Con trỏ float lưu giá trị độ lệch chuẩn lớn nhất đo được trên 3 trục
 * @return uint8_t 0 nếu thành công, 1 nếu lỗi I2C, 2 nếu phát hiện chuyển động (StdDev vượt ngưỡng)
 */
uint8_t mpu6050CalibrateGyro(float *p_stddev);

/**
 * @brief Hiệu chuẩn Accel một lần duy nhất (One-time Calibration).
 * @details Thu thập 4000 mẫu khi drone nằm thăng bằng. Tính toán offset và cập nhật biến nội bộ.
 * @return uint8_t 0 nếu thành công, 1 nếu lỗi I2C.
 */
uint8_t mpu6050CalibrateAccel();

/**
 * @brief Kiểm tra độ lớn vector gia tốc trọng trường tĩnh khi khởi động.
 * @details Yêu cầu tổng vector g nằm trong khoảng 0.95g ~ 1.05g.
 * @param p_g_total Con trỏ lưu giá trị độ lớn vector g thực tế đo được
 * @return uint8_t 0 nếu hợp lệ, 1 nếu vượt ngoài dải an toàn (lỗi IMU_ERROR).
 */
uint8_t mpu6050ValidateAccelStatic(float *p_g_total);

/**
 * @brief Thiết lập trực tiếp offset cho Gyro và Accel (ví dụ khi load từ EEPROM).
 */
void mpu6050SetOffsets(int16_t ax_off, int16_t ay_off, int16_t az_off,
                       int16_t gx_off, int16_t gy_off, int16_t gz_off);

/**
 * @brief Lấy các giá trị offset hiện tại (để lưu vào EEPROM).
 */
void mpu6050GetOffsets(int16_t *ax_off, int16_t *ay_off, int16_t *az_off,
                       int16_t *gx_off, int16_t *gy_off, int16_t *gz_off);

#endif // MPU6050_H
