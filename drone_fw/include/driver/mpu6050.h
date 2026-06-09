#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>

/**
 * @file mpu6050.h
 * @brief Driver cảm biến IMU MPU6050 thông qua SoftI2C.
 * @note Hỗ trợ Burst Read 14 bytes và thuật toán hiệu chuẩn (Calibration) tĩnh với 2000 mẫu.
 */

// Địa chỉ I2C mặc định của MPU6050
#define MPU6050_ADDR        0x68

// Độ nhạy cảm biến (dựa trên cấu hình đo)
#define MPU6050_ACCEL_SO_8G  4096.0f   // Độ nhạy Accel cho dải +-8g (LSB/g)
#define MPU6050_GYRO_SO_2000 16.4f     // Độ nhạy Gyro cho dải +-2000 deg/s (LSB/(deg/s))

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
 * @details Đánh thức cảm biến, cài đặt dải đo Gyro +-2000 deg/s, Accel +-8g và bộ lọc DLPF.
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
 * @brief Thực hiện hiệu chuẩn tĩnh Gyro và Accel (Calibration).
 * @details Thu thập 2000 mẫu khi drone nằm tĩnh trên mặt phẳng ngang để tìm sai số tĩnh (bias).
 *          Offset tìm được sẽ áp dụng trực tiếp trong hàm mpu6050Read.
 */
void mpu6050Calibrate();

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
