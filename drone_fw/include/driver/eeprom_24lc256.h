#ifndef EEPROM_24LC256_H
#define EEPROM_24LC256_H

#include <Arduino.h>

/**
 * @file eeprom_24lc256.h
 * @brief Driver đọc/ghi bộ nhớ EEPROM 24LC256 qua Software I2C.
 * @note Lưu trữ cấu hình PID và offset cảm biến. Chỉ đọc lúc khởi động (boot)
 *       và chỉ ghi khi ngắt động cơ (disarm). Không gọi trong vòng lặp điều khiển nhanh.
 */

// Địa chỉ I2C mặc định của EEPROM 24LC256
#define EEPROM_ADDR     0x50

// Địa chỉ ô nhớ bắt đầu lưu cấu hình PID và offsets
#define EEPROM_PID_START_ADDR   0x0000

// Chữ ký xác thực tính hợp lệ của dữ liệu cấu hình
#define DRONE_CONFIG_SIGNATURE  0xDEADBEEF

/**
 * @brief Cấu trúc lưu trữ cấu hình hệ thống (PID và Offsets cảm biến)
 */
struct DroneConfig {
  uint32_t signature;       // Chữ ký xác nhận dữ liệu hợp lệ (0xDEADBEEF)
  
  // Các tham số của bộ điều khiển PID (36 bytes)
  float kp_roll, ki_roll, kd_roll;
  float kp_pitch, ki_pitch, kd_pitch;
  float kp_yaw, ki_yaw, kd_yaw;
  
  // Offsets của Accelerometer (6 bytes)
  int16_t accel_offset_x;
  int16_t accel_offset_y;
  int16_t accel_offset_z;
  
  // Offsets của Gyroscope (6 bytes)
  int16_t gyro_offset_x;
  int16_t gyro_offset_y;
  int16_t gyro_offset_z;
  
  uint8_t esc_calibrated;   // Cờ trạng thái đã calibrate dải ga ESC (1: Đã calib)
  uint8_t crc8;             // Byte kiểm tra toàn vẹn dữ liệu
};

/**
 * @brief Khởi tạo giao tiếp với EEPROM.
 */
void eepromInit();

/**
 * @brief Đọc dữ liệu thô từ EEPROM.
 */
uint8_t eepromReadRaw(uint8_t *p_buf, uint16_t size);

/**
 * @brief Ghi dữ liệu thô vào EEPROM.
 */
uint8_t eepromWriteRaw(const uint8_t *p_buf, uint16_t size);

/**
 * @brief Tính toán CRC-8 cho cấu trúc cấu hình (bỏ qua trường crc8 cuối cùng).
 * @param config Con trỏ cấu hình cần tính toán
 * @return uint8_t Giá trị CRC-8
 */
uint8_t eepromCalculateCrc8(const DroneConfig *config);

/**
 * @brief Tải cấu hình hệ thống từ EEPROM hoặc Flash Backup (Lưu trữ kép).
 * @param p_config Con trỏ struct lưu cấu hình load được
 * @return uint8_t 0 nếu load từ EEPROM, 1 nếu load từ Flash Backup, 2 nếu dùng mặc định
 */
uint8_t configLoad(DroneConfig *p_config);

/**
 * @brief Ghi cấu hình hệ thống đồng thời vào EEPROM và Flash Backup.
 * @param p_config Con trỏ cấu hình cần lưu
 * @return uint8_t 0 nếu lưu thành công cả hai, 1 nếu lỗi EEPROM, 2 nếu lỗi Flash, 3 nếu lỗi cả hai
 */
uint8_t configSave(const DroneConfig *p_config);

#endif // EEPROM_24LC256_H
