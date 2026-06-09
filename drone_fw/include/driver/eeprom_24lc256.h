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

/**
 * @brief Khởi tạo giao tiếp với EEPROM.
 */
void eepromInit();

/**
 * @brief Đọc thông số PID và offset cảm biến từ EEPROM.
 * @param p_buf Bộ đệm chứa dữ liệu float (ví dụ lưu PID và Offset)
 * @param size Kích thước dữ liệu (byte)
 * @return uint8_t 0 nếu thành công, 1 nếu lỗi
 */
uint8_t eepromReadConfig(uint8_t *p_buf, uint16_t size);

/**
 * @brief Ghi thông số PID và offset cảm biến vào EEPROM.
 * @param p_buf Bộ đệm chứa dữ liệu cần ghi
 * @param size Kích thước dữ liệu (byte)
 * @return uint8_t 0 nếu thành công, 1 nếu lỗi
 */
uint8_t eepromWriteConfig(const uint8_t *p_buf, uint16_t size);

#endif // EEPROM_24LC256_H
