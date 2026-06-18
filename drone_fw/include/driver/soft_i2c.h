#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#include <Arduino.h>

/**
 * @file soft_i2c.h
 * @brief Driver giao tiếp I2C phần mềm (bit-bang) trên chân PA9 (SDA) và PA10 (SCL).
 * @note Thiết kế linh hoạt cho phép điều chỉnh tần số (100kHz - 400kHz) qua số vòng trễ,
 *       tích hợp cơ chế Timeout tránh treo vòng lặp điều khiển, và Bus Recovery.
 */

// Định nghĩa kết quả giao tiếp I2C
#define I2C_OK      0
#define I2C_ERROR   1

/**
 * @brief Khởi tạo giao tiếp Software I2C.
 * @details Cấu hình PA9/PA10 ở chế độ Output Open-Drain, Pull-up. Thực hiện kiểm tra
 *          và giải phóng bus nếu bị kẹt (Bus Recovery).
 */
void softI2cInit();

/**
 * @brief Thiết lập tần số hoạt động của Software I2C.
 * @param speed_khz Tốc độ truyền (ví dụ: 100 hoặc 400)
 */
void softI2cSetSpeed(uint16_t speed_khz);

/**
 * @brief Thực hiện khôi phục bus bị kẹt (Bus Recovery).
 * @details Gửi tối đa 9 xung clock SCL để nhả đường dây SDA nếu có thiết bị phụ đang giữ mức thấp.
 */
void softI2cBusRecovery();

/**
 * @brief Ghi dữ liệu vào một thanh ghi của thiết bị.
 * @param dev_addr Địa chỉ thiết bị I2C (7-bit)
 * @param reg_addr Địa chỉ thanh ghi cần ghi
 * @param val Giá trị dữ liệu cần ghi
 * @return uint8_t I2C_OK nếu thành công, I2C_ERROR nếu lỗi/timeout
 */
uint8_t softI2cWriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t val);

/**
 * @brief Đọc dữ liệu từ một thanh ghi của thiết bị.
 * @param dev_addr Địa chỉ thiết bị I2C (7-bit)
 * @param reg_addr Địa chỉ thanh ghi cần đọc
 * @param p_val Con trỏ lưu trữ giá trị đọc được
 * @return uint8_t I2C_OK nếu thành công, I2C_ERROR nếu lỗi/timeout
 */
uint8_t softI2cReadReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *p_val);

/**
 * @brief Đọc liên tiếp nhiều byte (Burst Read) từ thiết bị.
 * @param dev_addr Địa chỉ thiết bị I2C (7-bit)
 * @param start_reg_addr Địa chỉ thanh ghi bắt đầu đọc
 * @param p_buf Bộ đệm lưu trữ dữ liệu đọc được
 * @param len Số lượng byte cần đọc
 * @return uint8_t I2C_OK nếu thành công, I2C_ERROR nếu lỗi/timeout
 */
uint8_t softI2cReadBytes(uint8_t dev_addr, uint8_t start_reg_addr, uint8_t *p_buf, uint16_t len);

/**
 * @brief Đọc liên tiếp nhiều byte từ thiết bị I2C sử dụng địa chỉ ô nhớ 16-bit (ví dụ: EEPROM).
 * @param dev_addr Địa chỉ thiết bị I2C (7-bit)
 * @param mem_addr Địa chỉ ô nhớ 16-bit bắt đầu đọc
 * @param p_buf Bộ đệm lưu trữ dữ liệu đọc được
 * @param len Số lượng byte cần đọc
 * @return uint8_t I2C_OK nếu thành công, I2C_ERROR nếu lỗi/timeout
 */
uint8_t softI2cReadBytes16(uint8_t dev_addr, uint16_t mem_addr, uint8_t *p_buf, uint16_t len);

/**
 * @brief Ghi liên tiếp nhiều byte vào thiết bị I2C sử dụng địa chỉ ô nhớ 16-bit (ví dụ: EEPROM).
 * @param dev_addr Địa chỉ thiết bị I2C (7-bit)
 * @param mem_addr Địa chỉ ô nhớ 16-bit bắt đầu ghi
 * @param p_buf Bộ đệm chứa dữ liệu cần ghi
 * @param len Số lượng byte cần ghi
 * @return uint8_t I2C_OK nếu thành công, I2C_ERROR nếu lỗi/timeout
 */
uint8_t softI2cWriteBytes16(uint8_t dev_addr, uint16_t mem_addr, const uint8_t *p_buf, uint16_t len);

/**
 * @brief Kiểm tra sự hiện diện của một thiết bị tại địa chỉ dev_addr.
 * @param dev_addr Địa chỉ thiết bị cần kiểm tra
 * @return uint8_t I2C_OK nếu thiết bị phản hồi ACK, I2C_ERROR nếu NACK hoặc lỗi.
 */
uint8_t softI2cScanAddress(uint8_t dev_addr);

#endif // SOFT_I2C_H
