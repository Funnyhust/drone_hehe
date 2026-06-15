#ifndef FLASH_BACKUP_H
#define FLASH_BACKUP_H

#include <Arduino.h>

/**
 * @file flash_backup.h
 * @brief Driver ghi/đọc cấu hình dự phòng lên Flash nội (Internal Flash) của STM32F103C8T6.
 * @note Sử dụng Page 63 (Trang cuối cùng của dải Flash 64KB) tại địa chỉ 0x0800FC00.
 */

#define FLASH_BACKUP_ADDR   0x0800FC00 // Page 63 (1KB) của STM32F103C8T6

/**
 * @brief Ghi dữ liệu cấu hình vào Flash nội (tự động xóa page trước khi ghi).
 * @param p_data Con trỏ dữ liệu cần ghi
 * @param size Kích thước dữ liệu (byte)
 * @return uint8_t 0 nếu thành công, 1 nếu lỗi xóa, 2 nếu lỗi ghi
 */
uint8_t flashBackupWrite(const uint8_t *p_data, uint16_t size);

/**
 * @brief Đọc dữ liệu cấu hình từ Flash nội.
 * @param p_data Bộ đệm để chứa dữ liệu đọc ra
 * @param size Kích thước dữ liệu cần đọc (byte)
 * @return uint8_t 0 nếu thành công
 */
uint8_t flashBackupRead(uint8_t *p_data, uint16_t size);

#endif // FLASH_BACKUP_H
