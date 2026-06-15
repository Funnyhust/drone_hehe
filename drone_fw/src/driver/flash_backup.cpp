#include "driver/flash_backup.h"

uint8_t flashBackupWrite(const uint8_t *p_data, uint16_t size) {
  if (size == 0) return 0;

  // 1. Mở khóa Flash để cho phép xóa/ghi
  HAL_FLASH_Unlock();

  // 2. Cấu hình xóa Page 63
  FLASH_EraseInitTypeDef eraseInit;
  eraseInit.TypeErase   = FLASH_TYPEERASE_PAGES;
  eraseInit.PageAddress = FLASH_BACKUP_ADDR;
  eraseInit.NbPages     = 1;

  uint32_t pageError = 0;
  if (HAL_FLASHEx_Erase(&eraseInit, &pageError) != HAL_OK) {
    HAL_FLASH_Lock();
    return 1; // Lỗi xóa Page
  }

  // 3. Ghi dữ liệu vào Flash dưới dạng Half-Word (16-bit)
  uint32_t targetAddr = FLASH_BACKUP_ADDR;
  const uint16_t *p_word = (const uint16_t *)p_data;
  uint16_t numHalfWords = (size + 1) / 2; // Số lượng nửa từ 16-bit cần ghi

  for (uint16_t i = 0; i < numHalfWords; i++) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, targetAddr, p_word[i]) != HAL_OK) {
      HAL_FLASH_Lock();
      return 2; // Lỗi lập trình Flash
    }
    targetAddr += 2;
  }

  // 4. Khóa lại Flash để tránh ghi đè ngẫu nhiên
  HAL_FLASH_Lock();
  return 0; // Thành công
}

uint8_t flashBackupRead(uint8_t *p_data, uint16_t size) {
  if (size == 0) return 0;
  // STM32 cho phép đọc Flash nội trực tiếp bằng cách ánh xạ con trỏ bộ nhớ
  memcpy(p_data, (const void *)FLASH_BACKUP_ADDR, size);
  return 0;
}
