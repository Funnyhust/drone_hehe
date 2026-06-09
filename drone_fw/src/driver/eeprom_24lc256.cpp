#include "driver/eeprom_24lc256.h"
#include "driver/soft_i2c.h"

void eepromInit() {
  // EEPROM sử dụng chung bus SoftI2C được khởi tạo từ softI2cInit
}

uint8_t eepromReadConfig(uint8_t *p_buf, uint16_t size) {
  if (size == 0) return 0;
  return softI2cReadBytes16(EEPROM_ADDR, EEPROM_PID_START_ADDR, p_buf, size);
}

uint8_t eepromWriteConfig(const uint8_t *p_buf, uint16_t size) {
  if (size == 0) return 0;
  uint8_t status = softI2cWriteBytes16(EEPROM_ADDR, EEPROM_PID_START_ADDR, p_buf, size);
  
  // Sau khi ghi, eeprom cần tối đa 5ms để thực hiện ghi vào bộ nhớ
  delay(5); 
  
  return status;
}
