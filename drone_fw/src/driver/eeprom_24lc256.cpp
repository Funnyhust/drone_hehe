#include "driver/eeprom_24lc256.h"
#include "driver/soft_i2c.h"
#include "driver/flash_backup.h"

void eepromInit() {
  // EEPROM sử dụng chung bus SoftI2C được khởi tạo từ softI2cInit
}

uint8_t eepromReadRaw(uint8_t *p_buf, uint16_t size) {
  if (size == 0) return 0;
  return softI2cReadBytes16(EEPROM_ADDR, EEPROM_PID_START_ADDR, p_buf, size);
}

uint8_t eepromWriteRaw(const uint8_t *p_buf, uint16_t size) {
  if (size == 0) return 0;
  uint8_t status = softI2cWriteBytes16(EEPROM_ADDR, EEPROM_PID_START_ADDR, p_buf, size);
  
  // Sau khi ghi, eeprom cần tối đa 5ms để thực hiện ghi vào bộ nhớ
  delay(5); 
  
  return status;
}

uint8_t eepromCalculateCrc8(const DroneConfig *config) {
  if (config == nullptr) return 0;
  
  const uint8_t *data = (const uint8_t *)config;
  uint8_t crc = 0;
  // Kích thước của struct trừ đi 1 byte cuối cùng (chính là byte crc8)
  uint8_t len = sizeof(DroneConfig) - 1;
  
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0xD5; // Đa thức 0xD5 (giống CRSF)
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

uint8_t configLoad(DroneConfig *p_config) {
  if (p_config == nullptr) return 2;

  // 1. Thử đọc từ EEPROM ngoài (Ưu tiên số 1)
  uint8_t eeprom_status = eepromReadRaw((uint8_t *)p_config, sizeof(DroneConfig));
  if (eeprom_status == 0) {
    uint8_t calc_crc = eepromCalculateCrc8(p_config);
    if (p_config->signature == DRONE_CONFIG_SIGNATURE && p_config->crc8 == calc_crc) {
      // Đọc EEPROM thành công, kiểm tra xem Flash nội đã đồng bộ chưa
      DroneConfig flash_config;
      flashBackupRead((uint8_t *)&flash_config, sizeof(DroneConfig));
      if (flash_config.signature != DRONE_CONFIG_SIGNATURE || flash_config.crc8 != p_config->crc8) {
        // Đồng bộ lại sang Flash nội
        flashBackupWrite((const uint8_t *)p_config, sizeof(DroneConfig));
      }
      return 0; // Load từ EEPROM thành công
    }
  }

  // 2. Thử đọc từ Flash nội (Bản sao lưu Backup)
  uint8_t flash_status = flashBackupRead((uint8_t *)p_config, sizeof(DroneConfig));
  if (flash_status == 0) {
    uint8_t calc_crc = eepromCalculateCrc8(p_config);
    if (p_config->signature == DRONE_CONFIG_SIGNATURE && p_config->crc8 == calc_crc) {
      // Đọc Flash thành công, tự động khôi phục ngược lại sang EEPROM
      eepromWriteRaw((const uint8_t *)p_config, sizeof(DroneConfig));
      return 1; // Load từ Flash Backup thành công
    }
  }

  // 3. Nếu cả hai thất bại, nạp thông số mặc định (Default values)
  p_config->signature = DRONE_CONFIG_SIGNATURE;
  // PID mặc định
  p_config->kp_roll = 1.0f; p_config->ki_roll = 0.012f; p_config->kd_roll = 4.0f;
  p_config->kp_pitch = 1.0f; p_config->ki_pitch = 0.012f; p_config->kd_pitch = 4.0f;
  p_config->kp_yaw = 3.0f; p_config->ki_yaw = 0.01f; p_config->kd_yaw = 0.0f;
  
  // Offset Accelerometer
  p_config->accel_offset_x = 0;
  p_config->accel_offset_y = 0;
  p_config->accel_offset_z = 0;
  
  // Offset Gyroscope
  p_config->gyro_offset_x = 0;
  p_config->gyro_offset_y = 0;
  p_config->gyro_offset_z = 0;
  
  p_config->esc_calibrated = 0;
  p_config->crc8 = eepromCalculateCrc8(p_config);
  
  // Ghi lại cấu hình mặc định vào cả hai
  eepromWriteRaw((const uint8_t *)p_config, sizeof(DroneConfig));
  flashBackupWrite((const uint8_t *)p_config, sizeof(DroneConfig));

  return 2; // Dùng mặc định
}

uint8_t configSave(const DroneConfig *p_config) {
  if (p_config == nullptr) return 3;

  // Cập nhật CRC8 trước khi lưu
  DroneConfig temp_config = *p_config;
  temp_config.crc8 = eepromCalculateCrc8(&temp_config);

  uint8_t eeprom_err = eepromWriteRaw((const uint8_t *)&temp_config, sizeof(DroneConfig));
  uint8_t flash_err = flashBackupWrite((const uint8_t *)&temp_config, sizeof(DroneConfig));

  if (eeprom_err != 0 && flash_err != 0) return 3;
  if (eeprom_err != 0) return 1;
  if (flash_err != 0) return 2;
  return 0;
}
