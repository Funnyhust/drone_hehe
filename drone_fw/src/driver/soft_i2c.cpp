#include "driver/soft_i2c.h"
#include "board_pinmap.h"

// Biến lưu trữ số vòng lặp trễ (NOP) để cấu hình tần số I2C
// Mặc định: 15 loops tương ứng khoảng 400kHz trên STM32F103 72MHz
static volatile uint32_t soft_i2c_delay_loops = 15;

// Định nghĩa timeout ngắn (chu kỳ loop) tránh treo cứng hệ thống
#define I2C_TIMEOUT_COUNT   1000

// =============================================================================
// Hàm trễ phi chặn bằng NOP (No Operation)
// =============================================================================
static inline void i2c_delay() {
  for (volatile uint32_t i = 0; i < soft_i2c_delay_loops; i++) {
    __asm__("nop");
  }
}

// =============================================================================
// Các hàm thao tác nhanh trên thanh ghi GPIO của STM32F103 (GPIOA)
// =============================================================================
static inline void sda_high() { GPIOA->BSRR = (1 << 9); }
static inline void sda_low()  { GPIOA->BRR = (1 << 9); }
static inline void scl_high() { GPIOA->BSRR = (1 << 10); }
static inline void scl_low()  { GPIOA->BRR = (1 << 10); }
static inline uint8_t sda_read() { return (GPIOA->IDR & (1 << 9)) ? 1 : 0; }
static inline uint8_t scl_read() { return (GPIOA->IDR & (1 << 10)) ? 1 : 0; }

// =============================================================================
// Các tín hiệu cơ bản của giao thức I2C
// =============================================================================

static void i2c_start() {
  sda_high();
  scl_high();
  i2c_delay();
  sda_low();
  i2c_delay();
  scl_low();
  i2c_delay();
}

static void i2c_stop() {
  sda_low();
  i2c_delay();
  scl_high();
  i2c_delay();
  sda_high();
  i2c_delay();
}

// Ghi 1 byte dữ liệu lên bus, trả về 0 nếu nhận được ACK, 1 nếu nhận NACK hoặc Timeout
static uint8_t i2c_write_byte(uint8_t byte) {
  for (uint8_t i = 0; i < 8; i++) {
    if (byte & 0x80) {
      sda_high();
    } else {
      sda_low();
    }
    byte <<= 1;
    i2c_delay();
    scl_high();
    i2c_delay();
    scl_low();
  }

  // Đọc ACK từ thiết bị tớ
  sda_high(); // Nhả chân SDA cho thiết bị tớ kéo xuống
  i2c_delay();
  scl_high();
  i2c_delay();

  uint8_t ack = sda_read();
  scl_low();
  i2c_delay();

  return ack; // ack = 0 nghĩa là thành công
}

// Đọc 1 byte từ bus, ack_bit = 0 nghĩa là gửi ACK, 1 nghĩa là gửi NACK
static uint8_t i2c_read_byte(uint8_t ack_bit) {
  uint8_t byte = 0;
  sda_high(); // Nhả chân SDA để sẵn sàng đọc

  for (uint8_t i = 0; i < 8; i++) {
    byte <<= 1;
    scl_high();
    i2c_delay();
    if (sda_read()) {
      byte |= 1;
    }
    scl_low();
    i2c_delay();
  }

  // Gửi phản hồi ACK / NACK
  if (ack_bit) {
    sda_high(); // NACK
  } else {
    sda_low();  // ACK
  }
  i2c_delay();
  scl_high();
  i2c_delay();
  scl_low();
  sda_high(); // Nhả chân SDA
  i2c_delay();

  return byte;
}

// =============================================================================
// API thực thi
// =============================================================================

void softI2cInit() {
  // Cấu hình GPIO PA9 (SDA) và PA10 (SCL) ở chế độ Output Open-Drain có Pull-up
  pinMode(PIN_SOFT_I2C_SDA, OUTPUT_OPEN_DRAIN);
  pinMode(PIN_SOFT_I2C_SCL, OUTPUT_OPEN_DRAIN);

  // Mặc định kéo cả 2 chân lên mức HIGH
  sda_high();
  scl_high();

  // Khôi phục bus đề phòng trường hợp bị kẹt lúc khởi động
  softI2cBusRecovery();
}

void softI2cSetSpeed(uint16_t speed_khz) {
  if (speed_khz == 100) {
    soft_i2c_delay_loops = 60; // Trễ dài cho 100kHz (~5us)
  } else {
    soft_i2c_delay_loops = 15; // Trễ ngắn cho 400kHz (~1.25us)
  }
}

void softI2cBusRecovery() {
  sda_high();
  // Nếu chân SDA đang bị kéo xuống mức LOW do thiết bị phụ bị kẹt ngắt giữa chừng
  if (sda_read() == 0) {
    for (uint8_t i = 0; i < 9; i++) {
      scl_low();
      i2c_delay();
      scl_high();
      i2c_delay();
      // Nếu thiết bị tớ đã nhả đường SDA lên HIGH, thoát sớm
      if (sda_read() == 1) {
        break;
      }
    }
  }
  // Gửi tín hiệu STOP để reset trạng thái bus
  i2c_stop();
}

uint8_t softI2cScanAddress(uint8_t dev_addr) {
  i2c_start();
  uint8_t ack = i2c_write_byte(dev_addr << 1);
  i2c_stop();
  return (ack == 0) ? I2C_OK : I2C_ERROR;
}

uint8_t softI2cWriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t val) {
  i2c_start();
  
  // Gửi địa chỉ thiết bị với bit Write (0)
  if (i2c_write_byte(dev_addr << 1) != 0) {
    i2c_stop();
    return I2C_ERROR;
  }

  // Gửi địa chỉ thanh ghi
  if (i2c_write_byte(reg_addr) != 0) {
    i2c_stop();
    return I2C_ERROR;
  }

  // Gửi giá trị dữ liệu
  if (i2c_write_byte(val) != 0) {
    i2c_stop();
    return I2C_ERROR;
  }

  i2c_stop();
  return I2C_OK;
}

uint8_t softI2cReadReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *p_val) {
  i2c_start();

  // 1. Ghi địa chỉ thiết bị + địa chỉ thanh ghi cần đọc
  if (i2c_write_byte(dev_addr << 1) != 0) {
    i2c_stop();
    return I2C_ERROR;
  }
  if (i2c_write_byte(reg_addr) != 0) {
    i2c_stop();
    return I2C_ERROR;
  }

  // 2. Restart để đọc dữ liệu
  i2c_start();
  if (i2c_write_byte((dev_addr << 1) | 1) != 0) { // Gửi địa chỉ với bit Read (1)
    i2c_stop();
    return I2C_ERROR;
  }

  // Đọc dữ liệu và trả về NACK (1) để kết thúc phiên truyền
  *p_val = i2c_read_byte(1);

  i2c_stop();
  return I2C_OK;
}

uint8_t softI2cReadBytes(uint8_t dev_addr, uint8_t start_reg_addr, uint8_t *p_buf, uint16_t len) {
  if (len == 0) return I2C_OK;

  i2c_start();

  // 1. Ghi địa chỉ thiết bị + thanh ghi bắt đầu
  if (i2c_write_byte(dev_addr << 1) != 0) {
    i2c_stop();
    return I2C_ERROR;
  }
  if (i2c_write_byte(start_reg_addr) != 0) {
    i2c_stop();
    return I2C_ERROR;
  }

  // 2. Restart để chuyển sang chế độ đọc dữ liệu
  i2c_start();
  if (i2c_write_byte((dev_addr << 1) | 1) != 0) {
    i2c_stop();
    return I2C_ERROR;
  }

  // 3. Đọc dữ liệu liên tiếp (Burst Read)
  for (uint16_t i = 0; i < len; i++) {
    // Nếu là byte cuối cùng thì gửi NACK (1) để báo thiết bị tớ dừng truyền, ngược lại gửi ACK (0)
    p_buf[i] = i2c_read_byte((i == len - 1) ? 1 : 0);
  }

  i2c_stop();
  return I2C_OK;
}

uint8_t softI2cReadBytes16(uint8_t dev_addr, uint16_t mem_addr, uint8_t *p_buf, uint16_t len) {
  if (len == 0) return I2C_OK;

  i2c_start();

  // 1. Gửi địa chỉ thiết bị với bit Write (0)
  if (i2c_write_byte(dev_addr << 1) != 0) {
    i2c_stop();
    return I2C_ERROR;
  }

  // Gửi High Byte của địa chỉ ô nhớ
  if (i2c_write_byte((uint8_t)(mem_addr >> 8)) != 0) {
    i2c_stop();
    return I2C_ERROR;
  }

  // Gửi Low Byte của địa chỉ ô nhớ
  if (i2c_write_byte((uint8_t)(mem_addr & 0xFF)) != 0) {
    i2c_stop();
    return I2C_ERROR;
  }

  // 2. Restart để đọc dữ liệu
  i2c_start();
  if (i2c_write_byte((dev_addr << 1) | 1) != 0) {
    i2c_stop();
    return I2C_ERROR;
  }

  // 3. Đọc dữ liệu liên tiếp (Burst Read)
  for (uint16_t i = 0; i < len; i++) {
    p_buf[i] = i2c_read_byte((i == len - 1) ? 1 : 0);
  }

  i2c_stop();
  return I2C_OK;
}

uint8_t softI2cWriteBytes16(uint8_t dev_addr, uint16_t mem_addr, const uint8_t *p_buf, uint16_t len) {
  i2c_start();

  // Gửi địa chỉ thiết bị với bit Write (0)
  if (i2c_write_byte(dev_addr << 1) != 0) {
    i2c_stop();
    return I2C_ERROR;
  }

  // Gửi High Byte của địa chỉ ô nhớ
  if (i2c_write_byte((uint8_t)(mem_addr >> 8)) != 0) {
    i2c_stop();
    return I2C_ERROR;
  }

  // Gửi Low Byte của địa chỉ ô nhớ
  if (i2c_write_byte((uint8_t)(mem_addr & 0xFF)) != 0) {
    i2c_stop();
    return I2C_ERROR;
  }

  // Gửi dữ liệu ghi
  for (uint16_t i = 0; i < len; i++) {
    if (i2c_write_byte(p_buf[i]) != 0) {
      i2c_stop();
      return I2C_ERROR;
    }
  }

  i2c_stop();
  return I2C_OK;
}
