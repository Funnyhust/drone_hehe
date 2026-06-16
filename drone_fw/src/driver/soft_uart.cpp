#include "driver/soft_uart.h"

#if ENABLE_SOFT_UART

#include "board_pinmap.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

// Biến toàn cục lưu trữ Port và Bit Mask của chân TX để thao tác nhanh
static GPIO_TypeDef *tx_port = GPIOB;
static uint32_t tx_pin_mask = (1 << 7); // Mặc định PB7

// Số chu kỳ CPU tương ứng với 1 bit truyền
static uint32_t cycles_per_bit = 3750; // Mặc định cho 19200 baud ở 72MHz

// Các hàm inline thao tác nhanh trên thanh ghi GPIO
static inline void tx_high() {
  tx_port->BSRR = tx_pin_mask;
}

static inline void tx_low() {
  tx_port->BRR = tx_pin_mask;
}

// Hàm trễ chính xác tuyệt đối theo chu kỳ CPU sử dụng DWT CYCCNT
// Tích hợp cơ chế kiểm soát giới hạn (timeout) để ngăn chặn hoàn toàn việc treo CPU
static inline void delay_cycles_dwt(uint32_t cycles) {
  uint32_t start = DWT->CYCCNT;
  uint32_t timeout_limit = cycles * 4; // Ngăn chặn vòng lặp vô hạn
  uint32_t count = 0;
  
  while (((DWT->CYCCNT - start) < cycles) && (count < timeout_limit)) {
    count++;
  }
}

void softUartInit(uint32_t baudrate) {
  // 1. Cấu hình chân GPIO chế độ Output
  pinMode(SOFT_UART_PIN, OUTPUT);
  digitalWrite(SOFT_UART_PIN, HIGH); // Kéo HIGH mặc định (Idle)

  // 2. Ánh xạ thủ công PORT và PIN để đảm bảo an toàn tuyệt đối, tránh lỗi HardFault trên các phiên bản Core khác nhau
  tx_port = GPIOB;
  tx_pin_mask = (1 << 7); // Mặc định an toàn

  // Kiểm tra Runtime để gán đúng Port và Pin Mask
  #if defined(PA0)
  if (SOFT_UART_PIN == PA0) { tx_port = GPIOA; tx_pin_mask = (1 << 0); }
  #endif
  #if defined(PA1)
  if (SOFT_UART_PIN == PA1) { tx_port = GPIOA; tx_pin_mask = (1 << 1); }
  #endif
  #if defined(PA2)
  if (SOFT_UART_PIN == PA2) { tx_port = GPIOA; tx_pin_mask = (1 << 2); }
  #endif
  #if defined(PA3)
  if (SOFT_UART_PIN == PA3) { tx_port = GPIOA; tx_pin_mask = (1 << 3); }
  #endif
  #if defined(PA9)
  if (SOFT_UART_PIN == PA9) { tx_port = GPIOA; tx_pin_mask = (1 << 9); }
  #endif
  #if defined(PA10)
  if (SOFT_UART_PIN == PA10) { tx_port = GPIOA; tx_pin_mask = (1 << 10); }
  #endif
  #if defined(PB3)
  if (SOFT_UART_PIN == PB3) { tx_port = GPIOB; tx_pin_mask = (1 << 3); }
  #endif
  #if defined(PB4)
  if (SOFT_UART_PIN == PB4) { tx_port = GPIOB; tx_pin_mask = (1 << 4); }
  #endif
  #if defined(PB5)
  if (SOFT_UART_PIN == PB5) { tx_port = GPIOB; tx_pin_mask = (1 << 5); }
  #endif
  #if defined(PB6)
  if (SOFT_UART_PIN == PB6) { tx_port = GPIOB; tx_pin_mask = (1 << 6); }
  #endif
  #if defined(PB7)
  if (SOFT_UART_PIN == PB7) { tx_port = GPIOB; tx_pin_mask = (1 << 7); }
  #endif
  #if defined(PB10)
  if (SOFT_UART_PIN == PB10) { tx_port = GPIOB; tx_pin_mask = (1 << 10); }
  #endif
  #if defined(PB11)
  if (SOFT_UART_PIN == PB11) { tx_port = GPIOB; tx_pin_mask = (1 << 11); }
  #endif

  // Thiết lập trạng thái ban đầu là HIGH
  tx_high();

  // 3. Khởi tạo và kích hoạt bộ đếm chu kỳ CPU (DWT CYCCNT)
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  *((volatile uint32_t*)0xE0001FB0) = 0xC5ACCE55; 
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // 4. Tính toán số chu kỳ CPU cho mỗi bit
  cycles_per_bit = SystemCoreClock / baudrate;

  // Trừ bớt overhead nhỏ của các lệnh nạp bit và gán trạng thái (~12 chu kỳ CPU)
  if (cycles_per_bit > 12) {
    cycles_per_bit -= 12;
  }
}

void softUartWrite(uint8_t c) {
  // KHÔNG dùng noInterrupts() ở đây để tránh việc tắt ngắt làm trì hoãn ngắt Compare 
  // của Driver Motor PWM (Software PWM). Ở tốc độ thấp 19200 baud, thời gian 1 bit (52us) 
  // đủ lớn để việc xen ngắt 1-2us của timer động cơ không gây ảnh hưởng đến dữ liệu truyền.

  // 1. Start bit: Kéo xuống mức LOW
  tx_low();
  delay_cycles_dwt(cycles_per_bit);

  // 2. Data bits (Unrolled Loop để đảm bảo độ trễ giữa các bit đồng đều tuyệt đối):
  // Bit 0 (LSB)
  if (c & 0x01) tx_high(); else tx_low();
  delay_cycles_dwt(cycles_per_bit);

  // Bit 1
  if (c & 0x02) tx_high(); else tx_low();
  delay_cycles_dwt(cycles_per_bit);

  // Bit 2
  if (c & 0x04) tx_high(); else tx_low();
  delay_cycles_dwt(cycles_per_bit);

  // Bit 3
  if (c & 0x08) tx_high(); else tx_low();
  delay_cycles_dwt(cycles_per_bit);

  // Bit 4
  if (c & 0x10) tx_high(); else tx_low();
  delay_cycles_dwt(cycles_per_bit);

  // Bit 5
  if (c & 0x20) tx_high(); else tx_low();
  delay_cycles_dwt(cycles_per_bit);

  // Bit 6
  if (c & 0x40) tx_high(); else tx_low();
  delay_cycles_dwt(cycles_per_bit);

  // Bit 7 (MSB)
  if (c & 0x80) tx_high(); else tx_low();
  delay_cycles_dwt(cycles_per_bit);

  // 3. Stop bit: Kéo lên mức HIGH
  tx_high();
  delay_cycles_dwt(cycles_per_bit);

  // Feed Independent Watchdog (IWDG) để tránh bị reset khi in chuỗi debug dài
  IWDG->KR = 0xAAAA;

#if ENABLE_DEBUG
  Serial.write(c);
#endif
}

// =============================================================================
// Triển khai các hàm overload in ấn (Print)
// =============================================================================

void softUartPrint(const char *str) {
  if (!str) return;
  while (*str) {
    softUartWrite((uint8_t)*str++);
  }
}

void softUartPrint(char c) {
  softUartWrite((uint8_t)c);
}

void softUartPrint(int val, int base) {
  char buf[34];
  ltoa(val, buf, base);
  softUartPrint(buf);
}

void softUartPrint(unsigned int val, int base) {
  char buf[34];
  ultoa(val, buf, base);
  softUartPrint(buf);
}

void softUartPrint(long val, int base) {
  char buf[34];
  ltoa(val, buf, base);
  softUartPrint(buf);
}

void softUartPrint(unsigned long val, int base) {
  char buf[34];
  ultoa(val, buf, base);
  softUartPrint(buf);
}

void softUartPrint(double val, int decimals) {
  char buf[32];
  dtostrf(val, 0, decimals, buf);
  softUartPrint(buf);
}

// =============================================================================
// Triển khai các hàm in xuống dòng (Println)
// =============================================================================

void softUartPrintln() {
  softUartPrint("\r\n");
}

void softUartPrintln(const char *str) {
  softUartPrint(str);
  softUartPrintln();
}

void softUartPrintln(char c) {
  softUartPrint(c);
  softUartPrintln();
}

void softUartPrintln(int val, int base) {
  softUartPrint(val, base);
  softUartPrintln();
}

void softUartPrintln(unsigned int val, int base) {
  softUartPrint(val, base);
  softUartPrintln();
}

void softUartPrintln(long val, int base) {
  softUartPrint(val, base);
  softUartPrintln();
}

void softUartPrintln(unsigned long val, int base) {
  softUartPrint(val, base);
  softUartPrintln();
}

void softUartPrintln(double val, int decimals) {
  softUartPrint(val, decimals);
  softUartPrintln();
}

// =============================================================================
// Hàm in định dạng (Printf)
// =============================================================================

void softUartPrintf(const char *format, ...) {
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  softUartPrint(buffer);
}

#endif // ENABLE_SOFT_UART == 1
