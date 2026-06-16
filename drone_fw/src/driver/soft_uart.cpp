#include "driver/soft_uart.h"

#if ENABLE_SOFT_UART

#include "board_pinmap.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

void softUartInit(uint32_t baudrate) {
  // Serial2 đã được khởi tạo trong crsfInit() chạy ở 420000 baud bằng Hardware
  (void)baudrate;
}

void softUartWrite(uint8_t c) {
  Serial2.write(c);
  
  // Feed Independent Watchdog (IWDG) để tránh bị reset khi in log dài
  IWDG->KR = 0xAAAA;
}

// =============================================================================
// Triển khai các hàm overload in ấn (Print) chuyển hướng sang Serial2
// =============================================================================

void softUartPrint(const char *str) {
  if (!str) return;
  Serial2.print(str);
}

void softUartPrint(char c) {
  Serial2.print(c);
}

void softUartPrint(int val, int base) {
  Serial2.print(val, base);
}

void softUartPrint(unsigned int val, int base) {
  Serial2.print(val, base);
}

void softUartPrint(long val, int base) {
  Serial2.print(val, base);
}

void softUartPrint(unsigned long val, int base) {
  Serial2.print(val, base);
}

void softUartPrint(double val, int decimals) {
  Serial2.print(val, decimals);
}

// =============================================================================
// Triển khai các hàm in xuống dòng (Println) chuyển hướng sang Serial2
// =============================================================================

void softUartPrintln() {
  Serial2.println();
}

void softUartPrintln(const char *str) {
  Serial2.println(str);
}

void softUartPrintln(char c) {
  Serial2.println(c);
}

void softUartPrintln(int val, int base) {
  Serial2.println(val, base);
}

void softUartPrintln(unsigned int val, int base) {
  Serial2.println(val, base);
}

void softUartPrintln(long val, int base) {
  Serial2.println(val, base);
}

void softUartPrintln(unsigned long val, int base) {
  Serial2.println(val, base);
}

void softUartPrintln(double val, int decimals) {
  Serial2.println(val, decimals);
}

// =============================================================================
// Hàm in định dạng (Printf) chuyển hướng sang Serial2
// =============================================================================

void softUartPrintf(const char *format, ...) {
  char buffer[128];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  Serial2.print(buffer);
}

#endif // ENABLE_SOFT_UART
