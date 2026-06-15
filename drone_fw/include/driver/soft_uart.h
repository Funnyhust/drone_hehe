#ifndef SOFT_UART_H
#define SOFT_UART_H

#include <Arduino.h>

/**
 * @file soft_uart.h
 * @brief Driver Software UART (chỉ phát - TX) sử dụng chân SOFT_UART_PIN.
 * @note Hỗ trợ các overload in ấn tương tự Serial của Arduino để dễ dàng thay
 * thế.
 */
#define ENABLE_SOFT_UART 0
// Định nghĩa base mặc định cho in số nguyên
#ifndef DEC
#define DEC 10
#endif
#ifndef HEX
#define HEX 16
#endif
#ifndef OCT
#define OCT 8
#endif
#ifndef BIN
#define BIN 2
#endif

#if ENABLE_SOFT_UART == 1

/**
 * @brief Khởi tạo Software UART với chân TX.
 * @param baudrate Tốc độ baud truyền dữ liệu (mặc định 115200).
 */
void softUartInit(uint32_t baudrate = 19200);

/**
 * @brief Gửi một byte (ký tự) qua Software UART.
 * @param c Ký tự cần truyền.
 */
void softUartWrite(uint8_t c);

// Các hàm in ấn (Print) hỗ trợ nhiều kiểu dữ liệu khác nhau
void softUartPrint(const char *str);
void softUartPrint(char c);
void softUartPrint(int val, int base = DEC);
void softUartPrint(unsigned int val, int base = DEC);
void softUartPrint(long val, int base = DEC);
void softUartPrint(unsigned long val, int base = DEC);
void softUartPrint(double val, int decimals = 2);

// Các hàm in ấn xuống dòng (Println)
void softUartPrintln();
void softUartPrintln(const char *str);
void softUartPrintln(char c);
void softUartPrintln(int val, int base = DEC);
void softUartPrintln(unsigned int val, int base = DEC);
void softUartPrintln(long val, int base = DEC);
void softUartPrintln(unsigned long val, int base = DEC);
void softUartPrintln(double val, int decimals = 2);

/**
 * @brief Gửi chuỗi định dạng (printf-like) qua Software UART.
 * @param format Chuỗi định dạng.
 * @param ... Các tham số truyền vào.
 */
void softUartPrintf(const char *format, ...);

#else

static inline void softUartInit(uint32_t baudrate = 19200) {}
static inline void softUartWrite(uint8_t c) {}
static inline void softUartPrint(const char *str) {}
static inline void softUartPrint(char c) {}
static inline void softUartPrint(int val, int base = DEC) {}
static inline void softUartPrint(unsigned int val, int base = DEC) {}
static inline void softUartPrint(long val, int base = DEC) {}
static inline void softUartPrint(unsigned long val, int base = DEC) {}
static inline void softUartPrint(double val, int decimals = 2) {}
static inline void softUartPrintln() {}
static inline void softUartPrintln(const char *str) {}
static inline void softUartPrintln(char c) {}
static inline void softUartPrintln(int val, int base = DEC) {}
static inline void softUartPrintln(unsigned int val, int base = DEC) {}
static inline void softUartPrintln(long val, int base = DEC) {}
static inline void softUartPrintln(unsigned long val, int base = DEC) {}
static inline void softUartPrintln(double val, int decimals = 2) {}
static inline void softUartPrintf(const char *format, ...) {}

#endif // ENABLE_SOFT_UART

#endif // SOFT_UART_H
