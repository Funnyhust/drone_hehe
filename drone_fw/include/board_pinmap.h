#ifndef BOARD_PINMAP_H
#define BOARD_PINMAP_H

#include <Arduino.h>

/**
 * @file board_pinmap.h
 * @brief Định nghĩa tập trung toàn bộ chân GPIO của hệ thống Drone
 * STM32F103C8T6.
 * @note Tuyệt đối không hardcode số chân trong các file driver khác.
 */

// =============================================================================
// 1. Phân hệ Động cơ (Motor PWM)
// =============================================================================
// Trước: Trái PB7, Phải PB6
// Sau: Trái PB5, Phải PB4

#define PIN_MOTOR_1 PB6 // F1-DRV (Trước Phải - M1)
#define PIN_MOTOR_2 PB5 // F2-DRV (Sau Phải - M2)  
#define PIN_MOTOR_3 PB4 // F3-DRV (Sau Trái - M3) 
#define PIN_MOTOR_4 PB7 // F4-DRV (Trước Trái - M4) 

// =============================================================================
// 2. Phân hệ Viễn thông (ELRS CRSF UART2)
// =============================================================================
#define PIN_ELRS_TX PA2 // USART2_TX (Sử dụng chân PA2 làm ngõ phát log Hardware ở 420000 baud)
#define PIN_ELRS_RX PA3 // USART2_RX (STM32 nhận về từ ELRS TX)

// =============================================================================
// 3. Phân hệ Cảm biến & EEPROM (Software I2C)
// =============================================================================
#define PIN_SOFT_I2C_SDA                                                       \
  PA9 // Chân dữ liệu SDA (Open-drain, Pull-up ngoài 4.7k)
#define PIN_SOFT_I2C_SCL                                                       \
  PA10 // Chân nhịp clock SCL (Open-drain, Pull-up ngoài 4.7k)

// =============================================================================
// 4. Phân hệ Giám sát Nguồn (Battery ADC)
// =============================================================================
#define PIN_BATTERY_ADC PA1 // Đọc điện áp pin qua cầu phân áp (ADC1_CH1)

// =============================================================================
// 5. Phân hệ Software UART
// =============================================================================
#define SOFT_UART_PIN PA2 // Chân TX cho Software UART (Chuyển sang PA2 để tránh trùng chân Motor 4 PB7)

// =============================================================================
// 6. Enable Debug Mode
// =============================================================================

#define ENABLE_DEBUG 1

#if ENABLE_DEBUG
#include <HardwareSerial.h>
#define PIN_DEBUG_TX PB10 // USART3_TX (STM32 gửi log đi)
#define PIN_DEBUG_RX PB11 // USART3_RX (STM32 nhận lệnh CLI)
extern HardwareSerial SerialDebug;
#ifdef Serial
#undef Serial
#endif
#define Serial SerialDebug
#endif

#endif // BOARD_PINMAP_H
