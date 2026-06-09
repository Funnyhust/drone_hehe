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
#define PIN_MOTOR_1                                                            \
  PB3 // F1-DRV (Trước Phải) - TIM2_CH2 (Remap) hoặc Timer ngắt
#define PIN_MOTOR_2 PB4 // F2-DRV (Sau Phải)  - TIM3_CH1 (Remap) hoặc Timer ngắt
#define PIN_MOTOR_3 PB5 // F3-DRV (Sau Trái)  - TIM3_CH2 (Remap) hoặc Timer ngắt
#define PIN_MOTOR_4 PB6 // F4-DRV (Trước Trái) - TIM4_CH1 (Chuẩn)

// =============================================================================
// 2. Phân hệ Viễn thông (ELRS CRSF UART2)
// =============================================================================
#define PIN_ELRS_TX PA2 // USART2_TX (STM32 gửi đi)
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
// 5. Enable Debug Mode
// =============================================================================

#define ENABLE_DEBUG 1

#if ENABLE_DEBUG
#include <HardwareSerial.h>
#define PIN_DEBUG_TX        PB10  // USART3_TX (STM32 gửi log đi)
#define PIN_DEBUG_RX        PB11  // USART3_RX (STM32 nhận lệnh CLI)
extern HardwareSerial SerialDebug;
#ifdef Serial
#undef Serial
#endif
#define Serial SerialDebug
#endif

#endif // BOARD_PINMAP_H
