#ifndef BATTERY_ADC_H
#define BATTERY_ADC_H

#include <Arduino.h>

/**
 * @file battery_adc.h
 * @brief Driver giám sát nguồn điện áp Pin qua ADC1_CH1 (chân PA1).
 * @note Sử dụng bộ lọc trung bình động (Moving Average Filter) và phân loại trạng thái pin.
 */

// Định nghĩa các trạng thái Pin
enum BatteryState {
  BATTERY_NORMAL = 0,
  BATTERY_LOW,
  BATTERY_CRITICAL
};

// Định nghĩa các ngưỡng điện áp cho Pin LiPo 3S (V)
#define BATTERY_THRESHOLD_LOW       10.5f  // Ngưỡng báo pin yếu (3.5V/cell)
#define BATTERY_THRESHOLD_CRITICAL  9.9f   // Ngưỡng báo pin nguy kịch (3.3V/cell)

/**
 * @brief Khởi tạo hệ thống đọc ADC cho Pin.
 * @details Cấu hình chân PA1 ở chế độ INPUT.
 */
void batteryInit();

/**
 * @brief Thực hiện đọc ADC và cập nhật điện áp pin.
 * @details Cần được gọi định kỳ để lấy mẫu điện áp và chạy bộ lọc trung bình động.
 */
void batteryUpdate();

/**
 * @brief Lấy giá trị điện áp pin hiện tại (đã lọc).
 * @return float Điện áp pin (V)
 */
float batteryGetVoltage();

/**
 * @brief Lấy trạng thái pin hiện tại.
 * @return BatteryState Trạng thái pin (NORMAL, LOW, CRITICAL)
 */
BatteryState batteryGetState();

#endif // BATTERY_ADC_H
