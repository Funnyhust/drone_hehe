#ifndef LOGGING_H
#define LOGGING_H

#include <Arduino.h>
#include "driver/mpu6050.h"
#include "middleware/imu_estimator.h"

/**
 * @brief Khởi tạo hệ thống logging
 */
void loggingInit();

/**
 * @brief Cập nhật máy trạng thái logging và thực hiện in dữ liệu qua soft_uart
 * 
 * @param ch_roll Giá trị kênh Roll (us)
 * @param ch_pitch Giá trị kênh Pitch (us)
 * @param ch_throttle Giá trị kênh Throttle (us)
 * @param ch_yaw Giá trị kênh Yaw (us)
 * @param ch5 Giá trị kênh AUX 1 (us)
 * @param ch6 Giá trị kênh AUX 2 (us)
 * @param ch7 Giá trị kênh AUX 3 (us) dùng để chuyển mode
 * @param p_imu Con trỏ đến dữ liệu thô cảm biến
 * @param p_att Con trỏ đến tư thế góc hiện tại
 */
void loggingUpdate(uint16_t ch_roll, uint16_t ch_pitch, uint16_t ch_throttle, uint16_t ch_yaw,
                   uint16_t ch5, uint16_t ch6, uint16_t ch7,
                   const MpuData *p_imu, const Attitude *p_att);

/**
 * @brief Lấy lệnh điều khiển động cơ trực tiếp từ chế độ test rung động của logging
 * 
 * @param m1 Xung cho Motor 1 (us)
 * @param m2 Xung cho Motor 2 (us)
 * @param m3 Xung cho Motor 3 (us)
 * @param m4 Xung cho Motor 4 (us)
 * @return true nếu đang chạy test rung và muốn ghi đè ga động cơ, ngược lại false
 */
bool loggingGetMotorCommand(uint16_t *m1, uint16_t *m2, uint16_t *m3, uint16_t *m4);

#endif // LOGGING_H
