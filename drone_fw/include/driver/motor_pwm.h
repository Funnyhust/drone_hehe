#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include <Arduino.h>

/**
 * @file motor_pwm.h
 * @brief Driver điều khiển động cơ qua xung PWM tần số cao (400Hz mặc định / 50Hz fallback) cho ESC.
 * @note Sử dụng hoàn toàn chế độ Hardware PWM (Alternate Function) của Timer 3 và Timer 4 để đạt sai số 0ns
 *       và loại bỏ 100% ngắt (Interrupt), giúp CPU thảnh thơi chạy các tác vụ khác như Soft I2C.
 */

// Định nghĩa giới hạn độ rộng xung PWM (us)
#define PWM_PULSE_MIN       1000  // Giá trị tối thiểu (motor dừng / ga thấp nhất)
#define PWM_PULSE_MAX       2000  // Giá trị tối đa (ga tối đa)
#define PWM_PULSE_SAFE      1000  // Mức an toàn khi khởi động / failsafe

/**
 * @brief Khởi tạo hệ thống PWM cho động cơ.
 * @details Giải phóng JTAG trên PB3/PB4 nhưng giữ lại SWD, cấu hình các chân PB3, PB4, PB5, PB6 ở chế độ OUTPUT.
 *          Thiết lập Hardware Timer để lập lịch xung PWM với tần số xác định.
 * @param use_50hz If true, sử dụng tần số 50Hz (cho ESC cũ), ngược lại dùng 400Hz (chuẩn).
 */
void motorInit(bool use_50hz = false);

/**
 * @brief Ghi giá trị độ rộng xung (us) cho một động cơ cụ thể.
 * @param motor Chỉ số động cơ (0 đến 3 tương ứng với F1 đến F4)
 * @param us Độ rộng xung từ 1000us đến 2000us
 */
void motorWriteUs(uint8_t motor, uint16_t us);

/**
 * @brief Ghi giá trị độ rộng xung cho cả 4 động cơ đồng thời.
 * @param m1 Xung cho Motor 1 (PB6 - Kênh 1 Timer 4)
 * @param m2 Xung cho Motor 2 (PB5 - Kênh 2 Timer 3 Remap)
 * @param m3 Xung cho Motor 3 (PB4 - Kênh 1 Timer 3 Remap)
 * @param m4 Xung cho Motor 4 (PB7 - Kênh 2 Timer 4)
 */
void motorWriteAllUs(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);

/**
 * @brief Dừng khẩn cấp toàn bộ động cơ (thiết lập về mức an toàn 1000us).
 */
void motorStopAll();

#endif // MOTOR_PWM_H
