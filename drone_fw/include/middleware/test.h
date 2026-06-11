#ifndef TEST_H
#define TEST_H

#include <Arduino.h>

/**
 * @brief Thiết lập tay ga động cơ theo tỷ lệ phần trăm (0% - 100%).
 * @param percent Giá trị ga từ 0.0f đến 100.0f.
 * @note Áp dụng Output Clamp giới hạn cứng xung đầu ra trong dải [1000us, 2000us].
 */
void setThrottlePercent(float percent);

/**
 * @brief Tăng/giảm tốc độ động cơ từ từ để tránh thay đổi đột ngột.
 * @param targetPercent Giá trị phần trăm ga đích cần đạt (0.0f - 100.0f).
 * @note Thay đổi với bước 1% sau mỗi 50ms.
 */
void rampTo(float targetPercent);

/**
 * @brief Kiểm tra cổng Serial nhận lệnh dừng khẩn cấp.
 * @note Nếu nhận chuỗi "STOP", lập tức ngắt động cơ về 1000us và khóa hệ thống.
 */
void checkEmergencyStop();

/**
 * @brief Chạy các chế độ kiểm thử độc lập dựa trên TEST_MOTOR_MODE trong config.h.
 */
void runTestMode();

#endif // TEST_H
