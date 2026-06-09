#ifndef CONFIG_H
#define CONFIG_H

/**
 * @file config.h
 * @brief Chứa các cấu hình phần mềm, tham số PID mặc định và cài đặt hệ thống.
 */

// Tần số vòng lặp điều khiển chính (mặc định)
#define CONTROL_LOOP_FREQ_HZ    500
#define CONTROL_LOOP_PERIOD_US  (1000000 / CONTROL_LOOP_FREQ_HZ) // 2000us

// Tần số vòng lặp điều khiển fallback khi I2C chạy ở 100kHz
#define FALLBACK_LOOP_FREQ_HZ   250
#define FALLBACK_LOOP_PERIOD_US (1000000 / FALLBACK_LOOP_FREQ_HZ) // 4000us

// Cấu hình giới hạn điều khiển góc của Drone (độ)
#define MAX_ROLL_ANGLE_DEG      30.0f  // Góc nghiêng Roll tối đa (+-30 độ)
#define MAX_PITCH_ANGLE_DEG     30.0f  // Góc nghiêng Pitch tối đa (+-30 độ)
#define MAX_YAW_RATE_DEGS       150.0f // Tốc độ xoay đầu Yaw tối đa (+-150 deg/s)

// Giới hạn ga an toàn khi pin yếu (LOW_BATTERY)
#define THROTTLE_LIMIT_LOW_BAT  1600   // Giới hạn ga tối đa ở mức 1600us
#define THROTTLE_LIMIT_CRIT_BAT 1200   // Giới hạn ga tối đa khi pin nguy kịch trước khi chạm đất

#endif // CONFIG_H
