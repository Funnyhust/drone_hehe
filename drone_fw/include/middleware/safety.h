#ifndef SAFETY_H
#define SAFETY_H

#include <Arduino.h>

/**
 * @file safety.h
 * @brief Middleware quản lý trạng thái bay (ARM/DISARM State Machine), Watchdog và Failsafe.
 * @note Định nghĩa chu kỳ Failsafe 100ms warning / 200ms hard kill, cấu hình IWDG 200ms,
 *       và logic bảo vệ pin yếu / lỗi IMU.
 */

// Định nghĩa 4 trạng thái hệ thống
enum FlightState {
  STATE_DISARMED = 0, // Đã ngắt động cơ, khóa an toàn
  STATE_PRE_ARM,      // Đang kiểm tra điều kiện an toàn trước khi arm
  STATE_ARMED,        // Đã kích hoạt động cơ, bay tự do
  STATE_FAILSAFE      // Trạng thái sự cố khẩn cấp, ngắt toàn bộ động cơ
};

/**
 * @brief Khởi tạo mô hình an toàn và cấu hình IWDG Watchdog.
 * @details Thiết lập IWDG với timeout ban đầu khoảng 200ms, đặt trạng thái DISARMED.
 */
void safetyInit();

/**
 * @brief Cập nhật State Machine và kiểm tra các điều kiện an toàn.
 * @details Được gọi liên tục trong vòng lặp điều khiển chính (500Hz).
 *          Xử lý chuyển trạng thái, failsafe và feed watchdog.
 * @param imu_ok Trạng thái đọc cảm biến IMU hiện tại (true nếu đọc thành công, false nếu lỗi)
 */
void safetyUpdate(bool imu_ok);

/**
 * @brief Thực hiện xóa cờ IWDG Watchdog (Feed Watchdog).
 */
void safetyFeedWatchdog();

/**
 * @brief Trả về trạng thái bay hiện tại của hệ thống.
 */
FlightState safetyGetState();

/**
 * @brief Trả về chuỗi ký tự mô tả trạng thái hiện tại (phục vụ in debug).
 */
const char* safetyGetStateStr();

/**
 * @brief Yêu cầu chuyển sang trạng thái ARMED (gạt công tắc ARM).
 * @return true nếu Arm thành công, false nếu pre-arm check thất bại.
 */
bool safetyRequestArm();

/**
 * @brief Yêu cầu chuyển sang trạng thái DISARMED (gạt công tắc DISARM hoặc tắt khẩn cấp).
 */
void safetyRequestDisarm();

#endif // SAFETY_H
