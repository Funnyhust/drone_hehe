#ifndef SAFETY_H
#define SAFETY_H

#include <Arduino.h>

/**
 * @file safety.h
 * @brief Middleware quản lý trạng thái bay (ARM/DISARM State Machine), Watchdog và Failsafe.
 */

// Định nghĩa 4 trạng thái bay của hệ thống
enum FlightState {
  STATE_DISARMED = 0, // Đã ngắt động cơ, khóa an toàn
  STATE_PRE_ARM,      // Đang kiểm tra điều kiện an toàn trước khi arm
  STATE_ARMED,        // Đã kích hoạt động cơ, bay tự do
  STATE_FAILSAFE      // Trạng thái sự cố khẩn cấp, ngắt toàn bộ động cơ
};

// Định nghĩa các trạng thái của Startup State Machine
enum StartupState {
  STARTUP_BOOT = 0,
  STARTUP_IMU_INIT,
  STARTUP_GYRO_CALIB,
  STARTUP_RC_CHECK,
  STARTUP_ACC_CHECK,
  STARTUP_ESC_CHECK,
  STARTUP_READY,
  STARTUP_ERROR
};

// Định nghĩa các mã lỗi trong quá trình khởi động
enum StartupError {
  ERR_NONE = 0,
  ERR_IMU_INIT_FAIL,        // 1 nhịp chớp LED: Không giao tiếp được MPU6050
  ERR_GYRO_CALIB_MOVING,    // 2 nhịp chớp LED: Drone bị di chuyển/rung lắc lúc Calib
  ERR_RC_LINK_LOST,         // 3 nhịp chớp LED: Mất sóng tay điều khiển ELRS
  ERR_RC_CENTER_OUT,        // 4 nhịp chớp LED: Cần điều khiển lệch tâm lúc boot (Roll/Pitch/Yaw)
  ERR_THROTTLE_NOT_MIN,     // 5 nhịp chớp LED: Cần ga ở mức cao lúc boot
  ERR_ACC_VALIDATION_FAIL,  // 6 nhịp chớp LED: Tổng vector gia tốc g nằm ngoài 0.95g ~ 1.05g
  ERR_EEPROM_FAIL           // 7 nhịp chớp LED: Lỗi đọc ghi EEPROM / CRC dữ liệu
};

/**
 * @brief Khởi tạo mô hình an toàn và cấu hình IWDG Watchdog.
 */
void safetyInit();

/**
 * @brief Cập nhật State Machine và kiểm tra các điều kiện an toàn.
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
 * @brief Lấy chuỗi mô tả trạng thái bay tương ứng.
 */
const char* safetyGetStateStr(FlightState state);

/**
 * @brief Yêu cầu chuyển sang trạng thái ARMED.
 */
bool safetyRequestArm();

/**
 * @brief Yêu cầu chuyển sang trạng thái DISARMED.
 */
void safetyRequestDisarm();

// =============================================================================
// Các API quản lý Máy trạng thái Khởi động (Startup State Machine)
// =============================================================================

/**
 * @brief Lấy trạng thái khởi động hiện tại.
 */
StartupState safetyGetStartupState();

/**
 * @brief Thiết lập trạng thái khởi động.
 */
void safetySetStartupState(StartupState state);

/**
 * @brief Lấy mã lỗi khởi động.
 */
StartupError safetyGetStartupError();

/**
 * @brief Thiết lập mã lỗi khởi động.
 */
void safetySetStartupError(StartupError error);

/**
 * @brief Lấy chuỗi mô tả trạng thái khởi động.
 */
const char* safetyGetStartupStateStr(StartupState state);

/**
 * @brief Lấy chuỗi mô tả lỗi khởi động.
 */
const char* safetyGetStartupErrorStr(StartupError error);

#endif // SAFETY_H
