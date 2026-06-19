#ifndef CONFIG_H
#define CONFIG_H

/**
 * @file config.h
 * @brief Chứa các cấu hình phần mềm, tham số PID mặc định và cài đặt hệ thống.
 */

// Tần số vòng lặp điều khiển chính (mặc định)
#define CONTROL_LOOP_FREQ_HZ 250
#define CONTROL_LOOP_PERIOD_US (1000000 / CONTROL_LOOP_FREQ_HZ) // 4000us

// Bật/Tắt kiểm tra điện áp pin ảo (0: Luôn coi như pin đầy để dễ test trên USB,
// 1: Bật bảo vệ pin thực tế)
#define ENABLE_BATTERY_CHECK 0

// Tần số vòng lặp điều khiển fallback khi I2C chạy ở 100kHz
#define FALLBACK_LOOP_FREQ_HZ 250
#define FALLBACK_LOOP_PERIOD_US (1000000 / FALLBACK_LOOP_FREQ_HZ) // 4000us

// Cấu hình giới hạn điều khiển góc của Drone (độ)
#define MAX_ROLL_ANGLE_DEG 30.0f  // Góc nghiêng Roll tối đa (+-30 độ)
#define MAX_PITCH_ANGLE_DEG 30.0f // Góc nghiêng Pitch tối đa (+-30 độ)
#define MAX_YAW_RATE_DEGS 150.0f  // Tốc độ xoay đầu Yaw tối đa (+-150 deg/s)

// Hệ số vi chỉnh góc lệch (Software Trim) để sửa lỗi tự trôi khi thả cần
// Nếu drone tự trôi về PHÍA TRƯỚC: đặt giá trị dương (ví dụ 1.0f hoặc 1.5f)
// Nếu drone tự trôi về PHÍA SAU: đặt giá trị âm (ví dụ -1.0f hoặc -1.5f)
#define PITCH_TRIM_OFFSET 0.0f  // Điều chỉnh trôi tiến/lùi (độ)
#define ROLL_TRIM_OFFSET  0.0f  // Điều chỉnh trôi trái/phải (độ)

// Giới hạn ga an toàn khi pin yếu (LOW_BATTERY)
#define THROTTLE_LIMIT_LOW_BAT 1600 // Giới hạn ga tối đa ở mức 1600us
#define THROTTLE_LIMIT_CRIT_BAT                                                \
  1200 // Giới hạn ga tối đa khi pin nguy kịch trước khi chạm đất

// =============================================================================
// Cấu hình chế độ kiểm tra trước khi bay (Pre-flight Test Mode)
// =============================================================================
// 1: Bật chế độ test an toàn (giới hạn xung motor tối đa để không bao giờ bay
// lên) 0: Chế độ bay bình thường
#define PRE_FLIGHT_TEST 0
#define PRE_FLIGHT_MAX_PULSE                                                   \
  2012 // Xung ga tối đa khi test (us), động cơ chỉ quay nhẹ

#define TEST_MOTOR_MODE 0

// 0: Vô hiệu hóa (Chạy chế độ bay bình thường)
// 1: Ramp Test (Tự động tăng 0% -> 100% -> 0% theo bước 10%, mỗi mức giữ 5
// giây) 2: Smooth Ramp Test (Kiểm tra tăng ga từ từ 1% mỗi 50ms sử dụng rampTo)
// 3: ESC Calibration Mode (Hiệu chuẩn hành trình ga ESC: 2000us -> 3s ->
// 1000us) 4: Stress Test (Tăng/giảm ga đột ngột 0 -> 100% -> 0% lặp lại 100
// lần) 5: Motor Direction Test (Quay 10% ga ổn định để kiểm tra chiều quay động
// cơ) 6: Test MPU6050 (In dữ liệu Accel/Gyro) 7: Test ELRS CRSF (In dữ liệu
// kênh RC giải mã) 8: Direct RC Control (Điều khiển thẳng 4 động cơ bằng cần ga
// Throttle

#define CALIBRATION_MODE 0
/* - 1 Calibrate ESC: ESC phát xung 2000us khi CH5 > 1750us, sau 5s phát xung
 1000us khi CH5 <=1750
   - 2 Calibrate Accel: Đặt drone nằm ngang, CH5 > 1750us thì sẽ Calibrare
*/

#endif // CONFIG_H
