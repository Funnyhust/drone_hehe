#ifndef BLACKBOX_H
#define BLACKBOX_H

#include <Arduino.h>

/**
 * @file blackbox.h
 * @brief Bộ đệm vòng ghi log dữ liệu bay (Blackbox Ring Buffer) trong RAM.
 * @note Lưu trữ tối ưu các thông số: Loop Time, PID Output, VBAT, RSSI, LQ
 *       để phục vụ gỡ lỗi rung lắc, failsafe mà không sử dụng dynamic allocation.
 */

// Định nghĩa kích thước bộ đệm vòng (số lượng mẫu lưu trữ)
#define BLACKBOX_LIMIT  256

struct BlackboxEntry {
  uint32_t loop_time_us; // Thời gian thực thi vòng lặp (us)
  int16_t out_roll;      // Đầu ra Roll PID
  int16_t out_pitch;     // Đầu ra Pitch PID
  int16_t out_yaw;       // Đầu ra Yaw PID
  uint16_t vbat_mv;      // Điện áp pin (mV)
  uint8_t lq;            // Chất lượng liên kết ELRS (%)
  int8_t rssi;           // Cường độ tín hiệu (dBm)
};

/**
 * @brief Khởi tạo bộ đệm vòng Blackbox.
 */
void blackboxInit();

/**
 * @brief Ghi một mẫu dữ liệu bay mới vào bộ đệm vòng.
 * @note Hàm này chạy non-blocking và tối ưu hóa thời gian thực thi trong loop chính.
 */
void blackboxLog(uint32_t loop_time_us, float out_roll, float out_pitch, float out_yaw,
                 float vbat, uint8_t lq, int8_t rssi);

/**
 * @brief Xuất toàn bộ dữ liệu trong bộ đệm ra cổng Serial dưới dạng CSV.
 * @note Chỉ được gọi khi drone ở trạng thái DISARMED.
 */
void blackboxDumpSerial();

/**
 * @brief Reset bộ đệm vòng (xóa toàn bộ log cũ).
 */
void blackboxReset();

#endif // BLACKBOX_H
