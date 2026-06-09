#ifndef RC_CRSF_H
#define RC_CRSF_H

#include <Arduino.h>

/**
 * @file rc_crsf.h
 * @brief Driver nhận và giải mã tín hiệu RC giao thức CRSF (ExpressLRS) qua Hardware UART2 (PA2/PA3).
 * @note Lọc nhiễu bằng thuật toán CRC-8 (đa thức 0xD5), giải mã các gói RC_CHANNELS_PACKED và LINK_STATISTICS.
 */

// Định nghĩa số lượng kênh điều khiển CRSF hỗ trợ
#define CRSF_NUM_CHANNELS   16

// Giá trị mặc định an toàn cho các kênh khi chưa nhận được tín hiệu (us)
#define CRSF_CHANNEL_MIN_US   988
#define CRSF_CHANNEL_MID_US   1500
#define CRSF_CHANNEL_MAX_US   2012

/**
 * @brief Khởi tạo bộ thu UART2 kết nối với module ELRS.
 * @details Cấu hình Serial2 ở tốc độ 420000 baud trên các chân PA2 (TX) và PA3 (RX).
 */
void crsfInit();

/**
 * @brief Cập nhật dữ liệu từ cổng Serial2 và giải mã các gói tin CRSF.
 * @note Hàm này cần được gọi liên tục trong vòng lặp chính (loop()) để tránh đầy bộ đệm.
 */
void crsfUpdate();

/**
 * @brief Lấy giá trị chu kỳ xung (us) của một kênh cụ thể.
 * @param ch Chỉ số kênh (0 đến 15, ch0=Roll, ch1=Pitch, ch2=Throttle, ch3=Yaw, ch4=Arm, ch5=Mode...)
 * @return uint16_t Giá trị quy đổi từ 988us đến 2012us
 */
uint16_t crsfGetChannel(uint8_t ch);

/**
 * @brief Kiểm tra xem liên kết sóng điều khiển còn hoạt động hay không.
 * @return true nếu nhận được gói tin RC hợp lệ trong vòng 200ms gần nhất, ngược lại false (Failsafe).
 */
bool crsfIsLinkActive();

/**
 * @brief Kiểm tra xem có đang ở trạng thái cảnh báo mất sóng nhẹ hay không (mất tin từ 100ms - 200ms).
 */
bool crsfIsLinkWarning();

/**
 * @brief Lấy chất lượng liên kết (Link Quality - LQ %).
 * @return uint8_t Chất lượng từ 0 đến 100%
 */
uint8_t crsfGetLq();

/**
 * @brief Lấy chỉ số cường độ tín hiệu nhận được (RSSI dBm).
 * @return int8_t Cường độ tín hiệu dBm (ví dụ: -60dBm, -80dBm)
 */
int8_t crsfGetRssi();

#endif // RC_CRSF_H
