# Thuyết Trình: Kiến Trúc File `main.cpp` - Trái Tim Hệ Thống Điều Khiển Bay

---

File `main.cpp` không chỉ đơn thuần là một file code thông thường. Với gần 900 dòng code, nó là **điều phối viên trung tâm** của toàn bộ hệ thống bay. Firmware được xây dựng trên kiến trúc **bare-metal thuần túy** (không sử dụng RTOS), thay vào đó triển khai cơ chế lập lịch phi chặn (Non-blocking Super-loop Scheduler) đồng bộ qua bộ đếm thời gian `micros()` để loại bỏ hoàn toàn độ trễ chuyển ngữ cảnh. Toàn bộ dữ liệu trong file này chỉ là các biến số toán học nằm trong RAM, chưa có bất kỳ dòng điện nào xuất ra phần cứng — công việc đó được uỷ quyền hoàn toàn cho các tầng Driver bên dưới.

Em chia kiến trúc của `main.cpp` thành 4 phân hệ chạy tuần tự theo thứ tự thời gian:

---

## Phân hệ -1: Khai báo và Cấp phát Bộ nhớ Toàn cục (Dòng 1–105)

Trước khi bất kỳ hàm nào được thực thi, vi điều khiển sẽ nạp toàn bộ khu vực khai báo biến toàn cục lên RAM. Đây là bước chuẩn bị tài nguyên tĩnh (Static Allocation):

Em khai báo tất cả header của các tầng Driver và Middleware — từ `soft_i2c.h`, `mpu6050.h`, `rc_crsf.h` (phần cứng) cho đến `pid_controller.h`, `motor_mixer.h`, `safety.h` (thuật toán). Đây chính là bản đồ kiến trúc phân tầng của toàn bộ hệ thống thể hiện ngay ở phần `#include`.

Tiếp theo, em cấp phát vùng nhớ cố định cho các biến trạng thái sống còn của máy bay: `imu_raw` lưu dữ liệu cảm biến thô, `attitude_angles` lưu góc nghiêng hiện tại, `out_roll/pitch/yaw` lưu đầu ra PID, và `last_loop_time_us` theo dõi thời gian vòng lặp. Tất cả đều dùng từ khóa `static` để đảm bảo dữ liệu tồn tại vĩnh viễn trong RAM suốt vòng đời máy bay mà không bị phân mảnh bộ nhớ.

---

## Phân hệ 0: Khởi tạo Hệ thống — `setup()` (Từ dòng 595)

Hàm `setup()` chỉ chạy duy nhất một lần ngay khi cấp nguồn. Nhiệm vụ của nó là đánh thức và khởi tạo tuần tự toàn bộ hệ thống theo đúng thứ tự phụ thuộc:

Đầu tiên là khởi tạo đồng hồ thời gian thực RTC dùng thạch anh 32.768kHz, sau đó lần lượt là SoftI2C bus, EEPROM, bộ thu RC CRSF, Motor — được khóa cứng ở 1000µs ngay từ đầu để đảm bảo an toàn tuyệt đối, tiếp theo là Battery ADC, Safety FSM, Blackbox, IMU Estimator và cuối cùng là PID Controller.

Sau khi tất cả module sẵn sàng, hệ thống đặt trạng thái ban đầu thành `STARTUP_BOOT` để kích hoạt máy trạng thái khởi động ở phân hệ tiếp theo.

---

## Phân hệ 1: Máy Trạng Thái Khởi Động Phi Chặn — Non-blocking Startup FSM (Dòng 126–445)

Đây là **chốt chặn an toàn bắt buộc** trước khi máy bay được phép cất cánh. Điểm đặc biệt của phân hệ này là không sử dụng bất kỳ hàm `delay()` nào — tránh treo CPU — mà thay vào đó chạy theo kiểu phi chặn (non-blocking), mỗi lần `loop()` được gọi thì FSM tiến thêm một bước nhỏ.

Máy trạng thái gồm 8 bước tuần tự:

1. **`STARTUP_BOOT`** — Đọc EEPROM/Flash tải bộ thông số PID và Offset cảm biến. Kiểm tra xem ESC và Accel đã được hiệu chuẩn chưa.
2. **`STARTUP_IMU_INIT`** — Khởi tạo MPU6050 ở tốc độ I2C 400kHz, có cơ chế dự phòng tự động lùi về 100kHz nếu gặp nhiễu.
3. **`STARTUP_GYRO_CALIB`** — Thu 1000 mẫu Gyro trong 4 giây. Thuật toán tính **Kỳ vọng (Mean)** và **Độ lệch chuẩn (StdDev)**. Nếu StdDev vượt ngưỡng, hệ thống phát hiện drone đang bị rung lắc và từ chối khởi động — cho phép thử lại tối đa 5 lần.
4. **`STARTUP_RC_CHECK`** — Kiểm tra sóng CRSF. Bắt buộc ga phải ở mức MIN (<1050µs) và các cần Roll/Pitch/Yaw phải về đúng tâm (1500 ± 30µs). Đây là cơ chế **chống chém tay** — nếu sai là khóa, không cho bay.
5. **`STARTUP_ACC_CHECK`** — Đo gia tốc tĩnh qua 10 mẫu. Nếu tổng vector $g = \sqrt{ax^2 + ay^2 + az^2}$ không nằm trong dải 0.95g – 1.05g thì mạch báo lỗi phần cứng.
6. **`STARTUP_ESC_CHECK`** — Xác nhận cờ `esc_calibrated` trong EEPROM. Rẽ nhánh vào chế độ Calib ESC nếu phi công gạt CH5, hoặc đi thẳng vào vòng lặp bay.
7. **`STARTUP_READY`** — Trạng thái đích hoàn hảo. Toàn bộ hệ thống Pass 100%, máy bay được phép cất cánh.
8. **`STARTUP_ERROR`** — Trạng thái khoá chết. Bất kỳ bài test nào thất bại, hệ thống rơi vào đây, ngắt toàn bộ PWM động cơ và khoá vĩnh viễn để chống cháy nổ.

---

## Phân hệ 3: Vòng Lặp Điều Khiển Chính Thời Gian Thực — 250Hz (Từ dòng 656)

Đây là lõi động học của máy bay. Vòng lặp được kiểm soát thời gian nghiêm ngặt bằng cơ chế **non-blocking timer**: chỉ thực thi khi đã đủ 4000µs kể từ chu kỳ trước, đảm bảo tần số cố định 250Hz.

Mỗi chu kỳ 4ms, hệ thống thực hiện tuần tự:

1. **Thu thập dữ liệu** — Đọc sóng vô tuyến CRSF từ tay khiển và Burst Read toàn bộ 6 trục cảm biến MPU6050 qua I2C.
2. **Lọc và ước lượng tư thế** — Đẩy dữ liệu thô qua `imuEstimatorUpdate()` chạy Bộ lọc bù (Complementary Filter) để tính ra góc nghiêng Roll/Pitch chính xác.
3. **Cập nhật Safety Watchdog** — Gọi `safetyUpdate()` để theo dõi tín hiệu RC, phát hiện mất sóng và kích hoạt Failsafe nếu cần.
4. **Xử lý tín hiệu tay khiển** — Áp dụng vùng chết Deadband 8µs để loại nhiễu tay rung. Quy đổi độ lệch cần sang góc mục tiêu (chia 15.0 cho Roll/Pitch) và tốc độ góc Yaw (chia 3.0).
5. **Giám sát Pin tự động** — Nếu pin yếu (`BATTERY_LOW`) thì kẹp cứng ga không vượt ngưỡng an toàn. Nếu pin nguy kịch (`BATTERY_CRITICAL`) thì ép máy bay hạ cánh khẩn cấp.
6. **Chống Windup khi chạm đất** — Khi ga < 1050µs, gọi `pidReset()` xoá sạch khâu tích phân `i_mem`, ngăn tích lũy sai số trước khi cất cánh.
7. **Tính toán PID và xuất động cơ** (chỉ khi `STATE_ARMED`):
   - Nếu ESC đã calib: `pidCompute()` → `motorMixerCompute()` → `motorWriteAllUs()` — nạp 4 giá trị xung vào Hardware Timer.
   - Nếu ESC chưa calib: gọi `motorStopAll()`, từ chối cấp công suất hoàn toàn.
   - Nếu Disarmed hoặc Failsafe: khoá cứng `motorStopAll()` và `pidReset()`.
8. **Ghi Blackbox** — Ghi log bay vào bộ nhớ (chỉ khi `STARTUP_READY`).
9. **Giám sát Loop Budget** — Đếm số lần vòng lặp vượt ngân sách thời gian. Nếu vượt quá 50 lần liên tiếp, hệ thống tự động hạ tần số xuống chế độ dự phòng để cứu máy bay khỏi bị treo.
10. **Tác vụ nền (Background Tasks)** — Cứ 200ms gửi Telemetry Pin về tay khiển qua CRSF, cứ 1 giây gửi đồng hồ RTC về màn hình phi công.

---

> **Tóm lại:** `main.cpp` hoạt động theo nguyên tắc **phân tầng và phi chặn hoàn toàn** — không có bất kỳ hàm `delay()` nào trong vòng lặp chính. Mọi tác vụ đều được lập lịch theo thời gian thực, đảm bảo CPU không bao giờ bị chặn và vòng điều khiển 250Hz luôn chạy đúng tiến độ.
