# Kế hoạch Sửa lỗi và Bổ sung Tính năng Setup (Fix Plan v2)

Bản kế hoạch này tập trung giải quyết triệt để lỗi lộn nhào (Flip) do thuật toán trộn động cơ (Mixer) và bổ sung 3 tính năng hiệu chuẩn (Calibration) tinh hoa được học hỏi từ dự án YMFC-AL.

## Mục tiêu (Goal Description)
1. Cứu drone khỏi việc tự lộn nhào khi cất cánh do lỗi Positive Feedback ở trục Pitch.
2. Hoàn thiện trải nghiệm Setup phần cứng thông qua CLI: Hiệu chuẩn hành trình ga ESC, Cân bằng động cơ đo rung động (Vibration Balancing), và tự động tải thông số bù (Offset) từ EEPROM.
3. **[MỚI] Tích hợp kích hoạt lệnh Calib và Save bằng kênh AUX (CH5) từ tay điều khiển ELRS.**

> [!IMPORTANT]
> **Yêu cầu đối với người dùng (User Review Required):**
> Các tính năng này sẽ can thiệp vào cách khởi động của drone và thay đổi cấu trúc lưu trữ I2C EEPROM (24LC256). Đảm bảo mạch của bạn đã hàn đúng chip EEPROM theo Schematic.

## Các thay đổi dự kiến (Proposed Changes)

### 1. Phân hệ Lõi điều khiển bay (Flight Controller Core)

#### [MODIFY] `src/middleware/motor_mixer.cpp`
* **Sửa lỗi lộn nhào (Fatal Bug Fix):** Đảo ngược dấu của biến `pitch` (`pitch = -pitch;`) ngay phần đầu của hàm `motorMixerCompute()`. 
* Điều này giúp động cơ phía trước chậm lại và động cơ sau tăng tốc khi cần "chúc mũi", thay vì làm ngược lại.

### 2. Phân hệ Giao tiếp Cảm biến và Bộ nhớ

#### [MODIFY] `src/driver/mpu6050.h` & `src/driver/mpu6050.cpp`
* **Lưu trữ Offset:** Bổ sung các hàm `mpu6050LoadOffsetsFromEEPROM()` và `mpu6050SaveOffsetsToEEPROM()`.
* **Cơ chế:** Offset của Accel và Gyro sẽ được lưu vào 12 byte đầu tiên của bộ nhớ 24LC256.

### 3. Phân hệ Vòng lặp chính và Giao tiếp (CLI & RC)

#### [MODIFY] `src/main.cpp`
* **Tối ưu khởi động (Boot Optimization):** 
  * Xóa dòng gọi `mpu6050Calibrate()` tốn 2 giây mỗi lần bật nguồn.
  * Thay thế bằng lệnh load offset từ EEPROM. (Chỉ khi nào kích hoạt hàm Calib thì mới chạy và đè vào EEPROM).
* **Tính năng Cân bằng cánh quạt (Vibration Test - YMFC Style):** 
  * Chỉnh sửa lệnh `t` (Test Motor). Trong khi motor quay, MCU sẽ liên tục đọc gia tốc (Accel) và tính toán độ chênh lệch biên độ cực đại (Max - Min của `ax, ay, az`) để quy đổi thành **"Điểm rung động" (Vibration Score)**. In điểm số ra CLI.
* **Tính năng Hiệu chuẩn ESC (ESC Calibration Mode):**
  * Thêm lệnh `e` vào hệ thống CLI: Xác nhận thao tác tháo cánh quạt -> Xuất xung `2000us` ra cả 4 động cơ -> Báo người dùng cắm pin -> Chờ người dùng nhấn phím bất kỳ -> Rút xung về `1000us`.

#### [MODIFY] `src/middleware/safety.cpp` (Hoặc thêm file mới `rc_handler.cpp`)
* **[MỚI] Tích hợp Hiệu chuẩn qua Tay điều khiển (ELRS CH5):**
  * Tận dụng kênh AUX của ELRS (dải tín hiệu CRSF từ ~988 đến 2012) để kích hoạt các chế độ từ xa mà không cần cắm cáp USB.
  * *Ý tưởng phân bổ dải PWM (Sẽ nghiên cứu kỹ hơn vào ngày mai):*
    * `CH5 > 1950`: Chế độ Lưu thông số (Save Mode - Lưu PID/Offset vào EEPROM).
    * `1300 < CH5 < 1700`: Chế độ chờ (Standby / Normal Flight).
    * `CH5 < 1100`: Kích hoạt chế độ Hiệu chuẩn MPU6050 (Calib Mode).

---

## Kế hoạch Kiểm tra (Verification Plan)

### Kiểm tra tĩnh trên bàn (Không cánh quạt)
1. Bật nguồn: Gạt công tắc CH5 trên tay điều khiển xuống mức thấp (`<1100`), đèn báo hiệu quá trình calib MPU6050 đang diễn ra. Gạt CH5 lên mức cao (`>1950`) để save offset vào EEPROM.
2. Gõ `t` trên CLI: Bật thử 1 motor và gõ móng tay vào mạch. Điểm số Vibration Score phải nhảy số liên tục.
3. Rút nguồn cắm lại: Xem thông số góc Roll, Pitch có lập tức trở về ~0 độ nhờ EEPROM hay không.

### Kiểm tra chiều tương tác bù trừ (Anti-flip Check)
Cắm pin, đẩy ga nhẹ để 4 motor quay đều (Chưa lắp cánh quạt).
* **Nhấc đuôi máy bay lên (Chúc mũi xuống):** Motor phía TRƯỚC (FR, FL) phải rít mạnh lên, Motor phía SAU (RR, RL) phải giảm tốc độ. Lỗi lộn nhào đã được trị dứt điểm.
