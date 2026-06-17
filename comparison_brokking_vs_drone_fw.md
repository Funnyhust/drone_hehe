# So sánh chi tiết: Brokking (YMFC-32) vs Drone FW

Tài liệu này so sánh chi tiết hai kiến trúc firmware điều khiển Drone STM32F103C8T6: 
1. **Brokking (YMFC-32)**: Bản thiết kế phần mềm kiểu cũ dạng phẳng (flat) viết trên Arduino IDE.
2. **Drone FW**: Bản thiết kế phần mềm mới theo mô hình hướng đối tượng, mô-đun hóa cao, phát triển trên PlatformIO.

---

## Bảng so sánh tổng quan

| Đặc tính | Brokking (YMFC-32) | Drone FW |
| :--- | :--- | :--- |
| **Công cụ phát triển** | Arduino IDE | VS Code + PlatformIO |
| **Kiến trúc phần mềm** | Flat (các file `.ino` tự nối tiếp), biến toàn cục | Mô-đun hóa C++, tách biệt Driver & Middleware |
| **Bộ thu RC (Receiver)** | PWM/PPM (đọc xung thủ công bằng ngắt Input Capture) | ELRS CRSF (420000 baud, truyền nhận dữ liệu số) |
| **Telemetry** | Không hỗ trợ | Hỗ trợ gửi trạng thái pin ngược về tay điều khiển |
| **Giao thức I2C** | Hardware I2C 2 (Wire.h), cố định 400kHz | Software I2C, tự động khôi phục bus & tự hạ tốc độ |
| **Bộ lọc IMU** | Complementary Filter (99.96% Gyro, 0.04% Accel) | Complementary Filter (98% Gyro, 2% Accel) tách biệt |
| **Vòng lặp (Control Loop)**| Cố định 250 Hz (4000µs), khóa chết chu kỳ | Mặc định 500 Hz (2000µs), tự động hạ tần số nếu quá tải |
| **Thuật toán PID** | PID Đơn vòng (Single-loop Rate PID + bổ trợ góc) | PID Vòng kép (Cascaded Angle + Rate Loop) |
| **Bộ lọc D-term** | Không có | Bộ lọc thông thấp IIR khử nhiễu tần số cao |
| **Trạng thái an toàn** | Kiểm tra cơ bản lúc boot | State Machine: `DISARMED`, `PRE_ARM`, `ARMED`, `FAILSAFE` |
| **Công cụ debug** | Chỉ in Serial giá trị thô thông qua mã phím | CLI UART tương tác, Test Động cơ, Blackbox Logger |

---

## Phân tích chi tiết các điểm nâng cấp

### 1. Kiến trúc hệ thống & Quản lý dự án
* **Brokking**: Tất cả các file `.ino` được trình biên dịch Arduino tự động nối lại thành một file lớn. Các biến được chia sẻ toàn cục không kiểm soát. Điều này gây khó khăn lớn khi muốn thay đổi cấu hình chân (phải tìm từng file) hoặc port code sang dòng chip khác (ESP32, STM32F4...).
* **Drone FW**:
  * Mã nguồn được chia thành hai thư mục chính: `/driver` (phục vụ giao tiếp phần cứng thô như I2C, PWM, ADC) và `/middleware` (chứa các giải thuật toán điều khiển như PID, bộ lọc IMU, quản lý an toàn).
  * Sử dụng file cấu hình tập trung `board_pinmap.h` và `config.h`. Khi cần đổi chân GPIO, người lập trình chỉ cần chỉnh sửa tại một nơi duy nhất.

### 2. Phân hệ bộ thu tay điều khiển (RC Receiver)
* **Brokking**: Sử dụng các chân input của STM32 để đo trực tiếp xung PWM/PPM bằng ngắt phần cứng. Phương pháp này chiếm dụng tài nguyên Timer của chip và dễ bị sai lệch nếu có nhiễu điện từ trên dây tín hiệu.
* **Drone FW**: Giao tiếp với bộ thu ELRS hiện đại qua giao thức **CRSF**.
  * Tốc độ phản hồi cực nhanh ở **420,000 baud**.
  * Chạy máy trạng thái phân tích gói tin tuần tự, kiểm tra tính toàn vẹn dữ liệu bằng thuật toán CRC-8.
  * Tích hợp cơ chế kiểm tra mất sóng (Failsafe Hard Kill) nếu không nhận được gói tin quá 200ms.

### 3. Giao tiếp I2C & Đọc cảm biến
* **Brokking**: Sử dụng thư viện `Wire.h` phần cứng. Nếu bus I2C gặp nhiễu điện (ví dụ từ ESC hoặc động cơ) dẫn đến việc một thiết bị kéo đường SDA xuống thấp mãi mãi, chip STM32 sẽ bị khóa cứng (hành vi treo chip).
* **Drone FW**:
  * Sử dụng **Software I2C** (Bit-banging) cho phép linh hoạt cấu hình chân I2C trên bất kỳ GPIO nào.
  * Tích hợp hàm **`softI2cBusRecovery()`**: Tự động gửi 9 chu kỳ xung Clock để ép các thiết bị phụ (IMU) nhả đường truyền SDA nếu phát hiện bus bị kẹt.
  * **Fallback tốc độ**: Khi khởi tạo ở tốc độ cao 400kHz thất bại (do dây dài hoặc nhiễu), hệ thống tự động cấu hình lại sang 100kHz thay vì dừng hoạt động.

### 4. Thuật toán điều khiển PID
* **Brokking**:
  * Chỉ chạy một vòng lặp Rate. Khi ở chế độ tự cân bằng (Auto-level), góc lệch của drone được nhân với một hệ số P (`pitch_level_adjust = angle_pitch * 15`) để tạo ra một tốc độ góc ảo bù vào setpoint đầu vào. Cơ chế này đơn giản nhưng dễ gây dao động khi drone tiến gần về vị trí cân bằng.
* **Drone FW**:
  * Áp dụng **PID vòng kép (Cascaded PID)** tiêu chuẩn của các FC hiện đại (như Betaflight):
    * **Vòng ngoài (Angle Loop)**: So sánh góc đặt với góc ước lượng thực tế từ IMU để tính ra tốc độ xoay (°/s) cần thiết.
    * **Vòng trong (Rate Loop)**: So sánh tốc độ xoay yêu cầu với tốc độ đo được từ Gyro, tính toán đầu ra PID để cấp trực tiếp cho ESC.
  * Bộ lọc **D-term Lowpass IIR** lọc bỏ nhiễu tần số cao từ động cơ truyền vào cảm biến, giúp motor chạy êm hơn và tránh bị nóng.

### 5. Khả năng giám sát và bảo mật an toàn (Safety)
* **Brokking**: Không có Watchdog bảo vệ. Quy trình ARM/DISARM motor chỉ kiểm tra vị trí cần ga đơn giản.
* **Drone FW**:
  * Quản lý trạng thái bằng máy trạng thái hữu hạn (FSM) tại `safety.cpp`. Động cơ chỉ được phép ARM khi hệ thống vượt qua tất cả các bài test tự kiểm tra (IMU hoạt động tốt, pin không quá yếu, tay điều khiển kết nối ổn định).
  * Tích hợp **Watchdog Timer (IWDG)**: Nếu phần mềm bị treo quá thời hạn cho phép, hệ thống sẽ tự khởi động lại chip và ngắt toàn bộ ESC ngay lập tức để tránh tai nạn bay tự do.
  * Tích hợp **Blackbox** lưu trữ lịch sử bay, hỗ trợ xuất log qua Serial phục vụ việc tinh chỉnh (tune) PID.
