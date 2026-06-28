# Giải Thích Chi Tiết: Kiến Trúc Phân Tầng Từ Bộ Trộn (Mixer) Đến Xung PWM Phần Cứng

Quá trình luân chuyển dữ liệu từ Toán học (PID) ra Xung điện (PWM) là sự phối hợp của một kiến trúc phần mềm phân tầng nghiêm ngặt. Để giải thích cho hội đồng, chúng ta cần chia làm hai khía cạnh: **Luồng Dữ Liệu (Data Flow)** và **Giao Tiếp Phần Cứng (Hardware Abstraction)**.

---

## PHẦN 1: KIẾN TRÚC PHẦN MỀM VÀ ĐIỀU PHỐI TRUNG TÂM (`main.cpp`)

Ở module `main.cpp`, toàn bộ dữ liệu đang được xử lý **chỉ là các biến số toán học (biến float, uint16_t) nằm trong RAM**. Chưa có bất kỳ dòng điện nào được xuất ra phần cứng.

**3. Khối Điều Phối Trung Tâm (`main.cpp` - Kiến trúc bare-metal với bộ lập lịch phi chặn thủ công, gần 900 dòng code):**
Đóng vai trò là "Điều phối viên trung tâm" của hệ thống. Không sử dụng RTOS, thay vào đó dùng kiến trúc Super-loop phi chặn đồng bộ qua `micros()` để loại bỏ độ trễ chuyển ngữ cảnh. Gần 900 dòng code được chia thành các phân hệ chính:

*   **Phân hệ -1: Khai báo Thư viện và Cấp phát Bộ nhớ Toàn cục (Dòng 1 đến 105):**
    Trước khi hệ thống chạy bất cứ hàm nào, đây là khu vực chuẩn bị tài nguyên tĩnh (Static Allocation):
    - Khai báo tất cả các file header của Driver (phần cứng) và Middleware (thuật toán).
    - Cấp phát vùng nhớ cho các biến trạng thái sống còn của máy bay: Dữ liệu IMU thô (`imu_raw`), Góc nghiêng (`attitude_angles`), Đầu ra PID (`out_roll`, `out_pitch`, `out_yaw`), và Biến quản lý thời gian vòng lặp (`last_loop_time_us`). Việc dùng biến `static` giúp dữ liệu tồn tại vĩnh viễn trong RAM mà không bị phân mảnh bộ nhớ rác.

*   **Phân hệ 0: Khởi tạo hệ thống - `setup()` (Từ dòng 595):**
    Hàm `setup()` chạy duy nhất một lần khi cấp nguồn, khởi tạo tuần tự toàn bộ driver và middleware:
    - Khởi tạo đồng hồ thời gian thực RTC (Real-Time Clock).
    - Khởi tạo SoftI2C bus (Giao tiếp cảm biến) → EEPROM → CRSF (bộ thu RC) → Motor (khóa cứng ở 1000us) → Battery ADC → Safety FSM → Blackbox → IMU Estimator → PID Controller.
    - Sau khi tất cả driver sẵn sàng, hệ thống mới đặt trạng thái khởi động ban đầu thành `STARTUP_BOOT` để kích hoạt FSM.

*   **Phân hệ 1: Máy trạng thái Khởi động Phi chặn (Non-blocking Startup FSM - Từ dòng 126 đến 445):**
    Đây là chốt chặn an toàn sống còn trước khi bay. Máy trạng thái chia làm 8 bước, chạy không dùng hàm `delay()` để tránh treo CPU:
    - `STARTUP_BOOT`: Đọc EEPROM/Flash để tải bộ thông số PID (P=1.0, I=0.012, D=5.0) và Offset cảm biến. Kiểm tra xem người dùng đã Calib ESC và Accel chưa.
    - `STARTUP_IMU_INIT`: Khởi tạo MPU6050 ở tốc độ I2C 400kHz (có cơ chế dự phòng lùi về 100kHz nếu nhiễu).
    - `STARTUP_GYRO_CALIB`: Lấy 1000 mẫu Gyro (mỗi mẫu cách nhau 4ms). Thuật toán tính **Kỳ vọng (Mean)** và **Độ lệch chuẩn (StdDev)** của 1000 mẫu. Nếu StdDev vượt ngưỡng, hệ thống biết Drone đang bị rung lắc và từ chối khởi động. Cho phép thử lại tối đa 5 lần trước khi khóa.
    - `STARTUP_RC_CHECK`: Kiểm tra sóng CRSF. Đặc biệt, thuật toán bắt buộc Ga phải ở mức MIN (<1050us) và các cần Roll/Pitch/Yaw phải ở đúng tâm (1500 $\pm$ 30). Nếu sai, khóa khởi động để chống chém tay. Chờ tối đa 5 giây để tay điều khiển kết nối.
    - `STARTUP_ACC_CHECK`: Đo gia tốc tĩnh (Static Gravity) qua 10 mẫu. Nếu tổng vector gia tốc $g = \sqrt{ax^2 + ay^2 + az^2}$ không nằm trong khoảng $0.95g - 1.05g$, mạch sẽ báo lỗi phần cứng.
    - `STARTUP_ESC_CHECK`: Xác nhận lại cờ `esc_calibrated` và `accel_offset` trong EEPROM. Rẽ nhánh xem người dùng có muốn vào chế độ Calib ESC qua CH5 hay đi thẳng vào vòng lặp bay.
    - `STARTUP_READY`: Trạng thái đích hoàn hảo. Báo hiệu toàn bộ hệ thống cảm biến, tay khiển đã Pass 100%. Xác nhận cho phép máy bay cất cánh.
    - `STARTUP_ERROR`: Trạng thái khóa chết (Nhà tù). Bất cứ bài test nào (RC, Gyro, Accel, IMU) thất bại, hệ thống rơi vào state này, ngắt mọi PWM ra motor để chống cháy nổ và nháy đèn báo lỗi.

*   **Phân hệ 2: Chế độ Hiệu chuẩn Chuyên sâu (Calibration Modes - Từ dòng 447 đến 610):**
    - `runEscCalibration()`: Là một FSM 6 bước riêng (`IDLE → WAIT_HIGH → WAIT_LOW → SAVING → DONE → ERROR`). Người dùng gạt CH5 lên cao (>1750us) để phát xung 2000us (Max Throttle), sau đó gạt CH5 xuống thấp (≤1750us) để phát xung 1000us (Min Throttle). Hệ thống đợi 4 giây ở mức Min để ESC kịp nhận xong và kêu bíp, sau đó lưu cờ `esc_calibrated = 1` vào EEPROM.
    - `runAccelCalibration()`: Đợi 1 giây để người dùng ổn định tay, sau đó gọi xuống driver IMU để lấy **4000 mẫu gia tốc ở tần số 1000Hz (trong 4 giây)**. Việc lấy mẫu khổng lồ này giúp bù trừ nhiễu vi mô của mặt bàn. Tính xong offset sẽ lưu kép vào EEPROM + Flash Backup.

*   **Phân hệ 3: Vòng lặp Điều khiển Chính thời gian thực (Main Loop 250Hz - Dòng 656 trở đi):**
    Đây là lõi động học của máy bay, chạy khép kín chuẩn xác mỗi 4000 micro-giây (250Hz):
    1. Đọc sóng vô tuyến CRSF và Burst Read toàn bộ 6 trục cảm biến MPU6050.
    2. Đẩy dữ liệu thô vào `imuEstimatorUpdate()` để chạy Bộ lọc bù (Complementary Filter) tính ra góc nghiêng Roll/Pitch.
    3. Cập nhật Watchdog và Safety State Machine (`safetyUpdate()`).
    4. Thuật toán tạo "Vùng chết" (Deadband 8us) cho cần Joystick để chống nhiễu tay rung. Trích xuất khoảng lệch Stick chia cho hằng số (15.0 cho Roll/Pitch, 3.0 cho Yaw) để quy đổi ra Góc/Tốc độ góc mục tiêu.
    5. **Giám sát Pin và giới hạn Throttle tự động:** Đọc `batteryGetState()`. Nếu Pin yếu (`BATTERY_LOW`) tự động kẹp Throttle không vượt quá `THROTTLE_LIMIT_LOW_BAT`. Nếu Pin nguy kịch (`BATTERY_CRITICAL`) kẹp xuống `THROTTLE_LIMIT_CRIT_BAT` để ép máy bay hạ cánh khẩn cấp.
    6. **Reset I-term khi chạm đất:** Khi Throttle < 1050us (máy bay đang nằm đất), tự động gọi `pidReset()` để xóa sạch khâu tích phân `i_mem`, ngăn hiện tượng Windup tích lũy trước khi cất cánh.
    7. **Kiểm tra chéo an toàn cuối cùng:** Đọc trạng thái từ `safetyGetState()`.
        + Nếu `STATE_ARMED` và ESC đã calib: Gọi `pidCompute()` → `motorMixerCompute()` → `motorWriteAllUs()` nạp 4 con số vào Hardware Timer.
        + Nếu `STATE_ARMED` nhưng ESC chưa calib: Gọi `motorStopAll()` - từ chối cấp công suất.
        + Nếu `STATE_DISARMED` hoặc rớt sóng (Failsafe): Ép cứng `motorStopAll()` và `pidReset()`.
    8. Ghi dữ liệu vào Blackbox (chỉ khi `STARTUP_READY`).
    9. In bảng Telemetry Real-time nếu chế độ debug đang bật.
    10. **Giám sát Loop Budget:** Đếm số lần vòng lặp vượt ngân sách thời gian (`budget_overrun_counter`). Nếu vượt quá 50 lần liên tiếp (xảy ra khi I2C chạy chậm 100kHz), hệ thống tự động hạ tần số vòng lặp xuống chế độ dự phòng (`FALLBACK_LOOP_PERIOD_US`) để tránh tràn buffer.
    11. **Tác vụ nền - Telemetry Pin:** Cứ mỗi 200ms, gửi điện áp pin và phần trăm pin về tay điều khiển qua giao thức CRSF (`crsfSendTelemetryBattery`), giúp phi công theo dõi pin ngay trên màn hình Goggles/Radio.



---

## PHẦN 2: STM32 XUẤT XUNG PWM KIỂU GÌ? (Hardware Timer và Xung nhịp 72MHz)

Đây là điểm giao thoa giữa Kỹ thuật Lập trình và Kỹ thuật Điện tử. Nếu hiểu sai chỗ này, toàn bộ lý thuyết PID sẽ sụp đổ ngay trên phần cứng thực tế.

**1. Vấn đề cốt lõi: Tại sao CPU không thể tự băm xung?**

Xung PWM điều khiển ESC đòi hỏi độ chính xác **tuyệt đối** đến từng micro-giây. Cụ thể, một xung 1550µs phải giữ mức HIGH đúng 1.550.000 nano-giây, không được sai lệch, không được bị ngắt quãng.

Nếu giao nhiệm vụ này cho CPU, CPU phải bận rộn đơn giản là ngồi đếm thời gian theo kiểu:
```
bật chân lên HIGH
chờ đúng 1550µs ... (CPU chết đứng tại đây)
kéo chân xuống LOW
```
Trong thời gian "chết đứng" đó, CPU không thể đọc MPU6050, không thể tính PID, không thể nhận sóng CRSF. Vòng lặp 250Hz bị gián đoạn. Máy bay rớt ngay lập tức. **Đây là lý do Software PWM không bao giờ được dùng trong hệ thống nhúng thời gian thực.**

**2. Giải pháp kiến trúc: Uỷ quyền 100% cho Hardware Timer**

STM32F103 có các khối ngoại vi chuyên dụng gọi là **Hardware Timer** (Timer 3 và Timer 4), hoạt động hoàn toàn độc lập với nhân CPU bằng phần cứng vật lý. Nguyên lý như sau:

- Thạch anh 8MHz được PLL nhân lên **72MHz** (72 triệu xung/giây). Đây là nguồn xung clock cấp cho toàn bộ hệ thống.
- Bộ Timer nhận xung 72MHz, đếm lên liên tục theo từng tick. Với Prescaler được cấu hình phù hợp, mỗi tick tương đương đúng **1 micro-giây** trong thế giới thực.
- Module `motor_pwm.cpp` chỉ cần ghi một con số duy nhất (ví dụ: `1550`) vào thanh ghi **Capture/Compare Register (CCR)** của Timer. Thao tác này tốn chưa đến 10 nano-giây.
- Từ thời điểm đó, phần cứng Timer **tự hoạt động hoàn toàn**: khi bộ đếm đạt đến giá trị `1550`, một công tắc bán dẫn bên trong chip tự động kéo điện áp chân GPIO từ 3.3V xuống 0V. CPU không tham gia bất cứ bước nào trong quá trình này.

**3. Kết quả thực tế trên mạch:**

| Thông số | Giá trị |
|---|---|
| Độ phân giải xung | 1 µs (chính xác tuyệt đối) |
| Tần số PWM | 200 Hz (chu kỳ 5000 µs) |
| Dải điều khiển | 1000 µs (Motor dừng) → 2000 µs (Full gas) |
| % thời gian CPU bị chiếm | **0%** |

Nhờ kiến trúc này, CPU được giải phóng 100% để chạy vòng lặp 250Hz: đọc IMU, tính PID, xử lý CRSF — hoàn toàn song song với 4 xung PWM đang được phần cứng bơm liên tục ra 4 chân ESC.

---

## PHẦN 3: KIẾN TRÚC TELEMETRY HAI CHIỀU QUA CRSF

Hệ thống giao tiếp với bộ thu ELRS qua **Hardware UART2** trên 2 chân PA2 (TX) và PA3 (RX) ở tốc độ **420.000 baud** — nhanh hơn UART thông thường gần 4 lần. Kiến trúc này được thiết kế hai chiều với vai trò rõ ràng cho mỗi chiều:

**1. Chiều vào (PA3 - RX): Nhận lệnh điều khiển từ Phi công**
- ELRS Receiver liên tục bắn gói tin `RC_CHANNELS_PACKED` vào chân PA3 ở tần số 150Hz.
- Mỗi gói tin mang 16 kênh RC được nén nhị phân 11-bit/kênh, Driver `rc_crsf.cpp` giải mã và trích xuất thành giá trị microsecond (988µs – 2012µs).
- Đồng thời gói `LINK_STATISTICS` mang thông tin chất lượng sóng (LQ%) và cường độ tín hiệu (RSSI dBm) về để hệ thống an toàn (`safety.cpp`) ra quyết định Failsafe.

**2. Chiều ra (PA2 - TX): Gửi Telemetry ngược về tay điều khiển**
- Mỗi 200ms, hệ thống đóng gói điện áp pin và phần trăm dung lượng còn lại vào gói `CRSF Battery Telemetry (0x08)` và bắn ngược về bộ thu ELRS. Từ đó ELRS phát về tay điều khiển qua đường RF, phi công theo dõi pin thời gian thực trên màn hình Goggles mà không cần bất cứ phần cứng bổ sung nào.
- Mỗi 1000ms, hệ thống trích thời gian từ module **RTC nội** (thạch anh 32.768kHz) đóng gói vào gói `Flight Mode Telemetry (0x21)` và gửi về. Phi công thấy đồng hồ thời gian thực nhảy trên màn hình tay khiển.

**3. Tại sao chân PA2 không còn dùng để đọc Log ASCII?**
- Ở giai đoạn phát triển ban đầu, PA2 được dùng như một cổng xuất Log văn bản (ASCII) một chiều để quan sát trên máy tính. Tuy nhiên, khi luồng ASCII và luồng CRSF nhị phân cùng chia sẻ một đường truyền, chúng nhiễu nhau và làm **vỡ gói CRSF**, khiến bộ thu ELRS báo mất sóng giả — nguy hiểm trực tiếp cho sự an toàn bay.
- Trong phiên bản Release, toàn bộ luồng Log ASCII đã được **xóa bỏ hoàn toàn** khỏi codebase. Chân PA2 trở thành **đường CRSF TX thuần nhị phân**, không bị xen tạp chất.

**TỔNG KẾT BẢO VỆ ĐỒ ÁN:**
> *"Hệ thống của em chia công việc theo đúng nguyên tắc phân tầng: CPU lo tính toán, Hardware Timer lo băm xung, CRSF lo giao tiếp vô tuyến. Mỗi tầng hoạt động hoàn toàn độc lập. Đặc biệt, bằng cách tận dụng kiến trúc Telemetry hai chiều của giao thức CRSF, em không chỉ nhận lệnh từ phi công mà còn gửi ngược lại dữ liệu pin và thời gian thực từ RTC nội về màn hình tay khiển — không tốn thêm bất cứ dây dẫn hay phần cứng bổ sung nào."*
