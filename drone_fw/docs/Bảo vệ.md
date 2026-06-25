# 🚁 KỊCH BẢN BẢO VỆ ĐỒ ÁN DRONE - BẢN HOÀN CHỈNH
> File này đã được quy hoạch lại toàn bộ từ A-Z, sắp xếp theo đúng dòng chảy của dữ liệu và lời văn báo cáo. Hãy học theo đúng thứ tự từ trên xuống dưới.

---

## PHẦN 1: NỀN TẢNG PHẦN CỨNG (Sơ đồ chân & Cấu hình)

### 1. File `board_pinmap.h` (Quản lý chân cắm)
"Dạ thưa các thầy, mục đích của file này là gom tất cả cấu hình chân của mạch vào một chỗ. Lỡ sau này em có vẽ lại bo mạch, muốn đổi chân linh kiện thì chỉ cần vào đúng file này sửa 1 lần là xong. Cụ thể cấu hình phần cứng của em:"
- **4 chân Động cơ:** Dùng đúng các chân có chức năng Timer phần cứng để xuất xung PWM mượt mà.
- **Cổng kết nối:** Do không gian bo mạch hạn chế, em dùng chung **UART2 (PA2, PA3)** cho 2 việc: PA3(RX) nhận lệnh từ ELRS, PA2(TX) xuất Log Debug ra máy tính. Lúc bay thật, em sẽ tắt Debug để PA2 quay lại làm chân báo pin cho tay cầm.
- **Cảm biến:** Dùng **I2C phần mềm** (PA9, PA10) thay vì I2C cứng. Lý do là dòng chip STM32F103 có lỗi phần cứng ở I2C, dễ bị treo đường truyền giữa chừng làm rớt drone. I2C mềm tự viết giúp hệ thống miễn nhiễm với lỗi này.
- **Pin:** Chân PA1 dùng bộ Analog (ADC) để đo điện áp pin.

### 2. File `config.h` (Trung tâm thông số)
- **Tần số vòng lặp:** `CONTROL_LOOP_FREQ_HZ = 250`. Tức là 1 giây MCU tính toán 250 lần, suy ra mỗi vòng lặp có quỹ thời gian là **4000 micro-giây (4ms)**.
- **Trick Fallback I2C:** Bình thường I2C chạy tốc độ 400kHz. Nếu bị nhiễu, nó tự hạ xuống 100kHz. Lúc này vòng lặp dự phòng `FALLBACK_LOOP_FREQ` vẫn ép MCU chạy đúng ở 250Hz để máy bay không bị rớt.
- **Bảo vệ Pin:** Giới hạn tay ga khi pin yếu (Low = khóa ga 1600, Critical = khóa ga 1200 để ép hạ cánh).
- **Giới hạn góc:** Chỉ cho phép nghiêng tối đa 30 độ để an toàn cho người mới tập. Tốc độ xoay Yaw tối đa 150 độ/giây.

---

## PHẦN 2: DÒNG CHẢY DỮ LIỆU ĐIỀU KHIỂN

### Bước 1: Thu sóng vô tuyến (`rc_crsf.cpp`)
- Thu tín hiệu từ bộ thu ELRS qua UART2 với tốc độ rất cao: **420.000 baud**.
- **Giải mã:** Tay cầm nén 16 kênh điều khiển vào vỏn vẹn 22 Bytes (vì mỗi kênh là 1 số lẻ 11-bit). Em dùng phép dịch bit chéo để bung nén 22 Bytes này ra lại thành 16 con số thô (172 - 1811).
- **Quy đổi:** Đổi dải thô đó ra số đo Micro-giây chuẩn từ **988µs - 2012µs** để hệ thống sử dụng. 
- **Chống nhiễu:** Dùng hàm Checksum `crsf_crc8()` kiểm tra chập bit. Nếu sai mã, bỏ luôn gói tin đó. Nếu quá 200ms không nhận được gói tin nào đúng → Kích hoạt Failsafe ngắt động cơ.

### Bước 2: Đọc Cảm biến & Hiệu chuẩn (`mpu6050.cpp`)
- **Burst Read 14 Bytes:** Mỗi vòng lặp, MCU đọc một lèo 14 bytes (6 byte Accel + 2 byte Nhiệt + 6 byte Gyro) trong một lần giao tiếp I2C. Việc này đảm bảo 6 trục được chụp tại đúng một phần nghìn giây, triệt tiêu độ trễ lệch pha (phase lag) so với việc đọc rời rạc từng thanh ghi.
- **Hiệu chuẩn tĩnh (Calibration):** Cảm biến giá rẻ luôn có nhiễu và sai số bẩm sinh. Em dùng vòng lặp để lấy trung bình hàng ngàn mẫu nhằm chắt lọc ra độ lệch sạch nhất:
  - **Với Accel:** Cần mặt phẳng tuyệt đối nên phải lấy tới **4000 mẫu (tốn 4s)**. Chỉ cần làm 1 lần duy nhất trong đời máy, lưu vĩnh viễn vào chip nhớ EEPROM. Lần sau bật máy không cần chờ calib lại.
  - **Với Gyro:** Đặc tính đo tốc độ và dễ bị trôi theo nhiệt độ. Nên hệ thống KHÔNG lưu vào EEPROM. Thay vào đó, lập trình tự động đo lại **1000 mẫu (tốn 1s)** mỗi khi cắm pin để lấy mốc cân bằng mới nhất cho môi trường hôm đó.

### Bước 3: Thuật toán tính góc thực tế (`imu_estimator.cpp`)
Dữ liệu thô từ bước 2 được đưa vào đây để tính góc nghiêng bằng 3 bước toán không gian:
1. **Tích phân Gyro:** Cộng dồn `Tốc_độ_góc × Thời_gian_dt` (`dt = 0.004s`) ra góc hiện tại.
2. **Bù chéo Yaw:** Khi chúi mũi (Pitch) mà bẻ lái (Yaw), góc tọa độ bị xoay làm thông số Pitch chảy sang Roll. Dùng lượng giác (`sin_yaw`) hoán đổi chéo lại để Drone không lảo đảo khi rẽ.
3. **Bộ Lọc Bù (Complementary Filter):** Kết hợp Gyro (siêu nhanh nhưng trôi dần) và Accel (không trôi nhưng nhiễu do cánh quạt). Em chọn tỉ lệ tin tưởng **99.96% Gyro + 0.04% Accel** để ra góc chuẩn xác nhất.

### Bước 4: Bộ điều khiển cốt lõi (`pid_controller.cpp`)
Em thiết kế kiến trúc **PID 2 vòng lồng nhau (Cascaded PID)**:
- **Lọc tín hiệu:** Trước khi đưa Gyro vào tính toán, em cho qua bộ lọc thông thấp (Low-pass 70/30) lấy 70% giá trị cũ + 30% giá trị mới. Chống D-term bị giật cục do nhiễu cánh quạt.
- **Vòng ngoài (Angle Loop):** Chỉ dùng **P**, biến sai số góc thành Tốc độ quay mục tiêu.
- **Vòng trong (Rate Loop):** Dùng đủ **P-I-D**, biến sai số tốc độ thành Lực bù trừ cuối cùng.
- **Khóa Anti-Windup:** Khóa khâu Tích phân (I) ở mức tối đa 400. Nếu bay ngược gió bão, I không bị cộng dồn vô hạn gây tràn biến làm lật máy bay.
- **Điểm sáng - Vứt bỏ dt:** Bình thường PID phải nhân thêm `dt`. Nhưng vì hệ thống bị ép chạy đúng ở 250Hz (`dt = 0.004s`), em đã gộp thẳng hằng số này vào cấu hình `Ki`, `Kd` lúc Tune. Việc lược bỏ `dt` khỏi công thức toán giúp chip STM32 không phải tính toán số thực, tiết kiệm tối đa chu kỳ máy.

### Bước 5: Trộn tín hiệu & Xuất xung (`motor_mixer.cpp` & `motor_pwm.cpp`)
- **motor_mixer:** Nhận lực PID, cộng với Tay ga (Throttle) theo thiết kế chéo X-frame. Tính ra được 4 con số bù trừ cho 4 motor. Tuy nhiên, theo nguyên tắc Giảm phụ thuộc (Loose Coupling), Mixer không được phép kích hoạt motor trực tiếp.
- **main.cpp (Tổng chỉ huy):** Đứng ở giữa, hứng 4 kết quả từ Mixer, soát xét các lớp an toàn (Pin yếu không? Đã bật công tắc Arm chưa? ESC đã calib chưa?). Nếu mọi thứ an toàn tuyệt đối, `main.cpp` mới giao 4 con số đó cho tầng cứng.
- **motor_pwm:** Nhận lệnh, can thiệp vào thanh ghi Timer phần cứng để xuất xung PWM chuẩn xác ra 4 chân PB4, PB5, PB6, PB7 điều khiển tốc độ từng cục ESC.

---

## PHẦN 3: ĐIỀU PHỐI VÒNG LẶP (`main.cpp`)
> Đây là nơi lắp ghép toàn bộ các mảnh ghép bên trên để Drone hoạt động được thời gian thực (Real-time).

**Cơ chế Quỹ thời gian vòng lặp (Control Loop Budget):**
- Trong hàm `loop()`, em hoàn toàn KHÔNG sử dụng hàm `delay()`.
- Chu kỳ 250Hz nghĩa là hệ thống có **Quỹ thời gian là 4000 micro-giây** cho 1 vòng chạy.
- Thực tế việc tính toán cả 5 bước khổng lồ ở trên chỉ tốn có **1000us - 1500us** là xong.
- **Vậy ~3000us dư thừa CPU làm gì?** Nó tuyệt đối không ngủ. CPU sẽ liên tục chạy vòng vòng bên ngoài để nghe ngóng cổng Serial máy tính, chạy máy trạng thái FSM, và liên tục dòm đồng hồ `micros()` xem "Đã đủ 4000us chưa?". Vừa tròn số, nó lập tức kích hoạt vòng tính toán mới. Cơ chế này tạo ra nhịp đập 250Hz đều như vắt chanh mà vẫn không bỏ lỡ bất cứ tín hiệu tay cầm nào.

---

## PHẦN 4: LƯU TRỮ VÀ PIN
- **eeprom_24lc256.cpp:** Ghi thông số PID, Offset Accel ra bộ nhớ ngoài bằng I2C mềm. Để người dùng tinh chỉnh trên máy tính một lần là nhớ mãi mãi. (Có thêm `flash_backup.cpp` ghi dự phòng vào nhân chip STM32 nhỡ EEPROM hỏng).
- **battery_adc.cpp:** Đọc điện áp qua mạch cầu phân áp (R1=27k, R2=10k). Mạch này hạ 12.6V của pin Lipo xuống còn dưới 3.3V cho chân STM32 đọc được an toàn. Cứ 100ms đọc 1 lần lấy trung bình 20 mẫu để chống sụt áp ảo khi rồ ga.

---
> **TÓM LẠI - KHI THẦY CÔ HỎI VÀO BẤT CỨ ĐÂU:**
> *Hãy nhớ tư duy 3 nhịp: Thằng này LẤY DỮ LIỆU TỪ ĐÂU → Nó TÍNH TOÁN bằng CÔNG THỨC GÌ → Nó NÉM KẾT QUẢ ĐI ĐÂU.*
