# Hướng dẫn Kiểm thử Hệ thống Chẩn đoán & Cân bằng (YMFC-32 Diagnostics & Testing Guide)

Tài liệu này hướng dẫn cách vận hành hệ thống logging chẩn đoán thời gian thực qua **Software UART** (19200 baud) bằng cách chuyển chế độ trực tiếp trên tay điều khiển (kênh **CH7 / AUX 3**) thay thế cho các lệnh UART truyền thống.

---

## ⚠️ Cảnh báo An toàn Quan trọng
> [!CAUTION]
> **BẮT BUỘC THÁO TOÀN BỘ CÁNH QUẠT TRƯỚC KHI TEST**
> Các bài test có thể kích hoạt động cơ quay (đặc biệt là bài test rung động cơ). Để đảm bảo an toàn tuyệt đối cho người và thiết bị, hãy tháo hết cánh quạt của cả 4 động cơ trước khi cấp nguồn pin hoặc chạy test.

---

## 1. Chuẩn bị Kết nối & Phần cứng
1. **Kết nối USB-to-UART:**
   - Chân **TX** của Software UART được cấu hình trên chân **PA2** của STM32 Bluepill.
   - Kết nối chân **PA2** (TX) của STM32 với chân **RX** của mạch nạp/mạch USB-to-UART.
   - Kết nối chân **GND** chung giữa STM32 và mạch USB-to-UART.
2. **Cấu hình Serial Monitor:**
   - Mở phần mềm Serial Monitor (như Hercules, Arduino Serial Monitor, Miniterm, v.v.).
   - Chọn đúng cổng COM của mạch USB-to-UART.
   - Thiết lập tốc độ Baud: **19200**, Data bits: **8**, Parity: **None**, Stop bits: **1**.
3. **Kết nối tay phát RC:**
   - Bật tay phát điều khiển, đảm bảo đã kết nối ổn định với bộ thu ELRS trên drone.

---

## 2. Cách Điều khiển và Chuyển đổi Mode
Hệ thống chẩn đoán gồm **10 chế độ** (từ Mode 0 đến Mode 9) được kiểm soát hoàn toàn thông qua kênh **CH7 (AUX 3)** trên tay điều khiển:
- Mỗi khi bạn gạt công tắc **CH7** (thay đổi giá trị vượt qua các ngưỡng `1300us` và `1700us`), hệ thống sẽ tự động chuyển sang chế độ tiếp theo (`active_mode = (active_mode + 1) % 10`).
- Khi chuyển đổi chế độ thành công, màn hình Serial Monitor sẽ hiển thị thông báo chào mừng chế độ mới:
  `[MODE CHANGE] Entering Mode X: <Tên chế độ>`

---

## 3. Danh sách các Chế độ Test chi tiết

### Mode 0: IDLE / Main Menu
- **Mô tả:** Chế độ chờ hiển thị Menu hướng dẫn và danh sách các mode test.
- **Dữ liệu xuất ra:** Bảng danh sách các chế độ chẩn đoán.

### Mode 1: In 6 Kênh Tay Điều Khiển (YMFC-32 'a')
- **Mô tả:** Kiểm tra độ chính xác và chiều hoạt động của các kênh RC từ tay phát.
- **Dữ liệu xuất ra:** Giá trị dạng xung (us) kèm ký hiệu hướng:
  - `Roll`: Trái (`<<<`), Phải (`>>>`), Giữa (`-+-`).
  - `Pitch`: Chúc mũi (`^^^`), Ngửa mũi (`vvv`), Giữa (`-+-`).
  - `Throttle`: Thấp (`vvv`), Cao (`^^^`), Giữa (`-+-`).
  - `Yaw`: Trái (`<<<`), Phải (`>>>`), Giữa (`-+-`).
  - Trạng thái `Start` giả lập của YMFC-32 (`Start: 0` - Khóa, `1` - Đang khởi động, `2` - Sẵn sàng).

### Mode 2: Quét thiết bị trên I2C Bus (YMFC-32 'b')
- **Mô tả:** Tự động quét toàn bộ địa chỉ I2C trên bus phần mềm.
- **Dữ liệu xuất ra:**
  - `I2C device found at address 0x68` (Cảm biến MPU6050).
  - `I2C device found at address 0x50` (Bộ nhớ EEPROM 24LC256).

### Mode 3: In giá trị thô Gyroscope (YMFC-32 'c')
- **Mô tả:** Hiển thị trực tiếp giá trị thô từ cảm biến Gyroscope.
- **Dữ liệu xuất ra:** `Gyro_x = X  Gyro_y = Y  Gyro_z = Z` (giá trị thô chưa chia tỉ lệ, phục vụ kiểm tra nhiễu tĩnh).

### Mode 4: In giá trị thô Accelerometer (YMFC-32 'd')
- **Mô tả:** Hiển thị trực tiếp giá trị thô từ cảm biến Accelerometer.
- **Dữ liệu xuất ra:** `ACC_x = X  ACC_y = Y  ACC_z = Z`.

### Mode 5: In góc nghiêng IMU (YMFC-32 'e')
- **Mô tả:** Hiển thị các góc nghiêng ước lượng sau bộ lọc bù (Complementary Filter) chạy ở tần số 250Hz.
- **Dữ liệu xuất ra:** `Pitch: X  Roll: Y  Yaw: Z  Temp: T` (đơn vị: Độ và độ C).

### Mode 6: Test LED Cảnh Báo (YMFC-32 'f')
- **Mô tả:** Chu kỳ test giả lập cho LED đỏ và xanh.
- **Dữ liệu xuất ra:** Thông báo trạng thái LED sáng lần lượt trong 3 giây.

### Mode 7: In điện áp nguồn Pin (YMFC-32 'g')
- **Mô tả:** Đo và hiển thị điện áp pin LiPo đã qua bộ lọc ADC.
- **Dữ liệu xuất ra:** `Voltage = XX.XV`.

### Mode 8: In giá trị Offsets cảm biến (YMFC-32 'h')
- **Mô tả:** Xem lại các giá trị bù sai số cảm biến đã được lưu trong quá trình khởi động hoặc calib.
- **Dữ liệu xuất ra:** Các giá trị offset của 3 trục Accel và 3 trục Gyro.

---

## 4. Quy trình chạy Test Rung Động Cơ (Mode 9)
Đây là tính năng cực kỳ quan trọng dùng để đo và phát hiện rung động do cánh quạt hoặc trục động cơ bị lệch tâm.

### Các bước thực hiện:
1. **Chuyển sang Mode 9** bằng cách gạt cần **CH7** cho đến khi màn hình hiển thị:
   `[MODE CHANGE] Entering Mode 9: Motor Vibration Test...`
2. **Khởi tạo cần Ga (Throttle Safety Init):**
   - Nếu cần ga đang ở vị trí cao, màn hình sẽ báo: `Throttle is not in the lowest position.`
   - Kéo cần ga trên tay phát về mức thấp nhất (`< 1050us`).
   - Màn hình hiển thị: `Waiting for 10 seconds: ... OK!`
3. **Chọn động cơ cần kiểm tra bằng công tắc CH6:**
   - Hệ thống cho phép quay thử từng động cơ riêng lẻ hoặc toàn bộ bằng công tắc **CH6 (AUX 2)**:
     * **CH6 < 1200us:** Động cơ 1 (Trước Phải).
     * **1200us $\le$ CH6 < 1400us:** Động cơ 2 (Sau Phải).
     * **1400us $\le$ CH6 < 1600us:** Động cơ 3 (Sau Trái).
     * **1600us $\le$ CH6 < 1800us:** Động cơ 4 (Trước Trái).
     * **CH6 $\ge$ 1800us:** Chạy đồng thời cả 4 động cơ.
4. **Tăng ga và đọc mức rung:**
   - Đẩy cần ga lên từ từ để động cơ quay.
   - **Giới hạn an toàn:** Để bảo vệ drone trong phòng thí nghiệm, phần ga xuất ra động cơ được khống chế cứng tối đa ở mức **1500us** kể cả khi đẩy hết cần ga lên 2000us.
   - Đọc kết quả hiển thị trên Serial Monitor:
     `Vibration level: X`
   - **Đánh giá kết quả:**
     * `X < 30`: Rung động rất thấp, động cơ và cánh quạt cân bằng tốt (Đạt).
     * `30 <= X < 80`: Rung động mức trung bình, có thể bay được nhưng nên kiểm tra lại độ thẳng trục hoặc cân lại cánh.
     * `X >= 80`: Rung động rất cao. Bắt buộc dừng kiểm tra cơ khí, tránh nguy cơ gây nhiễu cảm biến làm drone bị lật khi bay.
