# Hướng dẫn Kiểm thử Hệ thống Chẩn đoán & Cân bằng (YMFC-32 Diagnostics & Testing Guide)

Tài liệu này hướng dẫn cách vận hành hệ thống logging chẩn đoán thời gian thực qua **Software UART** (19200 baud) bằng cách chuyển chế độ trực tiếp trên tay điều khiển (kênh **CH7 / AUX 3**).

---

## ⚠️ Cảnh báo An toàn Quan trọng
> [!CAUTION]
> **AN TOÀN TUYỆT ĐỐI KHI TEST ĐỘNG CƠ CÓ CÁNH QUẠT**
> - **Khi test KHÔNG lắp cánh quạt**: Bạn chỉ cần đặt drone nằm yên trên bàn phẳng. Lực nâng lúc này bằng 0 nên drone sẽ đứng yên an toàn.
> - **Khi test CÓ lắp cánh quạt**: Bắt buộc phải **dùng vật nặng đè chặt** (như bao cát, chai nước 5 lít, hoặc gạch) đè chắc chắn lên chân/khung của drone xuống sàn nhà hoặc bàn lớn. Tránh dùng tay giữ trực tiếp gần cánh quạt đang quay để ngăn ngừa nguy cơ chấn thương nghiêm trọng.

---

## 1. Chuẩn bị Kết nối & Phần cứng
1. **Kết nối USB-to-UART:**
   - Chân **TX** của Software UART được cấu hình trên chân **PA2** của STM32 Bluepill.
   - Kết nối chân **PA2** (TX) của STM32 với chân **RX** của mạch nạp/mạch USB-to-UART.
   - Kết nối chân **GND** chung giữa STM32 và mạch nạp.
2. **Cấu hình Serial Monitor:**
   - Thiết lập tốc độ Baud: **19200**, Data bits: **8**, Parity: **None**, Stop bits: **1**.
3. **Kết nối tay phát RC:**
   - Bật tay phát điều khiển, đảm bảo đã kết nối ổn định với bộ thu ELRS trên drone.

---

## 2. Cách Điều khiển và Chuyển đổi Mode
Hệ thống chẩn đoán gồm **10 chế độ** (từ Mode 0 đến Mode 9) được kiểm soát hoàn toàn thông qua kênh **CH7 (AUX 3)** trên tay điều khiển:
- Mỗi khi bạn gạt thay đổi trạng thái công tắc **CH7** (Thấp, Trung bình, Cao), hệ thống sẽ tự động chuyển sang chế độ tiếp theo (`active_mode = (active_mode + 1) % 10`).
- Khi chuyển đổi chế độ thành công, màn hình Serial Monitor sẽ hiển thị:
  `[MODE CHANGE] Entering Mode X: <Tên chế độ>`

---

## 3. Danh sách các Chế độ Test chi tiết

### Mode 0: IDLE / Main Menu
- **Mô tả:** Chế độ chờ hiển thị Menu hướng dẫn và danh sách các mode test.

### Mode 1: In 6 Kênh Tay Điều Khiển (YMFC-32 'a')
- **Mô tả:** Kiểm tra độ chính xác và chiều hoạt động của các kênh RC từ tay phát.
- **Dữ liệu xuất ra:** Giá trị dạng xung (us) kèm ký hiệu hướng:
  - `Roll`: Trái (`<<<`), Phải (`>>>`), Giữa (`-+-`).
  - `Pitch`: Chúc mũi (`^^^`), Ngửa mũi (`vvv`), Giữa (`-+-`).
  - `Throttle`: Thấp (`vvv`), Cao (`^^^`), Giữa (`-+-`).
  - `Yaw`: Trái (`<<<`), Phải (`>>>`), Giữa (`-+-`).
  - Trạng thái `Start` giả lập của YMFC-32 (`Start: 0` - Khóa, `1` - Đang khởi động, `2` - Sẵn sàng).

### Mode 2: Quét thiết bị trên I2C Bus (YMFC-32 'b')
- **Mô tả:** Tự động quét toàn bộ địa chỉ I2C trên bus phần mềm để xác định linh kiện kết nối.
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
- **Dữ liệu xuất ra:** `Pitch: X  Roll: Y  Yaw: Z  Temp: T` (đơn vị: Độ và độ C). *Đã được fix lỗi hiển thị trống số thực.*

### Mode 6: Test LED Cảnh Báo (YMFC-32 'f')
- **Mô tả:** Chu kỳ test giả lập cho LED đỏ và xanh.

### Mode 7: In điện áp nguồn Pin (YMFC-32 'g')
- **Mô tả:** Đo và hiển thị điện áp pin LiPo đã qua bộ lọc ADC.
- **Dữ liệu xuất ra:** `Voltage = XX.XV`. *Đã được fix lỗi hiển thị trống số thực.*

### Mode 8: In giá trị Offsets cảm biến (YMFC-32 'h')
- **Mô tả:** Xem lại các giá trị bù sai số cảm biến đã được lưu trong quá trình khởi động hoặc hiệu chuẩn.
- **Dữ liệu xuất ra:** Các giá trị offset của 3 trục Accel và 3 trục Gyro.

---

## 4. Quy trình chạy Test Rung Động Cơ (Mode 9)
Dùng để đo và phát hiện rung động do cánh quạt hoặc trục động cơ bị lệch tâm.

### Các bước thực hiện:
1. **Chuyển sang Mode 9** bằng cách gạt cần **CH7** cho đến khi màn hình hiển thị:
   `[MODE CHANGE] Entering Mode 9: Motor Vibration Test...`
2. **Khởi tạo cần Ga (Throttle Safety Init):**
   - Kéo cần ga trên tay phát về mức thấp nhất (`< 1050us`).
   - Màn hình hiển thị: `Waiting for 10 seconds: ... OK!`
3. **Chọn động cơ cần kiểm tra bằng cách gạt CH6 (Xoay vòng):**
   - Mỗi lần bạn gạt thay đổi trạng thái công tắc **CH6 (AUX 2)**, hệ thống sẽ chuyển đổi xoay vòng động cơ theo thứ tự:
     `M1 (Truoc Phai) -> M2 (Sau Phai) -> M3 (Sau Trai) -> M4 (Truoc Trai) -> ALL`
   - Log Serial Monitor hiển thị phản hồi ngay lập tức để bạn biết động cơ nào được chọn:
     `[MOTOR SELECT] Selected: M1 (Truoc Phai)`
4. **Tăng ga và đọc mức rung:**
   - Đẩy cần ga lên để bắt đầu quay motor.
   - **Tắt spam log tĩnh**: Khi tay ga ở mức tối thiểu (`< 1050us`), log độ rung sẽ **không in ra** để tránh làm bẩn màn hình. Log chỉ in khi ga bắt đầu mở ($\ge 1050us$).
   - **Giới hạn ga an toàn**: Ga tối đa cấp ra motor được khống chế cứng ở mức **1500us** để đảm bảo an toàn, tránh drone tự bay lên trên bàn test.
   - **Đọc kết quả hiển thị trên Serial Monitor:**
     `[M1 (Truoc Phai)] Vibration level: X`
   - **Đánh giá kết quả (khi chạy ga ổn định không cánh):**
     * `X < 20`: Động cơ hoạt động hoàn hảo, chạy êm và trục thẳng (Đạt).
     * `20 <= X < 50`: Rung động mức nhẹ, bình thường.
     * `X >= 50`: Rung động lớn, nên kiểm tra xem trục motor có bị cong hay bạc đạn bị mẻ không.
     * *(Nếu lắp cánh quạt, mức rung động thường sẽ cao hơn, nhưng nếu vượt quá 80 thì cánh quạt bị mất cân bằng động nghiêm trọng).*
