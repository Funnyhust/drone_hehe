# Kế hoạch Kiểm thử các Module Phần cứng (Drone Test Plan)

Tài liệu này hướng dẫn chi tiết các bước thực hiện kiểm thử độc lập cho từng phân hệ (động cơ, cảm biến IMU, bộ thu điều khiển ELRS) của Drone thông qua giao diện CLI Debug (qua phần mềm Serial Monitor kết nối với cổng Hardware UART hoặc Software UART chân **PB7** ở tốc độ **19200 baud**).

---

## ⚠️ Quy tắc an toàn chung trước khi kiểm thử
> [!CAUTION]
> **THÁO CÁNH QUẠT KHỎI TẤT CẢ ĐỘNG CƠ**
> Trước khi thực hiện bất kỳ bài kiểm tra nào liên quan đến động cơ hoặc cắm nguồn pin, bắt buộc phải tháo toàn bộ cánh quạt để tránh nguy cơ drone tự khởi động gây thương tích hoặc hỏng hóc thiết bị.

---

## 1. Kiểm thử Phân hệ Động cơ (Motor PWM Test)

### Mục tiêu
Kiểm tra xem 4 động cơ có quay đúng thứ tự và hướng quay chuẩn của cấu hình **Quad-X**, đồng thời đáp ứng chính xác dải xung điều khiển PWM từ **1000µs** (tốc độ tối thiểu) đến **2000µs** (tốc độ tối đa).

### Sơ đồ cấu hình động cơ (Quad-X Layout)
```
     [Trước Trái] (M4) ↻     ↺ (M1) [Trước Phải]
                           ▲
                           │ (Đầu Drone)
     [Sau Trái]   (M3) ↺     ↻ (M2) [Sau Phải]
```

### Các bước kiểm thử
1. **Chuẩn bị:** Đảm bảo drone đã tháo cánh quạt, kết nối mạch USB-to-UART với máy tính, mở Serial Monitor (tốc độ 19200 baud).
2. **Kích hoạt chế độ test:** Gửi ký tự `'t'` qua Serial Monitor.
3. **Thực hiện lệnh điều khiển:**
   * Gửi ký tự `'1'`: Toàn bộ động cơ xuất xung **1000µs** (động cơ sẽ bắt đầu quay nhẹ ở tốc độ không tải - Idle).
   * Gửi ký tự `'2'`: Toàn bộ động cơ tăng ga lên **1200µs** (kiểm tra xem tốc độ quay có tăng nhẹ đều ở cả 4 động cơ hay không).
   * Gửi ký tự `'3'`: Toàn bộ động cơ tăng ga lên **1500µs** (tốc độ trung bình).
   * Gửi ký tự `'4'`: Toàn bộ động cơ tăng ga lên **1800µs** (tốc độ cao - *thực hiện nhanh để tránh nóng motor*).
   * Gửi ký tự `'0'`: Dừng toàn bộ động cơ lập tức.
4. **Tiêu chí đánh giá Đạt (Pass):**
   * Khi gửi `'1'` - `'4'`, cả 4 động cơ quay đều, không bị kẹt hay giật cục.
   * Chiều quay của các motor phải đúng như sơ đồ: **M1 và M4 quay ngược chiều kim đồng hồ (CCW) hoặc theo cấu hình ngược, M2 và M3 quay cùng chiều kim đồng hồ (CW)**. 
   * Gửi `'0'` lập tức dừng quay hoàn toàn.

---

## 2. Kiểm thử Cảm biến MPU6050 (IMU Test)

### Mục tiêu
Đảm bảo cảm biến MPU6050 được vi điều khiển nhận dạng thành công qua bus Software I2C (địa chỉ `0x68`), hoàn thành quá trình hiệu chuẩn (Calibration) và cập nhật dữ liệu gia tốc (Accel) và vận tốc góc (Gyro) chính xác theo chuyển động vật lý.

### Các bước kiểm thử
1. **Kiểm tra I2C Bus:** 
   * Gửi ký tự `'s'` qua Serial Monitor để chạy I2C Scanner.
   * **Tiêu chí đạt:** Màn hình phải hiển thị dòng báo tìm thấy thiết bị:
     `I2C device found at address 0x68` (địa chỉ mặc định của MPU6050).
2. **Kiểm tra dữ liệu chuyển động:**
   * Gửi ký tự `'i'` qua Serial Monitor để in dữ liệu IMU thời gian thực.
   * **Đọc dữ liệu khi Drone đặt nằm phẳng tĩnh trên bàn:**
     * Giá trị Gia tốc (`ACC`): `ax` ≈ 0.0, `ay` ≈ 0.0, `az` ≈ 1.0 (hoặc ≈ 9.8 m/s² tùy theo đơn vị scale).
     * Giá trị Vận tốc góc (`GYRO`): `gx` ≈ 0.0, `gy` ≈ 0.0, `gz` ≈ 0.0.
   * **Di chuyển drone bằng tay để kiểm tra phản hồi:**
     * **Nghiêng cánh bên phải xuống (Roll dương):** Gia tốc `ay` phải thay đổi sang giá trị dương, vận tốc góc `gx` tăng giảm theo hướng xoay.
     * **Chúc mũi drone xuống dưới (Pitch âm):** Gia tốc `ax` thay đổi, vận tốc góc `gy` phản hồi tương ứng.
     * **Xoay drone quanh trục thẳng đứng (Yaw):** Vận tốc góc `gz` phản hồi mạnh khi xoay và trở về 0 khi dừng xoay.
3. **Thoát chế độ xem dữ liệu:** Gửi bất kỳ ký tự nào để dừng in dữ liệu IMU.

---

## 3. Kiểm thử Truyền thông ELRS CRSF (ExpressLRS Test)

### Mục tiêu
Kiểm tra kết nối và giải mã gói tin CRSF từ bộ thu ELRS RX kết nối với tay điều khiển (Transmitter). Đảm bảo nhận đúng giá trị của các kênh điều khiển (Roll, Pitch, Throttle, Yaw, Switch Arm, Flight Mode) và chất lượng sóng (LQ, RSSI).

### Các bước kiểm thử
1. **Kết nối tay điều khiển:** Bật tay điều khiển có module phát ELRS TX, đảm bảo đã liên kết (Bound) thành công với bộ thu ELRS RX trên drone (Đèn LED trên bộ thu ELRS RX sáng đứng, không nhấp nháy).
2. **Kích hoạt hiển thị dữ liệu RC:** Gửi ký tự `'r'` qua Serial Monitor.
3. **Kiểm tra các kênh cần điều khiển (Gimbal Sticks):**
   * **Kênh Ga (Throttle - CH3):**
     * Cần ga kéo hết cỡ xuống dưới: Giá trị hiển thị `CH3` ≈ **1000** (hoặc 988µs).
     * Cần ga đẩy hết cỡ lên trên: Giá trị hiển thị `CH3` ≈ **2000** (hoặc 2012µs).
     * Cần ga ở giữa: `CH3` ≈ **1500**.
   * **Kênh Lắc Ngang (Roll - CH1):**
     * Gạt hết cần sang trái: `CH1` ≈ **1000**.
     * Gạt hết cần sang phải: `CH1` ≈ **2000**.
     * Cần ở giữa: `CH1` ≈ **1500**.
   * **Kênh Chúc Mũi (Pitch - CH2):**
     * Đẩy hết cần lên trên (chúc mũi): `CH2` ≈ **1000** (hoặc 2000 tùy chế độ đảo kênh).
     * Kéo hết cần xuống dưới (ngửa mũi): `CH2` ≈ **2000**.
     * Cần ở giữa: `CH2` ≈ **1500**.
   * **Kênh Quay Đuôi (Yaw - CH4):**
     * Gạt cần sang trái: `CH4` ≈ **1000**.
     * Gạt cần sang phải: `CH4` ≈ **2000**.
     * Cần ở giữa: `CH4` ≈ **1500**.
4. **Kiểm tra các công tắc phụ (Auxiliary Switches):**
   * **Công tắc khóa động cơ (Arm - CH5 / AUX1):** Gạt công tắc ARM trên tay điều khiển để kiểm tra xem `CH5` có chuyển đổi giá trị dứt khoát giữa mức thấp (~1000) và mức cao (~2000) hay không.
5. **Kiểm tra chất lượng sóng (Link Statistics):**
   * **LQ (Link Quality):** Đạt khoảng `100%` (nếu ở khoảng cách gần).
   * **RSSI:** Dao động trong khoảng `-30 dBm` đến `-80 dBm` (giá trị càng âm ít càng tốt, ví dụ -40 tốt hơn -80).
6. **Thoát chế độ xem dữ liệu:** Gửi bất kỳ ký tự nào để dừng in dữ liệu RC.

---

## 4. Kiểm thử Giám sát Pin (Battery ADC Test)

### Mục tiêu
Đảm bảo mạch cầu phân áp đọc điện áp pin qua chân **PA1** hoạt động chính xác, bộ lọc trung bình động triệt tiêu được nhiễu động cơ và hệ thống phản hồi đúng khi pin yếu/nguy kịch.

### Các bước kiểm thử
1. **Kiểm tra điện áp đọc được:** 
   * Kết nối Serial Monitor. Trạng thái điện áp pin được in định kỳ trong Blackbox log (nhấn `'b'` để dump) hoặc in ra khi khởi động.
   * So sánh giá trị điện áp hiển thị với giá trị đo bằng đồng hồ vạn năng (Multimeter) đo trực tiếp tại jack cắm pin. Độ lệch cho phép $\le \pm 0.1\text{V}$.
2. **Kiểm tra mức cảnh báo Pin yếu:**
   * Sử dụng nguồn DC điều chỉnh được (hoặc dùng pin cũ có điện áp thấp $< 10.5\text{V}$):
     * Khi điện áp sụt dưới **10.5V** (LOW_BATTERY): Kiểm tra xem có cảnh báo LED nhấp nháy/còi kêu không. Nếu đang bay, kéo ga lên tối đa xem xung ga có bị giới hạn ở mức **1600µs** không.
     * Khi điện áp sụt dưới **9.9V** (CRITICAL_BATTERY): Hệ thống phải khóa không cho phép ARM nếu đang DISARMED.

---

## 5. Kiểm thử Tính năng An toàn Mất sóng (RC Failsafe Test)

### Mục tiêu
Xác nhận drone sẽ ngắt hoàn toàn động cơ ngay lập tức nếu mất kết nối với tay điều khiển, tránh việc drone bay mất kiểm soát (flyaway).

### Các bước kiểm thử
1. **Chuẩn bị:** Đặt drone cố định (không lắp cánh quạt), bật tay phát và ARM hệ thống, đẩy nhẹ ga để động cơ quay ở tốc độ thấp.
2. **Kích hoạt Failsafe:** Tắt nguồn tay phát điều khiển (Transmitter).
3. **Tiêu chí đánh giá Đạt (Pass):**
   * Trong vòng **100ms** đầu tiên: Hệ thống phát cảnh báo mất sóng qua Serial log.
   * Đúng **200ms** kể từ lúc tắt tay phát: Toàn bộ 4 động cơ phải dừng quay hoàn toàn và hệ thống tự động khóa chuyển về trạng thái `FAILSAFE`.
4. **Khôi phục:** Bật lại tay phát, kéo cần ga về vị trí thấp nhất và gạt công tắc DISARM rồi ARM lại để đảm bảo hệ thống có thể khôi phục bình thường.

---

## 6. Kiểm thử Tiện ích Hệ thống (Blackbox & I2C Scanner)

### Mục tiêu
Đảm bảo các tiện ích debug hoạt động bình thường để hỗ trợ phân tích dữ liệu sau khi bay.

### Các bước kiểm thử
1. **Kiểm tra I2C Scanner:**
   * Trên Serial Monitor (ở trạng thái `DISARMED`), gửi ký tự `'s'`.
   * Màn hình phải quét ra chính xác:
     * `0x68` (Cảm biến MPU6050)
     * `0x50` (Bộ nhớ EEPROM 24LC256)
2. **Kiểm tra Blackbox Dump:**
   * Cho drone hoạt động một khoảng thời gian ngắn (ở trạng thái ARMED hoặc chạy test motor).
   * Gạt công tắc chuyển về `DISARMED`.
   * Gửi ký tự `'b'` qua Serial Monitor.
   * Màn hình phải xuất ra toàn bộ danh sách dữ liệu log lưu trong Ring Buffer (bao gồm Loop Time, PID Output, VBAT, RSSI, LQ).
