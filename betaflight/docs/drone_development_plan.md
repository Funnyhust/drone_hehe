# Kế hoạch Phát triển Drone (Betaflight)

Tài liệu này tổng hợp toàn bộ các thành phần đã có trong mã nguồn hiện tại và những bước bạn cần thực hiện để hoàn thiện một chiếc drone.

## 1. Những gì ĐÃ CÓ (Source Code Core)
Project Betaflight này đã cực kỳ đầy đủ và được tối ưu hóa ở mức độ chuyên gia. Bạn không cần phải viết lại các thuật toán cơ bản.

### 🧠 Thuật toán Điều khiển & Cân bằng
- **PID Controller**: Hỗ trợ đầy đủ Acro, Angle, Horizon mode với các tính năng nâng cao (Feedforward, Anti-Gravity).
- **IMU (Mahony AHRS)**: Thuật toán ước lượng trạng thái dựa trên Quaternion, lọc nhiễu gyro/accel.
- **Hệ thống lọc (Filters)**: Lowpass, Biquad, Dynamic Notch Filter (theo RPM).

### 📡 Truyền tin và Điều khiển (RX)
- **ExpressLRS (ELRS)**: Đã tích hợp sẵn driver (`src/main/rx/expresslrs.c`).
- **Các giao thức khác**: Hỗ trợ SBUS, CRSF, IBUS, Spektrum...

### ⚙️ Điều khiển Động cơ (Motor)
- **DShot (300/600)**: Giao thức hiện đại nhất cho ESC.
- **PWM / OneShot**: Hỗ trợ các ESC đời cũ.
- **Mixer**: Cấu hình Quad X, Tri, Hexa, Cánh bằng...

### 🔌 Drivers Cảm biến
- **Gyro/Accel**: Hỗ trợ gần như tất cả chip (BMI270, MPU6000, ICM-42688P...).
- **Barometer (Áp suất)**: BMP280, DPS310... (để giữ độ cao).
- **Magnetometer (La bàn)**: HBM5883L... (để định hướng Bắc).
- **GPS**: Hỗ trợ giao thức UBLOX, NMEA.

---

## 2. Những gì BẠN CẦN LÀM (Lập trình & Cấu hình)
Vì mã nguồn core đã có, công việc chính của bạn là **Cấu hình (Targeting)** để khớp với phần cứng thực tế.

### 📍 Bước 1: Pin Mapping (Sơ đồ chân)
Bạn cần định nghĩa trong code các chân của vi điều khiển (MCU) nối với linh kiện nào:
- **Motor 1, 2, 3, 4**: Nối vào chân Timer/DMA nào?
- **ELRS Receiver**: Nối vào UART port nào (TX/RX)?
- **Gyro/Accel**: Kết nối qua SPI hay I2C? Sử dụng chân CS (Chip Select) nào?
- **Vốn dĩ (Battery/Current)**: Nối vào chân ADC nào để đo điện áp?

### 🛠️ Bước 2: Cấu hình Target (Board Configuration)
- Tạo thư mục target mới trong `src/platform/STM32/target/` (Ví dụ: `src/platform/STM32/target/MY_CUSTOM_FC/`).
- Viết file `target.mk` để định nghĩa dòng MCU (Vd: `TARGET_MCU := STM32F405xx`).
- Viết file `target.h` để định nghĩa các Hardware Resources và các tính năng được ENABLE.

### 🏗️ Bước 3: Biên dịch (Build)
- Sử dụng lệnh `make` để biên dịch binary tương ứng với target của bạn.
- Ví dụ: `make TARGET=MY_CUSTOM_FC`.

### 🧪 Bước 4: Cân chỉnh (Tuning)
- Sau khi flash code, bạn cần sử dụng **Betaflight Configurator** để:
    - Hiệu chỉnh cảm biến (Calibrate Accel).
    - Thiết lập hướng board (Board Alignment).
    - Tinh chỉnh thông số PID (PID Tuning) để drone bay mượt mà nhất.

---

## 3. Bảng Tổng hợp Toàn bộ Hệ thống Drone (Phần cứng & Mã nguồn)

Dưới đây là bảng tra cứu đầy đủ cho tất cả các thành phần cần thiết để xây dựng chiếc drone của bạn:

| Hệ thống | Thành phần | Linh kiện khuyên dùng | Vị trí Source Code | Chức năng & Ghi chú |
| :--- | :--- | :--- | :--- | :--- |
| **Xử lý trung tâm** | **Vi điều khiển (MCU)** | STM32F405 / F722 / H743 | `src/main/fc/` | "Bộ não" chạy toàn bộ thuật toán Betaflight. |
| **Bay (Flight)** | **Cảm biến Góc (IMU)** | **MPU6050**, BMI270, MPU6000 | `src/main/drivers/accgyro/` | Tự cân bằng và ổn định tư thế drone. |
| | **Động cơ (Motor)** | Brushless Motor (Vd: 2207) | `src/main/drivers/motor.c` | Lực đẩy để drone bay được. |
| | **Điều tốc (ESC)** | ESC hỗ trợ DShot 600 | `src/main/drivers/dshot.c` | Nhận lệnh từ MCU để điều khiển tốc độ Motor. |
| | **Cảm biến Áp suất** | BMP280, DPS310, MS5611 | `src/main/sensors/barometer.c` | Đo áp suất không khí để giữ độ cao. |
| **Điều khiển (RC)** | **Bộ thu sóng (RX)** | **SX127x (LoRa 433MHz)** | `src/main/rx/expresslrs.c` | Nhận tín hiệu điều khiển từ xa (ELRS). |
| **An toàn & LR** | **Hệ thống GPS** | UBLOX M8N, M10 | `src/main/io/gps.c` | Tọa độ GPS, tốc độ và chế độ GPS Rescue. |
| | **Năng lượng** | Cầu phân áp (Volt), Shunt (Curr) | `src/main/sensors/battery.c` | Theo dõi dung lượng pin (Battery Info). |
| | **Nhật ký bay** | SPI Flash, SD Card | `src/main/blackbox/` | Ghi log dữ liệu bay để phân tích lỗi. |
| **Hình ảnh (FPV)** | **Camera FPV** | **Caddx Ratel 2**, Foxeer Falkor | (Tín hiệu gửi trực tiếp sang VTX) | "Con mắt" giúp Pilot thấy đường lái. |
| | **OSD Chip** | **MAX7456** | `src/main/drivers/max7456.c` | Chèn thông số (Volt, GPS) lên màn hình. |
| | **Bộ phát hình (VTX)** | Rush Tank Solo, Foxeer Reaper | (Gửi sóng về Kính/Màn hình) | Truyền hình ảnh về trạm mặt đất. |

---

## 4. Sơ đồ Cấp nguồn & Quản lý Năng lượng

Để hệ thống hoạt động ổn định, bạn cần phân tầng nguồn điện như sau:

1.  **Nguồn VBAT (Điện áp pin trực tiếp):** 
    *   Cấp cho 4 ESC và bộ phát hình VTX (nếu VTX hỗ trợ điện áp cao).
2.  **Mạch hạ áp 5V (BEC):** 
    *   Hạ từ VBAT xuống 5V để cấp cho Module GPS, Module RX (SX127x), chip OSD, Beeper và LED.
3.  **Mạch ổn áp 3.3V (LDO):** 
    *   Hạ từ 5V xuống 3.3V để cấp cho MCU STM32 và cảm biến MPU6050 (đây là tầng nguồn nhạy cảm nhất).

---

## 5. Danh sách Linh kiện cần chuẩn bị (Kiểm tra cuối)
- [ ] Vi điều khiển STM32 (Khuyên dùng F405 trở lên cho Betaflight mới nhất).
- [ ] Mạch cảm biến MPU6050.
- [ ] Module SX127x (LoRa 433MHz).
- [ ] Module GPS M8N/M10.
- [ ] Khung Carbon (Frame), 4 Motor, 4 ESC.
- [ ] Camera FPV & VTX.
- [ ] Chip OSD MAX7456 (Nếu bạn tự thiết kế mạch FC).
- [ ] Pin LiPo (3S-6S) & Bộ sạc.

> [!TIP]
> Việc cách ly nguồn nhiễu giữa **Động cơ** và **MCU** là chìa khóa để drone bay ổn định. Hãy sử dụng thêm tụ điện (vd: 1000uF 35V) tại đầu vào pin để lọc nhiễu tốt hơn.
