# 📋 HƯỚNG DẪN ĐỌC HIỂU `main.cpp`
> File này có **954 dòng**. Bạn **KHÔNG cần đọc từng dòng**. Hãy đọc theo 4 khối chức năng lớn dưới đây.

---

## 🗺️ BẢN ĐỒ FILE main.cpp

| Dòng | Khối | Mô tả ngắn |
|------|------|------------|
| 1 - 51 | Khai báo | `#include`, biến toàn cục |
| 56 - 75 | Hàm tiện ích | `runI2cScanner()`, `printDroneConfig()` |
| 120 - 445 | **Khởi động FSM** | `updateStartupFsm()` — trình tự bật nguồn |
| 447 - 608 | Chế độ đặc biệt | `runEscCalibration()`, `runAccelCalibration()` |
| 613 - 643 | **`setup()`** | Chạy 1 lần khi cắm điện |
| 648 - 954 | **`loop()`** | Vòng lặp điều khiển 250Hz |

---

## KHỐI 1 — KHAI BÁO TOÀN CỤC (Dòng 1 - 51)
> *Chỉ cần nhìn lướt, không cần học thuộc.*

**Các biến quan trọng cần biết:**
- **Dòng 25-26**: `imu_raw` (dữ liệu thô cảm biến) và `attitude_angles` (góc Roll/Pitch/Yaw đã tính xong).
- **Dòng 30-31**: `target_loop_period_us = 4000` → Chu kỳ vòng lặp mục tiêu là 4000 micro-giây (= 250Hz).
- **Dòng 41-43**: `out_roll`, `out_pitch`, `out_yaw` → Nơi hứng kết quả đầu ra của bộ PID.
- **Dòng 51**: `global_config` → Biến lưu toàn bộ cấu hình (PID, Offset cảm biến, trạng thái ESC).

---

## KHỐI 2 — MÁY TRẠNG THÁI KHỞI ĐỘNG (Dòng 120 - 445)
> *Đây là phần ĐẶC BIỆT NHẤT và hay bị hỏi nhất! Phải nắm chắc.*

**Lý do có FSM (Finite State Machine):** Nếu dùng `delay()` để chờ calib Gyro 1 giây, CPU bị "đóng băng", không thể Feed Watchdog → chip tự reset. FSM cho phép calib từng bước nhỏ mà CPU vẫn chạy các tác vụ khác song song (Non-blocking).

**6 bước khởi động theo thứ tự:**

```
BOOT → IMU_INIT → GYRO_CALIB → RC_CHECK → ACC_CHECK → ESC_CHECK → READY
```

### Bước 1: STARTUP_BOOT (Dòng ~120)
- Đọc cấu hình từ EEPROM (hoặc Flash Backup nếu EEPROM hỏng).
- Nạp các thông số PID và Offset cảm biến đã lưu vào bộ điều khiển.
- **Câu trả lời thầy hỏi "Sao PID có thể chỉnh được?":** *"Dạ, lúc boot hệ thống đọc PID từ EEPROM. Em có thể chỉnh PID qua Serial rồi lưu lại, lần sau cắm điện nó tự nạp bộ mới."*

### Bước 2: STARTUP_IMU_INIT (Dòng 198)
- Thử khởi tạo MPU6050 ở **400kHz** (nhanh).
- Nếu thất bại → tự động thụt xuống **100kHz** (chậm hơn nhưng ổn định hơn).
- Đây là cơ chế **Fallback tự động** — không cần người dùng can thiệp.

### Bước 3: STARTUP_GYRO_CALIB (Dòng 227)
- Đây là Calib Gyro phi chặn (Non-blocking).
- Mỗi lần vào FSM chỉ đọc đúng **1 mẫu** (khác với `mpu6050CalibrateGyro()` là blocking).
- Sau **1000 mẫu** (= 4 giây) → tính Mean và StdDev → lưu Offset Gyro.
- Nếu drone bị rung (StdDev quá lớn) → retry tối đa 5 lần → nếu vẫn thất bại → `STARTUP_ERROR`.

### Bước 4: STARTUP_RC_CHECK (Dòng 321)
- Kiểm tra xem tay điều khiển ELRS có đang kết nối không.
- **Dòng 340**: Ga (Throttle) **BẮT BUỘC phải < 1050us**. Nếu Throttle đang cao → từ chối boot → báo lỗi. *(Bảo vệ tránh motor tự rú lên khi cắm điện).*
- **Dòng 349**: Roll/Pitch/Yaw **BẮT BUỘC phải ở giữa (1500us ± 30us)**. Cần gạt đòn phải ở vị trí trung tâm.

### Bước 5: STARTUP_ACC_CHECK (Dòng 367)
- Đọc 10 mẫu Accel → tính trung bình → tính tổng lực Pytago.
- **Dòng 384**: Tổng lực phải nằm trong khoảng **0.95g - 1.05g**. Nếu lệch → cảm biến lỗi → block.
- *Đây chính là "khám sức khỏe tổng quát" đã nhắc ở mpu6050.cpp, nhưng chạy lại ở tầng main.*

### Bước 6: STARTUP_ESC_CHECK (Dòng 399)
- Chỉ kiểm tra **cờ trạng thái** `esc_calibrated` trong EEPROM, không chạy lại calib.
- Nếu ESC chưa calib → cho phép bay nhưng `motorStopAll()` sẽ chặn motor dù PID đã tính xong.
- Xong → chuyển sang `STARTUP_READY` → vòng lặp điều khiển bắt đầu.

---

## KHỐI 3 — HÀM `setup()` (Dòng 613 - 643)
> *Chạy 1 lần duy nhất khi cắm điện. Học thuộc thứ tự là đủ.*

```
1. softUartInit()      → Bật log UART (19200 baud)
2. Serial.begin()      → Bật CLI Serial (115200 baud)
3. softI2cInit()       → Bật I2C mềm
4. eepromInit()        → Khởi tạo chip nhớ EEPROM
5. crsfInit()          → Khởi tạo bộ thu RC ELRS
6. motorInit(false)    → Khởi tạo PWM motor (false = 200Hz, không phải 50Hz)
7. batteryInit()       → Khởi tạo đọc ADC pin
8. safetyInit()        → Khởi tạo State Machine an toàn
9. blackboxInit()      → Khởi tạo hộp đen
10. imuEstimatorInit() → Khởi tạo bộ lọc bù
11. pidInit()          → Khởi tạo bộ PID
12. loggingInit()      → Khởi tạo hệ thống log debug
```

> **Câu hay hỏi:** *"Tại sao `motorInit(false)` chứ không phải `motorInit(true)`?"*
> **Trả lời:** *"false = 200Hz. Các ESC drone hiện đại hỗ trợ PWM 200Hz giúp phản hồi nhanh hơn. true = 50Hz là cho servo truyền thống."*

---

## KHỐI 4 — HÀM `loop()` (Dòng 648 - 954)
> *Đây là CỐT LÕI của toàn bộ hệ thống. Phải nắm từng bước.*

### Phần A — CLI Debug qua Serial (Dòng 663 - 734)
Người dùng có thể gõ lệnh qua cổng Serial để debug mà không cần bay:

| Phím | Tác dụng |
|------|----------|
| `s` | Quét tìm thiết bị I2C |
| `i` | In liên tục dữ liệu thô IMU |
| `r` | In liên tục giá trị 4 kênh RC |
| `b` | Dump log Blackbox ra Serial |
| `c` | Chạy lại Calib Gyro |
| `p` | In cấu hình PID hiện tại |
| `d` | Bật/tắt bảng Telemetry real-time |

### Phần B — Vòng lặp điều khiển chính (Dòng 739 - 954)

**Cơ chế căn chỉnh thời gian (Dòng 739-748):**
```cpp
uint32_t start_time_us = micros();
uint32_t dt_us = start_time_us - last_loop_time_us;
if (dt_us >= target_loop_period_us) { // Chỉ chạy khi đủ 4000us
```
- Không dùng `delay()` để giữ tần số 250Hz. Thay vào đó: đo thời gian trôi qua, chỉ thực thi khi đủ 4ms.
- CPU vẫn tự do chạy FSM khởi động và nhận lệnh Serial trong thời gian chờ.

**5 bước trong mỗi chu kỳ 4ms:**

#### Bước 1 — Đọc RC (Dòng 752)
```cpp
crsfUpdate(); // Hút dữ liệu từ bộ đệm UART của ELRS
```

#### Bước 2 — Đọc IMU (Dòng 758)
```cpp
imu_ok = (mpu6050Read(&imu_raw) == 0); // Burst Read 14 bytes
```
- Chỉ chạy sau khi FSM đã qua giai đoạn `STARTUP_IMU_INIT`.

#### Bước 3 — Tính góc (Dòng 762)
```cpp
imuEstimatorUpdate(&imu_raw, 0.004f); // dt = 4ms = 0.004 giây
```
- `0.004f` là `dt` được hardcode và truyền thẳng vào hàm.

#### Bước 4 — Kiểm tra an toàn (Dòng 771)
```cpp
safetyUpdate(imu_ok); // Cập nhật Watchdog và trạng thái Arm/Disarm
```

#### Bước 5 — Điều khiển Motor (Dòng 781 - 867)
Chỉ chạy khi `current_fstate == STATE_ARMED`:

**5a — Deadband (Dòng 789-801):**
```cpp
if (ch_roll > 1508) roll_diff = ch_roll - 1508;
else if (ch_roll < 1492) roll_diff = ch_roll - 1492;
// Nếu 1492 <= ch_roll <= 1508 → roll_diff = 0 (vùng chết)
```
Tay điều khiển cơ học không bao giờ về đúng 1500us. Deadband ±8us ngăn drone tự nghiêng nhẹ khi tay không chạm cần.

**5b — Quy đổi sang góc/tốc độ (Dòng 807-809):**
```cpp
float target_roll = (float)roll_diff / 15.0f;   // max ≈ 32.8 độ
float target_pitch = (float)pitch_diff / 15.0f;
float target_yaw_rate = (float)yaw_diff / 3.0f; // max ≈ 164 độ/s
```

**5c — Giới hạn ga theo pin (Dòng 812-823):**
- Pin `BATTERY_LOW` → ga tối đa bị hạ xuống.
- Pin `BATTERY_CRITICAL` → ga bị hạ thêm nữa → buộc người lái phải hạ cánh.

**5d — Reset I-term khi ga thấp (Dòng 827-829):**
```cpp
if (ch_throttle < 1050) { pidReset(); }
```
Khi drone đang trên mặt đất (ga < 1050us), reset khâu Tích phân về 0. Tránh I-term tích lũy trên mặt đất rồi khi cất cánh bị rú motor đột ngột.

**5e — Tính PID (Dòng 832-833):**
```cpp
pidCompute(&attitude_angles, &imu_raw, target_roll, target_pitch,
           target_yaw_rate, &out_roll, &out_pitch, &out_yaw);
```

**5f — Mixer → PWM (Dòng 836-850):**
```cpp
motorMixerCompute(throttle_limit, out_roll, out_pitch, out_yaw, &m1, &m2, &m3, &m4);
// Kiểm tra ESC đã calib chưa
if (global_config.esc_calibrated == 1) {
    motorWriteAllUs(m1, m2, m3, m4); // Xuất PWM
} else {
    motorStopAll(); // Chặn nếu ESC chưa calib
}
```

---

## ⚠️ CÁC CÂU HỎI HAY GẶP VỀ main.cpp

**Q: Tại sao không dùng `delay()` để giữ 250Hz?**
A: `delay()` chặn CPU hoàn toàn. Trong lúc chờ, dữ liệu UART từ ELRS đến sẽ bị mất, Watchdog không được feed sẽ reset chip. Dùng `micros()` cho phép CPU vừa chờ vừa làm việc khác.

**Q: Deadband là gì? Tại sao cần?**
A: Vùng chết ±8us quanh điểm giữa (1500us) của cần điều khiển. Tay điều khiển cơ học không bao giờ trả về đúng 1500us, luôn sai lệch vài us. Không có Deadband, drone sẽ luôn tự trôi nhẹ dù không chạm cần.

**Q: Tại sao PID chỉ chạy khi `STATE_ARMED`?**
A: Khi Disarmed, motor phải đứng yên. Nếu PID vẫn tính và tích I-term, đến khi Arm lại sẽ có lực bù lớn làm motor rú đột ngột gây nguy hiểm.

**Q: Tại sao cần 2 Serial (softUart và Serial cứng)?**
A: `softUart` (USART3 - chân PA10/PB10) → In log liên tục khi bay vì chân PA2 đang dùng cho ELRS. `Serial` cứng (USB/UART1) → Nhận lệnh CLI debug từ máy tính khi ngồi bàn.

**Q: `global_config.esc_calibrated` kiểm tra ở đâu?**
A: Kiểm tra ở **dòng 846** trong vòng lặp chính. Dù PID và Mixer đã tính xong đàng hoàng, nếu cờ này bằng 0 → `motorStopAll()` → motor không nhận được lệnh. Đây là lớp bảo vệ cuối cùng.
