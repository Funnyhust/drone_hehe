# Đặc tả Kỹ thuật và Cấu hình Firmware STM32F103C8T6 (Drone)

Tài liệu này quy chuẩn thông tin thiết kế phần cứng, cấu trúc phần mềm nhúng, các cơ chế an toàn và quy định lập trình cho máy bay không người lái Quadcopter sử dụng vi điều khiển STM32F103C8T6 (Bluepill), xây dựng trên nền tảng **PlatformIO** và **Arduino framework**.

---

## 1. Cấu hình Phần cứng & Ánh xạ GPIO

Tất cả các chân GPIO bắt buộc phải được định nghĩa tập trung duy nhất tại file `board_pinmap.h`. Tuyệt đối không được hardcode số chân trong driver.

### Hệ thống Clock & Debug
* **PA13 (SWDIO) & PA14 (SWCLK):** Chân nạp chương trình và debug qua mạch nạp ST-Link.
* **PD0 / PD1:** Sử dụng thạch anh ngoài cao tần HSE 8MHz.
* **PC14 / PC15:** Sử dụng thạch anh ngoài thấp tần LSE 32.768kHz.

### Phân hệ Động cơ (Motor PWM)
* **Chân điều khiển:**
  * **PB3** = F1-DRV (Động cơ 1 - Trước Phải)
  * **PB4** = F2-DRV (Động cơ 2 - Sau Phải)
  * **PB5** = F3-DRV (Động cơ 3 - Sau Trái)
  * **PB6** = F4-DRV (Động cơ 4 - Trước Trái)
* **Xử lý chân JTAG:** PB3 và PB4 mặc định thuộc cụm JTAG. Trong hàm khởi tạo `motorInit()`, ta gọi lệnh giải phóng debug port nhưng giữ SWD để có thể sử dụng các chân này làm GPIO thông thường xuất xung PWM:
  ```cpp
  disableDebugPorts(); // Giải phóng PB3, PB4, PA15
  ```
* **Tạo xung PWM:**
  * **Hạn chế phần cứng:** PB3, PB4, PB5 liên quan đến cụm chân JTAG/remap và không phân bổ đồng nhất được trên các kênh hardware timer độc lập mà không dính cấu hình phức tạp.
  * **Giải pháp bắt buộc:** **Không được sử dụng `digitalWrite` + `delayMicroseconds`** (gây tốn CPU và độ chính xác kém). Phải ưu tiên sử dụng Hardware Timer nếu khả thi. Nếu không đủ channel hardware timer, bắt buộc phải xây dựng một **bộ lập lịch xung PWM (PWM scheduler) sử dụng timer interrupt** để tạo xung PWM 400Hz (chu kỳ 2.5ms, độ rộng xung 1000us - 2000us) có độ chính xác cao nhất cho cả 4 ESC.
  * **Yêu cầu hỗ trợ từ ESC:** **ESC bắt buộc phải hỗ trợ tín hiệu PWM 400Hz**. Nếu ESC chỉ hỗ trợ servo PWM truyền thống (50Hz), hệ thống phải có cấu hình tùy chọn để fallback hạ tần số PWM xuống **50Hz** (chu kỳ 20ms).
  * Tần số mặc định: **400Hz** (Chu kỳ 2.5ms).

### Bộ thu vô tuyến ExpressLRS (CRSF Receiver)
* **PA3** (USART2_RX) = Nhận tín hiệu TX từ module ELRS.
* **PA2** (USART2_TX) = Truyền tín hiệu RX tới module ELRS.
* Tốc độ giao tiếp mặc định: **420000 baud**, 8N1.
* **Đấu nối:** Sử dụng bộ Hardware UART2 (`Serial2`) của STM32F103 làm mặc định để đảm bảo tốc độ 420000 baud hoạt động ổn định nhất, không bị mất gói tin hay trễ ngắt. Người dùng kết nối trực tiếp chân TX của module ELRS sang chân **PA3** và RX của module ELRS sang chân **PA2**.

### Giao tiếp I2C Software (Bit-Bang)
* **PA9** = SDA
* **PA10** = SCL
* Cấu hình chân dạng **Open-Drain, Pull-up** ngoài 4.7k.
* **Thiết kế Bit-bang linh hoạt:**
  * Hàm trễ được thiết kế linh hoạt bằng tham số cấu hình (không khóa cứng tần số 400kHz trong code) cho phép cấu hình động/tĩnh giữa 100kHz và 400kHz một cách linh hoạt.
  * Tích hợp chức năng chống treo hệ thống (timeout cực ngắn) và chu trình khôi phục bus bị kẹt (Bus Recovery - gửi 9 xung nhịp SCL).
* **Quản lý thiết bị I2C:**
  * **MPU6050** (địa chỉ `0x68`): Thiết bị chính trong vòng lặp điều khiển. Sử dụng phương thức **Burst Read 14 bytes** bắt đầu từ thanh ghi `0x3B` (đọc Accel X, Y, Z, Temp, Gyro X, Y, Z) trong 1 transaction nhằm giảm thiểu overhead giao tiếp. Lấy mẫu ở tần số **500Hz**.
  * **Hiệu chuẩn IMU (Calibration):** Tự động hiệu chuẩn Gyro và Accel khi khởi động hoặc qua cổng lệnh. Trong quá trình hiệu chuẩn, drone phải được đặt cố định nằm ngang. Hệ thống sẽ thu thập **2000 mẫu** (ở tần số 1kHz tương ứng với 2 giây) để tính toán offset gyro chính xác nhất nhằm giảm thiểu trôi (gyro bias) của cảm biến. Offset được lưu vào RAM và ghi vào EEPROM khi disarm.
  * **24LC256** (EEPROM - địa chỉ `0x50`): **Chỉ đọc khi khởi động (boot)** để tải cấu hình PID, và **chỉ ghi khi ngắt động cơ (disarm)**. Tuyệt đối không đọc/ghi EEPROM trong vòng lặp điều khiển nhanh.
  * **HDC2022:** Loại bỏ hoàn toàn khỏi dự án (không sử dụng nữa).
* **Cơ chế Fallback:**
  * Nếu phát hiện bit-bang 400kHz không ổn định trong lúc khởi tạo/kiểm tra, hệ thống tự động hạ tốc độ bus xuống **100kHz**.
  * Khi hạ tốc độ I2C xuống 100kHz, target vòng lặp PID vẫn được **mặc định cố gắng giữ ở 500Hz** (chu kỳ 2ms) nếu thời gian thực thi (loop time) cho phép.
  * Để bù đắp việc I2C chậm, hệ thống thực hiện giảm tần suất lấy mẫu của Gyro, giảm ghi logging debug và giảm telemetry. **Chỉ hạ PID loop xuống 250Hz (chu kỳ 4ms) nếu đo thực tế loop time vượt quá budget liên tục**.
  * Hệ thống sẽ xuất cảnh báo qua Serial log.

### Giám sát Nguồn (Battery ADC)
* **PA1** = Đọc điện áp pin thông qua mạch cầu phân áp (Điện trở $27\text{k}\Omega$ và $10\text{k}\Omega$).
* Công thức tính toán:
  $$V_{adc} = \text{analogRead}(PA1) \times \frac{3.3}{4095}$$
  $$V_{pin} = V_{adc} \times \frac{27000 + 10000}{10000} = V_{adc} \times 3.7$$
* Tích hợp bộ lọc số trung bình động (Moving Average Filter) để lọc nhiễu đọc điện áp khi động cơ chạy.
* **Phân lớp trạng thái điện áp nguồn (Battery States):**
  * `NORMAL`: Điện áp ổn định (đối với pin 3S: $\ge 10.5\text{V}$).
  * `LOW_BATTERY`: Điện áp bắt đầu sụt ($10.0\text{V} \le V_{in} < 10.5\text{V}$), kích hoạt cảnh báo LED/còi (nếu có) và giới hạn công suất cần ga tối đa (throttle limit).
  * `CRITICAL_BATTERY`: Điện áp sụt sâu ($V_{in} < 9.9\text{V}$). Ở chế độ Prototype mode: Nếu hệ thống đang bay (`ARMED`), chỉ thực hiện giới hạn ga (throttle limit) cực thấp và cảnh báo liên tục, **không tự động disarm giữa không trung** để tránh rơi tự do gây nguy hiểm cho thiết bị và người xung quanh. Chỉ cho phép disarm khi ga ở mức tối thiểu hoặc drone đã chạm đất. Nếu đang ở trạng thái `DISARMED`, khóa hoàn toàn không cho phép Arm.

---

## 2. Kiến trúc Phân tầng & Thuật toán

Dự án được phân chia thành 3 tầng chức năng độc lập:
1. **Lớp Driver (Trừu tượng hóa phần cứng):**
   * Đọc/ghi bit-bang I2C qua `soft_i2c`.
   * Giao tiếp trực tiếp với MPU6050, EEPROM.
   * Khởi tạo và cập nhật chu kỳ nhiệm vụ cho động cơ bằng Timer thông qua `motor_pwm` ở tần số mặc định 400Hz.
   * Thực hiện bộ đọc ADC pin và xử lý lọc số.
2. **Lớp Middleware (Xử lý thuật toán và giao thức):**
   * Giải mã gói dữ liệu CRSF (`crsf_parser`) thu được các kênh điều khiển (Roll, Pitch, Throttle, Yaw, Arm, Mode).
   * **Ước lượng Tư thế (Attitude Estimator):** Ưu tiên trích xuất và sử dụng thuật toán **gyro attitude estimator** tham khảo trực tiếp từ Betaflight. Thuật toán **Complementary Filter** chỉ được sử dụng làm phương án dự phòng (fallback) nếu việc tích hợp bộ estimator từ Betaflight gặp trục trặc kỹ thuật.
   * Thuật toán điều khiển PID vòng lặp kép (`pid_controller`) và phân bổ lực đẩy cho động cơ (`motor_mixer`) tối giản từ thiết kế của Betaflight.
3. **Lớp Application (Vòng lặp chính và an toàn):**
   * Vòng lặp chính chạy ở tần số **500Hz** (mặc định). Khi xảy ra fallback I2C 100kHz, loop chính vẫn duy trì ở 500Hz và chỉ giảm xuống 250Hz khi loop time đo thực tế vượt budget liên tục.
   * Quản lý mô hình trạng thái (State Machine) ARM/DISARM và Watchdog giám sát phần cứng.

### Bộ đệm giám sát debug (Debug Ring Buffer)
Triển khai một ring buffer để lưu trữ liên tục các thông số debug quan trọng nhất phục vụ phân tích sau chuyến bay (Blackbox):
* Loop Time (Thời gian thực thi vòng lặp).
* PID Output (Đầu ra PID Roll, Pitch, Yaw).
* VBAT (Điện áp pin đo được).
* RSSI & LQ (Độ mạnh và chất lượng tín hiệu RC từ ELRS).
* Dữ liệu này có thể được kết xuất qua Serial khi drone ở trạng thái `DISARMED`.

---

## 3. Quản lý Trạng thái & Cơ chế An toàn

### Mô hình Trạng thái (ARM/DISARM State Machine)
Định nghĩa rõ ràng 4 trạng thái hệ thống:
1. `DISARMED`: Động cơ bị khóa hoàn toàn ở mức 1000us. Cho phép hiệu chuẩn IMU, đọc/ghi EEPROM, hoặc chạy các lệnh cấu hình.
2. `PRE_ARM`: Trạng thái kiểm tra an toàn (Pre-arm Checks). Hệ thống quét các điều kiện cần thiết. Nếu tất cả đạt yêu cầu, cho phép chuyển sang `ARMED`.
3. `ARMED`: Động cơ hoạt động, bộ trộn Mixer và vòng PID bắt đầu tính toán liên tục. Khóa các tính năng hiệu chuẩn IMU và khóa ghi EEPROM để tránh trễ chu kỳ.
4. `FAILSAFE`: Kích hoạt ngay lập tức khi xảy ra sự cố nghiêm trọng. Động cơ lập tức tắt hoàn toàn (`motorStopAll()`), chuyển trạng thái về `DISARMED`.

### Các điều kiện Khóa liên động trước khi Arm (Pre-arm Checks)
* Động cơ luôn giữ ở mức an toàn **1000us** khi chưa Arm.
* Không cho phép Arm nếu cần ga (Throttle) lớn hơn **1050us**.
* Không cho phép Arm nếu mất tín hiệu điều khiển từ bộ thu ELRS.
* Không cho phép Arm nếu cảm biến IMU chưa hoàn thành hiệu chuẩn (lấy đủ 2000 mẫu) hoặc khởi tạo thất bại.
* Không cho phép Arm nếu điện áp pin ở trạng thái `CRITICAL_BATTERY`.

### Giám sát Watchdog & Protections
* **Watchdog bảo vệ:** Cấu hình **IWDG (Independent Watchdog)** của STM32F103 với chu kỳ timeout khoảng **100ms hoặc 200ms** khi bring-up và phát triển để tránh reset nhầm do các tác vụ I2C bit-bang hoặc ghi Serial log bị trễ nhẹ. Thực hiện xóa cờ (feed watchdog) liên tục trong vòng lặp điều khiển chính. Khi hệ thống đã chạy ổn định tối đa, IWDG timeout mới được hạ xuống **50ms**. Nếu chương trình bị treo ở bất cứ đâu, vi điều khiển sẽ tự động reset.
* **Mất sóng (RC Failsafe):**
  * **Failsafe Warning (100ms):** Nếu mất gói tin CRSF liên tục trong **100ms**, hệ thống sẽ bật cảnh báo mất tín hiệu nhưng vẫn giữ trạng thái bay.
  * **Failsafe Hard Kill (200ms):** Nếu mất gói tin liên tục vượt quá **200ms**, hệ thống ngay lập tức tắt toàn bộ động cơ và chuyển sang trạng thái `FAILSAFE`.
* **Lỗi cảm biến IMU liên tục:** Nếu giao dịch I2C đọc MPU6050 bị lỗi liên tiếp quá **5 lần**, hệ thống ngay lập tức chuyển sang trạng thái `FAILSAFE`, ngắt động cơ để tránh sự cố.

---

## 4. Quy định Lập trình & Biên dịch (Coding Standards)

Để đảm bảo code biên dịch tối ưu và không bị lỗi thời gian thực, toàn bộ firmware bắt buộc tuân thủ các quy tắc lập trình nhúng sau:

1. **Cấu hình Biên dịch:** Chương trình phải compile thành công với các cờ cảnh báo nghiêm ngặt: `-Wall -Wextra`. Không bỏ qua các cảnh báo về kiểu dữ liệu và biến không sử dụng.
2. **Quản lý bộ nhớ:** **Tuyệt đối không sử dụng `malloc()`, `free()`, `new`, `delete`** trong vòng lặp điều khiển chính (PID loop) để tránh rác bộ nhớ và phân mảnh RAM gây trễ ngắt bất định.
3. **Kiểu dữ liệu chuỗi:** **Tuyệt đối không sử dụng lớp `String`** của Arduino. Mọi chuỗi ký tự phải được quản lý bằng mảng char tĩnh (`char[]`) với kích thước xác định trước.
4. **Hàm trễ (Delay):** **Tuyệt đối không sử dụng `delay()`** trong thời gian chạy (runtime). Mọi tác vụ phải chạy dạng phi chặn (**non-blocking**), quản lý bằng bộ đếm thời gian hoặc so sánh `micros()`.

---

## 5. Chế độ Thử nghiệm tại chỗ (Bring-up Mode)

Để hỗ trợ kiểm thử phần cứng cô lập, firmware cung cấp menu tương tác qua cổng nạp UART (Serial Monitor 115200 baud):
1. **Boot Status:** Hiển thị thông số cấu hình và kết quả tự kiểm tra cảm biến lúc khởi động.
2. **I2C Scan:** Tìm kiếm các địa chỉ phản hồi trên đường truyền I2C mềm.
3. **Raw IMU Data:** In liên tục giá trị thô và offset của MPU6050 để kiểm định hướng trục.
4. **CRSF Channels Monitor:** Kiểm tra trạng thái và giá trị các kênh nhận được từ tay điều khiển.
5. **Motor Test Mode (Compile Flag `-D TEST_MOTOR_OUTPUT`):** Cho phép đặt tốc độ cụ thể cho từng động cơ qua Serial để kiểm tra dây nối và chiều quay.

> [!WARNING]
> Bắt buộc phải tháo toàn bộ cánh quạt khi thực hiện kiểm tra động cơ bằng chế độ `TEST_MOTOR_OUTPUT` để tránh nguy hiểm.

---

## 6. Lộ trình Triển khai Dự án (Implementation Order)

Để đảm bảo quá trình tích hợp trơn tru, hạn chế tối đa lỗi chéo giữa các tầng ngoại vi, thứ tự phát triển và kiểm tra bắt buộc tuân thủ lộ trình sau:
1. **board_pinmap.h**: Tạo file pinmap chứa tất cả các cấu hình chân GPIO.
2. **motor_pwm test không cánh**: Implement driver PWM điều khiển động cơ bằng timer interrupt hoặc hardware timer ở tần số 400Hz (hoặc 50Hz), giải phóng JTAG, test xuất xung an toàn.
3. **soft_i2c + MPU6050 burst read**: Hoàn thành bit-bang I2C điều chỉnh được trễ, test burst read 14 byte từ cảm biến ổn định.
4. **CRSF parser**: Giải mã gói tin từ Hardware UART2 ở tốc độ 420000 baud.
5. **battery ADC**: Đọc pin qua PA1, thực hiện lọc trung bình động và phân loại trạng thái pin (NORMAL, LOW, CRITICAL).
6. **state machine + failsafe**: Hoàn thành hệ thống quản lý trạng thái ARM/DISARM, IWDG và logic failsafe 100ms/200ms.
7. **PID/mixer**: Tích hợp gyro attitude estimator và Mixer của Betaflight.
8. **blackbox/debug buffer**: Triển khai buffer ghi log để phục vụ gỡ lỗi thực tế.
