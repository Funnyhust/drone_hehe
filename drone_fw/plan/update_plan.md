# Kế hoạch Nâng cấp Firmware: Chuyển đổi Hardware PWM và Tối ưu hóa I2C

Bản kế hoạch này tổng hợp các phân tích kỹ thuật và các bước thực thi để giải quyết triệt để vấn đề nhiễu (jitter) vòng lặp điều khiển, đảm bảo MPU6050 đọc dữ liệu chính xác ở tốc độ 400kHz trên phần cứng cố định (STM32F103C8T6).

## Mục tiêu (Goal Description)
Loại bỏ hoàn toàn **Software PWM** (sử dụng ngắt Timer) để giải phóng CPU. Sử dụng **Hardware PWM** trên các chân động cơ hiện có (PB4, PB5, PB6, PB7) thông qua tính năng Alternate Function Remap của STM32F103. Nhờ đó, **Soft I2C** (PA9, PA10) sẽ chạy ổn định ở **400kHz** mà không bị ngắt xen ngang làm sai lệch thời gian (timing).

> [!IMPORTANT]
> **Yêu cầu đối với người dùng (User Review Required):**
> 1. Đảm bảo mạch của bạn đã có điện trở kéo lên (Pull-up Resistor) giá trị **2.2 kΩ - 4.7 kΩ** nối từ 3.3V vào 2 chân PA9 và PA10. (Module GY-521 thường đã tích hợp sẵn).
> 2. Đọc kỹ phần Cảnh báo Thử nghiệm động cơ ở cuối file này. Xin hãy phê duyệt kế hoạch để tôi bắt đầu sửa code.

## Các thay đổi dự kiến (Proposed Changes)

### 1. Phân hệ Cấu hình Chân (Pinmap)
Không thay đổi file mạch, giữ nguyên thiết kế:
* **Motor 1:** PB6 (`TIM4_CH1`)
* **Motor 2:** PB5 (`TIM3_CH2` - Yêu cầu AFIO Partial Remap)
* **Motor 3:** PB4 (`TIM3_CH1` - Yêu cầu AFIO Partial Remap)
* **Motor 4:** PB7 (`TIM4_CH2`)
* **Soft I2C:** PA9 (SDA), PA10 (SCL)

### 2. Phân hệ Điều khiển Động cơ (Motor PWM)

#### [MODIFY] `src/driver/motor_pwm.cpp`
* **Xóa bỏ ngắt phần mềm:** Xóa toàn bộ các hàm ngắt `motor_overflow_callback`, `motor_compare1_callback`, v.v.
* **Cấu hình AFIO:** 
  * Gọi hàm tắt JTAG (giữ lại SWD) để giải phóng PB4: `__HAL_AFIO_REMAP_SWJ_NOJTAG()`.
  * Gọi hàm bật Partial Remap cho TIM3: `__HAL_AFIO_REMAP_TIM3_PARTIAL()`.
* **Khởi tạo HardwareTimer:**
  * Dùng `HardwareTimer(TIM3)` điều khiển PB4 (Kênh 1) và PB5 (Kênh 2).
  * Dùng `HardwareTimer(TIM4)` điều khiển PB6 (Kênh 1) và PB7 (Kênh 2).
  * Chuyển chế độ sang `TIMER_PWM`.
* **Tốc độ PWM:** 
  * Sẽ đặt mặc định thử nghiệm ở **400Hz** để tận dụng tối đa vòng lặp PID 500Hz.
  * Giữ lại đối số `use_50hz` trong hàm khởi tạo để dự phòng hạ cấp xuống 50Hz nếu ESC quá cũ.

#### [MODIFY] `src/driver/motor_pwm.h`
* Loại bỏ khai báo biến/cấu trúc liên quan đến ngắt Timer 2.
* Điều chỉnh để phù hợp với kiến trúc HardwareTimer mới.

### 3. Phân hệ I2C và Vòng lặp chính

#### [MODIFY] `src/main.cpp`
* Giữ nguyên cơ chế Fallback tốc độ MPU6050: Ban đầu thử khởi tạo ở `400kHz`. Nếu thất bại, tự động hạ xuống `100kHz`.
* Không cần thay đổi vòng lặp chính. Nhờ việc gỡ bỏ ngắt PWM, `budget_overrun_counter` (đếm vượt quá chu kỳ 2000us) sẽ ít khi bị kích hoạt hơn.

#### [MODIFY] `src/driver/soft_i2c.cpp`
* Không sửa đổi lõi bit-banging (vì nó đã an toàn).
* Không thêm hàm `noInterrupts()` (tắt ngắt toàn cục) để đảm bảo ngắt UART CRSF vẫn hoạt động bình thường, duy trì kết nối tay điều khiển.

---

## Kế hoạch Kiểm tra (Verification Plan)

> [!CAUTION]
> BẮT BUỘC THÁO TOÀN BỘ 4 CÁNH QUẠT TRƯỚC KHI CẤP NGUỒN PIN LIPO.
> Việc test ESC ở tần số 400Hz có thể gây hiểu nhầm tín hiệu thành Full-Throttle trên một số dòng ESC rất cũ, dẫn đến động cơ tự quay max tốc độ.

### Các bước kiểm tra thủ công (Manual Verification):
1. **Kiểm tra Soft I2C:** Cắm cáp USB (không cắm pin). Dùng lệnh CLI `i` trên Serial Monitor để in dữ liệu MPU6050. Nếu luồng số liệu chạy mượt mà, không bị khựng, và báo init 400kHz thành công -> I2C đã ổn định.
2. **Kiểm tra Âm thanh ESC:** Cắm pin LiPo. ESC phải phát ra tiếng bíp khởi động chuẩn (VD: 3 bíp ngắn + 1 bíp dài). Nếu ESC kêu tít tít báo lỗi liên tục, chuyển `motorInit(true)` về 50Hz và nạp lại code.
3. **Kiểm tra Chiều quay Động cơ:** Dùng lệnh CLI `t` để bật Motor Test Mode. Bật từng motor (1, 2, 3, 4) xem có quay đúng vị trí PB6, PB5, PB4, PB7 và đúng chiều hay không. Mạch Hardware PWM vẫn sẽ đánh map chân y như cũ nhưng bằng phần cứng.
