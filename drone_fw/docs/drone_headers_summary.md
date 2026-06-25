# Tài liệu Tổng hợp Cấu trúc Header Files Drone Firmware

Tài liệu này tổng hợp chi tiết các thông số, cấu trúc dữ liệu, hằng số và các hàm được định nghĩa trong các file `.h` thuộc hai phân hệ chính: **Driver** (Giao tiếp phần cứng) và **Middleware** (Thuật toán và Quản lý bay).

---

## Phần 1: Phân hệ DRIVER (`include/driver`)
Đây là tầng giao tiếp trực tiếp với các phần cứng ngoại vi.

### 1. `battery_adc.h` (Quản lý điện áp Pin)
File này chứa các thông số và hàm dùng để đọc điện áp pin qua ADC (chân PA1) và có bộ lọc trung bình động.
*   **Enum `BatteryState`**: Trạng thái của pin gồm `BATTERY_NORMAL` (Bình thường), `BATTERY_LOW` (Yếu), `BATTERY_CRITICAL` (Nguy kịch).
*   **Hằng số ngưỡng điện áp (Pin LiPo 3S)**:
    *   `BATTERY_THRESHOLD_LOW` (10.5V): Ngưỡng báo pin yếu (3.5V/cell).
    *   `BATTERY_THRESHOLD_CRITICAL` (9.9V): Ngưỡng báo pin nguy kịch (3.3V/cell).
*   **Các hàm**: `batteryInit()` (khởi tạo), `batteryUpdate()` (đọc định kỳ), `batteryGetVoltage()` (lấy điện áp đã lọc), `batteryGetState()` (lấy trạng thái pin).

### 2. `eeprom_24lc256.h` (Bộ nhớ EEPROM I2C)
File này dùng để lưu trữ cấu hình hệ thống (PID, offset cảm biến) lên chip EEPROM rời.
*   **Hằng số**:
    *   `EEPROM_ADDR` (0x50): Địa chỉ I2C của EEPROM.
    *   `EEPROM_PID_START_ADDR` (0x0000): Địa chỉ bắt đầu lưu.
    *   `DRONE_CONFIG_SIGNATURE` (0xDEADBEEF): Chữ ký xác thực tính toàn vẹn.
*   **Cấu trúc dữ liệu `DroneConfig`**:
    *   `signature`: Chữ ký hợp lệ.
    *   `kp_roll`, `ki_roll`, `kd_roll` / `pitch` / `yaw`: Hệ số PID cho 3 trục.
    *   `accel_offset...`, `gyro_offset...`: Offset của cảm biến IMU.
    *   `esc_calibrated`: Cờ đánh dấu đã calib ESC.
    *   `crc8`: Mã kiểm tra lỗi dữ liệu (CRC).
*   **Các hàm**: Khởi tạo (`eepromInit`), Đọc/Ghi thô (`eepromReadRaw`, `eepromWriteRaw`), Tính CRC (`eepromCalculateCrc8`), Lưu/Tải cấu hình dự phòng (`configLoad`, `configSave`).

### 3. `flash_backup.h` (Lưu trữ dự phòng trên Flash nội)
File này dùng để lưu dự phòng cấu hình vào bộ nhớ Flash bên trong MCU (đề phòng EEPROM lỗi).
*   **Hằng số**: `FLASH_BACKUP_ADDR` (0x0800FC00) - Tương ứng với Page 63 (Trang cuối cùng) của STM32F103C8T6.
*   **Các hàm**: `flashBackupWrite(p_data, size)`, `flashBackupRead(p_data, size)`.

### 4. `motor_pwm.h` (Điều khiển Động cơ / ESC)
File này điều khiển 4 động cơ thông qua xung Hardware PWM.
*   **Hằng số độ rộng xung (us)**:
    *   `PWM_PULSE_MIN` (1000): Xung tối thiểu (Động cơ dừng).
    *   `PWM_PULSE_MAX` (2000): Xung tối đa (Ga max 100%).
    *   `PWM_PULSE_SAFE` (1000): Mức xung an toàn.
*   **Các hàm**: `motorInit()`, `motorWriteUs(motor, us)`, `motorWriteAllUs(m1, m2, m3, m4)`, `motorStopAll()`.

### 5. `mpu6050.h` (Cảm biến IMU - Gia tốc & Góc nghiêng)
Driver xử lý cảm biến quán tính MPU6050.
*   **Hằng số**:
    *   `MPU6050_ADDR` (0x68): Địa chỉ I2C.
    *   `MPU6050_ACCEL_SO_8G` (4096.0f), `MPU6050_GYRO_SO_500` (65.5f): Độ nhạy Accel và Gyro.
    *   `GYRO_CALIB_STDDEV_THRESHOLD` (50.0f): Ngưỡng rung lắc tối đa khi calib.
*   **Cấu trúc `MpuData`**: Chứa dữ liệu thô (`raw`) và dữ liệu vật lý (`ax, ay, az, gx, gy, gz, temp`).
*   **Các hàm**: `mpu6050Init()`, `mpu6050Read()`, Calib (`mpu6050CalibrateGyro`, `mpu6050CalibrateAccel`), Kiểm tra tĩnh (`mpu6050ValidateAccelStatic`), Set/Get Offsets.

### 6. `rc_crsf.h` (Bộ thu sóng điều khiển CRSF / ELRS)
Driver giao tiếp tay điều khiển giao thức CRSF qua UART2.
*   **Hằng số**: `CRSF_NUM_CHANNELS` (16), `CRSF_CHANNEL_MIN_US` (988), `CRSF_CHANNEL_MID_US` (1500), `CRSF_CHANNEL_MAX_US` (2012).
*   **Các hàm**: Khởi tạo (`crsfInit`), Cập nhật (`crsfUpdate`), Lấy giá trị kênh (`crsfGetChannel`), Kiểm tra kết nối (`crsfIsLinkActive`), Lấy LQ/RSSI, Gửi Telemetry pin (`crsfSendTelemetryBattery`).

### 7. `soft_i2c.h` (I2C bằng phần mềm - Bitbanging)
Định nghĩa bus I2C giả lập bằng phần mềm trên 2 chân GPIO (PA9, PA10).
*   **Hằng số**: `I2C_OK` (0), `I2C_ERROR` (1).
*   **Các hàm**: `softI2cInit()`, `softI2cSetSpeed()`, `softI2cBusRecovery()`, Đọc/Ghi 1 byte hoặc nhiều byte, Đọc/Ghi qua địa chỉ 16-bit, `softI2cScanAddress()`.

### 8. `soft_uart.h` (UART bằng phần mềm - Chỉ TX)
Driver giả lập cổng UART (TX) dùng để debug log ra màn hình.
*   **Macro**: `ENABLE_SOFT_UART` (1) để bật/tắt tính năng debug nhằm tối ưu code.
*   **Các hàm**: Khởi tạo (`softUartInit`), Hỗ trợ in ấn giống Arduino Serial: `softUartPrint`, `softUartPrintln`, `softUartPrintf`.

---

## Phần 2: Phân hệ MIDDLEWARE (`include/middleware`)
Đây là tầng trung gian, xử lý các thuật toán bay, PID, an toàn và ghi log.

### 1. `blackbox.h` (Lưu trữ dữ liệu bay - Hộp đen)
Quản lý bộ đệm vòng (Ring Buffer) trên RAM để ghi lại log dữ liệu bay.
*   **Hằng số**: `BLACKBOX_LIMIT` (256).
*   **Cấu trúc `BlackboxEntry`**: `loop_time_us`, `out_roll`, `out_pitch`, `out_yaw`, `vbat_mv`, `lq`, `rssi`.
*   **Các hàm**: `blackboxInit()`, `blackboxLog()`, `blackboxDumpSerial()` (xuất ra CSV), `blackboxReset()`.

### 2. `imu_estimator.h` (Bộ ước lượng góc tư thế)
Thuật toán kết hợp dữ liệu cảm biến để tính ra góc nghiêng thực tế ở tần số cao.
*   **Cấu trúc `Attitude`**: Góc `roll`, `pitch`, `yaw` (độ).
*   **Các hàm**: `imuEstimatorInit()`, `imuEstimatorUpdate(p_imu, dt)`, `imuEstimatorGetAttitude()`.

### 3. `logging.h` (Quản lý in ấn và Debug)
Middleware xử lý việc in log ra màn hình thông qua cổng Soft_UART.
*   **Các hàm**: `loggingInit()`, `loggingUpdate(...)` (in trạng thái các kênh và IMU), `loggingGetMotorCommand(...)` (hỗ trợ test động cơ).

### 4. `motor_mixer.h` (Bộ trộn tín hiệu Động cơ - Mixer)
Thuật toán trộn (Mixer) cho cấu hình Quad-X.
*   **Hàm chính**: `motorMixerCompute(throttle, roll, pitch, yaw, &m1, &m2, &m3, &m4)`. Kết hợp ga và PID để tạo xung PWM cho 4 motor.

### 5. `pid_controller.h` (Bộ điều khiển tự động PID)
Thuật toán PID vòng kép (Angle Loop + Rate Loop).
*   **Hằng số**: `AXIS_ROLL` (0), `AXIS_PITCH` (1), `AXIS_YAW` (2).
*   **Cấu trúc `PidGains`**: Hệ số `kp`, `ki`, `kd`.
*   **Các hàm**: Khởi tạo (`pidInit`), `pidSetGains` / `pidGetGains`, `pidReset()` (tránh wind-up khi disarm), hàm cốt lõi `pidCompute(...)` để tính toán lệnh bù lực.

### 6. `safety.h` (Hệ thống an toàn, Failsafe & ARM)
Quản lý trạng thái an toàn, quy trình khởi động, khóa/mở máy và sự cố (Failsafe).
*   **Enum `FlightState`**: `STATE_DISARMED`, `STATE_PRE_ARM`, `STATE_ARMED`, `STATE_FAILSAFE`.
*   **Enum `StartupState` & `StartupError`**: Quản lý chuỗi boot (Lỗi MPU, Lỗi RC, Bị lệch tâm...) với các mã nháy đèn cụ thể (1-7 nhịp).
*   **Các hàm**: Khởi tạo (`safetyInit`), Kiểm tra/Feed Watchdog (`safetyUpdate`, `safetyFeedWatchdog`), Lệnh khóa/mở máy (`safetyRequestArm`, `safetyRequestDisarm`).

### 7. `test.h` (Module Test Động cơ)
Các tiện ích để test độc lập động cơ (khi bật `TEST_MOTOR_MODE`).
*   **Các hàm**: `setThrottlePercent()`, `rampTo()`, `checkEmergencyStop()` (lệnh STOP khẩn cấp), `runTestMode()`.
