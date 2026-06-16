# Tài Liệu Hướng Dẫn Chi Tiết Thuật Toán Điều Khiển PID Vòng Kép (Cascade PID) cho Drone

Tài liệu này tổng hợp toàn bộ cấu trúc thuật toán điều khiển PID đang được triển khai trong firmware drone của bạn (dựa trên mã nguồn tại [pid_controller.cpp](file:///d:/Duong/Drone/drone_hehe/drone_fw/src/middleware/pid_controller.cpp)), giải thích chi tiết ý nghĩa toán học, cơ chế vật lý và cách thiết lập thông số để tránh các lỗi phần cứng nguy hiểm.

---

## 1. Kiến Trúc Bộ Điều Khiển PID Vòng Kép (Cascade PID)

Drone của bạn sử dụng cấu trúc **PID vòng kép (Cascade PID)** cho hai trục **Roll** và **Pitch**. Trục **Yaw** chỉ sử dụng **PID vòng đơn (Rate Loop)**.

```mermaid
graph TD
    subgraph Vòng Ngoài - Angle Loop (Góc)
        TargetAngle[Góc đặt - Target Angle] --> SubAngle[Sai số Góc]
        CurrentAngle[Góc thực tế - IMU] --> SubAngle
        SubAngle --> KpAngle[Kp Angle]
        KpAngle --> TargetRate[Tốc độ góc đặt - Target Rate]
    end

    subgraph Vòng Trong - Rate Loop (Tốc độ góc)
        TargetRate --> SubRate[Sai số Tốc độ góc]
        CurrentGyro[Tốc độ góc thực tế - Gyro] --> SubRate
        SubRate --> PIDRate[Bộ tính PID Rate]
        PIDRate --> MotorOut[Lực điều khiển Motor]
    end
```

### A. Vòng Ngoài: Điều khiển góc (Angle / Outer Loop)
* **Mục tiêu**: Đưa góc nghiêng thực tế của drone về đúng góc nghiêng mong muốn từ cần gạt (đơn vị: Độ - `deg`).
* **Đầu vào**: Sai số góc giữa góc đặt (`target_angle`) và góc nghiêng thực tế (`current_angle` ước lượng từ bộ lọc IMU).
* **Đầu ra**: Tốc độ xoay góc mong muốn (đơn vị: Độ/giây - `deg/s`).
* **Công thức**:
  $$\text{Target Rate} = Kp_{\text{angle}} \times (\text{Target Angle} - \text{Current Angle})$$
  *(Tốc độ góc đặt được giới hạn trong dải $[-250.0, 250.0] \text{ deg/s}$ để đảm bảo drone không tự xoay quá nhanh gây mất kiểm soát).*

### B. Vòng Trong: Điều khiển tốc độ góc (Rate / Inner Loop)
* **Mục tiêu**: Đưa tốc độ xoay thực tế đo từ con quay hồi chuyển (Gyroscope) về đúng tốc độ xoay mong muốn được tính từ vòng ngoài.
* **Đầu vào**: Sai số tốc độ góc giữa tốc độ đặt (`target_rate`) và tốc độ quay thực tế đo trực tiếp từ cảm biến Gyro (`current_gyro`).
* **Đầu ra**: Tín hiệu xung PWM điều chỉnh công suất đưa tới mạch trộn động cơ (Mixer) để tăng/giảm tốc độ quay của từng motor.
* **Công thức**:
  $$\text{Output} = P_{\text{term}} + I_{\text{term}} + D_{\text{term}}$$

---

## 2. Chi Tiết Thuật Toán Tính Toán PID Trên Từng Trục

Dưới đây là chi tiết mã nguồn tính toán thực tế trên các trục điều khiển trong vi điều khiển:

### Trục Roll và Pitch (Vòng kép - Cascade)
1. **Tính sai số góc (Vòng ngoài)**:
   ```cpp
   float error_angle_roll = target_roll_deg - current_att->roll;
   float target_rate_roll = angle_gains[AXIS_ROLL].kp * error_angle_roll;
   
   // Giới hạn an toàn tốc độ góc
   if (target_rate_roll > 250.0f) target_rate_roll = 250.0f;
   else if (target_rate_roll < -250.0f) target_rate_roll = -250.0f;
   ```

2. **Tính thành phần tỷ lệ P-Term (Vòng trong)**:
   Phản ứng ngay lập tức với sai số tốc độ góc hiện tại.
   ```cpp
   float error_rate_roll = target_rate_roll - current_imu->gx;
   float p_term_roll = rate_gains[AXIS_ROLL].kp * error_rate_roll;
   ```

3. **Tính thành phần tích phân I-Term & Khử bão hòa (Anti-windup)**:
   Tích lũy sai số theo thời gian để bù đắp các lực lệch tĩnh (trọng tâm lệch, gió thổi liên tục). Giới hạn I-term ngăn chặn việc bộ tích phân tự tăng kịch trần khi drone bị kẹt.
   ```cpp
   i_mem_roll += rate_gains[AXIS_ROLL].ki * error_rate_roll * dt;
   if (i_mem_roll > 250.0f) i_mem_roll = 250.0f;
   else if (i_mem_roll < -250.0f) i_mem_roll = -250.0f;
   ```

4. **Tính thành phần vi phân D-Term & Bộ lọc thông thấp (IIR Low-pass Filter)**:
   Dự báo hướng di chuyển để giảm chấn dao động. Do vi phân nhạy cảm với nhiễu tần số cao của động cơ truyền qua khung sườn đến IMU, thuật toán bắt buộc phải dùng bộ lọc IIR thông thấp với hệ số lọc $\alpha = 0.2$ để làm mịn tín hiệu D-term trước khi xuất ra motor:
   ```cpp
   // Tính vi phân thô
   float d_raw_roll = rate_gains[AXIS_ROLL].kd * (error_rate_roll - last_error_roll) / dt;
   
   // Lọc thông thấp IIR (tần số cắt khoảng 15-20Hz)
   d_filtered_roll = d_filtered_roll + Alpha_D * (d_raw_roll - d_filtered_roll);
   ```

5. **Tổng hợp đầu ra điều khiển**:
   ```cpp
   *out_roll = p_term_roll + i_mem_roll + d_filtered_roll;
   last_error_roll = error_rate_roll; // Lưu lại sai số cho chu kỳ sau
   ```

### Trục Yaw (Chỉ dùng Vòng đơn - Rate Loop)
Vì trục Yaw không cần điều khiển giữ góc nghiêng cố định mà chỉ cần xoay đầu theo lệnh của phi công, trục này bỏ qua vòng ngoài (Angle Loop) và đưa thẳng lệnh gạt cần tốc độ xoay đầu vào vòng trong (Rate Loop):
```cpp
float error_rate_yaw = target_yaw_rate - current_imu->gz;
float p_term_yaw = rate_gains[AXIS_YAW].kp * error_rate_yaw;

i_mem_yaw += rate_gains[AXIS_YAW].ki * error_rate_yaw * dt;
if (i_mem_yaw > 150.0f) i_mem_yaw = 150.0f;
else if (i_mem_yaw < -150.0f) i_mem_yaw = -150.0f;

float d_raw_yaw = rate_gains[AXIS_YAW].kd * (error_rate_yaw - last_error_yaw) / dt;
d_filtered_yaw = d_filtered_yaw + Alpha_D * (d_raw_yaw - d_filtered_yaw);

*out_yaw = p_term_yaw + i_mem_yaw + d_filtered_yaw;
last_error_yaw = error_rate_yaw;
```

---

## 3. Ý Nghĩa Vật Lý Và Ảnh Hưởng Của Các Hệ Số

| Hệ số | Vai trò vật lý | Hiện tượng khi đặt QUÁ THẤP | Hiện tượng khi đặt QUÁ CAO |
| :---: | :--- | :--- | :--- |
| **Kp**<br>*(Tỷ lệ)* | Tạo lực phản ứng ngay lập tức để bẻ drone về trạng thái cân bằng. | Drone phản hồi lờ đờ, trôi dạt góc nghiêng, có cảm giác "nhão" tay lái. | Drone dao động liên tục ở tần số thấp (overshoot, tự lắc nhanh qua lại). |
| **Ki**<br>*(Tích phân)* | Tích lũy sai số qua thời gian để bù đắp các lỗi lệch trọng tâm (do pin lệch) hoặc kháng gió cố định. | Drone từ từ bị trôi dạt hướng bay hoặc lệch góc tự cân bằng sau một khoảng thời gian bay ngắn. | Drone tự xuất hiện dao động rất chậm, biên độ lớn (lững lờ trồi sụt). |
| **Kd**<br>*(Vi phân)* | "Phanh giảm chấn" hoạt động ngược chiều với tốc độ dao động để hãm quán tính quay. | Drone bị nẩy ngược góc khi dừng gạt cần nhanh, dao động tự do sau khi lắc mạnh. | **NGUY HIỂM**: Động cơ giật cục dữ dội (jitter), nóng ran động cơ nhanh chóng, có thể làm cháy ESC và gây nhiễu loạn mạch lái. |

---

## 4. Tầm Quan Trọng Của Chu Kỳ Vòng Lặp Chính (`dt`)
Chu kỳ tính toán vi phân và tích phân phụ thuộc hoàn toàn vào biến thời gian thực thi giữa hai vòng lặp điều khiển liên tiếp (`dt`):
* Trong thuật toán vi phân: $$D_{\text{term}} \propto \frac{1}{dt}$$
* Trong thuật toán tích phân: $$I_{\text{term}} \propto dt$$

> [!IMPORTANT]
> Nếu hệ thống xảy ra hiện tượng trễ chu kỳ (Loop budget overrun) do vi điều khiển bận làm các tác vụ khác (như Software UART giả lập bằng hàm delay), giá trị `dt` thực tế sẽ tăng đột biến từ $2.0\text{ms}$ lên đến $5.0 - 10.0\text{ms}$. Sự bất ổn định này sẽ phá vỡ hoàn toàn toán học của bộ PID, làm các phép nhân/chia với `dt` ra kết quả sai lệch khổng lồ, khiến các động cơ giật loạn xạ không kiểm soát.

---

## 5. Hướng Dẫn Cân Chỉnh PID (Tuning Workflow) Cho Người Mới
Để cân chỉnh drone an toàn, bạn nên tuân thủ quy trình sau:

1. **Khởi động với PID lý tưởng mặc định**:
   Sử dụng bộ số chuẩn:
   * **Angle Kp**: `4.5`
   * **Rate Roll/Pitch**: Kp=`3.5`, Ki=`2.5`, Kd=`0.05`
   * **Rate Yaw**: Kp=`4.0`, Ki=`1.5`, Kd=`0.02`
2. **Cân chỉnh Kp trước**: 
   Tăng nhẹ Kp lên dần cho đến khi drone phản ứng nhanh nhạy và đứng yên đầm chắc. Nếu tăng quá đà, drone sẽ bắt đầu dao động nhanh tự phát $\rightarrow$ Giảm Kp đi 20-30% so với điểm bắt đầu dao động.
3. **Cân chỉnh Kd để giảm chấn**:
   Nếu drone dừng đột ngột bị nẩy nhẹ góc $\rightarrow$ Tăng nhẹ Kd (mỗi lần tăng rất nhỏ, khoảng `0.01`). Luôn sờ thử động cơ sau 30 giây bay thử, nếu động cơ ấm hoặc nóng ran thì phải hạ Kd ngay lập tức.
4. **Cân chỉnh Ki**:
   Nếu drone bay tiến/lùi bị trôi góc nghiêng không giữ vững đường bay thẳng $\rightarrow$ Tăng nhẹ Ki (mỗi lần tăng `0.2`).
