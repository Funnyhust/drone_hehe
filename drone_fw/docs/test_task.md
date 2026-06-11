# Danh sách Nhiệm vụ Kiểm thử Hệ thống (Drone Test Tasks Checklist)

Dưới đây là danh sách các bài kiểm tra tích hợp và tính năng an toàn quan trọng cần thực hiện trước khi cho drone cất cánh thực tế. Bạn đã kiểm thử độc lập thành công **Motor PWM, ELRS và MPU6050**. Dưới đây là các phần còn lại cần kiểm thử.

---

## 📋 Checklist Kiểm thử (Test Tasks)

### [ ] 1. Kiểm thử Giám sát Pin (Battery ADC Test)
*   [ ] **Độ chính xác phép đo:** 
    *   Cắm pin vào drone, dùng đồng hồ vạn năng (VOM) đo trực tiếp điện áp pin tại đầu giắc XT60.
    *   Kết nối Serial Debug (115200 baud) hoặc Software UART (19200 baud), xem giá trị điện áp báo về trên màn hình log.
    *   *Tiêu chí Đạt:* Độ lệch điện áp giữa VOM và giá trị hiển thị trên log $\le \pm 0.1\text{V}$.
*   [ ] **Cảnh báo Pin yếu (Low Battery Warning):**
    *   Giả lập sụt áp pin xuống dưới **10.5V** (bằng cách dùng pin cũ hoặc nguồn DC chỉnh dòng).
    *   *Tiêu chí Đạt:* Đèn LED cảnh báo nhấp nháy liên tục (hoặc còi kêu báo động). Khi đẩy cần ga lên tối đa, xung PWM xuất ra 4 động cơ bị giới hạn cứng ở mức **1600µs** để bảo vệ pin.
*   [ ] **Khóa an toàn Pin nguy kịch (Critical Battery Lock):**
    *   Giả lập sụt áp pin xuống dưới **9.9V**.
    *   *Tiêu chí Đạt:* Hệ thống khóa hoàn toàn, không cho phép thực hiện hành động **ARM** (mở khóa động cơ) để ngăn chặn cất cánh khi pin không đủ an toàn.

---

### [ ] 2. Kiểm thử Tính năng An toàn Mất sóng (RC Failsafe Test)
*   > [!CAUTION]
    > **ĐẢM BẢO ĐÃ THÁO CÁNH QUẠT TRƯỚC KHI THỰC HIỆN BÀI TEST NÀY!**
*   [ ] **Thời gian phản hồi Failsafe:**
    *   Bật tay phát, thực hiện **ARM** drone trên tay phát.
    *   Đẩy nhẹ cần ga lên để 4 động cơ quay đều ở tốc độ thấp.
    *   Tắt nguồn tay phát đột ngột để giả lập mất sóng RF.
    *   *Tiêu chí Đạt:* 
        *   Trong vòng tối đa **200ms** kể từ khi tắt tay phát, toàn bộ 4 động cơ phải dừng quay hoàn toàn lập tức.
        *   Trên Serial Debug xuất hiện dòng log thông báo hệ thống chuyển sang trạng thái an toàn `FAILSAFE` và tự động khóa động cơ.
*   [ ] **Khôi phục sau Failsafe (Recovery):**
    *   Bật lại tay phát.
    *   Kéo cần ga về vị trí thấp nhất, gạt công tắc ARM về vị trí DISARM rồi ARM lại.
    *   *Tiêu chí Đạt:* Hệ thống mở khóa thành công và động cơ quay trở lại bình thường.

---

### [ ] 3. Kiểm thử Tiện ích Hệ thống (Blackbox & I2C Scanner)
*   [ ] **Kiểm tra EEPROM 24LC256:**
    *   Gửi ký tự `'s'` (I2C Scanner) qua cổng Serial CLI.
    *   *Tiêu chí Đạt:* Quét ra chính xác địa chỉ cảm biến **`0x68`** (MPU6050) và địa chỉ bộ nhớ EEPROM **`0x50`**.
*   [ ] **Kiểm tra ghi và trích xuất Blackbox log:**
    *   Cho drone chạy test động cơ hoặc ARM xoay nhẹ trong vòng 10 giây.
    *   Chuyển về trạng thái **DISARMED**.
    *   Gửi ký tự `'b'` qua cổng Serial CLI.
    *   *Tiêu chí Đạt:* Màn hình dump ra đúng chuỗi dữ liệu nhị phân hoặc bảng log chi tiết ghi lại toàn bộ quá trình hoạt động (Loop time, góc Roll/Pitch/Yaw, đầu ra PID, Vbat, RSSI, LQ).

---

### [ ] 4. Kiểm thử Hồi tiếp Cân bằng PID (PID Feedback Test ở chế độ PRE_FLIGHT_TEST)
*   [ ] **Kiểm tra hướng phản hồi của PID:**
    *   Bật chế độ `#define PRE_FLIGHT_TEST 1` trong [config.h](file:///d:/Duong/Drone/drone_hehe/drone_fw/include/config.h) (để giới hạn xung ga an toàn tối đa 1180us, chỉ quay motor nhẹ trên bàn).
    *   Thực hiện **ARM** drone và đẩy ga lên khoảng 10-20% để cả 4 motor quay đều ổn định.
    *   Cầm drone lên tay, nghiêng drone sang các hướng và quan sát tốc độ motor thay đổi để tự cân bằng:
        *   *Nghiêng sang Phải:* 2 motor bên phải (M1, M2) phải quay nhanh hơn (hoặc chậm hơn tùy thiết kế cánh) để tạo lực đẩy nâng cánh phải lên cân bằng lại.
        *   *Nghiêng sang Trái:* 2 motor bên trái (M3, M4) tăng tốc để nâng cánh trái lên.
        *   *Chúc mũi về phía trước:* 2 motor phía trước (M1, M4) tăng tốc để đẩy mũi lên.
        *   *Ngửa đuôi về phía sau:* 2 motor phía sau (M2, M3) tăng tốc để đẩy đuôi lên.
    *   *Tiêu chí Đạt:* Các động cơ tăng/giảm tốc độ đúng hướng phản hồi bù lại lực nghiêng của tay bạn. Nếu động cơ phản ứng ngược lại (ví dụ nghiêng phải mà motor phải lại giảm tốc làm drone nghiêng sâu hơn) $\to$ **PID bị ngược chiều hệ số P, I, D**.
