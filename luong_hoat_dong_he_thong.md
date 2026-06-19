# LUỒNG HOẠT ĐỘNG HỆ THỐNG CHÍNH — TÀI LIỆU HỌC TẬP
## Tổng hợp từ Q&A — Phiên 19-20/06/2026

---

# GIAI ĐOẠN 1 — KHỞI ĐỘNG HỆ THỐNG (`he_thong_chinh.py`)

Toàn bộ hệ thống được khởi động từ hàm `main()` trong file `he_thong_chinh.py`. Hàm này lần lượt gọi ra các thành phần sau:

**Bước 1 — Xác định thư mục gốc và tạo thư mục đầu ra:**
`he_thong_chinh.py` gọi hàm `ensure_dirs()` từ `utils.py` (dòng 29–40) để tạo sẵn toàn bộ thư mục lưu trữ đầu ra nếu chúng chưa tồn tại:
- `logs/` — lưu file CSV sự kiện và file lịch sử text
- `canh_bao_full/` — lưu ảnh toàn frame khi có cảnh báo
- `canh_bao_crop/` — lưu ảnh cắt phóng to vùng target
- `canh_bao_video/` — lưu video replay
- `reports/` — lưu báo cáo HTML
- `config/` — lưu cấu hình ROI

**Bước 2 — Khởi động Camera:**
`he_thong_chinh.py` gọi hàm `open_camera()` (dòng 44–52) để khởi động webcam. Hàm này dùng `cv2.VideoCapture()` để mở camera theo `CAMERA_INDEX = 0` được cấu hình trong `config.py`. Sau khi mở thành công, nó cài đặt tốc độ khung hình là `CAMERA_FPS = 30` khung/giây. Nếu không mở được camera, hệ thống sẽ tạo ra một khung hình giả (SMPTE Color Bars màu cầu vồng + chữ "NO CAMERA SIGNAL") để vẫn chạy được — nhờ cờ `FALLBACK_SYNTHETIC_FRAME = True` trong `config.py`.

**Bước 3 — Nạp 3 mô hình AI:**
`he_thong_chinh.py` khởi tạo đối tượng `Detector(root)` từ `detector.py`. Khi khởi tạo, `Detector.__init__()` tự động gọi `_load_models()` (dòng 58–98) để nạp lần lượt 3 mô hình YOLO vào bộ nhớ GPU:
- `yolov8n-pose.pt` — mô hình gốc pretrained của Ultralytics, chuyên phát hiện người và 17 điểm khớp xương
- `Model/Posture/weights/best.pt` — mô hình tự train, chuyên phân loại tư thế Đứng/Ngồi/Nằm (3 class)
- `Model/Rescue_v4/weights/best.pt` (hoặc bản mới nhất) — mô hình tự train, chuyên phân loại Normal/Victim (2 class)

**Bước 4 — Tải cấu hình vùng ROI:**
`he_thong_chinh.py` gọi `load_roi_config()` từ `roi_manager.py` (dòng 46–67). Hàm này đọc file `config/roi_config.json`. Nếu file tồn tại và hợp lệ (có ít nhất 3 điểm đỉnh), nó tải lại vùng quan sát nguy hiểm từ lần trước. Nếu không có file, ROI được đặt về trạng thái trống, chờ người vận hành vẽ mới.

**Bước 5 — Khởi tạo các thành phần phụ trợ:**
- `EventLogger(paths)` từ `event_logger.py` — quản lý ghi log sự kiện ra CSV và hiển thị thông báo
- `EventReplayBuffer(paths)` từ `event_logger.py` — khởi tạo bộ đệm video trong RAM, mở thread ghi video chạy ngầm
- `ReportGenerator(paths, root)` từ `report_generator.py` — chuẩn bị sẵn bộ tạo báo cáo HTML
- `WeatherProvider()` từ `weather_telemetry.py` — bắt đầu thread lấy dữ liệu thời tiết từ Open-Meteo API (chạy nền, cập nhật mỗi 10 phút)
- `OverlayRenderer()` từ `renderer.py` — chuẩn bị bộ vẽ giao diện
- `TabHoldController()` từ `ui_controller.py` — chuẩn bị xử lý phím Tab

---

# GIAI ĐOẠN 2 — VÒNG LẶP CHÍNH (`he_thong_chinh.py`)

Sau khi khởi động xong, hệ thống bước vào vòng lặp `while True` chạy liên tục. **Mỗi một vòng lặp = xử lý 1 khung hình.**

## 2.1 — Đọc Frame từ Camera

`he_thong_chinh.py` gọi `cap.read()` (dòng 198). Đây KHÔNG phải là "cắt 30 ảnh rồi lấy ra" — mà là **đọc đúng 1 khung hình tại thời điểm đó** từ luồng video của camera, rồi vòng lặp quay lại và đọc tiếp 1 khung mới. Tốc độ camera là 30 khung/giây nên mỗi giây vòng lặp sẽ đọc được khoảng 30 khung.

Sau khi đọc frame, hệ thống dùng `cv2.resize()` (dòng 204) để chuẩn hóa kích thước về đúng `1280 × 720 pixel` theo cấu hình `CAMERA_WIDTH` và `CAMERA_HEIGHT` trong `config.py` (dòng 24–25).

## 2.2 — Lưu Frame vào Bộ Đệm Video (Replay Buffer)

Ngay sau khi đọc frame, `he_thong_chinh.py` gọi `replay_buffer.add_frame(frame, now)` (dòng 211). Đây là cơ chế **lưu frame vào RAM** (không phải ổ cứng) thông qua class `EventReplayBuffer` trong `event_logger.py`.

Cơ chế hoạt động: buffer duy trì một hàng đợi vòng (deque) trong RAM. Nó không lưu mọi frame mà chỉ snapshot theo tần suất `REPLAY_FPS = 20` lần/giây (tức cứ 50ms lấy 1 frame). Các frame cũ hơn `REPLAY_PRE_SECONDS = 10 giây` sẽ tự động bị xóa khỏi deque để không chiếm hết RAM. Kết quả là deque luôn giữ khoảng **10 giây frame gần nhất** trong bộ nhớ.

**Thời lượng video replay thực tế:** Khi có sự kiện nguy hiểm, hệ thống lấy 10 giây frame đã lưu trong deque (phần "trước"), rồi tiếp tục thu thêm `REPLAY_POST_SECONDS = 10 giây` frame (phần "sau"). Tổng cộng là **~20 giây video thực tế** xuất ra file `.mp4`. Không có chuyện "x2 nên còn 10 giây" — `REPLAY_FPS = 20` chỉ là tốc độ lấy mẫu và phát lại, không phải hệ số tăng tốc.

## 2.3 — Phát Hiện (Detection)

`he_thong_chinh.py` gọi `detector.detect(frame, now)` (dòng 215), truyền 1 frame ảnh vào `detector.py` để xử lý qua 3 mô hình AI.

---

# GIAI ĐOẠN 3 — XỬ LÝ PHÁT HIỆN (`detector.py`)

Hàm điều phối chính là `_detect_pose_rescue()` (dòng 116). Hàm này KHÔNG tự phát hiện gì mà đóng vai trò **điều phối**, gọi lần lượt 3 hàm con và tổng hợp kết quả.

## 3.1 — Ba Luồng Phát Hiện Song Song

Với mỗi frame, cả 3 model được gọi chạy:

**Luồng 1 — `_detect_pose_boxes()` (dòng 181):** Dùng `model_pose` (yolov8n-pose.pt) để phát hiện tất cả người trong frame. Mỗi người được đóng khung bounding box và lấy ra **17 tọa độ điểm khớp xương** (keypoints) gồm mũi, vai, khuỷu tay, cổ tay, hông, đầu gối, cổ chân... kèm theo độ tin cậy của từng điểm. Model này còn dùng ByteTrack để gán ID ổn định cho từng người qua các frame.

**Luồng 2 — `_detect_rescue_boxes()` (dòng 297):** Dùng `model_rescue` để phát hiện người và phân loại thành `normal_person` hoặc `victim`. Model này trả về bounding box + class + confidence cho mỗi người nó nhìn thấy.

**Luồng 3 — `_detect_posture_boxes()` (dòng 245):** Dùng `model_posture` để phát hiện người và phân loại tư thế thành `Stand`, `Sit`, hoặc `Lie`. Tương tự, trả về bounding box + class + confidence.

## 3.2 — Phân Tích Hình Học Keypoints: `_analyze_person_state()` (dòng 518)

Với **mỗi người** tìm được từ Pose model, hàm `_analyze_person_state()` **LUÔN LUÔN được gọi** — không phụ thuộc vào việc Rescue hay Posture có tìm thấy người đó không. Đây KHÔNG phải hàm backup mà là **nguồn dữ liệu song song độc lập** luôn chạy cùng lúc.

Hàm này dùng **tính toán hình học thuần túy** (không dùng AI) trên 17 tọa độ keypoint để phán đoán:
- Người có đang **nằm/ngã** không? → So sánh tỉ lệ chiều rộng/chiều cao bounding box, khoảng cách giữa vai-hông, vị trí xương sống
- Người có đang **giơ tay** không? → So sánh tọa độ Y của cổ tay với tọa độ Y của vai (cổ tay cao hơn vai một ngưỡng `HAND_RAISE_MARGIN_PIXELS = 40px`)
- Người có đang **vẫy tay** không? → Theo dõi lịch sử tọa độ cổ tay trong 2 giây, đếm số lần đổi chiều (≥ 3 lần đổi + biên độ ≥ 60px → xác nhận vẫy)
- Người có **bất động lâu** không? → Theo dõi lịch sử tọa độ trung tâm trong `STILLNESS_SECONDS = 20 giây`
- Người có **gục đầu/ngã nghiêng** không? → Phân tích góc của cột sống so với trục đứng

Kết quả hàm này trả về một dictionary chứa các cờ boolean (`is_lying`, `is_collapsed`, `is_immobile`, `is_waving`...) và trạng thái cử chỉ (`gesture_candidate`).

## 3.3 — Khớp Khung: `_best_match()` (dòng 504)

Vì 3 model phát hiện độc lập nhau và mỗi model có bounding box riêng, hệ thống cần biết: "Cái box của Rescue model kia có đang nói về CÙNG 1 người với box của Pose model này không?"

`_best_match()` dùng **chỉ số IoU (Intersection over Union)** để đo mức độ chồng lấp giữa box Pose và box Rescue/Posture. Nếu IoU ≥ `RESCUE_IOU_THRESHOLD = 0.20` (20% diện tích chồng nhau) → hai box được coi là cùng 1 người → lấy kết quả kết hợp. Nếu IoU < 0.20 → không khớp → bỏ qua, coi như Rescue/Posture không tìm thấy người đó.

## 3.4 — Dung Hợp Tư Thế: `_fuse_posture()` (dòng 354)

Hàm này kết hợp **2 nguồn** để ra 1 kết luận tư thế duy nhất:
- **Nguồn 1:** Kết quả từ YOLO-Posture model (nếu `_best_match` tìm được khớp) — độ tin cậy 0.0–1.0
- **Nguồn 2:** Kết quả phân tích keypoint từ `_analyze_person_state()` — trạng thái standing/sitting/lying_or_fallen/unknown

Logic ưu tiên:
- Nếu YOLO-Posture tìm được khớp **và** confidence ≥ 0.55 → tin ngay, trả về kết quả của model Posture luôn
- Ngược lại → cộng điểm tích lũy cho 3 trạng thái từ cả 2 nguồn (theo trọng số `POSTURE_MODEL_WEIGHT = 0.85` cho model, `POSE_POSTURE_WEIGHT = 0.15` cho keypoint), chọn trạng thái nào có điểm cao nhất nếu vượt ngưỡng tin cậy tối thiểu

## 3.5 — Dung Hợp Xác Nhận Nạn Nhân: `_fuse_rescue()` (dòng 469)

Đây là bước cuối cùng quyết định người này là **"victim"** hay **"normal_person"**. Hệ thống dùng **công thức xác suất tích lũy độc lập (Probability Union)**:

```
P(Victim) = 1 − (1 − P_rescue) × (1 − 0.78 × P_posture) × (1 − 0.65 × P_pose)
```

Trong đó:
- `P_rescue` = confidence từ YOLO-Rescue model (chỉ tính nếu nó phân loại là victim, nếu là normal_person thì = 0)
- `P_posture` = điểm fusion tư thế / POSTURE_MODEL_WEIGHT — **chỉ tính nếu tư thế = lying_or_fallen**, tư thế khác = 0
- `P_pose` = bằng chứng từ keypoints: nằm (0.72), gục/ngã (0.82), bất động+nằm (0.90), vẫy 2 tay (0.90)

Nếu `P(Victim) ≥ VICTIM_FUSION_THRESHOLD = 0.85` → `class_name = "victim"`. Ngược lại → `class_name = "normal_person"`.

**Lưu ý quan trọng:** `class_name` từ hàm này và `Danger Score` từ `target_manager.py` là **2 hệ thống hoàn toàn độc lập** phục vụ 2 mục đích khác nhau. `victim_probability` quyết định **nhãn hiển thị và điều kiện auto-save**; Danger Score quyết định **mức độ cảnh báo NORMAL/WARNING/DANGER**. Cả hai phải cùng vượt ngưỡng mới kích hoạt lưu sự kiện tự động.

Ví dụ: người nằm ngủ nhưng YOLO-Rescue nhận ra "normal_person" với confidence cao → P(Victim) thấp → class_name = "normal_person" → auto-save không kích hoạt dù Danger Score có thể đạt 55 (WARNING). Đây là cơ chế lọc False Positive quan trọng.

## 3.6 — Đóng Gói Kết Quả

Toàn bộ thông tin của 1 người sau khi xử lý xong được đóng gói vào `RawDetection` (models.py dòng 25–30):
- `object_id` — ID tracking
- `bbox` — tọa độ bounding box (x1, y1, x2, y2)
- `class_name` — "victim" hoặc "normal_person"
- `confidence` — độ tin cậy tổng hợp
- `analysis` — dictionary chứa toàn bộ thông tin chi tiết: keypoints, pose_flags, posture_status, gesture_candidate, victim_probability...

`detector.detect()` trả về một **list[RawDetection]** — danh sách tất cả người được phát hiện trong frame đó.

---

# GIAI ĐOẠN 4 — QUẢN LÝ TARGET VÀ TÍNH ĐIỂM (`target_manager.py`)

`he_thong_chinh.py` gọi `manager.update(detections, frame, now)` (dòng 225). Hàm `update()` trong `TargetManager` làm 6 việc theo thứ tự:

## 4.1 — Gán/Tìm ID và Tạo Hồ Sơ

`TargetState` (models.py dòng 72–141) là "hồ sơ bệnh nhân" của mỗi người được theo dõi — nó **tồn tại liên tục trong RAM** suốt cả session, được cập nhật mỗi frame chứ không tạo mới mỗi frame. `target_manager` dùng `self.targets: dict[int, TargetState]` để lưu tất cả hồ sơ theo object_id.

Nếu người này đã có hồ sơ (cùng ID tracker) → lấy hồ sơ cũ ra cập nhật. Nếu lần đầu xuất hiện → tạo `TargetState` mới, lưu vào dict.

## 4.2 — Cập Nhật Thông Tin Thô

Copy trực tiếp từ `RawDetection` vào `TargetState`: bbox, class_name, confidence, keypoints, pose_flags (is_lying, is_collapsed, is_waving, is_immobile...).

## 4.3 — Làm Mịn Class (Chống Class Nhảy Loạn)

Hệ thống không tin ngay kết quả của 1 frame duy nhất. Mỗi frame, class_name mới được đẩy vào `class_history` (deque lưu 30 frame gần nhất). `stable_class` = class xuất hiện nhiều nhất trong 30 frame đó. Sau khi tích lũy đủ ≥ 3 frame, `target.class_name` mới được cập nhật từ `stable_class`. Cơ chế này tránh tình trạng label nhảy "victim → normal → victim" liên tục do nhiễu model.

## 4.4 — Phân Tích Chuyển Động, ROI, Cử Chỉ

- **`_analyze_motion()`:** Theo dõi lịch sử tọa độ trung tâm bounding box trong cửa sổ `MOTION_WINDOW_SECONDS = 3 giây`. Nếu di chuyển > `MOVING_THRESHOLD = 80px` → "moving"; 30–80px → "slight_motion"; < 30px → "stable"; bất động ≥ `LONG_IMMOBILE_SECONDS = 15 giây` trong khi có dấu hiệu nguy hiểm → "dangerous_motionless"
- **ROI check:** Dùng thuật toán Ray Casting kiểm tra tọa độ trung tâm target có nằm trong polygon ROI không → cập nhật `roi_status`
- **`_update_gesture_status()`:** Xác nhận cử chỉ phải duy trì ≥ `HAND_RAISE_HOLD_SECONDS = 1 giây` liên tục mới được xác nhận. Sau khi cử chỉ biến mất, giữ hiển thị thêm `GESTURE_HOLD_AFTER_DETECT_SECONDS = 2 giây`

## 4.5 — Tính Điểm Nguy Hiểm: `_score_target()` (dòng 454)

Đây là bước cốt lõi nhất của `target_manager`. Điểm được tính theo công thức cộng tích lũy:

```
Danger Score = YOLO_Score + Posture_Score + Motion_Score + Gesture_Score + ROI_Score
```

| Thành phần | Điều kiện | Điểm |
|---|---|---|
| YOLO_Score | Victim + posture = lying_or_fallen | +25 (Pose boost) |
| | Victim + vẫy 2 tay | +25 (Pose boost) |
| | Victim + giơ 2 tay | +20 (Pose boost) |
| | Victim + gục/ngã | +15 (Pose boost) |
| Posture_Score | Tư thế = lying_or_fallen | +30 |
| Motion_Score | Bất động nguy hiểm (≥15s) | +20 |
| | Ít chuyển động đáng ngờ | +5 |
| Gesture_Score | Giơ 1 tay | +15 |
| | Giơ 2 tay | +30 |
| | Vẫy 2 tay rõ | +50 |
| ROI_Score | Trong vùng giám sát | +10~30 (tối đa 30) |

**Điều chỉnh đặc biệt:** Nằm/ngã + vẫy 2 tay → Score tối thiểu 75. Người bình thường trong ROI không có dấu hiệu nguy hiểm → Score bị giới hạn ≤ 20.

**Ngưỡng phân loại:**
- 0 – 54: `NORMAL` (xanh lá)
- 55 – 84: `WARNING` (vàng) — `WARNING_THRESHOLD = 55`
- ≥ 85: `DANGER` (đỏ) — `DANGER_THRESHOLD = 85`

## 4.6 — Chụp Ảnh Crop Luôn Tại Chỗ

Mỗi frame, `target_manager` gọi `target.crop_from(frame)` để cắt vùng ảnh của target với padding 8px 4 phía, lưu vào `target.latest_crop`. Điều này để khi cần lưu sự kiện khỏi phải cắt lại từ đầu.

---

# GIAI ĐOẠN 5 — CHỌN FOCUS TARGET & KIỂM TRA AUTO-SAVE (`he_thong_chinh.py`)

Sau khi có danh sách `list[TargetState]` đã cập nhật, `he_thong_chinh.py` thực hiện thêm 2 việc:

**Chọn Focus Target:** Gọi `manager.selected_target(now)` để chọn ra 1 target "ưu tiên nhất" để phóng to ảnh crop và hiển thị thông tin chi tiết. Ưu tiên theo: DANGER > WARNING > NORMAL, rồi theo score cao nhất, rồi theo thời gian mới nhất.

**Kiểm tra Auto-Save:** Với mỗi target, `he_thong_chinh.py` gọi `logger.should_auto_log_danger(target, now)` để kiểm tra có cần tự động lưu sự kiện không. Điều kiện bắt buộc cả 3 phải đúng đồng thời:
1. `class_name == "victim"` (do victim_probability ≥ 0.85 từ detector)
2. `display_status == "DANGER"` (do Danger Score ≥ 85 từ target_manager)
3. `rescue_score >= AUTO_SAVE_MIN_SCORE = 85`

Ngoài ra còn kiểm tra cooldown (60 giây giữa 2 lần lưu cùng target) và delta score (phải tăng thêm ≥ 15 điểm so với lần lưu trước).

Nếu điều kiện thỏa → `logger.save_target_event()` được gọi:
- Lưu **ảnh full** (có vẽ bbox + label) vào `canh_bao_full/`
- Lưu **ảnh crop** vào `canh_bao_crop/`
- Gọi `replay_buffer.request_save()` → lấy 10s frame từ deque + thu thêm 10s → ghép thành ~20s video `.mp4` vào `canh_bao_video/`
- Ghi 1 dòng vào file CSV `logs/rescue_events.csv`

---

# GIAI ĐOẠN 6 — VẼ GIAO DIỆN (`renderer.py`)

`he_thong_chinh.py` gọi `overlay.render(frame, targets, selected, telemetry, env, events, show_overlay, fps, now)` từ `renderer.py`. Hàm này vẽ toàn bộ UI lên frame:
- Bounding box màu theo status (xanh/vàng/đỏ) cho từng target
- Label: ID, class_name, Danger Score, tư thế, cử chỉ
- Ảnh crop phóng to của Focus Target ở góc phải
- Panel thông tin chi tiết: score breakdown, lý do, motion status
- Header: thời gian hiện tại, thời tiết, model badge
- Nếu Tab đang giữ → hiển thị overlay chi tiết toàn màn hình
- Nếu ROI_EDIT_MODE → hiển thị polygon đang vẽ

Sau đó `cv2.imshow()` hiển thị frame đã vẽ ra màn hình full-screen.

---

# GIAI ĐOẠN 7 — XỬ LÝ PHÍM BẤM (`he_thong_chinh.py`)

`cv2.waitKey(1)` bắt phím bấm với timeout 1ms (không chặn vòng lặp). Phím bấm được xử lý như sau:

| Phím | File xử lý | Chức năng |
|---|---|---|
| `Tab` (giữ) | `ui_controller.py` — `TabHoldController` | Bật/tắt overlay thông tin chi tiết |
| `A` | `he_thong_chinh.py` dòng 255–256 | Mở cửa sổ nhật ký Tkinter |
| `W` | `he_thong_chinh.py` dòng 257–262 | Cập nhật thời tiết ngay lập tức |
| `N` | `he_thong_chinh.py` → `handle_key()` | Chuyển sang target tiếp theo |
| `S` | `he_thong_chinh.py` → `handle_key()` | Khóa focus target ưu tiên nhất |
| `C` | `he_thong_chinh.py` → `handle_key()` | Operator xác nhận là nạn nhân → lưu sự kiện |
| `F` | `he_thong_chinh.py` → `handle_key()` | Operator báo nhầm → xóa media đã lưu |
| `T` | `he_thong_chinh.py` → `handle_key()` | Operator khóa theo dõi thêm |
| `R` | `he_thong_chinh.py` → `handle_key()` | Xuất báo cáo HTML ngay |
| `O` | `he_thong_chinh.py` → `handle_key()` | Bật/tắt ROI |
| `E` | `he_thong_chinh.py` → `handle_key()` | Vào chế độ vẽ ROI (click chuột) |
| `X` | `he_thong_chinh.py` → `handle_key()` | Xóa ROI |
| `P` | `he_thong_chinh.py` → `handle_key()` | Bật/tắt skeleton 17 keypoints |
| `B` | `he_thong_chinh.py` → `handle_key()` | Bật/tắt crosshair |
| `Q` / `Esc` | `he_thong_chinh.py` | Thoát → xuất báo cáo HTML tự động |

> **Không có phím 1, 2, 3. Không có phím V.** Phím operator là C/F/T. Video replay được lưu tự động, không có lệnh lưu thủ công. Báo cáo xuất ra là HTML (không phải PDF).

---

# KẾT THÚC PHIÊN

Khi người dùng nhấn `Q` hoặc `Esc`, vòng lặp while dừng lại. Khối `finally` trong `he_thong_chinh.py` (dòng 265–276) thực hiện:
1. `replay_buffer.stop()` — kết thúc thread ghi video, hoàn thành các video đang dang dở
2. `log_window.close()` — đóng cửa sổ Tkinter nếu đang mở
3. `report_generator.generate()` — tạo báo cáo HTML tổng kết phiên làm việc (nếu `GENERATE_REPORT_ON_EXIT = True`)
4. `cap.release()` — giải phóng camera
5. `cv2.destroyAllWindows()` — đóng tất cả cửa sổ OpenCV
