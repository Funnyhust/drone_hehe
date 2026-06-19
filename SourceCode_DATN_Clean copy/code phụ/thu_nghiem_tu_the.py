import cv2
from ultralytics import YOLO

model = YOLO(r"Model\Posture\weights\best.pt")

cap = cv2.VideoCapture(0)

print("🎥 Đang mở Webcam Test Real-time cho Posture V8... (Bấm phím 'q' để thoát)")

cv2.namedWindow("Test Do Tin Cay - Posture V8", cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty("Test Do Tin Cay - Posture V8", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        print("Lỗi: Không đọc được frame từ Camera.")
        break
        
    results = model.predict(
        source=frame,
        conf=0.6,         
        verbose=False      
    )
    
    annotated_frame = results[0].plot()
    
    cv2.imshow("Test Do Tin Cay - Posture V8", annotated_frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
