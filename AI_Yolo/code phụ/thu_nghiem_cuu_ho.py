import cv2
from ultralytics import YOLO

model = YOLO(r"Model\Rescue\weights\best.pt")

cap = cv2.VideoCapture(0)

print("🎥 Đang mở Webcam Test Real-time cho Rescue V2... (Bấm phím 'q' để thoát)")

cv2.namedWindow("Test Do Tin Cay - Rescue V2", cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty("Test Do Tin Cay - Rescue V2", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        print("Lỗi: Không đọc được frame từ Camera.")
        break
        
    results = model.predict(
        source=frame,
        conf=0.35,         
        agnostic_nms=True,
        verbose=False      
    )
    
    annotated_frame = frame.copy()
    boxes = results[0].boxes
    
    if boxes is not None:
        for box in boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            
            box_w = max(1, x2 - x1)
            box_h = max(1, y2 - y1)
            
            is_victim = False
            label_text = ""
            color = (0, 255, 0) 
            
            if cls_id == 1:
                is_victim = True
                label_text = f"Victim (YOLO): {conf:.2f}"
                color = (0, 0, 255) # Đỏ
            elif box_w / box_h > 1.8:
                is_victim = True
                label_text = f"Victim (LOGIC NGÃ): {conf:.2f}"
                color = (0, 0, 255)
            else:
                label_text = f"Normal: {conf:.2f}"
                
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(annotated_frame, label_text, (x1, max(20, y1 - 10)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    cv2.imshow("Test Do Tin Cay - Rescue V2", annotated_frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
