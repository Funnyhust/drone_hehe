import cv2
import json
import numpy as np
from pathlib import Path

BASE_DIR = Path(r"d:\DATN\AI YOLO\data_train_AI\data_tong_hop")

DATASETS = {
    "ds0": "01_Disaster Victim.v1i.yolov8.zip",
    "ds1": "02_Drowning Detection.v1-first.yolov8.zip",
    "ds2": "03_Emergency Response.v1i.yolov8.zip",
    "ds3": "04_People Detection -General-.v8i.yolov8.zip",
    "ds4": "05_coco-person.v1i.yolov8.zip",
    "ds5": "06_disaster.v1-for_yolo.yolov8.zip",
    "ds6": "07_drone person detection.v6i.yolov8.zip",
    "ds7": "08_general person dataset.v1-v1.yolov8.zip",
    "ds8": "09_visdrone2019.v3i.yolov8.zip",
    "ds9": "10_Injured people detection.v1i.yolov8.zip",
    "ds10": "11_injured people.v1i.yolov8.zip",
    "ds11": "12_People Detection - Thermal...zip",
    "ds13": "14_rescue.v1i.yolov8.zip",
    "ds14": "15_sos.v1i.yolov8.zip",
    "ds15": "17_drowning.v2i.yolov8.zip",
    "ds16": "18_Rubble and Person Detection.v1i.yolov8.zip",
    "ds17": "19_victim detection.v1i.yolov8.zip",
    "dsA": "A_Fall_Detection_v1i.zip (Nam)",
    "dsB": "B_Fall_Detection_raw.zip (Nam)",
    "dsC": "C_Fall_Down_Detection.zip (Nam)",
    "dsE": "E_Sitting_Person_2.zip (Ngoi)",
    "dsF": "F_person_standing.zip (Dung)",
    "dsG": "G_standing_person.zip (Dung)",
    "dsH": "H_standing.zip (Dung)",
    "dsI": "I_sitting.zip (Ngoi)",
}

WINDOW_MAX_W, WINDOW_MAX_H = 1200, 800

def get_config(ds_id):
    if ds_id.isdigit():
        return {
            "colors": {0: (0, 255, 0), 1: (0, 0, 255)},
            "names": {0: "0: Normal", 1: "1: Victim"},
            "help_text": "F:Lat Class(0<>1) | E:Lui | Space:Bo qua | Q:Thoat"
        }
    else:
        return {
            "colors": {1: (0, 255, 0), 2: (255, 0, 0), 3: (0, 255, 255)},
            "names": {1: "1: Dung", 2: "2: Ngoi", 3: "3: Nam"},
            "help_text": "1:Dung | 2:Ngoi | 3:Nam | E:Lui | Space:Bo qua | Q:Thoat"
        }
def read_labels(label_path):
    if not label_path.exists(): return []
    lines = []
    with open(label_path, "r", encoding='utf-8') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) == 5:
                lines.append([int(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])])
    return lines

def write_labels(label_path, labels):
    with open(label_path, "w", encoding='utf-8') as f:
        for cls, x, y, w, h in labels:
            f.write(f"{cls} {x:.6f} {y:.6f} {w:.6f} {h:.6f}\n")

def read_img_unicode(path):
    stream = open(path, "rb")
    bytes = bytearray(stream.read())
    numpyarray = np.asarray(bytes, dtype=np.uint8)
    return cv2.imdecode(numpyarray, cv2.IMREAD_COLOR)

def draw_boxes(img, labels, config):
    h_img, w_img = img.shape[:2]
    display = img.copy()
    for cls, x, y, w, h in labels:
        x1, y1 = int((x - w / 2) * w_img), int((y - h / 2) * h_img)
        x2, y2 = int((x + w / 2) * w_img), int((y + h / 2) * h_img)
        color = config["colors"].get(cls, (255, 255, 0))
        cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)
        cv2.putText(display, config["names"].get(cls, str(cls)), (x1, max(y1 - 5, 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    return display

def resize_fit(img, max_w, max_h):
    h, w = img.shape[:2]
    scale = min(max_w / w, max_h / h, 1.0)
    if scale < 1.0:
        img = cv2.resize(img, (int(w * scale), int(h * scale)))
    return img

def add_info_bar(img, index, total, filename, action_msg, ds_id, config):
    bar = np.zeros((80, img.shape[1], 3), dtype=np.uint8)
    cv2.putText(bar, f"Bo: ds{ds_id} | [{index+1}/{total}] {filename[:60]}", (10, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    cv2.putText(bar, config["help_text"], (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 255, 180), 1)
    if action_msg:
        cv2.putText(bar, action_msg, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    return np.vstack([bar, img])

def run_reviewer(ds_id):
    prefix = f"ds{ds_id}_"
    progress_file = Path(rf"d:\DATN\AI YOLO\duyet_data_roboflow\review_progress_ds{ds_id}.json")
    config = get_config(ds_id)
    
    label_files = []
    for split in ["train", "valid"]:
        label_files.extend(list((BASE_DIR / split / "labels").glob(f"{prefix}*.txt")))
    
    label_files = sorted(label_files)
    total = len(label_files)
    
    if total == 0:
        print(f"\n[!] Khong tim thay anh nao thuoc bo ds{ds_id}!\n")
        return
        
    print(f"\n>>> Dang mo bo ds{ds_id} (Tong cong: {total} anh)...")
    
    if progress_file.exists():
        with open(progress_file, "r") as f:
            progress = json.load(f)
    else:
        progress = {"current_index": 0}
        
    idx = progress.get("current_index", 0)
    if idx >= total:
        idx = 0 
    
    window_name = f"Review ds{ds_id}"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, WINDOW_MAX_W, WINDOW_MAX_H + 80)
    
    action_msg = ""
    while idx < total:
        lf = label_files[idx]
        split = lf.parent.parent.name
        imgs_dir = BASE_DIR / split / "images"
        
        img_path = None
        for ext in [".jpg", ".jpeg", ".png"]:
            if (imgs_dir / (lf.stem + ext)).exists():
                img_path = imgs_dir / (lf.stem + ext)
                break
                
        if not img_path:
            idx += 1
            continue
            
        img = read_img_unicode(str(img_path))
        if img is None:
            idx += 1
            continue
            
        labels = read_labels(lf)
        display = draw_boxes(img, labels, config)
        display = resize_fit(display, WINDOW_MAX_W, WINDOW_MAX_H)
        display = add_info_bar(display, idx, total, img_path.name, action_msg, ds_id, config)
        
        cv2.imshow(window_name, display)
        action_msg = ""
        key = cv2.waitKey(0) & 0xFF
        
        if ds_id.isdigit():
            if key in [ord('f'), ord('F')]:
                labels = [[1 - c, x, y, w, h] for c, x, y, w, h in labels]
                write_labels(lf, labels)
                action_msg = ">> DA L\u1eacT CLASS (0<>1) <<<"
                idx += 1
                with open(progress_file, "w") as f: json.dump({"current_index": idx}, f)
                continue
        else:
            if key == ord('1'):
                labels = [[1, x, y, w, h] for _, x, y, w, h in labels]
                write_labels(lf, labels)
                action_msg = ">> DA SET THANH 1 (DUNG) <<<"
                idx += 1
                with open(progress_file, "w") as f: json.dump({"current_index": idx}, f)
                continue
            elif key == ord('2'):
                labels = [[2, x, y, w, h] for _, x, y, w, h in labels]
                write_labels(lf, labels)
                action_msg = ">> DA SET THANH 2 (NGOI) <<<"
                idx += 1
                with open(progress_file, "w") as f: json.dump({"current_index": idx}, f)
                continue
            elif key == ord('3'):
                labels = [[3, x, y, w, h] for _, x, y, w, h in labels]
                write_labels(lf, labels)
                action_msg = ">> DA SET THANH 3 (NAM) <<<"
                idx += 1
                with open(progress_file, "w") as f: json.dump({"current_index": idx}, f)
                continue

        if key in [ord('e'), ord('E')]:
            if idx > 0:
                idx -= 1
                action_msg = "<< LUI LAI 1 ANH"
                with open(progress_file, "w") as f: json.dump({"current_index": idx}, f)
            else:
                action_msg = "DA LA ANH DAU TIEN!"
        elif key == ord(' '):
            action_msg = "Bo qua"
            idx += 1
            with open(progress_file, "w") as f: json.dump({"current_index": idx}, f)
        elif key in [ord('q'), ord('Q'), 27]:
            with open(progress_file, "w") as f: json.dump({"current_index": idx}, f)
            print(f"\nDa thoat va luu tien trinh ds{ds_id} tai {idx}/{total}\n")
            break

    else:
        print(f"\nHOAN THANH TOAN BO {total} ANH CUA ds{ds_id}!\n")
        with open(progress_file, "w") as f: json.dump({"current_index": total}, f)
        
    cv2.destroyAllWindows()

def main():
    while True:
        print("\n" + "="*70)
        print("TOOL KIEM TRA VA LAT NHAN TUNG BO DATA")
        print("="*70)
        for k, v in DATASETS.items():
            print(f"{k:<6} | {v}")
        print("="*70)
        
        choice = input(">> Nhap ma so muon kiem tra (vd: 1, 16, hoac A de mo dsA, hoac Q de thoat): ").strip().lower()
        
        if choice == 'q':
            print("Da thoat tool.")
            break
            
        if choice.startswith('ds'):
            choice = choice[2:]
            
        if choice.isalnum() and f"ds{choice.upper()}" in DATASETS:
            run_reviewer(choice.upper())
        elif choice.isdigit() and f"ds{choice}" in DATASETS:
            run_reviewer(choice)
        else:
            print("[!] Lua chon khong hop le, vui long nhap lai!")

if __name__ == "__main__":
    main()
