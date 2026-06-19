import os
import shutil
from pathlib import Path

SOURCE_DIR = Path(r"d:\DATN\AI YOLO\data_train_AI\data_tong_hop")
DEST_DIR = Path(r"d:\DATN\AI YOLO\data_train_AI\rescue")

PREFIXES = [
    "ds0_", "ds1_", "ds2_", "ds3_", "ds4_", "ds5_", "ds6_", "ds7_", "ds8_", 
    "ds9_", "ds10_", "ds11_", "ds13_", "ds14_", "ds15_", "ds16_", "ds17_"
]

def main():
    print("=== BAT DAU LOC VA GOM DATA CUU HO ===")
    for split in ["train", "valid"]:
        (DEST_DIR / split / "images").mkdir(parents=True, exist_ok=True)
        (DEST_DIR / split / "labels").mkdir(parents=True, exist_ok=True)

    total_images_copied = 0

    for split in ["train", "valid"]:
        src_labels_dir = SOURCE_DIR / split / "labels"
        src_images_dir = SOURCE_DIR / split / "images"
        dest_labels_dir = DEST_DIR / split / "labels"
        dest_images_dir = DEST_DIR / split / "images"

        if not src_labels_dir.exists():
            continue

        for label_file in src_labels_dir.glob("*.txt"):
            if PREFIXES and not any(label_file.name.startswith(p) for p in PREFIXES):
                continue
            
            img_path = None
            for ext in [".jpg", ".jpeg", ".png"]:
                temp_img = src_images_dir / (label_file.stem + ext)
                if temp_img.exists():
                    img_path = temp_img
                    break
            
            if not img_path:
                continue

            valid_lines = []
            with open(label_file, "r", encoding="utf-8") as f:
                lines = f.readlines()
                for line in lines:
                    parts = line.strip().split()
                    if len(parts) == 5:
                        cls_id = int(parts[0])
                        if cls_id in [0, 1]:
                            valid_lines.append(f"{cls_id} {parts[1]} {parts[2]} {parts[3]} {parts[4]}\n")
                        else:
                            pass

            if not valid_lines:
                continue

            dest_label_path = dest_labels_dir / label_file.name
            with open(dest_label_path, "w", encoding="utf-8") as f:
                f.writelines(valid_lines)
            
            dest_img_path = dest_images_dir / img_path.name
            shutil.copy2(img_path, dest_img_path)
            
            total_images_copied += 1
            if total_images_copied % 1000 == 0:
                print(f"Da xu ly {total_images_copied} file...")

    print(f"\n[XONG] Da gom thanh cong {total_images_copied} anh va nhan sang thu muc {DEST_DIR}")

    yaml_path = DEST_DIR / "data_rescue_only.yaml"
    yaml_content = f"""train: {DEST_DIR / 'train' / 'images'}
val: {DEST_DIR / 'valid' / 'images'}

nc: 2
names: 
  0: 'normal_person'
  1: 'victim'
"""
    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write(yaml_content)
    
    print(f"Da tao xong file cau hinh: {yaml_path}")

if __name__ == "__main__":
    main()
