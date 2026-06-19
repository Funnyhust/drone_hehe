import os
import shutil
from pathlib import Path

THU_MUC_TONG_HOP = Path(r"d:\DATN\AI YOLO\data_train_AI\data_tong_hop")

def doi_ten_va_gop(thu_muc_goc, tien_to):
    thu_muc_goc = Path(thu_muc_goc)
    count = 0
    
    print(f"\n[DANG XU LY] Bat dau gan ma '{tien_to}_' va copy vao data_tong_hop...")
    
    for split in ['train', 'valid', 'test']:
        for loai in ['images', 'labels']:
            thu_muc_nguon = thu_muc_goc / split / loai
            thu_muc_dich = THU_MUC_TONG_HOP / split / loai
            
            if not thu_muc_nguon.exists():
                continue
                
            thu_muc_dich.mkdir(parents=True, exist_ok=True)
            
            for file_path in thu_muc_nguon.iterdir():
                if file_path.is_file():
                    ten_moi = f"{tien_to}_{file_path.name}"
                    duong_dan_moi = thu_muc_dich / ten_moi
                    shutil.copy2(file_path, duong_dan_moi)
                    count += 1
                    
    print(f"[XONG] Da xu ly thanh cong {count} file!\n")

def main():
    print("="*60)
    print("TOOL TU DONG DOI TEN VA GOP DATASET TU ROBOFLOW")
    print("="*60)
    
    thu_muc_giai_nen = input("1. Nhap duong dan thu muc ban vua giai nen: ").strip()
    if thu_muc_giai_nen.startswith('"') and thu_muc_giai_nen.endswith('"'):
        thu_muc_giai_nen = thu_muc_giai_nen[1:-1]
        
    tien_to = input("2. Nhap ma dataset muon gan (VD: ds18 hoac dsK): ").strip()
    
    if Path(thu_muc_giai_nen).exists():
        doi_ten_va_gop(thu_muc_giai_nen, tien_to)
    else:
        print("[LOI] Khong tim thay thu muc ban vua nhap!")

if __name__ == "__main__":
    main()
