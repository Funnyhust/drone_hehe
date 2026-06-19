from ultralytics import YOLO
import os
import time
import psutil
import torch
import sys
from pathlib import Path

PROJECT_DIR = "."
RUN_NAME = "Model/Rescue_v4"
YAML_PATH = r"d:\DATN\AI YOLO\data_train_AI\rescue\data_rescue_only.yaml"
LOG_FILE = "train_monitor_v4.log"

def log_msg(msg):
    with open(LOG_FILE, "a", encoding="utf-8") as f:
        f.write(msg + "\n")
    print(msg)

def on_train_epoch_end(trainer):
    epoch = trainer.epoch + 1
    ram_usage = psutil.virtual_memory().percent
    
    vram_msg = ""
    if torch.cuda.is_available():
        vram_allocated = torch.cuda.memory_allocated() / (1024**3)
        vram_reserved = torch.cuda.memory_reserved() / (1024**3)
        vram_msg = f" | VRAM Allocated: {vram_allocated:.2f}GB, Reserved: {vram_reserved:.2f}GB"
        
    metrics = trainer.metrics
    map50 = metrics.get('metrics/mAP50(B)', 0)
    
    log_msg(f"[MONITOR] Epoch {epoch} Xong | RAM: {ram_usage}%{vram_msg} | mAP50: {map50:.4f}")

def start_training():
    log_msg(f"=== BAT DAU TIEN TRINH TRAIN luc {time.ctime()} ===")
    
    last_pt = Path(PROJECT_DIR) / "runs/detect" / RUN_NAME / "weights/last.pt"
    
    if last_pt.exists():
        log_msg(f"[RESUME] Tim thay {last_pt}, tiep tuc train...")
        model = YOLO(str(last_pt))
        model.add_callback("on_train_epoch_end", on_train_epoch_end)
        try:
            model.train(resume=True)
        except Exception as e:
            log_msg(f"[CRASH] Loi roi: {e}")
            sys.exit(1) 
    else:
        log_msg("[NEW] Bat dau train moi tu yolov8s.pt...")
        model = YOLO("yolov8s.pt")
        model.add_callback("on_train_epoch_end", on_train_epoch_end)
        try:
            model.train(
                data=YAML_PATH,
                epochs=50,
                imgsz=640,
                batch=4,
                device=0,
                project=PROJECT_DIR,
                name=RUN_NAME,
                exist_ok=True,
                patience=20,
                cache=False,
                verbose=False 
                )
        except Exception as e:
            log_msg(f"[CRASH] Loi roi: {e}")
            sys.exit(1)

if __name__ == "__main__":
    start_training()
