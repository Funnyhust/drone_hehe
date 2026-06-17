from __future__ import annotations

import csv
import html
import json
import math
import os
import queue
import socket
import threading
import time
import unicodedata
from collections import defaultdict, deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any
from urllib.parse import urlencode
from urllib.request import urlopen

import cv2
import numpy as np

import config
import models
import utils
import roi_manager
import weather_telemetry
import ui_controller
import detector
import target_manager
import event_logger
import report_generator
import renderer
from models import *
from utils import *
from roi_manager import *
from weather_telemetry import *
from ui_controller import *
from detector import *
from target_manager import *
from event_logger import *
from report_generator import *
from renderer import *
def open_camera():
    if config.USE_DSHOW_ON_WINDOWS and isinstance(config.CAMERA_INDEX, int):
        cap = cv2.VideoCapture(config.CAMERA_INDEX, cv2.CAP_DSHOW)
    else:
        cap = cv2.VideoCapture(config.CAMERA_INDEX)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FPS, config.CAMERA_FPS)
        return cap
    return None


def synthetic_frame(width: int, height: int, message: str) -> np.ndarray:
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    frame[:] = (18, 20, 24)
    cv2.putText(frame, message, (40, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (210, 220, 230), 2, cv2.LINE_AA)
    return frame


def handle_key(key: int, selected, manager, logger, frame, telemetry, env, replay_buffer, report_generator, paths) -> bool:
    if key == 27:
        if config.ROI_EDIT_MODE:
            cancel_roi_edit(logger)
            return True
        return True
    if key in (ord("q"), ord("Q")):
        return False
    if key < 0:
        return True
    if key in (13, 10):
        if config.ROI_EDIT_MODE:
            save_roi_edit(paths, logger)
            refresh_target_roi_status(manager)
        return True

    key_char = chr(key).lower() if 0 <= key <= 255 else ""
    if key_char == "r":
        try:
            path = report_generator.generate()
            logger.remember(f"Đã tạo báo cáo HTML: {path.name}", "ACTION")
        except Exception as exc:
            logger.remember(f"Lỗi tạo báo cáo HTML: {exc}", "WARNING")
        return True
    if key_char == "o":
        toggle_roi(paths, logger)
        refresh_target_roi_status(manager)
        return True
    if key_char == "e":
        start_roi_edit(logger)
        return True
    if key_char == "x":
        clear_roi(paths, logger)
        refresh_target_roi_status(manager)
        return True
    if key_char == "p":

        config.SHOW_SKELETON = not config.SHOW_SKELETON
        logger.remember(f"{'Bật' if config.SHOW_SKELETON else 'Tắt'} khung xương 17 điểm.", "ACTION")
        return True

    now = time.time()
    if key_char == "n":
        target = manager.cycle_focus(now)
        if target is None:
            logger.remember("Không có mục tiêu để chuyển.", "WARNING")
        elif manager.focus_locked:
            logger.remember(f"Chuyển khóa mục tiêu sang ID {target.object_id}.", "ACTION", dedupe_seconds=0)
        else:
            logger.remember(f"Chuyển mục tiêu sang ID {target.object_id}.", "ACTION", dedupe_seconds=0)
        return True
    if key_char == "b":

        config.SHOW_CROSSHAIR = not config.SHOW_CROSSHAIR
        logger.remember(f"{'Bật' if config.SHOW_CROSSHAIR else 'Tắt'} tia ngắm trung tâm.", "ACTION")
        return True
    if key_char == "s":
        target = manager.focus_highest_priority(now, lock=True)
        if target is None:
            logger.remember("Không có mục tiêu ưu tiên để khóa.", "WARNING")
        else:
            logger.remember(f"Đã khóa mục tiêu ưu tiên nhất: ID {target.object_id}.", "ACTION", dedupe_seconds=0)
        return True

    if selected is None:
        if key_char in {"c", "f", "t"}:
            logger.remember("Không có mục tiêu đang chọn.", "WARNING")
        return True

    if key_char == "c":
        manager.apply_operator_decision(selected.object_id, "CONFIRMED", now)
        logger.save_target_event("OPERATOR_CONFIRMED", selected, frame, telemetry, env, "CONFIRMED", replay_buffer)
        logger.remember(f"Người vận hành xác nhận ID {selected.object_id} là nạn nhân.", "ACTION", dedupe_seconds=0)
    elif key_char == "f":
        manager.apply_operator_decision(selected.object_id, "FALSE_ALARM", now)
        logger.remove_saved_media_for_target(selected.object_id, replay_buffer)
        logger.save_target_event("OPERATOR_FALSE_ALARM", selected, frame, telemetry, env, "FALSE_ALARM", replay_buffer)
        logger.remember(f"Người vận hành đánh dấu ID {selected.object_id} là báo nhầm.", "ACTION", dedupe_seconds=0)
        manager.remove_target(selected.object_id)
    elif key_char == "t":
        manager.apply_operator_decision(selected.object_id, "TRACK_MORE", now)
        manager.lock_focus(selected.object_id, now)
        logger.save_target_event("OPERATOR_TRACK_MORE", selected, frame, telemetry, env, "TRACK_MORE", replay_buffer)
        logger.remember(f"Khóa mục tiêu ID {selected.object_id} để tiếp tục theo dõi.", "ACTION", dedupe_seconds=0)
    return True

def main() -> int:
    root = app_root()
    paths = ensure_dirs(root)

    detector = Detector(root)
    manager = TargetManager()
    env_provider = WeatherProvider()
    telemetry_provider = TelemetryProvider()
    logger = EventLogger(paths)
    logger.remember(detector.model_summary())
    load_roi_config(paths, logger)
    replay_buffer = EventReplayBuffer(paths)
    report_generator = ReportGenerator(paths, root)
    overlay = OverlayRenderer(detector.yolo_badge())
    tab = TabHoldController()
    log_window = EventLogWindow(logger)

    cap = open_camera()
    if cap is None:
        print(f"Khong mo duoc camera index {config.CAMERA_INDEX}.")
        logger.remember(f"Không mở được camera index {config.CAMERA_INDEX}.", "WARNING")
        if not config.FALLBACK_SYNTHETIC_FRAME:
            return 1

    cv2.namedWindow(config.WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(config.WINDOW_NAME, roi_mouse_callback)
    if config.FULLSCREEN:
        cv2.setWindowProperty(config.WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    last_time = time.time()
    fps = 0.0
    running = True
    show_overlay = False
    frame_index = 0

    try:
        while running:
            frame = None
            ok = False
            if cap is not None:
                ok, frame = cap.read()
            if not ok or frame is None:
                if not config.FALLBACK_SYNTHETIC_FRAME:
                    break
                frame = synthetic_frame(config.CAMERA_WIDTH, config.CAMERA_HEIGHT, "Camera chua san sang - placeholder")
            else:
                frame = cv2.resize(frame, (config.CAMERA_WIDTH, config.CAMERA_HEIGHT))

            now = time.time()
            frame_index += 1
            dt = max(1e-6, now - last_time)
            last_time = now
            fps = (0.9 * fps) + (0.1 * (1.0 / dt)) if fps > 0 else (1.0 / dt)
            replay_buffer.add_frame(frame, now)

            env = env_provider.read()
            if frame_index % max(1, config.DETECTION_EVERY_N_FRAMES) == 0:
                raw_detections = detector.detect(frame, now)
            else:
                raw_detections = []

            for object_id, _path, ok in replay_buffer.get_completed():
                if ok:
                    logger.remember(f"Đã lưu replay video cho ID {object_id}")
                else:
                    logger.remember(f"Replay video lỗi hoặc quá ngắn cho ID {object_id}")

            targets = manager.update(raw_detections, frame, now)
            logger.observe_targets(targets)
            mission_state = mission_state_from_targets(targets)
            telemetry = telemetry_provider.read(fps, mission_state)
            selected = manager.selected_target(now)

            for target in targets:
                if logger.should_auto_log_danger(target, now):
                    logger.save_target_event("AUTO_DANGER", target, frame, telemetry, env, "AUTO", replay_buffer)

            tab.poll()
            show_overlay = tab.is_overlay_visible()
            rendered = overlay.render(
                frame,
                targets,
                selected,
                telemetry,
                env,
                logger.recent_events(),
                show_overlay,
                fps,
                now,
            )
            cv2.imshow(config.WINDOW_NAME, rendered)
            log_window.poll()

            key = cv2.waitKey(1) & 0xFF
            if key != 255:
                if key == 9:
                    tab.update_from_cv_key(key)
                elif key in (ord("a"), ord("A")):
                    log_window.open()
                elif key in (ord("w"), ord("W")):
                    env = env_provider.read(force=True)
                    if env.has_data and telemetry.internet_status != "Offline":
                        logger.remember(f"Đã cập nhật thời tiết từ Open-Meteo: {env.assessment}.", "SYSTEM")
                    else:
                        logger.remember("Cập nhật thời tiết thất bại: Mất kết nối mạng.", "WARNING")
                else:
                    running = handle_key(key, selected, manager, logger, frame, telemetry, env, replay_buffer, report_generator, paths)
    finally:
        replay_buffer.stop()
        log_window.close()
        if config.GENERATE_REPORT_ON_EXIT:
            try:
                path = report_generator.generate()
                print(f"[REPORT] {path}")
            except Exception as exc:
                print(f"[REPORT_ERROR] {exc}")
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()

    return 0

if __name__ == "__main__":
    import os
    if os.name == 'nt':
        import ctypes
        ctypes.windll.kernel32.SetThreadExecutionState(0x80000000 | 0x00000001 | 0x00000002)
    raise SystemExit(main())
