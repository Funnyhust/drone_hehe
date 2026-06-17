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
from models import *
def app_root() -> Path:
    return Path(__file__).resolve().parent.parent


def ensure_dirs(root: Path) -> dict[str, Path]:
    paths = {
        "logs": root / "logs",
        "full": root / "canh_bao_full",
        "crop": root / "canh_bao_crop",
        "video": root / "canh_bao_video",
        "reports": root / "reports",
        "config": root / "config",
    }
    for path in paths.values():
        path.mkdir(parents=True, exist_ok=True)
    return paths


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def bbox_center(bbox: tuple[int, int, int, int]) -> tuple[float, float]:
    x1, y1, x2, y2 = bbox
    return (x1 + x2) / 2.0, (y1 + y2) / 2.0


def iou_overlap(box_a, box_b) -> float:
    x_a = max(box_a[0], box_b[0])
    y_a = max(box_a[1], box_b[1])
    x_b = min(box_a[2], box_b[2])
    y_b = min(box_a[3], box_b[3])
    inter = max(0, x_b - x_a) * max(0, y_b - y_a)
    if inter == 0:
        return 0.0
    area_a = max(1, (box_a[2] - box_a[0]) * (box_a[3] - box_a[1]))
    area_b = max(1, (box_b[2] - box_b[0]) * (box_b[3] - box_b[1]))
    union = area_a + area_b - inter
    return inter / float(union) if union > 0 else 0.0


def get_keypoint(kps_xy, kps_conf, index: int, min_conf: float = 0.35):
    if kps_xy is None or kps_conf is None or index >= len(kps_xy):
        return None
    if kps_conf[index] < min_conf:
        return None
    x, y = kps_xy[index]
    return float(x), float(y)


def _direction_changes(values: list[float], min_delta: float = 6.0) -> int:
    if len(values) < 3:
        return 0
    signs: list[int] = []
    anchor = values[0]
    for value in values[1:]:
        delta = value - anchor
        if abs(delta) < min_delta:
            continue
        sign = 1 if delta > 0 else -1
        if not signs or sign != signs[-1]:
            signs.append(sign)
        anchor = value
    return max(0, len(signs) - 1)


def detect_wrist_waving(history, current_time: float) -> bool:
    recent = [item for item in history if current_time - float(item[0]) <= config.WAVING_WINDOW_SECONDS]
    if len(recent) < 4:
        return False
    xs = [float(item[1]) for item in recent]
    ys = [float(item[2]) for item in recent]
    above_values = [bool(item[3]) for item in recent]
    if sum(1 for value in above_values if value) < 2:
        return False
    move_x = max(xs) - min(xs)
    move_y = max(ys) - min(ys)
    if max(move_x, move_y) < config.WRIST_MOVE_THRESHOLD:
        return False
    above_changes = sum(1 for idx in range(1, len(above_values)) if above_values[idx] != above_values[idx - 1])
    direction_changes = max(_direction_changes(xs), _direction_changes(ys), above_changes)
    return direction_changes >= config.WAVING_MIN_DIRECTION_CHANGES


def normalize_posture_class(cls: str) -> str:
    class_name = str(cls or "").lower()
    if class_name in {"standing", "standing_person", "stand", "0"}:
        return "standing_person"
    if class_name in {"sitting", "sitting_person", "sit", "1"}:
        return "sitting_person"
    if class_name in {"lying", "lying_person", "fallen", "fall", "lie", "2"}:
        return "lying_person"
    return "unknown"


def normalize_rescue_class(cls: str, class_id: int | None = None) -> str:
    class_name = str(cls or "").strip().lower()
    if class_name in {"victim", "injured", "casualty", "nguoi_gap_nan"}:
        return "victim"
    if class_name in {"normal_person", "normal", "person", "nguoi_binh_thuong"}:
        return "normal_person"
    if class_id is not None:
        return "victim" if int(class_id) == 1 else "normal_person"
    return "normal_person"


def normalize_class(cls: str) -> str:
    return normalize_posture_class(cls)


