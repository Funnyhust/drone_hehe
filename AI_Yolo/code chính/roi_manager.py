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
from utils import *
def _clean_roi_points(points: Any) -> list[tuple[int, int]]:
    cleaned: list[tuple[int, int]] = []
    if not isinstance(points, list):
        return cleaned
    for item in points:
        if not isinstance(item, (list, tuple)) or len(item) != 2:
            continue
        try:
            cleaned.append((int(item[0]), int(item[1])))
        except Exception:
            continue
    return cleaned


def roi_config_path(paths: dict[str, Path] | None = None) -> Path:
    if paths is not None and "config" in paths:
        return paths["config"] / config.ROI_CONFIG_FILENAME
    return app_root() / "config" / config.ROI_CONFIG_FILENAME


def load_roi_config(paths: dict[str, Path], logger=None) -> None:

    config.ROI_ENABLED = False
    ROI_POINTS = []
    config.ROI_EDIT_MODE = False
    ROI_EDIT_POINTS = []

    path = roi_config_path(paths)
    if not path.exists():
        return
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        points = _clean_roi_points(data.get("points", []))
        ROI_POINTS = points if len(points) >= 3 else []
        config.ROI_ENABLED = bool(data.get("roi_enabled", False)) and len(ROI_POINTS) >= 3
        if logger is not None:
            logger.remember("ROI loaded" if config.ROI_ENABLED else "ROI config loaded, ROI disabled")
    except Exception:
        config.ROI_ENABLED = False
        ROI_POINTS = []
        if logger is not None:
            logger.remember("Cảnh báo: lỗi đọc roi_config.json, ROI tắt")


def save_roi_config(paths: dict[str, Path] | None = None) -> None:
    path = roi_config_path(paths)
    path.parent.mkdir(parents=True, exist_ok=True)
    enabled = bool(config.ROI_ENABLED and len(ROI_POINTS) >= 3)
    payload = {
        "roi_enabled": enabled,
        "points": [[int(x), int(y)] for x, y in ROI_POINTS],
    }
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def start_roi_edit(logger=None) -> None:

    config.ROI_EDIT_MODE = True
    ROI_EDIT_POINTS = []
    if logger is not None:
        logger.remember("ROI edit mode")


def save_roi_edit(paths: dict[str, Path], logger=None) -> None:

    if len(ROI_EDIT_POINTS) < 3:
        if logger is not None:
            logger.remember("ROI cần ít nhất 3 điểm")
        return
    ROI_POINTS = list(ROI_EDIT_POINTS)
    config.ROI_ENABLED = True
    config.ROI_EDIT_MODE = False
    ROI_EDIT_POINTS = []
    save_roi_config(paths)
    if logger is not None:
        logger.remember("ROI saved")


def cancel_roi_edit(logger=None) -> None:

    config.ROI_EDIT_MODE = False
    ROI_EDIT_POINTS = []
    if logger is not None:
        logger.remember("ROI edit canceled")


def clear_roi(paths: dict[str, Path], logger=None) -> None:

    config.ROI_ENABLED = False
    ROI_POINTS = []
    config.ROI_EDIT_MODE = False
    ROI_EDIT_POINTS = []
    save_roi_config(paths)
    if logger is not None:
        logger.remember("ROI cleared")


def toggle_roi(paths: dict[str, Path], logger=None) -> None:

    if config.ROI_ENABLED:
        config.ROI_ENABLED = False
        save_roi_config(paths)
        if logger is not None:
            logger.remember("ROI disabled")
        return
    if len(ROI_POINTS) < 3:
        if logger is not None:
            logger.remember("Chưa có ROI. Bấm E để vẽ ROI.")
        return
    config.ROI_ENABLED = True
    save_roi_config(paths)
    if logger is not None:
        logger.remember("ROI enabled")


def roi_polygon_pixels(frame_shape) -> list[tuple[int, int]]:
    h, w = frame_shape[:2]
    if not config.ROI_ENABLED or len(ROI_POINTS) < 3:
        return []
    clipped = []
    for x, y in ROI_POINTS:
        clipped.append((int(clamp(x, 0, max(0, w - 1))), int(clamp(y, 0, max(0, h - 1)))))
    return clipped


def point_in_polygon(point: tuple[float, float], polygon: list[tuple[int, int]]) -> bool:
    if len(polygon) < 3:
        return False
    pts = np.array(polygon, dtype=np.int32)
    return cv2.pointPolygonTest(pts, point, False) >= 0


def roi_mouse_callback(event, x, y, flags, param) -> None:
    del flags, param

    if event == cv2.EVENT_LBUTTONDOWN and config.ROI_EDIT_MODE:
        ROI_EDIT_POINTS.append((int(x), int(y)))


def refresh_target_roi_status(manager) -> None:
    polygon = list(ROI_POINTS) if config.ROI_ENABLED and len(ROI_POINTS) >= 3 else []
    for target in manager.targets.values():
        if not polygon:
            target.roi_status = "roi_disabled"
            continue
        cx, cy = bbox_center(target.bbox)
        target.roi_status = "inside_roi" if point_in_polygon((cx, cy), polygon) else "outside_roi"


