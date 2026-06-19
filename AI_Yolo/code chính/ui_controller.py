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
class TabHoldController:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._overlay_visible = False
        self._last_tab_down = False
        self._last_cv_toggle = 0.0
        self._backend = "opencv"
        self._win_user32 = None
        self._install_global_backend()

    def poll(self) -> None:
        if self._backend != "win32" or self._win_user32 is None:
            return
        try:
            tab_down = bool(self._win_user32.GetAsyncKeyState(0x09) & 0x8000)
        except Exception:
            self._backend = "opencv"
            return

        if tab_down and not self._last_tab_down:
            self._toggle()
        self._last_tab_down = tab_down

    def update_from_cv_key(self, key: int) -> None:
        if self._backend != "opencv":
            return
        now = time.time()
        if key == 9 and now - self._last_cv_toggle > 0.35:
            self._toggle()
            self._last_cv_toggle = now

    def is_overlay_visible(self) -> bool:
        with self._lock:
            return self._overlay_visible

    def _toggle(self) -> None:
        with self._lock:
            self._overlay_visible = not self._overlay_visible

    def _install_global_backend(self) -> None:
        try:
            import ctypes

            self._win_user32 = ctypes.windll.user32
            self._backend = "win32"
            return
        except Exception:
            self._win_user32 = None
            self._backend = "opencv"


