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
def internet_available(timeout: float = config.INTERNET_CHECK_TIMEOUT_SECONDS) -> bool:
    try:
        with socket.create_connection(("1.1.1.1", 53), timeout=timeout):
            return True
    except OSError:
        return False


def evaluate_flight_weather(data: dict[str, float | int | None] | None) -> tuple[str, str]:
    if not data:
        return "KHÔNG CÓ DỮ LIỆU", "Không thể lấy dự báo thời tiết online"
    rain_probability_1h = float(data.get("rain_probability_1h") or 0.0)
    precipitation_1h = float(data.get("precipitation_1h_mm") or 0.0)
    rain_probability_3h = float(data.get("rain_probability_3h") or 0.0)
    precipitation_3h = float(data.get("precipitation_3h_mm") or 0.0)
    wind_speed = float(data.get("wind_speed_ms") or 0.0)
    temperature = float(data.get("temperature_c") or 0.0)
    humidity = float(data.get("humidity_percent") or 0.0)

    if wind_speed >= 8.0:
        return "CẢNH BÁO GIÓ", "Gió mạnh, không nên bay drone nhẹ"
    if rain_probability_1h >= 60 or precipitation_1h > 1.0:
        return "KHÔNG KHUYẾN NGHỊ BAY", "Có khả năng mưa trong 1 giờ tới, nên hoãn hoặc bay rất ngắn"
    if rain_probability_3h >= 60 or precipitation_3h > 2.0:
        return "CẦN THEO DÕI", "Có khả năng mưa trong 3 giờ tới, không nên kéo dài nhiệm vụ"
    if humidity >= 90.0:
        return "CẦN THEO DÕI ĐỘ ẨM", "Độ ẩm cao, theo dõi nguy cơ đọng sương hoặc camera mờ"
    if temperature > 38.0:
        return "CẢNH BÁO NÓNG", "Nhiệt độ cao, pin và linh kiện có thể giảm hiệu năng"
    return "TỐT", "Điều kiện thời tiết phù hợp để bay thử nghiệm"


class WeatherProvider:
    def __init__(self) -> None:
        self.last_update_attempt = 0.0
        self.last_success_time = 0.0
        self.last_reading: EnvironmentReading | None = None

    def read(self, force: bool = False) -> EnvironmentReading:
        now = time.time()
        if not config.WEATHER_ENABLED:
            self.last_reading = self._no_data("Đã tắt thời tiết online")
            return self.last_reading
        if config.WEATHER_PROVIDER != "open_meteo":
            self.last_reading = self._no_data(f"Weather provider chưa hỗ trợ: {config.WEATHER_PROVIDER}")
            return self.last_reading
        if not force and self.last_reading is not None and now - self.last_update_attempt < config.WEATHER_UPDATE_INTERVAL_SECONDS:
            return self.last_reading

        self.last_update_attempt = now
        if not internet_available():
            return self._handle_fetch_error("Không có Internet")

        try:
            reading = self._fetch_open_meteo()
            self.last_reading = reading
            self.last_success_time = now
            return reading
        except Exception:
            return self._handle_fetch_error("Lỗi lấy dữ liệu thời tiết")

    def _fetch_open_meteo(self) -> EnvironmentReading:
        params = {
            "latitude": config.WEATHER_LAT,
            "longitude": config.WEATHER_LON,
            "current": "temperature_2m,relative_humidity_2m,precipitation,weather_code,wind_speed_10m",
            "hourly": "temperature_2m,relative_humidity_2m,precipitation_probability,precipitation,wind_speed_10m,weather_code",
            "forecast_days": 1,
            "timezone": "auto",
        }
        url = "https://api.open-meteo.com/v1/forecast?" + urlencode(params)
        with urlopen(url, timeout=config.WEATHER_HTTP_TIMEOUT_SECONDS) as response:
            payload = json.loads(response.read().decode("utf-8"))

        current = payload.get("current", {}) or {}
        hourly = payload.get("hourly", {}) or {}
        current_time = str(current.get("time") or "")
        indexes = self._next_hour_indexes(hourly.get("time", []), current_time)
        rain_probs = self._hourly_values(hourly.get("precipitation_probability", []), indexes)
        precip = self._hourly_values(hourly.get("precipitation", []), indexes)

        data = {
            "temperature_c": self._to_float(current.get("temperature_2m")),
            "humidity_percent": self._to_float(current.get("relative_humidity_2m")),
            "wind_speed_ms": self._to_float(current.get("wind_speed_10m")),
            "rain_probability_1h": rain_probs[0] if rain_probs else None,
            "precipitation_1h_mm": precip[0] if precip else None,
            "rain_probability_3h": max(rain_probs) if rain_probs else None,
            "precipitation_3h_mm": sum(precip) if precip else None,
            "weather_code": self._to_int(current.get("weather_code")),
        }
        assessment, recommendation = evaluate_flight_weather(data)
        return EnvironmentReading(
            source="Open-Meteo",
            location_name=config.WEATHER_LOCATION_NAME,
            temperature_c=data["temperature_c"],
            humidity_percent=data["humidity_percent"],
            wind_speed_ms=data["wind_speed_ms"],
            rain_probability_1h=data["rain_probability_1h"],
            precipitation_1h_mm=data["precipitation_1h_mm"],
            rain_probability_3h=data["rain_probability_3h"],
            precipitation_3h_mm=data["precipitation_3h_mm"],
            weather_code=data["weather_code"],
            assessment=assessment,
            recommendation=recommendation,
            status_text="Đã cập nhật",
            updated_text=time.strftime("%H:%M"),
            has_data=True,
            is_stale=False,
        )

    def _handle_fetch_error(self, status_text: str) -> EnvironmentReading:
        if self.last_reading is not None and self.last_reading.has_data:
            stale = EnvironmentReading(
                source=self.last_reading.source,
                location_name=self.last_reading.location_name,
                temperature_c=self.last_reading.temperature_c,
                humidity_percent=self.last_reading.humidity_percent,
                wind_speed_ms=self.last_reading.wind_speed_ms,
                rain_probability_1h=self.last_reading.rain_probability_1h,
                precipitation_1h_mm=self.last_reading.precipitation_1h_mm,
                rain_probability_3h=self.last_reading.rain_probability_3h,
                precipitation_3h_mm=self.last_reading.precipitation_3h_mm,
                weather_code=self.last_reading.weather_code,
                assessment=self.last_reading.assessment,
                recommendation=self.last_reading.recommendation,
                status_text="Dữ liệu cũ",
                updated_text=self.last_reading.updated_text,
                has_data=True,
                is_stale=True,
            )
            self.last_reading = stale
            return stale
        self.last_reading = self._no_data(status_text)
        return self.last_reading

    @staticmethod
    def _no_data(status_text: str) -> EnvironmentReading:
        return EnvironmentReading(
            source="Open-Meteo",
            location_name=config.WEATHER_LOCATION_NAME,
            temperature_c=None,
            humidity_percent=None,
            wind_speed_ms=None,
            rain_probability_1h=None,
            precipitation_1h_mm=None,
            rain_probability_3h=None,
            precipitation_3h_mm=None,
            weather_code=None,
            assessment="KHÔNG CÓ DỮ LIỆU",
            recommendation="Không thể lấy dự báo thời tiết online",
            status_text=status_text,
            updated_text="--",
            has_data=False,
            is_stale=False,
        )

    @staticmethod
    def _next_hour_indexes(times: list[Any], current_time: str) -> list[int]:
        if not times:
            return []
        start = 0
        if current_time:
            for idx, item in enumerate(times):
                if str(item) >= current_time:
                    start = idx
                    break
        return list(range(start, min(start + 3, len(times))))

    @staticmethod
    def _hourly_values(values: list[Any], indexes: list[int]) -> list[float]:
        out: list[float] = []
        for idx in indexes:
            if idx >= len(values):
                continue
            value = WeatherProvider._to_float(values[idx])
            if value is not None:
                out.append(value)
        return out

    @staticmethod
    def _to_float(value: Any) -> float | None:
        try:
            if value is None:
                return None
            return float(value)
        except Exception:
            return None

    @staticmethod
    def _to_int(value: Any) -> int | None:
        try:
            if value is None:
                return None
            return int(value)
        except Exception:
            return None


class TelemetryProvider:
    def __init__(self) -> None:
        self.start = time.time()
        self._last_internet_check = 0.0
        self._internet_status = "Offline"

    def read(self, fps: float, mission_state: str) -> TelemetrySnapshot:
        laptop_battery, laptop_charging = self._read_laptop_power()
        internet_status = self._read_internet_status()
        gps_connected = False
        gps_lat = gps_lon = altitude = heading = None
        map_status = "Tạm tắt"
        victim_text = "Chưa có tọa độ"
        return TelemetrySnapshot(
            mission_state,
            laptop_battery,
            laptop_charging,
            internet_status,
            fps,
            gps_connected,
            gps_lat,
            gps_lon,
            altitude,
            heading,
            map_status,
            victim_text,
            "Chưa kết nối",
            time.strftime("%H:%M:%S"),
        )

    @staticmethod
    def _read_laptop_power() -> tuple[str, str]:
        try:
            import psutil

            battery = psutil.sensors_battery()
        except Exception:
            battery = None
        if battery is None:
            return "Không hỗ trợ", "Không hỗ trợ"
        percent = int(round(float(getattr(battery, "percent", 0.0))))
        plugged = bool(getattr(battery, "power_plugged", False))
        return f"{percent} %", "Có" if plugged else "Không"

    def _read_internet_status(self) -> str:
        now = time.time()
        if now - self._last_internet_check < config.INTERNET_CHECK_INTERVAL_SECONDS:
            return self._internet_status
        self._last_internet_check = now
        self._internet_status = "Online" if internet_available() else "Offline"
        return self._internet_status


def mission_state_from_targets(targets: list[TargetState]) -> str:
    if not targets:
        return "SEARCHING"
    if any(t.final_status == "CONFIRMED_VICTIM" for t in targets):
        return "VICTIM_CONFIRMED"
    if any(t.display_status == "DANGER" for t in targets):
        return "DANGER_ALERT"
    if any(t.display_status == "WARNING" for t in targets):
        return "WARNING"
    return "TRACKING"


