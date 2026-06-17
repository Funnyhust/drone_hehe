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
class Detector:
    def __init__(self, root: Path) -> None:
        self.root = root
        self.model_pose = None
        self.model_posture = None
        self.model_rescue = None
        self.pose_available = False
        self.posture_available = False
        self.rescue_available = False
        self.rescue_model_version = 0
        self.rescue_model_name = "NONE"
        self.movement_history = defaultdict(deque)
        self.left_wrist_history = defaultdict(lambda: deque(maxlen=120))
        self.right_wrist_history = defaultdict(lambda: deque(maxlen=120))
        self._load_models()

    def model_summary(self) -> str:
        s = "MODEL:"
        if self.pose_available: s += " Pose"
        if self.posture_available: s += " + Posture"
        if self.rescue_available: s += " + Rescue"
        if not (self.pose_available or self.posture_available or self.rescue_available): return "MODEL: chua load duoc YOLO"
        return s

    def yolo_badge(self) -> str:
        parts = []
        if self.pose_available: parts.append("Pose")
        if self.posture_available: parts.append("Posture")
        if self.rescue_available: parts.append("Rescue")
        if not parts: return "Not loaded"
        return " + ".join(parts)

    def _load_models(self) -> None:
        try:
            from ultralytics import YOLO
        except Exception as exc:
            print(f"[CANH BAO] Chua co ultralytics YOLO: {exc}")
            return

        try:
            self.model_pose = YOLO(config.POSE_MODEL_PATH)
            self.pose_available = True
            print(f"[OK] Pose model: {config.POSE_MODEL_PATH}")
        except Exception as exc:
            print(f"[CANH BAO] Khong load duoc pose model: {exc}")
            self.model_pose = None

        posture_path = self._find_model(config.POSTURE_MODEL_CANDIDATES)
        if posture_path is not None:
            try:
                self.model_posture = YOLO(str(posture_path))
                self.posture_available = True
                print(f"[OK] Posture model: {posture_path.name}")
            except Exception as exc:
                print(f"[CANH BAO] Khong load duoc posture model: {exc}")
                self.model_posture = None

        rescue_path = self._find_model(config.RESCUE_MODEL_CANDIDATES)
        if rescue_path is None:
            print("[CANH BAO] Khong tim thay rescue_detection_v2/weights/best.pt. Van chay Pose + Posture neu co.")
        else:
            try:
                self.model_rescue = YOLO(str(rescue_path))
                nc = getattr(self.model_rescue.model, "nc", 2)
                self.rescue_model_version = 2
                self.rescue_model_name = rescue_path.name
                self.rescue_available = True
                print(f"[OK] Rescue V2 model: {rescue_path.name}, classes={nc}")
            except Exception as exc:
                print(f"[CANH BAO] Khong load duoc Rescue V2 model: {exc}")
                self.model_rescue = None

    def _find_model(self, candidates) -> Path | None:
        for candidate in candidates:
            path = Path(candidate)
            checks = [self.root / path, self.root.parent / path, self.root / "models" / path.name, path]
            for item in checks:
                if item.exists():
                    return item
        return None

    def detect(self, frame, timestamp: float) -> list[RawDetection]:
        if self.pose_available and self.model_pose is not None:
            return self._detect_pose_rescue(frame, timestamp)
        if config.SIMULATE_WHEN_MODEL_MISSING:
            return self._simulate(frame, timestamp)
        return []

    def _detect_pose_rescue(self, frame, timestamp: float) -> list[RawDetection]:
        pose_boxes = self._detect_pose_boxes(frame)
        if not pose_boxes:
            return []

        rescue_boxes = self._detect_rescue_boxes(frame)
        posture_boxes = self._detect_posture_boxes(frame)
        h, w = frame.shape[:2]

        detections: list[RawDetection] = []
        for pose_item in pose_boxes:
            raw_box = pose_item["bbox"]
            x1, y1, x2, y2 = [int(v) for v in raw_box]
            x1 = max(0, min(w - 1, x1))
            y1 = max(0, min(h - 1, y1))
            x2 = max(0, min(w, x2))
            y2 = max(0, min(h, y2))
            if x2 <= x1 or y2 <= y1:
                continue

            box = (x1, y1, x2, y2)
            object_id = int(pose_item["object_id"])
            kps_xy = pose_item.get("kps_xy")
            kps_conf = pose_item.get("kps_conf")
            pose = self._analyze_person_state(box, kps_xy, kps_conf, object_id, timestamp)
            pose["kps_xy"] = kps_xy
            pose["kps_conf"] = kps_conf
            pose["bbox"] = box  

            posture_match = self._best_match(box, posture_boxes)
            rescue_match = self._best_match(box, rescue_boxes)

            if posture_match is not None:
                pm_class = normalize_posture_class(posture_match.get("class_name", ""))
                pm_conf = float(posture_match.get("confidence", 0.0))
                pose["posture_v8_hint"] = pm_class
                pose["posture_v8_hint_conf"] = pm_conf
            else:
                pose["posture_v8_hint"] = "unknown"
                pose["posture_v8_hint_conf"] = 0.0

            posture_result = self._fuse_posture(posture_match, pose)
            pose.update(posture_result)
            if posture_match is not None:
                pose["posture_match"] = posture_match

            rescue_result = self._fuse_rescue(rescue_match, posture_result, pose)
            class_name = rescue_result["class_name"]
            confidence = rescue_result["confidence"]
            if rescue_match is None and class_name == "normal_person":
                confidence = float(pose_item["confidence"])
            pose.update(rescue_result)
            pose["rescue_match"] = {
                "is_victim": class_name == "victim",
                "is_high_conf": class_name == "victim" and confidence >= config.RESCUE_HIGH_CONF_THRESHOLD,
                "confidence": confidence,
                "class_name": class_name,
                "raw_class_name": (rescue_match or {}).get("class_name", "missing"),
                "raw_confidence": float((rescue_match or {}).get("confidence", 0.0)),
            }

            detections.append(RawDetection(object_id, box, class_name, confidence, pose))

        return detections

    def _detect_pose_boxes(self, frame) -> list[dict]:
        if not self.pose_available or self.model_pose is None:
            return []
        try:
            results = self.model_pose.track(
                source=frame,
                persist=True,
                conf=config.POSE_CONFIDENCE,
                tracker=config.TRACKER_CONFIG,
                show=False,
                verbose=False,
            )
        except Exception:
            return []
        if not results:
            return []
        result = results[0]
        boxes = getattr(result, "boxes", None)
        keypoints = getattr(result, "keypoints", None)
        if boxes is None or boxes.xyxy is None or keypoints is None:
            return []

        xyxy = boxes.xyxy.cpu().numpy().astype(int)
        confs = boxes.conf.cpu().numpy()
        ids = boxes.id.cpu().numpy().astype(int) if boxes.id is not None else list(range(len(xyxy)))
        kps_xy_all = keypoints.xy.cpu().numpy()
        kps_conf_all = keypoints.conf.cpu().numpy() if keypoints.conf is not None else None
        out = [
            {
                "bbox": tuple(int(v) for v in box),
                "object_id": int(ids[index]),
                "confidence": float(confs[index]),
                "kps_xy": kps_xy_all[index],
                "kps_conf": kps_conf_all[index] if kps_conf_all is not None else None,
            }
            for index, box in enumerate(xyxy)
        ]
        
        if len(out) <= 1:
            return out
        out.sort(key=lambda x: x["confidence"], reverse=True)
        kept = []
        suppressed = set()
        for i, a in enumerate(out):
            if i in suppressed:
                continue
            kept.append(a)
            a_area = max(1, (a["bbox"][2] - a["bbox"][0]) * (a["bbox"][3] - a["bbox"][1]))
            for j, b in enumerate(out):
                if j <= i or j in suppressed:
                    continue
                b_area = max(1, (b["bbox"][2] - b["bbox"][0]) * (b["bbox"][3] - b["bbox"][1]))
                x_a = max(a["bbox"][0], b["bbox"][0])
                y_a = max(a["bbox"][1], b["bbox"][1])
                x_b = min(a["bbox"][2], b["bbox"][2])
                y_b = min(a["bbox"][3], b["bbox"][3])
                inter = max(0, x_b - x_a) * max(0, y_b - y_a)
                min_area = min(a_area, b_area)
                ioa = inter / min_area if min_area > 0 else 0
                if ioa >= 0.40:
                    suppressed.add(j)
        return kept


    def _detect_posture_boxes(self, frame) -> list[dict]:
        if not self.posture_available or self.model_posture is None:
            return []
        try:
            results = self.model_posture.track(
                source=frame,
                persist=True,
                conf=config.POSTURE_CONFIDENCE,
                tracker=config.TRACKER_CONFIG,
                show=False,
                verbose=False,
            )
        except Exception:
            return []
        if not results or results[0].boxes is None:
            return []

        boxes = results[0].boxes
        xyxy = boxes.xyxy.cpu().numpy().astype(int)
        confs = boxes.conf.cpu().numpy()
        cls_values = boxes.cls.cpu().numpy().astype(int) if boxes.cls is not None else [0] * len(xyxy)
        ids = boxes.id.cpu().numpy().astype(int) if boxes.id is not None else list(range(len(xyxy)))
        names = getattr(results[0], "names", None) or getattr(self.model_posture, "names", {}) or {}

        out = []
        for index, (box, conf, cls_id) in enumerate(zip(xyxy, confs, cls_values)):
            raw_name = str(names.get(int(cls_id), int(cls_id))).strip().lower()
            cname = normalize_posture_class(raw_name)
            if cname == "unknown":
                continue
            out.append({
                "bbox": tuple(int(v) for v in box),
                "confidence": float(conf),
                "class_name": cname,
                "object_id": int(ids[index]),
            })
        if len(out) <= 1:
            return out
        out.sort(key=lambda x: x["confidence"], reverse=True)
        kept = []
        suppressed = set()
        for i, a in enumerate(out):
            if i in suppressed:
                continue
            kept.append(a)
            for j, b in enumerate(out):
                if j <= i or j in suppressed:
                    continue
                if iou_overlap(a["bbox"], b["bbox"]) >= 0.30:
                    suppressed.add(j)
        return kept

    def _detect_rescue_boxes(self, frame) -> list[dict]:
        if not self.rescue_available or self.model_rescue is None:
            return []
        try:
            results = self.model_rescue.predict(
                source=frame,
                conf=config.RESCUE_CONFIDENCE,
                show=False,
                verbose=False,
            )
        except Exception:
            return []
        if not results or results[0].boxes is None:
            return []

        boxes = results[0].boxes
        xyxy = boxes.xyxy.cpu().numpy().astype(int)
        confs = boxes.conf.cpu().numpy()
        cls_values = boxes.cls.cpu().numpy().astype(int) if boxes.cls is not None else [0] * len(xyxy)
        names = getattr(results[0], "names", None) or getattr(self.model_rescue, "names", {}) or {}

        out = []
        for box, conf, cls_id in zip(xyxy, confs, cls_values):
            raw_class_name = str(names.get(int(cls_id), int(cls_id))).strip().lower()
            class_name = normalize_rescue_class(raw_class_name, int(cls_id))
            is_victim = class_name == "victim"
            is_person = True
            out.append(
                {
                    "bbox": tuple(int(v) for v in box),
                    "confidence": float(conf),
                    "is_victim": bool(is_victim),
                    "is_person": bool(is_person),
                    "is_high_conf": bool(is_victim) and float(conf) >= config.RESCUE_HIGH_CONF_THRESHOLD,
                    "class_name": class_name,
                }
            )
        return out

    @staticmethod
    def _pose_posture_confidence(pose: dict) -> float:
        status = str(pose.get("posture_status", "unknown") or "unknown")
        if status == "unknown":
            return 0.0
        confidence = str(pose.get("posture_confidence", "weak") or "weak")
        base = 0.80 if confidence == "strong" else 0.60
        kps_conf = pose.get("kps_conf")
        if kps_conf is not None:
            try:
                arr = np.asarray(kps_conf)
                visible_count = float(np.sum(arr >= 0.30))
                visible_ratio = min(1.0, visible_count / 9.0)  
                base *= 0.70 + 0.30 * visible_ratio
            except Exception:
                pass
        return float(clamp(base, 0.0, 1.0))

    def _fuse_posture(self, posture_match: dict | None, pose: dict) -> dict:
        scores = {"standing": 0.0, "sitting": 0.0, "lying_or_fallen": 0.0}
        custom_status = "unknown"
        custom_conf = 0.0
        has_posture_match = posture_match is not None

        if has_posture_match:
            custom_status = {
                "standing_person": "standing",
                "sitting_person": "sitting",
                "lying_person": "lying_or_fallen",
                "stand": "standing",
                "sit": "sitting",
                "lie": "lying_or_fallen",
            }.get(normalize_posture_class(posture_match.get("class_name", "unknown")), "unknown")
            custom_conf = float(clamp(float(posture_match.get("confidence", 0.0)), 0.0, 1.0))
            if custom_status in scores:
                scores[custom_status] += custom_conf * config.POSTURE_MODEL_WEIGHT

            if custom_conf >= 0.55 and custom_status != "unknown":
                return {
                    "posture_status": custom_status,
                    "posture_class": {
                        "standing": "standing_person",
                        "sitting": "sitting_person",
                        "lying_or_fallen": "lying_person",
                    }.get(custom_status, "unknown"),
                    "posture_confidence": "strong",
                    "posture_fusion_score": round(custom_conf * config.POSTURE_MODEL_WEIGHT, 3),
                    "posture_model_status": custom_status,
                    "posture_model_confidence": round(custom_conf, 3),
                    "pose_posture_status": str(pose.get("posture_status", "unknown") or "unknown"),
                    "pose_posture_confidence": 0.0,
                    "posture_votes": scores,
                    "posture_match_found": True,
                }
        else:
            hint_status = {
                "standing_person": "standing",
                "sitting_person": "sitting",
                "lying_person": "lying_or_fallen",
                "stand": "standing",
                "sit": "sitting",
                "lie": "lying_or_fallen",
            }.get(str(pose.get("posture_v8_hint", "unknown")), "unknown")
            hint_conf = float(pose.get("posture_v8_hint_conf", 0.0))
            if hint_status in scores and hint_conf > 0:
                scores[hint_status] += hint_conf * config.POSTURE_MODEL_WEIGHT * 0.75

        pose_status = str(pose.get("posture_status", "unknown") or "unknown")
        pose_conf = self._pose_posture_confidence(pose)
        effective_pose_weight = config.POSE_POSTURE_WEIGHT if has_posture_match else min(0.50, config.POSE_POSTURE_WEIGHT + config.POSTURE_MODEL_WEIGHT * 0.35)
        if pose_status in scores:
            scores[pose_status] += pose_conf * effective_pose_weight

        pose_box = pose.get("bbox")
        if pose_box is not None and not has_posture_match:
            try:
                bx1, by1, bx2, by2 = pose_box
                bh = max(1, by2 - by1)
                bw = max(1, bx2 - bx1)
                ratio = bh / bw
                if 0.80 <= ratio < 1.65:
                    if scores["sitting"] > 0.15:
                        scores["sitting"] += 0.10
                elif ratio >= 1.65:
                    if scores["standing"] > 0.15:
                        scores["standing"] += 0.08
            except Exception:
                pass

        ordered = sorted(scores.items(), key=lambda item: item[1], reverse=True)
        best_status, best_score = ordered[0]
        second_score = ordered[1][1]
        margin = best_score - second_score
        min_conf = config.POSTURE_FUSION_MIN_CONFIDENCE if has_posture_match else (config.POSTURE_FUSION_MIN_CONFIDENCE - 0.08)
        min_margin = config.POSTURE_FUSION_MIN_MARGIN if has_posture_match else (config.POSTURE_FUSION_MIN_MARGIN * 0.5)
        if best_score < min_conf or margin < min_margin:
            best_status = "unknown"

        return {
            "posture_status": best_status,
            "posture_class": {
                "standing": "standing_person",
                "sitting": "sitting_person",
                "lying_or_fallen": "lying_person",
            }.get(best_status, "unknown"),
            "posture_confidence": "strong" if best_score >= 0.55 else "weak",
            "posture_fusion_score": round(best_score, 3),
            "posture_model_status": custom_status,
            "posture_model_confidence": round(custom_conf, 3),
            "pose_posture_status": pose_status,
            "pose_posture_confidence": round(pose_conf, 3),
            "posture_votes": {key: round(value, 3) for key, value in scores.items()},
            "posture_match_found": has_posture_match,
        }

    @staticmethod
    def _pose_victim_evidence(pose: dict) -> float:
        evidence = 0.0
        if pose.get("is_lying"):
            evidence = max(evidence, 0.72)
        if pose.get("is_collapsed"):
            evidence = max(evidence, 0.82)
        if pose.get("is_immobile") and (pose.get("is_lying") or pose.get("is_collapsed")):
            evidence = max(evidence, 0.90)
        gesture = str(pose.get("gesture_candidate", "none") or "none")
        if gesture == "two_hands_waving":
            evidence = max(evidence, 0.90)
        elif gesture == "two_hands_raised":
            evidence = max(evidence, 0.65)
        elif gesture == "one_hand_raised":
            evidence = max(evidence, 0.25)
        return evidence

    def _fuse_rescue(self, rescue_match: dict | None, posture: dict, pose: dict) -> dict:
        rescue_class = str((rescue_match or {}).get("class_name", "normal_person"))
        rescue_conf = float(clamp(float((rescue_match or {}).get("confidence", 0.0)), 0.0, 1.0))
        rescue_victim = rescue_conf if normalize_rescue_class(rescue_class) == "victim" else 0.0

        posture_victim = 0.0
        if posture.get("posture_status") == "lying_or_fallen":
            posture_victim = float(clamp(float(posture.get("posture_fusion_score", 0.0)) / config.POSTURE_MODEL_WEIGHT, 0.0, 1.0))
        pose_victim = self._pose_victim_evidence(pose)

        victim_probability = 1.0 - (
            (1.0 - rescue_victim)
            * (1.0 - 0.78 * posture_victim)
            * (1.0 - 0.65 * pose_victim)
        )
        class_name = "victim" if victim_probability >= config.VICTIM_FUSION_THRESHOLD else "normal_person"
        if class_name == "victim":
            confidence = victim_probability
        else:
            pose_conf = float(pose.get("confidence", 0.0))
            if normalize_rescue_class(rescue_class) == "normal_person" and rescue_conf > 0:
                confidence = max(rescue_conf, pose_conf)
            else:
                confidence = pose_conf

        return {
            "class_name": class_name,
            "confidence": float(clamp(confidence, 0.0, 1.0)),
            "victim_probability": round(victim_probability, 3),
            "rescue_v2_victim_evidence": round(rescue_victim, 3),
            "posture_victim_evidence": round(posture_victim, 3),
            "pose_victim_evidence": round(pose_victim, 3),
            "source": "rescue_v2+posture_v8+pose_v8",
        }

    def _best_match(self, pose_box, boxes: list[dict]) -> dict | None:
        best = None
        best_iou = 0.0
        for item in boxes:
            overlap = iou_overlap(pose_box, item["bbox"])
            if overlap > best_iou:
                best_iou = overlap
                best = item
        if best is None or best_iou < config.RESCUE_IOU_THRESHOLD:
            return None
        matched = dict(best)
        matched["iou"] = best_iou
        return matched

    def _analyze_person_state(self, box, kps_xy, kps_conf, box_id: int, current_time: float) -> dict:
        x1, y1, x2, y2 = box
        box_w = max(1, x2 - x1)
        box_h = max(1, y2 - y1)
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)
        reasons = []
        danger_score = 0

        is_lying = False
        if box_w / box_h > 1.8:
            ls_early = get_keypoint(kps_xy, kps_conf, 5, min_conf=0.25)
            rs_early = get_keypoint(kps_xy, kps_conf, 6, min_conf=0.25)
            lw_early = get_keypoint(kps_xy, kps_conf, 9, min_conf=0.25)
            rw_early = get_keypoint(kps_xy, kps_conf, 10, min_conf=0.25)
            shoulder_y_avg = None
            if ls_early and rs_early:
                shoulder_y_avg = (ls_early[1] + rs_early[1]) / 2.0
            elif ls_early:
                shoulder_y_avg = ls_early[1]
            elif rs_early:
                shoulder_y_avg = rs_early[1]
            is_raising_now = False
            if shoulder_y_avg is not None:
                if (lw_early and lw_early[1] < shoulder_y_avg - 15) or (rw_early and rw_early[1] < shoulder_y_avg - 15):
                    is_raising_now = True
            nose_check = get_keypoint(kps_xy, kps_conf, 0, min_conf=0.3)
            if not is_raising_now:
                if nose_check is None or nose_check[1] >= y1 + box_h * 0.45:
                    is_lying = True
                    danger_score += 2
                    reasons.append("nam/nga")

        left_shoulder = get_keypoint(kps_xy, kps_conf, 5)
        right_shoulder = get_keypoint(kps_xy, kps_conf, 6)
        left_wrist = get_keypoint(kps_xy, kps_conf, 9)
        right_wrist = get_keypoint(kps_xy, kps_conf, 10)
        nose = get_keypoint(kps_xy, kps_conf, 0)
        left_hip = get_keypoint(kps_xy, kps_conf, 11)
        right_hip = get_keypoint(kps_xy, kps_conf, 12)
        left_knee = get_keypoint(kps_xy, kps_conf, 13)
        right_knee = get_keypoint(kps_xy, kps_conf, 14)
        left_ankle = get_keypoint(kps_xy, kps_conf, 15)
        right_ankle = get_keypoint(kps_xy, kps_conf, 16)

        is_raising_hand = False
        is_one_hand_raised = False
        is_two_hands_raised = False
        is_waving = False
        gesture_candidate = "none"
        left_wrist_sample = None
        right_wrist_sample = None
        shoulder_points = [p for p in (left_shoulder, right_shoulder) if p is not None]
        if shoulder_points:
            shoulder_y = sum(p[1] for p in shoulder_points) / len(shoulder_points)
            raised_count = 0
            left_above = False
            right_above = False
            if left_wrist is not None and left_wrist[1] < shoulder_y - config.HAND_RAISE_MARGIN_PIXELS:
                raised_count += 1
                left_above = True
            if right_wrist is not None and right_wrist[1] < shoulder_y - config.HAND_RAISE_MARGIN_PIXELS:
                raised_count += 1
                right_above = True

            if left_wrist is not None:
                left_wrist_sample = (current_time, float(left_wrist[0]), float(left_wrist[1]), bool(left_above))
                self.left_wrist_history[box_id].append(left_wrist_sample)
            if right_wrist is not None:
                right_wrist_sample = (current_time, float(right_wrist[0]), float(right_wrist[1]), bool(right_above))
                self.right_wrist_history[box_id].append(right_wrist_sample)

            left_wave = detect_wrist_waving(self.left_wrist_history[box_id], current_time)
            right_wave = detect_wrist_waving(self.right_wrist_history[box_id], current_time)
            if raised_count >= 2:
                is_raising_hand = True
                is_two_hands_raised = True
                gesture_candidate = "two_hands_raised"
                danger_score += 3
                reasons.append("2 tay gio len")
            elif raised_count == 1:
                is_raising_hand = True
                is_one_hand_raised = True
                gesture_candidate = "one_hand_raised"
                danger_score += 1
                reasons.append("gio tay")

            if raised_count >= 2 and left_wave and right_wave:
                is_waving = True
                gesture_candidate = "two_hands_waving"
                danger_score += 5
                reasons.append("2 tay vay cau cuu")

        hip_points = [p for p in (left_hip, right_hip) if p is not None]
        knee_points = [p for p in (left_knee, right_knee) if p is not None]
        ankle_points = [p for p in (left_ankle, right_ankle) if p is not None]
        is_collapsed = False
        is_sitting_pose = False
        is_standing_pose = False
        posture_confidence = "weak"
        if hip_points and shoulder_points:
            avg_hip_y = sum(p[1] for p in hip_points) / len(hip_points)
            avg_hip_x = sum(p[0] for p in hip_points) / len(hip_points)
            avg_shoulder_y = sum(p[1] for p in shoulder_points) / len(shoulder_points)
            avg_shoulder_x = sum(p[0] for p in shoulder_points) / len(shoulder_points)
            torso_height = abs(avg_hip_y - avg_shoulder_y)
            spine_x_span = abs(avg_hip_x - avg_shoulder_x)

            if len(shoulder_points) == 2:
                shoulder_width = abs(shoulder_points[0][0] - shoulder_points[1][0])
            else:
                shoulder_width = max(30.0, box_w * 0.4)

            body_points_torso = shoulder_points + hip_points + knee_points + ankle_points
            shoulder_hip_gap = abs(avg_hip_y - avg_shoulder_y)
            torso_is_horizontal = shoulder_hip_gap < box_h * 0.20
            if len(body_points_torso) >= 4:
                body_x_span = max(p[0] for p in body_points_torso) - min(p[0] for p in body_points_torso)
                body_y_span = max(p[1] for p in body_points_torso) - min(p[1] for p in body_points_torso)
                if body_x_span > max(45, body_y_span * config.LYING_KEYPOINT_RATIO_THRESHOLD) and torso_is_horizontal:
                    is_lying = True
                    danger_score += 2
                    reasons.append("truc co the nam ngang")
            if spine_x_span > max(40, torso_height * 1.35) and torso_height < box_h * 0.35 and torso_is_horizontal:
                is_lying = True
                danger_score += 2
                reasons.append("than nguoi nam ngang")

            if knee_points and not is_lying:
                avg_knee_y = sum(p[1] for p in knee_points) / len(knee_points)
                head_to_hip_y = (avg_hip_y - nose[1]) if nose else (torso_height * 1.3)
                head_to_hip_y = max(1.0, head_to_hip_y)
                upper_leg_y = max(1.0, avg_knee_y - avg_hip_y)
                vertical_ratio = box_h / max(1.0, float(box_w))

                if ankle_points:
                    avg_ankle_y = sum(p[1] for p in ankle_points) / len(ankle_points)
                    lower_leg_y = max(0.0, avg_ankle_y - avg_knee_y)
                    if lower_leg_y < max(35.0, upper_leg_y * 0.60) and vertical_ratio < 2.3:
                        is_sitting_pose = True
                        posture_confidence = "strong"
                        reasons.append("dang ngoi: chan gap")
                    elif (
                        not is_sitting_pose
                        and vertical_ratio >= 1.45
                        and lower_leg_y >= upper_leg_y * 0.70
                        and upper_leg_y >= head_to_hip_y * 0.40
                    ):
                        is_standing_pose = True
                        posture_confidence = "strong"
                        reasons.append("dang dung: thay truc chan thang")
                    elif not is_sitting_pose and upper_leg_y <= head_to_hip_y * 0.35 and vertical_ratio < 2.6:
                        is_sitting_pose = True
                        posture_confidence = "weak"
                        reasons.append("co the ngoi: dui ngan du thay co chan")
                else:
                    if upper_leg_y <= head_to_hip_y * 0.35 and vertical_ratio < 2.6:
                        is_sitting_pose = True
                        posture_confidence = "strong"
                        reasons.append("dang ngoi: dui ngan gap goc (khuat co chan)")
                    elif upper_leg_y >= head_to_hip_y * 0.45 and vertical_ratio >= 1.3 and not is_sitting_pose:
                        is_standing_pose = True
                        posture_confidence = "strong"
                        reasons.append("dang dung: dui dai doc xuong (khuat co chan)")

            head_tilted = False
            if nose is not None:
                if abs(nose[0] - avg_shoulder_x) > max(30.0, shoulder_width * 0.65):
                    head_tilted = True
                elif nose[1] - avg_shoulder_y > max(15.0, torso_height * 0.2):
                    head_tilted = True
            body_leaning = False
            if spine_x_span > max(35.0, torso_height * 0.45):
                body_leaning = True
                    
            torso_crushed = torso_height / max(1.0, float(box_h)) < 0.15
            if head_tilted or torso_crushed or body_leaning:
                is_collapsed = True
                danger_score += 2
                reasons.append("guc dau / nghieng / sup do")

        history = self.movement_history[box_id]
        history.append((current_time, cx, cy))
        while history and current_time - history[0][0] > config.STILLNESS_SECONDS:
            history.popleft()

        is_immobile = False
        if len(history) >= 5:
            times = [p[0] for p in history]
            xs = [p[1] for p in history]
            ys = [p[2] for p in history]
            duration = max(times) - min(times)
            if duration >= config.STILLNESS_SECONDS - 0.5 and max(xs) - min(xs) < config.CENTER_JITTER_THRESHOLD and max(ys) - min(ys) < config.CENTER_JITTER_THRESHOLD:
                is_immobile = True
                if is_lying or is_collapsed:
                    danger_score += 1
                    reasons.append("bat dong")

        if is_lying and is_immobile:
            danger_score += 2
            reasons.append("nam lau")
        if is_raising_hand and is_immobile:
            danger_score += 1
            reasons.append("gio tay + bat dong")

        posture_status = "unknown"
        posture_confidence = "weak"

        if danger_score >= 4:
            state = "CAN_CUU_GIUP"
        elif danger_score >= 2:
            state = "NGHI_NGO"
        else:
            state = "BINH_THUONG"

        return {
            "source": "pose_rescue",
            "state": state,
            "danger_score": int(danger_score),
            "reasons": reasons,
            "is_lying": is_lying,
            "is_immobile": is_immobile,
            "is_raising_hand": is_raising_hand,
            "is_one_hand_raised": is_one_hand_raised,
            "is_two_hands_raised": is_two_hands_raised,
            "is_waving": is_waving,
            "is_collapsed": is_collapsed,
            "posture_status": posture_status,
            "posture_confidence": posture_confidence,
            "gesture_candidate": gesture_candidate,
            "left_wrist_sample": left_wrist_sample,
            "right_wrist_sample": right_wrist_sample,
        }

    def _simulate(self, frame, timestamp: float) -> list[RawDetection]:
        h, w = frame.shape[:2]
        cx = int(w * (0.45 + 0.12 * math.sin(timestamp / 3.0)))
        cy = int(h * 0.54)
        return [
            RawDetection(
                1,
                (cx - 95, cy - 42, cx + 95, cy + 42),
                "victim",
                0.82,
                {
                    "source": "simulated",
                    "state": "CAN_CUU_GIUP",
                    "danger_score": 4,
                    "reasons": ["demo victim"],
                    "is_lying": True,
                    "is_immobile": False,
                    "is_raising_hand": False,
                    "is_one_hand_raised": False,
                    "is_two_hands_raised": False,
                    "is_waving": False,
                    "is_collapsed": False,
                    "posture_status": "lying_or_fallen",
                    "gesture_candidate": "none",
                    "left_wrist_sample": None,
                    "right_wrist_sample": None,
                    "rescue_match": {"is_high_conf": True},
                },
            )
        ]


