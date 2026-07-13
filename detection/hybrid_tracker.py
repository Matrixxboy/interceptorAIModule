"""
Hybrid YOLO + OpenCV lock tracker for MSP intercept loop.

- YOLO finds / refreshes the target (accurate, slower)
- KCF/CSRT fills frames between YOLO runs (fast)
- IoU reacquire when OpenCV drifts or loses lock
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import cv2
import numpy as np

from config import CONFIG, DetectionConfig, TrackerConfig
from detection.yolo_detector import YOLODetector
from utils.helpers import BBox
from utils.logger import setup_logger

log = setup_logger("cuas.hybrid")

CvKind = Literal["kcf", "csrt"]


def _create_cv_tracker(kind: str):
    kind = kind.lower()
    creators = []
    if kind == "csrt":
        creators = [
            lambda: cv2.TrackerCSRT_create(),
            lambda: cv2.legacy.TrackerCSRT_create(),
        ]
    else:
        creators = [
            lambda: cv2.TrackerKCF_create(),
            lambda: cv2.legacy.TrackerKCF_create(),
        ]
    for fn in creators:
        try:
            return fn()
        except Exception:  # noqa: BLE001
            continue
    return None


@dataclass
class HybridResult:
    ok: bool
    bbox_xywh: tuple[int, int, int, int] | None
    source: str  # "yolo" | "opencv" | "iou" | "lost"
    label: str
    conf: float
    detections: list[BBox]


class HybridYoloLockTracker:
    """Manual ROI or YOLO auto-lock with periodic YOLO refresh."""

    def __init__(
        self,
        det_cfg: DetectionConfig | None = None,
        tracker_cfg: TrackerConfig | None = None,
        cv_kind: CvKind = "kcf",
        yolo_every_n: int = 3,
        reacquire_iou: float = 0.15,
    ) -> None:
        self.det_cfg = det_cfg or CONFIG.detection
        self.tcfg = tracker_cfg or CONFIG.tracker
        self.cv_kind = cv_kind
        self.yolo_every_n = max(1, int(yolo_every_n))
        self.reacquire_iou = reacquire_iou

        self.detector: YOLODetector | None = None
        self._cv = None
        self._locked = False
        self._bbox: tuple[int, int, int, int] | None = None
        self._label = ""
        self._conf = 0.0
        self._frame_i = 0
        self._lost = 0
        self._last_dets: list[BBox] = []

    def ensure_detector(self) -> YOLODetector:
        if self.detector is None:
            log.info("Loading YOLO for hybrid lock…")
            self.detector = YOLODetector(self.det_cfg)
        return self.detector

    @property
    def locked(self) -> bool:
        return self._locked

    @property
    def bbox(self) -> tuple[int, int, int, int] | None:
        return self._bbox

    def reset(self) -> None:
        self._cv = None
        self._locked = False
        self._bbox = None
        self._label = ""
        self._conf = 0.0
        self._lost = 0
        self._frame_i = 0

    def detect_only(self, frame: np.ndarray) -> list[BBox]:
        dets = self.ensure_detector().detect(frame)
        self._last_dets = dets
        return dets

    def lock_xywh(self, frame: np.ndarray, xywh: tuple[int, int, int, int], label: str = "manual") -> bool:
        x, y, w, h = [int(v) for v in xywh]
        if w < 8 or h < 8:
            return False
        self._start_cv(frame, (x, y, w, h))
        self._bbox = (x, y, w, h)
        self._label = label
        self._conf = 1.0
        self._locked = True
        self._lost = 0
        self._frame_i = 0
        return True

    def lock_bbox(self, frame: np.ndarray, box: BBox) -> bool:
        return self.lock_xywh(frame, box.as_int_xywh(), label=box.label or "yolo")

    def lock_nearest(self, frame: np.ndarray, x: float, y: float) -> BBox | None:
        dets = self.detect_only(frame)
        if not dets:
            return None
        best = min(dets, key=lambda b: (b.cx - x) ** 2 + (b.cy - y) ** 2)
        self.lock_bbox(frame, best)
        return best

    def lock_best(self, frame: np.ndarray) -> BBox | None:
        dets = self.detect_only(frame)
        if not dets:
            return None
        best = max(dets, key=lambda b: b.conf)
        self.lock_bbox(frame, best)
        return best

    def _start_cv(self, frame: np.ndarray, xywh: tuple[int, int, int, int]) -> None:
        tracker = _create_cv_tracker(self.cv_kind)
        if tracker is None:
            self._cv = None
            return
        ok = tracker.init(frame, tuple(int(v) for v in xywh))
        self._cv = tracker if ok else None

    def _xywh_from_box(self, box: BBox) -> tuple[int, int, int, int]:
        return box.as_int_xywh()

    def _iou_xywh(self, a: tuple[int, int, int, int], b: BBox) -> float:
        ax, ay, aw, ah = a
        a_box = BBox(ax, ay, ax + aw, ay + ah)
        return a_box.iou(b)

    def update(self, frame: np.ndarray) -> HybridResult:
        self._frame_i += 1
        dets: list[BBox] = []
        run_yolo = (self._frame_i % self.yolo_every_n) == 0

        if run_yolo or not self._locked:
            try:
                dets = self.detect_only(frame)
            except Exception as exc:  # noqa: BLE001
                log.warning("YOLO detect failed: %s", exc)
                dets = self._last_dets

        if not self._locked:
            return HybridResult(False, None, "lost", "", 0.0, dets)

        source = "opencv"
        ok = False
        xywh = self._bbox

        # 1) OpenCV update
        if self._cv is not None:
            tracking_ok, new_bb = self._cv.update(frame)
            if tracking_ok:
                x, y, w, h = [int(v) for v in new_bb]
                if w * h >= 64:
                    xywh = (x, y, w, h)
                    ok = True
                    source = "opencv"

        # 2) Periodic YOLO refresh / reacquire by IoU
        if run_yolo and dets and xywh is not None:
            best_iou = 0.0
            best: BBox | None = None
            for d in dets:
                iou = self._iou_xywh(xywh, d)
                if iou > best_iou:
                    best_iou = iou
                    best = d
            # Also allow nearest-center if IoU weak but close
            if best is None or best_iou < self.reacquire_iou:
                cx = xywh[0] + xywh[2] * 0.5
                cy = xywh[1] + xywh[3] * 0.5
                near = min(dets, key=lambda b: (b.cx - cx) ** 2 + (b.cy - cy) ** 2)
                dist = ((near.cx - cx) ** 2 + (near.cy - cy) ** 2) ** 0.5
                max_jump = max(xywh[2], xywh[3]) * 2.5
                if dist < max_jump:
                    best = near
                    best_iou = max(best_iou, self._iou_xywh(xywh, near))

            if best is not None and (best_iou >= self.reacquire_iou or not ok):
                xywh = self._xywh_from_box(best)
                self._start_cv(frame, xywh)
                self._label = best.label or self._label
                self._conf = best.conf
                ok = True
                source = "yolo" if best_iou < 0.5 else "iou"

        if ok and xywh is not None:
            self._bbox = xywh
            self._lost = 0
            return HybridResult(True, xywh, source, self._label, self._conf, dets)

        self._lost += 1
        return HybridResult(False, self._bbox, "lost", self._label, self._conf, dets)
