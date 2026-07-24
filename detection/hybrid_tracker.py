"""Hybrid YOLO + Sub-pixel Optical Flow + Anti-Jumping Color Histogram Lock Tracker."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import cv2
import numpy as np

from config import CONFIG, DetectionConfig, TrackerConfig
from detection.pixel_lock import PixelLockEngine
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
    source: str  # "pixel_lock" | "csrt" | "yolo" | "hold" | "lost"
    label: str
    conf: float
    detections: list[BBox]


class HybridYoloLockTracker:
    """Sub-pixel keypoint optical flow + color histogram anti-jumping lock tracker."""

    def __init__(
        self,
        det_cfg: DetectionConfig | None = None,
        tracker_cfg: TrackerConfig | None = None,
        cv_kind: CvKind = "csrt",
        yolo_every_n: int = 5,
        reacquire_iou: float = 0.25,
        max_hold_frames: int = 20,
    ) -> None:
        self.det_cfg = det_cfg or CONFIG.detection
        self.tcfg = tracker_cfg or CONFIG.tracker
        self.cv_kind = cv_kind
        self.yolo_every_n = max(1, int(yolo_every_n))
        self.reacquire_iou = reacquire_iou
        self.max_hold_frames = max_hold_frames

        self.detector: YOLODetector | None = None
        self.pixel_engine = PixelLockEngine()
        self._cv = None
        self._locked = False
        self._bbox: tuple[int, int, int, int] | None = None
        self._label = ""
        self._cls_id = -1
        self._target_hist: np.ndarray | None = None
        self._conf = 0.0
        self._frame_i = 0
        self._lost = 0
        self._last_dets: list[BBox] = []
        
        # Load YOLO model at startup
        self.ensure_detector()

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
        self.pixel_engine.initialized = False
        self._cv = None
        self._locked = False
        self._bbox = None
        self._label = ""
        self._cls_id = -1
        self._target_hist = None
        self._conf = 0.0
        self._lost = 0
        self._frame_i = 0

    def detect_only(self, frame: np.ndarray) -> list[BBox]:
        dets = self.ensure_detector().detect(frame)
        self._last_dets = dets
        return dets

    def _compute_hist(self, frame_bgr: np.ndarray, xywh: tuple[int, int, int, int]) -> np.ndarray | None:
        x, y, w, h = xywh
        img_h, img_w = frame_bgr.shape[:2]
        ix, iy, iw, ih = max(0, x), max(0, y), min(img_w - x, w), min(img_h - y, h)
        if iw < 6 or ih < 6:
            return None
        roi = frame_bgr[iy : iy + ih, ix : ix + iw]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1], None, [16, 16], [0, 180, 0, 256])
        cv2.normalize(hist, hist, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX)
        return hist

    def _compare_hist(self, frame_bgr: np.ndarray, candidate_box: BBox) -> float:
        if self._target_hist is None:
            return 1.0
        cand_hist = self._compute_hist(frame_bgr, candidate_box.as_int_xywh())
        if cand_hist is None:
            return 0.0
        return float(cv2.compareHist(self._target_hist, cand_hist, cv2.HISTCMP_CORREL))

    def lock_xywh(self, frame: np.ndarray, xywh: tuple[int, int, int, int], label: str = "manual") -> bool:
        x, y, w, h = [int(v) for v in xywh]
        if w < 8 or h < 8:
            return False
        self.pixel_engine.init_lock(frame, (x, y, w, h), label=label)
        self._start_cv(frame, (x, y, w, h))
        self._target_hist = self._compute_hist(frame, (x, y, w, h))
        self._bbox = (x, y, w, h)
        self._label = label
        self._cls_id = -1
        self._conf = 1.0
        self._locked = True
        self._lost = 0
        self._frame_i = 0
        return True

    def lock_bbox(self, frame: np.ndarray, box: BBox) -> bool:
        ok = self.lock_xywh(frame, box.as_int_xywh(), label=box.label or "yolo")
        if ok:
            self._cls_id = box.cls_id
        return ok

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

        # 1. Sub-pixel Pyramidal Optical Flow Update
        pix_ok, pix_box, pix_conf, _ = self.pixel_engine.update(frame)

        # 2. OpenCV CSRT/KCF Tracker Update
        cv_ok = False
        cv_box = None
        if self._cv is not None:
            tracking_ok, new_bb = self._cv.update(frame)
            if tracking_ok:
                x, y, w, h = [int(v) for v in new_bb]
                if w * h >= 36:
                    cv_box = (x, y, w, h)
                    cv_ok = True

        target_ok = False
        xywh = self._bbox
        source = "pixel_lock"

        if pix_ok and pix_box is not None:
            px, py, pw, ph = pix_box
            if cv_ok and cv_box is not None:
                cx, cy, cw, ch = cv_box
                f_x = int(round(0.75 * px + 0.25 * cx))
                f_y = int(round(0.75 * py + 0.25 * cy))
                f_w = int(round(0.75 * pw + 0.25 * cw))
                f_h = int(round(0.75 * ph + 0.25 * ch))
                xywh = (f_x, f_y, f_w, f_h)
            else:
                xywh = (int(round(px)), int(round(py)), int(round(pw)), int(round(ph)))
            target_ok = True
            self._conf = max(0.5, float(pix_conf))
        elif cv_ok and cv_box is not None:
            xywh = cv_box
            target_ok = True
            source = "csrt"

        # 3. Anti-Jumping Gated YOLO Verification
        if run_yolo and dets and xywh is not None:
            # Rank candidates by IoU + Appearance Histogram Similarity
            candidates = []
            for d in dets:
                iou = self._iou_xywh(xywh, d)
                if iou >= 0.10:
                    hist_sim = self._compare_hist(frame, d)
                    cls_match = 1.2 if (self._cls_id >= 0 and d.cls_id == self._cls_id) else 1.0
                    score = (iou * 0.6 + hist_sim * 0.4) * cls_match
                    candidates.append((score, iou, hist_sim, d))

            if candidates:
                candidates.sort(key=lambda item: item[0], reverse=True)
                best_score, best_iou, best_hist, best_box = candidates[0]

                # STRICT GATING: Only re-anchor if appearance matches AND (target lost OR high IoU)
                if target_ok:
                    # Target is currently tracked by optical flow -> DO NOT JUMP unless high match
                    if best_iou >= 0.40 and best_hist >= 0.40:
                        yolo_xywh = self._xywh_from_box(best_box)
                        self.pixel_engine.init_lock(frame, yolo_xywh, label=best_box.label or self._label)
                        self._start_cv(frame, yolo_xywh)
                        self._target_hist = self._compute_hist(frame, yolo_xywh)
                        xywh = yolo_xywh
                        self._conf = best_box.conf
                else:
                    # Target was lost -> require appearance match before locking
                    if best_hist >= 0.45:
                        yolo_xywh = self._xywh_from_box(best_box)
                        self.pixel_engine.init_lock(frame, yolo_xywh, label=best_box.label or self._label)
                        self._start_cv(frame, yolo_xywh)
                        self._target_hist = self._compute_hist(frame, yolo_xywh)
                        xywh = yolo_xywh
                        target_ok = True
                        self._conf = best_box.conf
                        source = "yolo"

        # 4. Result output & grace period hold
        if target_ok and xywh is not None:
            self._bbox = xywh
            self._lost = 0
            return HybridResult(True, xywh, source, self._label, self._conf, dets)
        # Grace period: Coast with last known box during brief dropouts
        self._lost += 1
        if self._lost <= self.max_hold_frames and self._bbox is not None:
            return HybridResult(True, self._bbox, "hold", self._label, self._conf * 0.85, dets)

        self._locked = False
        return HybridResult(False, self._bbox, "lost", self._label, 0.0, dets)

