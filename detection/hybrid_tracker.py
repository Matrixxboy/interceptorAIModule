"""Hybrid YOLO + scale-aware lock + optical-flow assist tracker."""

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
from vision.scale_aware_lock import ScaleAwareLock

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
    source: str  # "scale_lock" | "pixel_lock" | "csrt" | "yolo" | "hold" | "lost"
    label: str
    conf: float
    detections: list[BBox]


class HybridYoloLockTracker:
    """Scale-aware box size (distance-grade) + flow/CSRT for center, YOLO gated."""

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
        # Prefer config lock_tracker when set
        cfg_kind = getattr(self.tcfg, "lock_tracker", cv_kind)
        self.cv_kind: CvKind = cfg_kind if cfg_kind in ("csrt", "kcf") else cv_kind
        self.yolo_every_n = max(1, int(yolo_every_n))
        self.reacquire_iou = reacquire_iou
        self.max_hold_frames = max_hold_frames

        self.detector: YOLODetector | None = None
        self.pixel_engine = PixelLockEngine()
        # Size authority for distance — no CSRT inside (hybrid blends center separately)
        self.scale_lock = ScaleAwareLock(use_csrt=False)
        self._cv = None
        self._locked = False
        self._bbox: tuple[int, int, int, int] | None = None
        self._bbox_f: tuple[float, float, float, float] | None = None
        self._label = ""
        self._cls_id = -1
        self._target_hist: np.ndarray | None = None
        self._conf = 0.0
        self._frame_i = 0
        self._lost = 0
        self._last_dets: list[BBox] = []
        self._manual_lock = False

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
        self.scale_lock.reset()
        self._cv = None
        self._locked = False
        self._bbox = None
        self._bbox_f = None
        self._label = ""
        self._cls_id = -1
        self._target_hist = None
        self._conf = 0.0
        self._lost = 0
        self._frame_i = 0
        self._manual_lock = False

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

    @staticmethod
    def _as_int(xywh: tuple[float, float, float, float]) -> tuple[int, int, int, int]:
        x, y, w, h = xywh
        return (int(round(x)), int(round(y)), max(1, int(round(w))), max(1, int(round(h))))

    def _set_bbox(self, xywh: tuple[float, float, float, float]) -> tuple[int, int, int, int]:
        self._bbox_f = (float(xywh[0]), float(xywh[1]), float(xywh[2]), float(xywh[3]))
        self._bbox = self._as_int(self._bbox_f)
        return self._bbox

    def lock_xywh(self, frame: np.ndarray, xywh: tuple[int, int, int, int], label: str = "manual") -> bool:
        x, y, w, h = [int(round(float(v))) for v in xywh]
        if w < 8 or h < 8:
            return False
        box = (x, y, w, h)
        self.pixel_engine.init_lock(frame, box, label=label)
        self.scale_lock.init(frame, box)
        self._start_cv(frame, box)
        self._target_hist = self._compute_hist(frame, box)
        self._set_bbox((float(x), float(y), float(w), float(h)))
        self._label = label
        self._cls_id = -1
        self._conf = 1.0
        self._locked = True
        self._lost = 0
        self._frame_i = 0
        self._manual_lock = label == "manual" or label.startswith("manual")
        return True

    def lock_bbox(self, frame: np.ndarray, box: BBox) -> bool:
        ok = self.lock_xywh(frame, box.as_int_xywh(), label=box.label or "yolo")
        if ok:
            self._cls_id = box.cls_id
            self._manual_lock = False
        return ok

    def lock_best(self, frame: np.ndarray) -> BBox | None:
        dets = self.detect_only(frame)
        if not dets:
            return None
        best = max(dets, key=lambda b: b.conf)
        self.lock_bbox(frame, best)
        return best

    def _start_cv(self, frame: np.ndarray, xywh: tuple[int, int, int, int]) -> None:
        if getattr(self.tcfg, "lock_tracker", "csrt") == "none":
            self._cv = None
            return
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

    def _reinit_trackers(self, frame: np.ndarray, xywh: tuple[int, int, int, int], label: str | None = None) -> None:
        self.pixel_engine.init_lock(frame, xywh, label=label or self._label)
        self.scale_lock.init(frame, xywh)
        self._start_cv(frame, xywh)
        self._target_hist = self._compute_hist(frame, xywh)

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

        # 1) Scale-aware lock — authoritative for W/H (distance)
        sc_ok, sc_box = self.scale_lock.update(frame)

        # 2) Optical flow — assist center
        pix_ok, pix_box, pix_conf, _ = self.pixel_engine.update(frame)

        # 3) CSRT/KCF — center only (size capped / ignored)
        cv_ok = False
        cv_box = None
        if self._cv is not None:
            tracking_ok, new_bb = self._cv.update(frame)
            if tracking_ok:
                x, y, w, h = [float(v) for v in new_bb]
                if w * h >= 36:
                    cv_box = (x, y, w, h)
                    cv_ok = True

        target_ok = False
        xywh_f: tuple[float, float, float, float] | None = self._bbox_f
        source = "scale_lock"

        if sc_ok and sc_box is not None:
            sx, sy, sw, sh = sc_box
            cx = sx + sw * 0.5
            cy = sy + sh * 0.5

            if pix_ok and pix_box is not None:
                px, py, pw, ph = pix_box
                cx = 0.60 * cx + 0.40 * (px + pw * 0.5)
                cy = 0.60 * cy + 0.40 * (py + ph * 0.5)
            elif cv_ok and cv_box is not None:
                cx = 0.65 * cx + 0.35 * (cv_box[0] + cv_box[2] * 0.5)
                cy = 0.65 * cy + 0.35 * (cv_box[1] + cv_box[3] * 0.5)

            xywh_f = (cx - sw * 0.5, cy - sh * 0.5, sw, sh)
            target_ok = True
            source = "scale_lock"
            self._conf = max(0.55, float(self.scale_lock.last_score))

        elif pix_ok and pix_box is not None:
            # Fallback: flow box, but do not let CSRT inflate size
            px, py, pw, ph = pix_box
            if cv_ok and cv_box is not None:
                cx = 0.75 * (px + pw * 0.5) + 0.25 * (cv_box[0] + cv_box[2] * 0.5)
                cy = 0.75 * (py + ph * 0.5) + 0.25 * (cv_box[1] + cv_box[3] * 0.5)
                # Keep flow size; ignore CSRT size
                xywh_f = (cx - pw * 0.5, cy - ph * 0.5, pw, ph)
            else:
                xywh_f = (float(px), float(py), float(pw), float(ph))
            target_ok = True
            source = "pixel_lock"
            self._conf = max(0.5, float(pix_conf))

        elif cv_ok and cv_box is not None:
            if self._bbox_f is not None:
                lw, lh = self._bbox_f[2], self._bbox_f[3]
                cw, ch = cv_box[2], cv_box[3]
                cx = cv_box[0] + cw * 0.5
                cy = cv_box[1] + ch * 0.5
                if cw > 2.0 * lw or ch > 2.0 * lh or cw < 0.5 * lw or ch < 0.5 * lh:
                    xywh_f = (cx - lw * 0.5, cy - lh * 0.5, lw, lh)
                else:
                    xywh_f = (cv_box[0], cv_box[1], cw, ch)
            else:
                xywh_f = cv_box
            target_ok = True
            source = "csrt"

        xywh = self._as_int(xywh_f) if xywh_f is not None else self._bbox

        # 4) YOLO verification — never fatten a tight manual box
        if run_yolo and dets and xywh is not None:
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
                yolo_xywh = self._xywh_from_box(best_box)

                if target_ok and xywh_f is not None:
                    # While tracking: only accept YOLO size if close to current lock size
                    yw, yh = float(yolo_xywh[2]), float(yolo_xywh[3])
                    sw, sh = xywh_f[2], xywh_f[3]
                    size_ok = (
                        0.72 <= (yw / max(1.0, sw)) <= 1.35
                        and 0.72 <= (yh / max(1.0, sh)) <= 1.35
                    )
                    if best_iou >= 0.45 and best_hist >= 0.45 and size_ok and not self._manual_lock:
                        self._reinit_trackers(frame, yolo_xywh, best_box.label or self._label)
                        xywh_f = (float(yolo_xywh[0]), float(yolo_xywh[1]), yw, yh)
                        xywh = yolo_xywh
                        self._conf = best_box.conf
                        source = "yolo"
                    elif best_iou >= 0.50 and best_hist >= 0.40:
                        # Center-only snap — keep current measured size (critical for distance)
                        cx = yolo_xywh[0] + yolo_xywh[2] * 0.5
                        cy = yolo_xywh[1] + yolo_xywh[3] * 0.5
                        sw, sh = xywh_f[2], xywh_f[3]
                        xywh_f = (cx - sw * 0.5, cy - sh * 0.5, sw, sh)
                        xywh = self._as_int(xywh_f)
                else:
                    # Lost → allow YOLO reacquire with appearance match
                    if best_hist >= 0.45:
                        self._reinit_trackers(frame, yolo_xywh, best_box.label or self._label)
                        xywh_f = (
                            float(yolo_xywh[0]),
                            float(yolo_xywh[1]),
                            float(yolo_xywh[2]),
                            float(yolo_xywh[3]),
                        )
                        xywh = yolo_xywh
                        target_ok = True
                        self._conf = best_box.conf
                        source = "yolo"
                        self._manual_lock = False

        if target_ok and xywh_f is not None:
            out = self._set_bbox(xywh_f)
            # Keep pixel engine bbox in sync so flow assist stays coherent
            if hasattr(self.pixel_engine, "bbox_xywh"):
                self.pixel_engine.bbox_xywh = xywh_f
            self._lost = 0
            return HybridResult(True, out, source, self._label, self._conf, dets)

        self._lost += 1
        if self._lost <= self.max_hold_frames and self._bbox is not None:
            return HybridResult(True, self._bbox, "hold", self._label, self._conf * 0.85, dets)

        self._locked = False
        return HybridResult(False, self._bbox, "lost", self._label, 0.0, dets)
