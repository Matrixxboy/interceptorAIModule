"""Hybrid YOLO + scale-aware lock + optical-flow assist tracker."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import cv2
import numpy as np

from config import CONFIG, DetectionConfig, TrackerConfig, known_width_m
from detection.dual_detector import DualDetector
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
        yolo_every_n: int | None = None,
        reacquire_iou: float = 0.25,
        max_hold_frames: int = 45,
    ) -> None:
        self.det_cfg = det_cfg or CONFIG.detection
        self.tcfg = tracker_cfg or CONFIG.tracker
        cfg_kind = getattr(self.tcfg, "lock_tracker", cv_kind)
        self.cv_kind: CvKind = cfg_kind if cfg_kind in ("csrt", "kcf") else cv_kind
        n = yolo_every_n if yolo_every_n is not None else getattr(self.det_cfg, "detect_every_n", 6)
        self.yolo_every_n = max(1, int(n))
        self.yolo_every_n_healthy = max(
            self.yolo_every_n,
            int(getattr(self.det_cfg, "detect_every_n_healthy", 24)),
        )
        self.reacquire_iou = float(getattr(self.tcfg, "reacquire_iou", reacquire_iou))
        self.max_hold_frames = max(
            int(max_hold_frames),
            int(getattr(self.tcfg, "reacquire_max_frames", 90)),
        )

        self.dual = DualDetector(self.det_cfg)
        self.detector: DualDetector | YOLODetector | None = self.dual
        self.pixel_engine = PixelLockEngine()
        self.scale_lock = ScaleAwareLock(use_csrt=False)
        self._cv = None
        self._locked = False
        self._bbox: tuple[int, int, int, int] | None = None
        self._bbox_f: tuple[float, float, float, float] | None = None
        self._label = ""
        self._family = ""
        self._known_width_m = 0.30
        self._cls_id = -1
        self._target_hist: np.ndarray | None = None
        self._conf = 0.0
        self._frame_i = 0
        self._lost = 0
        self._last_dets: list[BBox] = []
        self._manual_lock = False
        self._detector_error: str | None = None
        self._vx, self._vy = 0.0, 0.0
        self._offscreen = False

    @property
    def family(self) -> str:
        return self._family

    @property
    def known_width_m(self) -> float:
        return self._known_width_m

    def apply_detection_config(self, det_cfg: DetectionConfig) -> None:
        self.det_cfg = det_cfg
        self.yolo_every_n = max(1, int(getattr(det_cfg, "detect_every_n", 6)))
        self.yolo_every_n_healthy = max(
            self.yolo_every_n,
            int(getattr(det_cfg, "detect_every_n_healthy", 24)),
        )
        self.dual.apply_config(det_cfg)
        self.detector = self.dual

    def ensure_detector(self) -> DualDetector:
        if self._detector_error is not None:
            raise RuntimeError(self._detector_error)
        try:
            self.dual.ensure()
        except Exception as exc:
            self._detector_error = f"YOLO unavailable: {exc}"
            log.exception(self._detector_error)
            raise RuntimeError(self._detector_error) from exc
        self.detector = self.dual
        return self.dual

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
        self._family = ""
        self._known_width_m = 0.30
        self._cls_id = -1
        self._target_hist = None
        self._conf = 0.0
        self._lost = 0
        self._frame_i = 0
        self._manual_lock = False
        self._vx, self._vy = 0.0, 0.0
        self._offscreen = False

    def _wanted_families(self) -> tuple[str, ...]:
        want = str(getattr(self.det_cfg, "target_type", "auto") or "auto").lower()
        if self._locked and self._family in ("aerial", "ground"):
            return (self._family,)
        if want == "aerial":
            return ("aerial",)
        if want == "ground":
            return ("ground",)
        return ("aerial", "ground")

    def detect_only(self, frame: np.ndarray) -> list[BBox]:
        families = self._wanted_families()
        allow_crop = (not self._locked) and ("aerial" in families)
        dets = self.ensure_detector().detect(
            frame, families=families, allow_aerial_crop=allow_crop
        )
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

    def _blend_hist(self, frame: np.ndarray, xywh: tuple[int, int, int, int]) -> None:
        fresh = self._compute_hist(frame, xywh)
        if fresh is None:
            return
        if self._target_hist is None:
            self._target_hist = fresh
            return
        self._target_hist = 0.92 * self._target_hist + 0.08 * fresh

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
        if not self._family:
            self._family = "ground" if label.lower() in ("car", "truck", "bus", "motorcycle", "motorbike") else "aerial"
        self._known_width_m = known_width_m(self._label, self._family, 0.30)
        self._cls_id = -1
        self._conf = 1.0
        self._locked = True
        self._lost = 0
        self._frame_i = 0
        self._offscreen = False
        self._manual_lock = label == "manual" or label.startswith("manual")
        return True

    def lock_bbox(self, frame: np.ndarray, box: BBox) -> bool:
        self._family = box.family or self._family
        ok = self.lock_xywh(frame, box.as_int_xywh(), label=box.label or "yolo")
        if ok:
            self._cls_id = box.cls_id
            self._family = box.family or self._family
            self._known_width_m = known_width_m(self._label, self._family, 0.30)
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

    def _search_roi(self, frame_w: int, frame_h: int) -> tuple[float, float, float, float] | None:
        if self._bbox_f is None:
            return None
        x, y, w, h = self._bbox_f
        pad_x = abs(self._vx) * float(self._lost + 1) + 0.35 * w + 24.0
        pad_y = abs(self._vy) * float(self._lost + 1) + 0.35 * h + 24.0
        x1 = max(0.0, x - pad_x)
        y1 = max(0.0, y - pad_y)
        x2 = min(float(frame_w), x + w + pad_x)
        y2 = min(float(frame_h), y + h + pad_y)
        return (x1, y1, x2, y2)

    def _center_in_roi(self, box: BBox, roi: tuple[float, float, float, float]) -> bool:
        return roi[0] <= box.cx <= roi[2] and roi[1] <= box.cy <= roi[3]

    def _same_family(self, box: BBox) -> bool:
        if not self._family or not box.family:
            return True
        return box.family == self._family

    def update(self, frame: np.ndarray) -> HybridResult:
        self._frame_i += 1
        dets: list[BBox] = []
        fh, fw = frame.shape[:2]

        if not self._locked:
            run_yolo = True
        elif self.scale_lock.locked and self.scale_lock.last_score >= 0.50:
            run_yolo = (self._frame_i % self.yolo_every_n_healthy) == 0
        else:
            run_yolo = (self._frame_i % self.yolo_every_n) == 0

        if run_yolo:
            try:
                dets = self.detect_only(frame)
            except Exception as exc:  # noqa: BLE001
                log.warning("YOLO detect failed: %s", exc)
                dets = self._last_dets
        else:
            dets = self._last_dets

        if not self._locked:
            return HybridResult(False, None, "lost", "", 0.0, dets)

        sc_ok, sc_box = self.scale_lock.update(frame)

        pix_ok = False
        pix_box = None
        pix_conf = 0.0
        run_flow = (self._frame_i % 2 == 0) or not sc_ok or self.scale_lock.last_score < 0.55
        if run_flow:
            pix_ok, pix_box, pix_conf, _ = self.pixel_engine.update(frame)

        cv_ok = False
        cv_box = None
        if self._cv is not None and (not sc_ok or self._frame_i % 2 == 1):
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
            score = float(self.scale_lock.last_score)
            self._conf = max(0.70, min(1.0, 0.55 + 0.60 * max(0.0, score)))

        elif pix_ok and pix_box is not None:
            px, py, pw, ph = pix_box
            if cv_ok and cv_box is not None:
                cx = 0.75 * (px + pw * 0.5) + 0.25 * (cv_box[0] + cv_box[2] * 0.5)
                cy = 0.75 * (py + ph * 0.5) + 0.25 * (cv_box[1] + cv_box[3] * 0.5)
                xywh_f = (cx - pw * 0.5, cy - ph * 0.5, pw, ph)
            else:
                xywh_f = (float(px), float(py), float(pw), float(ph))
            target_ok = True
            source = "pixel_lock"
            self._conf = max(0.65, min(1.0, float(pix_conf)))

        elif cv_ok and cv_box is not None:
            if self._bbox_f is not None:
                lw, lh = max(1.0, self._bbox_f[2]), max(1.0, self._bbox_f[3])
                cw, ch = max(1.0, cv_box[2]), max(1.0, cv_box[3])
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
            self._conf = max(0.65, float(self._conf or 0.65))

        xywh = self._as_int(xywh_f) if xywh_f is not None else self._bbox

        healthy_lock = (
            target_ok
            and self.scale_lock.locked
            and float(self.scale_lock.last_score) >= 0.50
        )
        if healthy_lock and xywh is not None:
            self._blend_hist(frame, xywh)

        # YOLO may reacquire when the lock is weak/lost — never steal a healthy lock.
        if run_yolo and dets and xywh is not None and not healthy_lock:
            roi = self._search_roi(fw, fh)
            hist_thr = float(getattr(self.tcfg, "template_match_threshold", 0.45))
            candidates = []
            for d in dets:
                if not self._same_family(d):
                    continue
                iou = self._iou_xywh(xywh, d)
                in_roi = roi is not None and self._center_in_roi(d, roi)
                if iou < 0.08 and not in_roi:
                    continue
                hist_sim = self._compare_hist(frame, d)
                cls_match = 1.2 if (self._cls_id >= 0 and d.cls_id == self._cls_id) else 1.0
                score = (max(iou, 0.15 if in_roi else 0.0) * 0.6 + hist_sim * 0.4) * cls_match
                candidates.append((score, iou, hist_sim, in_roi, d))

            if candidates:
                candidates.sort(key=lambda item: item[0], reverse=True)
                best_score, best_iou, best_hist, in_roi, best_box = candidates[0]
                yolo_xywh = self._xywh_from_box(best_box)

                if target_ok and xywh_f is not None:
                    yw, yh = float(yolo_xywh[2]), float(yolo_xywh[3])
                    sw, sh = xywh_f[2], xywh_f[3]
                    size_ok = (
                        0.72 <= (yw / max(1.0, sw)) <= 1.35
                        and 0.72 <= (yh / max(1.0, sh)) <= 1.35
                    )
                    if (
                        best_iou >= 0.45
                        and best_hist >= hist_thr
                        and size_ok
                        and not self._manual_lock
                    ):
                        self._reinit_trackers(frame, yolo_xywh, best_box.label or self._label)
                        xywh_f = (float(yolo_xywh[0]), float(yolo_xywh[1]), yw, yh)
                        xywh = yolo_xywh
                        self._conf = best_box.conf
                        self._label = best_box.label or self._label
                        source = "yolo"
                    elif best_iou >= 0.50 and best_hist >= 0.40:
                        cx = yolo_xywh[0] + yolo_xywh[2] * 0.5
                        cy = yolo_xywh[1] + yolo_xywh[3] * 0.5
                        sw, sh = xywh_f[2], xywh_f[3]
                        xywh_f = (cx - sw * 0.5, cy - sh * 0.5, sw, sh)
                        xywh = self._as_int(xywh_f)
                elif best_hist >= hist_thr and (best_iou >= self.reacquire_iou or in_roi):
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
                    self._label = best_box.label or self._label
                    source = "yolo"
                    self._manual_lock = False
                    self._offscreen = False

        if target_ok and xywh_f is not None:
            if self._bbox_f is not None:
                self._vx = xywh_f[0] - self._bbox_f[0]
                self._vy = xywh_f[1] - self._bbox_f[1]
            out = self._set_bbox(xywh_f)
            if hasattr(self.pixel_engine, "bbox_xywh"):
                self.pixel_engine.bbox_xywh = xywh_f
            self._lost = 0
            cx = xywh_f[0] + xywh_f[2] * 0.5
            cy = xywh_f[1] + xywh_f[3] * 0.5
            margin = 8.0
            self._offscreen = (
                cx < margin or cy < margin or cx > fw - margin or cy > fh - margin
            )
            return HybridResult(True, out, source, self._label, self._conf, dets)

        self._lost += 1
        hold_limit = self.max_hold_frames
        if self._offscreen:
            hold_limit = max(hold_limit, 75)
        if self._lost <= hold_limit and self._bbox is not None:
            if self._bbox_f is not None:
                nx = self._bbox_f[0] + self._vx
                ny = self._bbox_f[1] + self._vy
                nx = float(np.clip(nx, -self._bbox_f[2] * 0.4, fw - self._bbox_f[2] * 0.6))
                ny = float(np.clip(ny, -self._bbox_f[3] * 0.4, fh - self._bbox_f[3] * 0.6))
                self._set_bbox((nx, ny, self._bbox_f[2], self._bbox_f[3]))
            return HybridResult(True, self._bbox, "hold", self._label, self._conf * 0.85, dets)

        self._locked = False
        return HybridResult(False, self._bbox, "lost", self._label, 0.0, dets)
