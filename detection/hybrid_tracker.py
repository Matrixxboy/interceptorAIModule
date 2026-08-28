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
    if kind == "kcf":
        creators = [
            lambda: cv2.TrackerKCF_create(),
            lambda: cv2.legacy.TrackerKCF_create(),
            lambda: cv2.legacy.TrackerMOSSE_create(),
        ]
    elif kind == "csrt":
        creators = [
            lambda: cv2.TrackerCSRT_create(),
            lambda: cv2.legacy.TrackerCSRT_create(),
            lambda: cv2.TrackerKCF_create(),
        ]
    else:
        creators = [
            lambda: cv2.TrackerKCF_create(),
            lambda: cv2.legacy.TrackerKCF_create(),
            lambda: cv2.legacy.TrackerMOSSE_create(),
            lambda: cv2.TrackerMIL_create(),
        ]
    for fn in creators:
        try:
            tr = fn()
            if tr is not None:
                return tr
        except Exception:
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
    """Interceptor-grade target lock: Scale-aware + flow + velocity-compensated predictive tracking."""

    def __init__(
        self,
        det_cfg: DetectionConfig | None = None,
        tracker_cfg: TrackerConfig | None = None,
        cv_kind: CvKind = "csrt",
        yolo_every_n: int = 12,
        reacquire_iou: float = 0.25,
        max_hold_frames: int = 60,
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
        self._detector_error: str | None = None
        self._target_id_seq = 1
        self._locked_target_id = -1
        # Interceptor-grade velocity tracking for predictive hold
        self._vx = 0.0  # Estimated target velocity X (pixels/frame)
        self._vy = 0.0  # Estimated target velocity Y (pixels/frame)
        self._last_cx = 0.0  # Previous center X
        self._last_cy = 0.0  # Previous center Y
        self._vel_alpha = 0.35  # EMA smoothing for velocity estimate

    def ensure_detector(self) -> YOLODetector:
        if self._detector_error is not None:
            raise RuntimeError(self._detector_error)
        if self.detector is None:
            log.info("Loading YOLO for hybrid lock…")
            try:
                self.detector = YOLODetector(self.det_cfg)
            except Exception as exc:
                self._detector_error = f"YOLO unavailable: {exc}"
                log.exception(self._detector_error)
                raise RuntimeError(self._detector_error) from exc
        return self.detector

    @property
    def locked(self) -> bool:
        return self._locked

    @property
    def target_id(self) -> int:
        return self._locked_target_id

    @property
    def bbox(self) -> tuple[int, int, int, int] | None:
        return self._bbox

    def reset(self) -> None:
        self.pixel_engine.initialized = False
        self.scale_lock.reset()
        self._cv = None
        self._locked = False
        self._locked_target_id = -1
        self._bbox = None
        self._bbox_f = None
        self._label = ""
        self._cls_id = -1
        self._target_hist = None
        self._conf = 0.0
        self._lost = 0
        self._frame_i = 0
        self._manual_lock = False
        self._vx = 0.0
        self._vy = 0.0
        self._last_cx = 0.0
        self._last_cy = 0.0

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

    def _set_bbox(self, xywh: tuple[float, float, float, float], snap: bool = False) -> tuple[int, int, int, int]:
        rx, ry, rw, rh = [float(v) for v in xywh]
        
        # Update velocity estimate (interceptor-grade predictive tracking)
        new_cx = rx + rw * 0.5
        new_cy = ry + rh * 0.5
        if self._bbox_f is not None and not snap:
            raw_vx = new_cx - self._last_cx
            raw_vy = new_cy - self._last_cy
            self._vx = self._vel_alpha * raw_vx + (1.0 - self._vel_alpha) * self._vx
            self._vy = self._vel_alpha * raw_vy + (1.0 - self._vel_alpha) * self._vy
        else:
            self._vx = 0.0
            self._vy = 0.0
        self._last_cx = new_cx
        self._last_cy = new_cy

        if snap or self._bbox_f is None:
            self._bbox_f = (rx, ry, rw, rh)
        else:
            px, py, pw, ph = self._bbox_f
            # Calculate step displacement (speed of target movement)
            step = float(np.hypot(rx - px, ry - py))
            
            # Dynamic alpha: fast target → high alpha (instant response); slow target → heavy smoothing
            alpha_pos = max(0.35, min(0.92, 0.35 + 0.05 * step))
            alpha_size = max(0.25, min(0.75, 0.25 + 0.03 * step))
            
            sx = alpha_pos * rx + (1.0 - alpha_pos) * px
            sy = alpha_pos * ry + (1.0 - alpha_pos) * py
            sw = alpha_size * rw + (1.0 - alpha_size) * pw
            sh = alpha_size * rh + (1.0 - alpha_size) * ph
            
            self._bbox_f = (sx, sy, sw, sh)
            
        self._bbox = self._as_int(self._bbox_f)
        return self._bbox

    def lock_xywh(self, frame: np.ndarray, xywh: tuple[int, int, int, int], label: str = "manual") -> bool:
        x, y, w, h = [int(round(float(v))) for v in xywh]
        if w < 8 or h < 8:
            return False
        box = (x, y, w, h)
        if self._locked_target_id <= 0:
            self._locked_target_id = self._target_id_seq
            self._target_id_seq += 1
        label_with_id = f"{label} #ID:{self._locked_target_id}"
        self.pixel_engine.init_lock(frame, box, label=label_with_id)
        self.scale_lock.init(frame, box)
        self._start_cv(frame, box)
        self._target_hist = self._compute_hist(frame, box)
        self._set_bbox((float(x), float(y), float(w), float(h)), snap=True)
        self._label = label_with_id
        self._cls_id = -1
        self._conf = 1.0
        self._locked = True
        self._lost = 0
        self._frame_i = 0
        self._manual_lock = label == "manual" or label.startswith("manual")
        return True

    def lock_bbox(self, frame: np.ndarray, box: BBox) -> bool:
        if box.track_id > 0:
            self._locked_target_id = box.track_id
        else:
            if self._locked_target_id <= 0:
                self._locked_target_id = self._target_id_seq
                self._target_id_seq += 1
            box.track_id = self._locked_target_id
        ok = self.lock_xywh(frame, box.as_int_xywh(), label=box.label or "yolo")
        if ok:
            self._cls_id = box.cls_id
            self._manual_lock = False
        return ok

    def lock_nearest_to_center(self, frame: np.ndarray, center_xy: tuple[int, int] | None = None) -> BBox | None:
        dets = self.detect_only(frame)
        if center_xy is None:
            h, w = frame.shape[:2]
            center_xy = (w // 2, h // 2)
        cx, cy = center_xy

        if dets:
            def dist_to_center(b: BBox) -> float:
                bcx = (b.x1 + b.x2) * 0.5
                bcy = (b.y1 + b.y2) * 0.5
                return (bcx - cx) ** 2 + (bcy - cy) ** 2

            best = min(dets, key=dist_to_center)
            self.lock_bbox(frame, best)
            return best
        else:
            # Fallback: Lock nominal center crosshair region (80x80 box)
            w_box, h_box = 80, 80
            nominal_xywh = (max(0, cx - w_box // 2), max(0, cy - h_box // 2), w_box, h_box)
            self.lock_xywh(frame, nominal_xywh, label="center_lock")
            return None

    def lock_best(self, frame: np.ndarray, center_xy: tuple[int, int] | None = None) -> BBox | None:
        return self.lock_nearest_to_center(frame, center_xy)

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

        # YOLO is expensive — skip most frames while locked with a healthy scale lock
        if not self._locked:
            run_yolo = True
        elif self.scale_lock.locked and self.scale_lock.last_score >= 0.50:
            run_yolo = (self._frame_i % 24) == 0
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

        # 1) Scale-aware lock — authoritative for W/H (distance)
        sc_ok, sc_box = self.scale_lock.update(frame)

        # 2) Optical flow — assist center (every other frame when scale lock is strong)
        pix_ok = False
        pix_box = None
        pix_conf = 0.0
        run_flow = (self._frame_i % 2 == 0) or not sc_ok or self.scale_lock.last_score < 0.55
        if run_flow:
            pix_ok, pix_box, pix_conf, _ = self.pixel_engine.update(frame)

        # 3) CSRT/KCF — only if enabled (default off for speed)
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
            # Map NCC score → follow confidence. Raw NCC ~0.55 used to sit under the
            # 60% follow gate and silently disable AI stick output.
            score = float(self.scale_lock.last_score)
            self._conf = max(0.70, min(1.0, 0.55 + 0.60 * max(0.0, score)))

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
            self._conf = max(0.65, min(1.0, float(pix_conf)))

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
            self._conf = max(0.65, float(self._conf or 0.65))

        xywh = self._as_int(xywh_f) if xywh_f is not None else self._bbox

        # 4) YOLO verification — strict anti-hijacking check so lock never switches to a background distractor
        if run_yolo and dets and xywh is not None:
            tcx = xywh[0] + xywh[2] * 0.5
            tcy = xywh[1] + xywh[3] * 0.5
            t_diag = float(np.hypot(xywh[2], xywh[3]))

            candidates = []
            for d in dets:
                iou = self._iou_xywh(xywh, d)
                dcx = d.x1 + (d.x2 - d.x1) * 0.5
                dcy = d.y1 + (d.y2 - d.y1) * 0.5
                dist = float(np.hypot(dcx - tcx, dcy - tcy))
                
                # Spatial jump gate: reject any candidate box that jumped more than 1.8x target size
                if dist > 1.8 * max(20.0, t_diag):
                    continue

                if iou >= 0.35:
                    hist_sim = self._compare_hist(frame, d)
                    cls_match = 1.2 if (self._cls_id >= 0 and d.cls_id == self._cls_id) else 1.0
                    score = (iou * 0.5 + hist_sim * 0.5) * cls_match
                    candidates.append((score, iou, hist_sim, d))

            if candidates:
                candidates.sort(key=lambda item: item[0], reverse=True)
                best_score, best_iou, best_hist, best_box = candidates[0]
                yolo_xywh = self._xywh_from_box(best_box)

                if target_ok and xywh_f is not None:
                    # While tracking: only accept YOLO size/center if appearance (hist >= 0.60) & IoU (>= 0.40) match strictly
                    yw, yh = float(yolo_xywh[2]), float(yolo_xywh[3])
                    sw, sh = xywh_f[2], xywh_f[3]
                    size_ok = (
                        0.75 <= (yw / max(1.0, sw)) <= 1.30
                        and 0.75 <= (yh / max(1.0, sh)) <= 1.30
                    )
                    if best_iou >= 0.45 and best_hist >= 0.60 and size_ok and not self._manual_lock:
                        self._reinit_trackers(frame, yolo_xywh, best_box.label or self._label)
                        xywh_f = (float(yolo_xywh[0]), float(yolo_xywh[1]), yw, yh)
                        xywh = yolo_xywh
                        self._conf = best_box.conf
                        source = "yolo"
                    elif best_iou >= 0.40 and best_hist >= 0.55:
                        # Center-only snap — keep current measured size
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

        # ── Interceptor-grade predictive hold ──
        # When target is temporarily lost, coast using velocity vector
        # instead of holding a static position. This keeps the lock box
        # moving with the target during brief occlusions (smoke, flare, jitter).
        self._lost += 1
        if self._lost <= self.max_hold_frames and self._bbox_f is not None:
            px, py, pw, ph = self._bbox_f
            # Apply velocity-compensated coast (predict where target moved)
            speed = float(np.hypot(self._vx, self._vy))
            if speed > 0.5 and self._lost <= 30:
                # Active coast: project position using velocity
                coast_x = px + self._vx
                coast_y = py + self._vy
                self._bbox_f = (coast_x, coast_y, pw, ph)
                self._last_cx = coast_x + pw * 0.5
                self._last_cy = coast_y + ph * 0.5
                self._bbox = self._as_int(self._bbox_f)
                # Decay velocity gradually so coast slows down
                self._vx *= 0.92
                self._vy *= 0.92
                # Decay confidence more slowly during active coast
                hold_conf = self._conf * max(0.50, 1.0 - self._lost * 0.015)
                return HybridResult(True, self._bbox, "coast", self._label, hold_conf, dets)
            else:
                # Static hold: target was stationary or coast exhausted
                hold_conf = self._conf * max(0.40, 1.0 - self._lost * 0.012)
                return HybridResult(True, self._bbox, "hold", self._label, hold_conf, dets)

        self._locked = False
        return HybridResult(False, self._bbox, "lost", self._label, 0.0, dets)
