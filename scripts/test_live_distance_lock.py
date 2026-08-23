"""
Live distance lock — accurate pinhole distance after calibration.

WHY IT WAS WRONG BEFORE
  The tracker used a fixed-size template, so the box did not shrink/grow
  when you moved. Distance = (W * f) / pixels needs the box size to change.

FIX
  Multi-scale template match (box scales with distance) + smoothed size.

CONTROLS
  Drag tight box  = lock
  C               = calibrate (tape measure + object width in CM)
  R               = unlock
  A               = measure axis: width / height / max
  S               = save calib
  Q               = quit

CALIBRATION
  1) Stand at a known distance (tape)
  2) Drag a TIGHT box on the object (edges of the real object only)
  3) Press C
  4) Q1 = distance in METERS (e.g. 1)
  5) Q2 = object size in CENTIMETERS along the box (e.g. phone width 7)

Run:
  python scripts/test_live_distance_lock.py
  python scripts/test_live_distance_lock.py --camera 1
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from config import SystemConfig
from control.fpv_follow import FPVFollowController
from estimation.distance_estimator import DistanceEstimator

CALIB_PATH = ROOT / "presets" / "distance_calib.json"


# ---------------------------------------------------------------------------
# Multi-scale lock — box size changes with distance (required for accuracy)
# ---------------------------------------------------------------------------

class ScaleAwareLock:
    """Template lock that searches across scales so pixel size tracks distance."""

    def __init__(self) -> None:
        self._template: np.ndarray | None = None  # grayscale
        self._bbox: tuple[float, float, float, float] | None = None
        self._base_w = 0.0
        self._base_h = 0.0
        self._scale = 1.0
        self._misses = 0
        self._csrt = None

    @staticmethod
    def _make_csrt():
        if hasattr(cv2, "TrackerCSRT_create"):
            return cv2.TrackerCSRT_create()
        if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerCSRT_create"):
            return cv2.legacy.TrackerCSRT_create()
        return None

    def init(self, frame_bgr: np.ndarray, bbox_xywh: tuple[int, int, int, int]) -> bool:
        x, y, w, h = [int(v) for v in bbox_xywh]
        fh, fw = frame_bgr.shape[:2]
        x = max(0, min(x, fw - 2))
        y = max(0, min(y, fh - 2))
        w = max(12, min(w, fw - x))
        h = max(12, min(h, fh - y))
        patch = frame_bgr[y : y + h, x : x + w]
        if patch.size == 0:
            return False
        self._template = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
        self._bbox = (float(x), float(y), float(w), float(h))
        self._base_w = float(w)
        self._base_h = float(h)
        self._scale = 1.0
        self._misses = 0
        self._csrt = self._make_csrt()
        if self._csrt is not None:
            self._csrt.init(frame_bgr, (x, y, w, h))
        return True

    def _multiscale_match(self, gray: np.ndarray) -> tuple[float, float, float, float, float] | None:
        """Returns (x, y, w, h, score) with scale-aware size."""
        if self._template is None or self._bbox is None:
            return None
        x, y, w, h = self._bbox
        fh, fw = gray.shape[:2]
        tw0, th0 = self._template.shape[1], self._template.shape[0]

        # Search scales around current scale (object nearer = larger)
        scales = []
        for s in np.linspace(max(0.45, self._scale * 0.70), min(2.4, self._scale * 1.35), 13):
            scales.append(float(s))

        pad = max(100, int(max(w, h) * 1.5))
        x0 = max(0, int(x) - pad)
        y0 = max(0, int(y) - pad)
        x1 = min(fw, int(x + w) + pad)
        y1 = min(fh, int(y + h) + pad)
        roi = gray[y0:y1, x0:x1]
        if roi.size == 0:
            return None

        best = None  # (score, nx, ny, nw, nh, scale)
        for s in scales:
            tw = max(8, int(round(tw0 * s)))
            th = max(8, int(round(th0 * s)))
            if tw >= roi.shape[1] or th >= roi.shape[0]:
                continue
            tmpl = cv2.resize(self._template, (tw, th), interpolation=cv2.INTER_AREA)
            res = cv2.matchTemplate(roi, tmpl, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, max_loc = cv2.minMaxLoc(res)
            if best is None or max_val > best[0]:
                best = (float(max_val), float(x0 + max_loc[0]), float(y0 + max_loc[1]), float(tw), float(th), s)

        if best is None or best[0] < 0.42:
            return None
        score, nx, ny, nw, nh, s = best
        self._scale = 0.7 * self._scale + 0.3 * s  # smooth scale
        return (nx, ny, nw, nh, score)

    def update(self, frame_bgr: np.ndarray) -> tuple[bool, tuple[float, float, float, float] | None]:
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        ms = self._multiscale_match(gray)

        csrt_box = None
        if self._csrt is not None:
            ok, box = self._csrt.update(frame_bgr)
            if ok:
                csrt_box = tuple(float(v) for v in box)

        if ms is not None:
            nx, ny, nw, nh, score = ms
            # If CSRT agrees roughly on center, blend sizes for stability
            if csrt_box is not None:
                cx, cy, cw, ch = csrt_box
                # Reject CSRT if size exploded (>2.5x) vs multi-scale
                if 0.4 * nw < cw < 2.5 * nw and 0.4 * nh < ch < 2.5 * nh:
                    nw = 0.65 * nw + 0.35 * cw
                    nh = 0.65 * nh + 0.35 * ch
                    nx = 0.65 * nx + 0.35 * cx
                    ny = 0.65 * ny + 0.35 * cy
            self._bbox = (nx, ny, nw, nh)
            self._misses = 0
            return True, self._bbox

        if csrt_box is not None:
            # Cap CSRT size drift vs last known size
            lx, ly, lw, lh = self._bbox if self._bbox else csrt_box
            cx, cy, cw, ch = csrt_box
            if cw > 3.0 * lw or ch > 3.0 * lh or cw < 0.3 * lw or ch < 0.3 * lh:
                # Keep last size, only move center
                self._bbox = (cx, cy, lw, lh)
            else:
                self._bbox = csrt_box
            self._misses = 0
            return True, self._bbox

        self._misses += 1
        if self._misses > 25:
            return False, None
        return True, self._bbox


def focal_from_sample(pixel_size: float, known_distance_m: float, known_width_m: float) -> float:
    return (max(1.0, pixel_size) * known_distance_m) / max(0.005, known_width_m)


def bbox_size_px(bbox: tuple[float, float, float, float], axis: str) -> float:
    _, _, w, h = bbox
    if axis == "height":
        return max(1.0, h)
    if axis == "max":
        return max(1.0, w, h)
    if axis == "diag":
        return max(1.0, math.hypot(w, h))
    return max(1.0, w)


def load_calib(cfg: SystemConfig) -> dict:
    if not CALIB_PATH.exists():
        return {}
    try:
        data = json.loads(CALIB_PATH.read_text(encoding="utf-8"))
        w = float(data.get("known_object_width_m", cfg.distance.known_object_width_m))
        if 0.01 <= w <= 2.5:
            cfg.distance.known_object_width_m = w
        if "focal_length_px" in data:
            cfg.distance.focal_length_px = float(data["focal_length_px"])
        if "desired_distance_m" in data:
            cfg.distance.desired_distance_m = float(data["desired_distance_m"])
        print(f" Loaded calibration from {CALIB_PATH.name}")
        print(f"   focal={cfg.distance.focal_length_px:.1f}px  object={cfg.distance.known_object_width_m*100:.1f}cm")
        return data
    except Exception as exc:
        print(f" Could not load calib: {exc}")
        return {}


def save_calib(cfg: SystemConfig, axis: str) -> None:
    CALIB_PATH.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "focal_length_px": cfg.distance.focal_length_px,
        "known_object_width_m": cfg.distance.known_object_width_m,
        "desired_distance_m": cfg.distance.desired_distance_m,
        "fov_h_deg": cfg.camera.fov_h_deg,
        "size_axis": axis,
    }
    CALIB_PATH.write_text(json.dumps(data, indent=2), encoding="utf-8")
    print(f" Saved -> {CALIB_PATH}")


def ask_float(prompt: str, default: float) -> float:
    raw = input(f"{prompt} [{default}]: ").strip()
    if not raw:
        return default
    return float(raw)


class RoiDrag:
    def __init__(self) -> None:
        self.dragging = False
        self.p0 = (0, 0)
        self.p1 = (0, 0)
        self.finished: tuple[int, int, int, int] | None = None
        self.img_h = 9999

    def on_mouse(self, event, x, y, flags, param) -> None:
        if y >= self.img_h and not self.dragging:
            return
        y = min(y, self.img_h - 1)
        if event == cv2.EVENT_LBUTTONDOWN:
            self.dragging = True
            self.p0 = (x, y)
            self.p1 = (x, y)
            self.finished = None
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
            self.p1 = (x, min(y, self.img_h - 1))
        elif event == cv2.EVENT_LBUTTONUP and self.dragging:
            self.dragging = False
            self.p1 = (x, min(y, self.img_h - 1))
            x0, y0 = self.p0
            x1, y1 = self.p1
            rx, ry = min(x0, x1), min(y0, y1)
            rw, rh = abs(x1 - x0), abs(y1 - y0)
            if rw > 12 and rh > 12:
                self.finished = (rx, ry, rw, rh)

    def draw(self, frame: np.ndarray) -> None:
        if self.dragging:
            cv2.rectangle(frame, self.p0, self.p1, (0, 200, 255), 2)


def open_camera(index: int) -> cv2.VideoCapture | None:
    for backend in (cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY):
        cap = cv2.VideoCapture(index, backend)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            ok, frame = cap.read()
            if ok and frame is not None:
                return cap
            cap.release()
    return None


def draw_hud(frame, *, locked, size_px, dist_m, raw_dist, fps, cfg, axis, calibrated, scale):
    h, w = frame.shape[:2]
    cv2.drawMarker(frame, (w // 2, h // 2), (80, 80, 80), cv2.MARKER_CROSS, 22, 1)
    panel = np.zeros((170, w, 3), dtype=np.uint8)
    panel[:] = (18, 18, 18)
    known_cm = cfg.distance.known_object_width_m * 100.0
    cal = "CALIBRATED" if calibrated else "NOT CALIBRATED — press C"

    lines = [
        f"ACCURATE DISTANCE LOCK   FPS {fps:.0f}   {cal}",
        "Drag TIGHT box | C=calibrate | R=unlock | A=axis | S=save | Q=quit",
        f"Object={known_cm:.1f}cm  Focal={cfg.distance.focal_length_px:.1f}px  Axis={axis}  Scale={scale:.2f}",
        "Dist(m) = (object_m * focal_px) / box_px     box must grow when you move closer",
    ]
    if locked and dist_m is not None:
        lines += [
            f"box={size_px:.1f}px   DIST={dist_m:.2f} m ({dist_m*100:.0f} cm)   raw={raw_dist:.2f} m",
        ]
    else:
        lines += ["NO LOCK — draw a tight box on the object edges"]

    for i, text in enumerate(lines):
        color = (200, 200, 200)
        if i == 0:
            color = (80, 200, 120) if calibrated else (40, 40, 255)
        if locked and i == 4:
            color = (0, 255, 160)
        cv2.putText(panel, text, (10, 26 + i * 28), cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 1, cv2.LINE_AA)
    return np.vstack([frame, panel])


def run_calibration(cfg, estimator, size_px: float, axis: str) -> bool:
    print()
    print("=" * 64)
    print(" CALIBRATION")
    print("  Q1 = tape-measure distance camera → object, in METERS")
    print("  Q2 = real size of what is INSIDE the green box, in CENTIMETERS")
    print("       (phone width ~7, bottle ~6-8, book ~15, laptop ~30)")
    print("  Tip: box must be TIGHT on the object. Loose box = wrong distance.")
    print("=" * 64)
    try:
        dist_m = ask_float("Q1) Distance in METERS", 1.0)
        width_cm = ask_float("Q2) Object size in CENTIMETERS", max(1.0, cfg.distance.known_object_width_m * 100))
    except Exception as exc:
        print(f" Cancelled: {exc}")
        return False

    if not (0.05 <= dist_m <= 30.0):
        print(f" Distance {dist_m} m invalid (use 0.05–30).")
        return False
    if not (2.0 <= width_cm <= 200.0):
        print(f" Width {width_cm} cm invalid. Use centimeters (phone=7, not 0.07).")
        return False

    width_m = width_cm / 100.0
    new_f = focal_from_sample(size_px, dist_m, width_m)
    cfg.distance.focal_length_px = new_f
    cfg.distance.known_object_width_m = width_m
    estimator.update_config(cfg.distance)
    check = estimator.estimate_distance(size_px)

    print()
    print(f" Averaged box size : {size_px:.1f} px   axis={axis}")
    print(f" You entered       : {dist_m:.3f} m away, object {width_cm:.1f} cm")
    print(f" New focal length  : {new_f:.2f} px")
    print(f" Verify distance   : {check:.3f} m  (must match {dist_m:.3f})")
    if abs(check - dist_m) < 0.02:
        print(" Calibration OK — now move closer/farther; distance should change.")
    else:
        print(" Verify mismatch — try again with a tighter box.")
    print("=" * 64)
    print()
    return True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width-cm", type=float, default=None)
    parser.add_argument("--axis", choices=("width", "height", "max", "diag"), default="width")
    args = parser.parse_args()

    cfg = SystemConfig()
    if cfg.distance.known_object_width_m > 2.5 or cfg.distance.known_object_width_m < 0.01:
        cfg.distance.known_object_width_m = 0.30

    meta = load_calib(cfg)
    axis = meta.get("size_axis", args.axis)
    if axis not in ("width", "height", "max", "diag"):
        axis = "width"
    calibrated = "focal_length_px" in meta and 0.01 <= cfg.distance.known_object_width_m <= 2.5

    if args.width_cm is not None:
        cfg.distance.known_object_width_m = args.width_cm / 100.0

    cfg.prediction.enable_kalman = False

    cap = open_camera(args.camera)
    if cap is None:
        print(f"ERROR: cannot open camera {args.camera}")
        return 1

    if not calibrated:
        ok, sample = cap.read()
        fw = int(sample.shape[1]) if ok else 1280
        cfg.distance.focal_length_px = (fw * 0.5) / math.tan(math.radians(max(10.0, cfg.camera.fov_h_deg) * 0.5))
        print(f" No calib yet — FOV estimate focal={cfg.distance.focal_length_px:.0f}px. Press C after lock.")

    win = "T.R.I.V.E.N.I Accurate Distance Lock"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1100, 820)
    drag = RoiDrag()
    cv2.setMouseCallback(win, drag.on_mouse)

    controller = FPVFollowController(cfg)
    estimator = DistanceEstimator(cfg.distance)
    lock = ScaleAwareLock()
    locked = False
    bbox = None

    size_hist: deque[float] = deque(maxlen=9)
    dist_hist: deque[float] = deque(maxlen=7)

    print("=" * 64)
    print(" For accuracy: tight box + correct CM width + multi-scale lock")
    print(" After C: walk closer → distance must DROP; walk away → must RISE")
    print("=" * 64)

    t0 = time.perf_counter()
    n = 0
    fps = 0.0
    cam_idx = args.camera

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(frame, "No camera", (40, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 80, 255), 2)

        fh, fw = frame.shape[:2]
        drag.img_h = fh

        if drag.finished is not None:
            rx, ry, rw, rh = drag.finished
            drag.finished = None
            if lock.init(frame, (rx, ry, rw, rh)):
                bbox = (float(rx), float(ry), float(rw), float(rh))
                locked = True
                size_hist.clear()
                dist_hist.clear()
                controller.reset()
                print(f" LOCKED {rw}x{rh}px — press C to calibrate at a known distance")
            else:
                locked = False
                bbox = None

        size_px = 0.0
        scale = lock._scale if locked else 1.0
        if locked:
            ok_tr, box = lock.update(frame)
            if ok_tr and box is not None:
                bbox = box
                x, y, bw, bh = bbox
                raw_size = bbox_size_px(bbox, axis)
                size_hist.append(raw_size)
                size_px = float(np.median(size_hist))
                cv2.rectangle(frame, (int(x), int(y)), (int(x + bw), int(y + bh)), (0, 220, 80), 2)
                cx, cy = int(x + bw / 2), int(y + bh / 2)
                cv2.circle(frame, (cx, cy), 4, (0, 220, 80), -1)
                cv2.line(frame, (fw // 2, fh // 2), (cx, cy), (0, 160, 255), 1)
            else:
                print(" LOCK LOST")
                locked = False
                bbox = None
                size_hist.clear()
                dist_hist.clear()

        drag.draw(frame)

        dist_m = None
        raw_dist = None
        estimator.update_config(cfg.distance)
        controller.distance_estimator.update_config(cfg.distance)

        if locked and bbox is not None and size_px > 1:
            x, y, bw, bh = bbox
            synth = (x, y, size_px if axis == "width" else bw, bh)
            controller.update(synth, fw, fh, base_throttle=1500)
            raw_dist = estimator.estimate_distance(size_px)
            dist_hist.append(raw_dist)
            dist_m = float(np.median(dist_hist))
            cv2.putText(
                frame, f"{dist_m:.2f} m", (int(x), max(22, int(y) - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 180), 2, cv2.LINE_AA,
            )
        else:
            controller.update(None, fw, fh, base_throttle=1500)

        n += 1
        now = time.perf_counter()
        if now - t0 >= 0.5:
            fps = n / (now - t0)
            n = 0
            t0 = now

        display = draw_hud(
            frame, locked=locked, size_px=size_px, dist_m=dist_m, raw_dist=raw_dist or 0.0,
            fps=fps, cfg=cfg, axis=axis, calibrated=calibrated, scale=scale,
        )
        cv2.imshow(win, display)
        key = cv2.waitKey(1) & 0xFF

        if key in (ord("q"), ord("Q")):
            break
        if key == 27:
            if locked:
                locked, bbox = False, None
                size_hist.clear()
                dist_hist.clear()
            else:
                break
        if key in (ord("r"), ord("R")):
            locked, bbox = False, None
            size_hist.clear()
            dist_hist.clear()
            print(" UNLOCKED")
        if key in (ord("a"), ord("A")):
            order = ["width", "height", "max", "diag"]
            axis = order[(order.index(axis) + 1) % len(order)]
            size_hist.clear()
            dist_hist.clear()
            print(f" Axis -> {axis}  (re-calibrate with C after changing axis)")
        if key in (ord("s"), ord("S")):
            save_calib(cfg, axis)
            calibrated = True
        if key in (ord("c"), ord("C")):
            if not locked or not size_hist:
                print(" Lock first, wait ~0.5s for stable box, then press C")
            else:
                avg_size = float(np.median(size_hist))
                if run_calibration(cfg, estimator, avg_size, axis):
                    calibrated = True
                    save_calib(cfg, axis)
                    dist_hist.clear()

        if ord("0") <= key <= ord("9"):
            new_idx = key - ord("0")
            if new_idx != cam_idx:
                new_cap = open_camera(new_idx)
                if new_cap is not None:
                    cap.release()
                    cap = new_cap
                    cam_idx = new_idx
                    locked, bbox = False, None
                    size_hist.clear()
                    dist_hist.clear()
                    print(f" Camera {cam_idx}")

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
