"""
Live distance lock test — clear calibration + stronger pixel lock.

CONTROLS
  Drag box on the video  = lock (draw tightly around the object)
  C  = calibrate (answer 2 simple questions in the terminal)
  R  = unlock
  A  = size axis (width / height / max)
  S  = save calibration
  Q  = quit

CALIBRATION (press C while locked) — two numbers only:
  1) DISTANCE  = how far YOU are from the object (meters), e.g. 1.0 = one meter
  2) WIDTH_CM  = how wide the object is in CENTIMETERS, e.g. phone~7, bottle~7, person~45
     NOT meters. NOT the distance.

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
# Stronger lock: CSRT + template matching refine
# ---------------------------------------------------------------------------

class PixelLockTracker:
    """Keeps the lock on the selected pixels more reliably than CSRT alone."""

    def __init__(self) -> None:
        self._cv_tracker = None
        self._template: np.ndarray | None = None
        self._bbox: tuple[float, float, float, float] | None = None
        self._frame_shape: tuple[int, int] | None = None
        self._misses = 0

    @staticmethod
    def _create_cv_tracker():
        if hasattr(cv2, "TrackerCSRT_create"):
            return cv2.TrackerCSRT_create()
        if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerCSRT_create"):
            return cv2.legacy.TrackerCSRT_create()
        if hasattr(cv2, "TrackerKCF_create"):
            return cv2.TrackerKCF_create()
        if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerKCF_create"):
            return cv2.legacy.TrackerKCF_create()
        return None

    def init(self, frame_bgr: np.ndarray, bbox_xywh: tuple[int, int, int, int]) -> bool:
        x, y, w, h = [int(v) for v in bbox_xywh]
        fh, fw = frame_bgr.shape[:2]
        x = max(0, min(x, fw - 2))
        y = max(0, min(y, fh - 2))
        w = max(8, min(w, fw - x))
        h = max(8, min(h, fh - y))

        patch = frame_bgr[y : y + h, x : x + w]
        if patch.size == 0:
            return False

        self._template = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
        self._bbox = (float(x), float(y), float(w), float(h))
        self._frame_shape = (fh, fw)
        self._misses = 0

        self._cv_tracker = self._create_cv_tracker()
        if self._cv_tracker is not None:
            self._cv_tracker.init(frame_bgr, (x, y, w, h))
        return True

    def _template_search(self, gray: np.ndarray) -> tuple[float, float, float, float] | None:
        if self._template is None or self._bbox is None:
            return None
        x, y, w, h = self._bbox
        fh, fw = gray.shape[:2]
        tw, th = self._template.shape[1], self._template.shape[0]

        # Search window around last position (2x box, min 80px pad)
        pad = max(80, int(max(w, h) * 1.2))
        x0 = max(0, int(x) - pad)
        y0 = max(0, int(y) - pad)
        x1 = min(fw, int(x + w) + pad)
        y1 = min(fh, int(y + h) + pad)
        roi = gray[y0:y1, x0:x1]
        if roi.shape[0] < th or roi.shape[1] < tw:
            return None

        res = cv2.matchTemplate(roi, self._template, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, max_loc = cv2.minMaxLoc(res)
        if max_val < 0.45:
            return None

        nx = float(x0 + max_loc[0])
        ny = float(y0 + max_loc[1])
        return (nx, ny, float(tw), float(th))

    def update(self, frame_bgr: np.ndarray) -> tuple[bool, tuple[float, float, float, float] | None]:
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        cand_cv = None

        if self._cv_tracker is not None:
            ok, box = self._cv_tracker.update(frame_bgr)
            if ok:
                cand_cv = tuple(float(v) for v in box)

        cand_tm = self._template_search(gray)

        # Prefer template when confident; else CSRT; else fail
        chosen = None
        if cand_tm is not None:
            chosen = cand_tm
            self._misses = 0
        elif cand_cv is not None:
            chosen = cand_cv
            self._misses = 0
            # Periodically refresh template from CSRT box so lock adapts a bit
            x, y, w, h = [int(v) for v in cand_cv]
            fh, fw = frame_bgr.shape[:2]
            if 0 <= x < fw and 0 <= y < fh and w > 4 and h > 4:
                patch = frame_bgr[y : min(fh, y + h), x : min(fw, x + w)]
                if patch.size > 0 and patch.shape[0] > 4 and patch.shape[1] > 4:
                    # Slow template update (blend) to avoid drift
                    new_t = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
                    if self._template is not None and new_t.shape == self._template.shape:
                        self._template = cv2.addWeighted(self._template, 0.85, new_t, 0.15, 0)
        else:
            self._misses += 1
            if self._misses > 20:
                return False, None
            return True, self._bbox  # hold last box briefly

        self._bbox = chosen
        return True, chosen

    @property
    def bbox(self):
        return self._bbox


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def focal_from_fov(frame_w: int, fov_h_deg: float) -> float:
    fov = max(10.0, min(170.0, float(fov_h_deg)))
    return (frame_w * 0.5) / math.tan(math.radians(fov) * 0.5)


def focal_from_sample(pixel_size: float, known_distance_m: float, known_width_m: float) -> float:
    return (max(1.0, pixel_size) * known_distance_m) / max(0.01, known_width_m)


def bbox_size_px(bbox: tuple[float, float, float, float], axis: str) -> float:
    _, _, w, h = bbox
    if axis == "height":
        return max(1.0, h)
    if axis == "max":
        return max(1.0, w, h)
    return max(1.0, w)


def load_calib(cfg: SystemConfig) -> dict:
    if not CALIB_PATH.exists():
        return {}
    try:
        data = json.loads(CALIB_PATH.read_text(encoding="utf-8"))
        # Ignore absurd saved widths (e.g. user entered meters as 3.0 by mistake)
        w = float(data.get("known_object_width_m", cfg.distance.known_object_width_m))
        if 0.01 <= w <= 2.5:
            cfg.distance.known_object_width_m = w
        if "focal_length_px" in data:
            cfg.distance.focal_length_px = float(data["focal_length_px"])
        if "desired_distance_m" in data:
            cfg.distance.desired_distance_m = float(data["desired_distance_m"])
        if "fov_h_deg" in data:
            cfg.camera.fov_h_deg = float(data["fov_h_deg"])
        print(f" Loaded calibration from {CALIB_PATH.name}")
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


def draw_hud(frame, *, locked, bbox, size_px, dist_m, dist_status, pitch_off, rc, fps, cfg, axis, calibrated):
    h, w = frame.shape[:2]
    cv2.drawMarker(frame, (w // 2, h // 2), (80, 80, 80), cv2.MARKER_CROSS, 22, 1)

    panel = np.zeros((180, w, 3), dtype=np.uint8)
    panel[:] = (18, 18, 18)
    known_cm = cfg.distance.known_object_width_m * 100.0
    cal = "OK calibrated" if calibrated else "NOT calibrated - press C"

    lines = [
        f"LIVE LOCK + DISTANCE   FPS {fps:.0f}   {cal}",
        "Drag TIGHT box on object | C=calibrate | R=unlock | A=axis | S=save | Q=quit",
        f"Object size setting: {known_cm:.1f} cm    Focal: {cfg.distance.focal_length_px:.0f} px    Axis: {axis}",
        "C asks: (1) how far is the object in METERS   (2) how WIDE is it in CENTIMETERS",
    ]
    if locked and bbox is not None and dist_m is not None:
        roll, pitch, yaw, thr = rc
        lines += [
            f"LOCKED  box={size_px:.0f}px   DIST={dist_m:.2f} m ({dist_m*100:.0f} cm)  [{dist_status}]  pitch={pitch_off:+.0f}",
            f"RC  R{roll} P{pitch} Y{yaw} T{thr}",
        ]
    else:
        lines += ["NO LOCK - drag a box around the target", ""]

    for i, text in enumerate(lines):
        color = (200, 200, 200)
        if i == 0:
            color = (80, 180, 255) if calibrated else (40, 40, 255)
        if locked and i == 4:
            color = (0, 230, 140)
        cv2.putText(panel, text, (10, 24 + i * 26), cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 1, cv2.LINE_AA)
    return np.vstack([frame, panel])


def run_calibration(cfg, estimator, controller, bbox, axis) -> bool:
    """Ask two clear questions. Returns True if calibrated."""
    print()
    print("=" * 60)
    print(" CALIBRATION — answer these 2 questions")
    print("=" * 60)
    print(" Q1 DISTANCE = how far the object is from the camera.")
    print("    Use a tape measure. Example: stand 1 meter away -> type 1")
    print("    Example: 1.5 meters away -> type 1.5")
    print()
    print(" Q2 OBJECT WIDTH = how wide the thing inside your box is,")
    print("    in CENTIMETERS (not meters!).")
    print("    Examples: phone ~7   water bottle ~7   laptop ~30   person shoulders ~45")
    print("=" * 60)

    try:
        dist_m = ask_float("Q1) Distance to object in METERS", 1.0)
        width_cm = ask_float("Q2) Object width in CENTIMETERS", 30.0)
    except Exception as exc:
        print(f" Cancelled: {exc}")
        return False

    if not (0.15 <= dist_m <= 40.0):
        print(f" Distance {dist_m} m looks wrong. Use something like 0.5 to 10.")
        return False
    if not (1.0 <= width_cm <= 250.0):
        print(f" Width {width_cm} cm looks wrong. Use centimeters (phone=7, not 0.07).")
        return False

    width_m = width_cm / 100.0
    size_px = bbox_size_px(bbox, axis)
    new_f = focal_from_sample(size_px, dist_m, width_m)

    cfg.distance.focal_length_px = new_f
    cfg.distance.known_object_width_m = width_m
    estimator.update_config(cfg.distance)
    controller.distance_estimator.update_config(cfg.distance)

    check = estimator.estimate_distance(size_px)
    print()
    print(f" Box size on screen : {size_px:.0f} pixels")
    print(f" You said distance  : {dist_m:.2f} m")
    print(f" You said width     : {width_cm:.1f} cm ({width_m:.3f} m)")
    print(f" Computed focal     : {new_f:.1f} px")
    print(f" Check (should ~{dist_m:.2f} m): {check:.2f} m")
    if abs(check - dist_m) < 0.05:
        print(" Calibration looks GOOD.")
    else:
        print(" Warning: check mismatch — redraw a tighter box and try C again.")
    print("=" * 60)
    print()
    return True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width-cm", type=float, default=None, help="Object width in centimeters")
    parser.add_argument("--fov", type=float, default=None)
    parser.add_argument("--axis", choices=("width", "height", "max"), default="width")
    args = parser.parse_args()

    cfg = SystemConfig()
    # Sensible default object size (30 cm) — never leave absurd values
    if cfg.distance.known_object_width_m > 2.5 or cfg.distance.known_object_width_m < 0.01:
        cfg.distance.known_object_width_m = 0.30

    meta = load_calib(cfg)
    axis = meta.get("size_axis", args.axis)
    calibrated = "focal_length_px" in meta and 0.01 <= cfg.distance.known_object_width_m <= 2.5

    if args.width_cm is not None:
        cfg.distance.known_object_width_m = args.width_cm / 100.0
    if args.fov is not None:
        cfg.camera.fov_h_deg = args.fov

    cfg.prediction.enable_kalman = False
    cfg.offsets.deadzone_norm = 0.01

    cap = open_camera(args.camera)
    if cap is None:
        print(f"ERROR: cannot open camera {args.camera}")
        return 1

    ok, sample = cap.read()
    frame_w = int(sample.shape[1]) if ok else 1280
    if not calibrated:
        cfg.distance.focal_length_px = focal_from_fov(frame_w, cfg.camera.fov_h_deg)
        print(f" Temporary FOV focal={cfg.distance.focal_length_px:.0f}px — press C to calibrate properly.")

    win = "Arjuna Live Distance Lock"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1100, 820)

    drag = RoiDrag()
    cv2.setMouseCallback(win, drag.on_mouse)

    controller = FPVFollowController(cfg)
    estimator = DistanceEstimator(cfg.distance)
    lock = PixelLockTracker()
    locked = False
    bbox = None

    print("=" * 60)
    print(" HOW DISTANCE WORKS")
    print("   Dist(m) = (object_width_m * focal_px) / box_width_px")
    print()
    print(" WHAT THOSE TWO QUESTIONS MEAN (press C):")
    print("   Q1 Distance (meters)  = tape-measure distance camera -> object")
    print("                           You typed 1  => object is 1 meter away")
    print("   Q2 Width (centimeters)= real size of the object in the box")
    print("                           Phone ~7 cm, bottle ~7 cm, person ~45 cm")
    print("                           If you type 1 here it means 1 cm (tiny!)")
    print("                           Earlier [3.0] was wrongly in METERS — that")
    print("                           made the math nonsense. Now Q2 is in CM.")
    print("=" * 60)

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
                controller.reset()
                print(f" LOCKED {rw}x{rh}px — keep object similar; press C to calibrate")
            else:
                locked = False
                bbox = None

        size_px = 0.0
        if locked:
            ok_tr, box = lock.update(frame)
            if ok_tr and box is not None:
                bbox = box
                x, y, bw, bh = bbox
                size_px = bbox_size_px(bbox, axis)
                cv2.rectangle(frame, (int(x), int(y)), (int(x + bw), int(y + bh)), (0, 220, 80), 2)
                cx, cy = int(x + bw / 2), int(y + bh / 2)
                cv2.circle(frame, (cx, cy), 4, (0, 220, 80), -1)
                cv2.line(frame, (fw // 2, fh // 2), (cx, cy), (0, 160, 255), 1)
            else:
                print(" LOCK LOST — draw the box again")
                locked = False
                bbox = None

        drag.draw(frame)

        dist_m = None
        dist_status = "-"
        pitch_off = 0.0
        rc = (1500, 1500, 1500, 1500)
        estimator.update_config(cfg.distance)
        controller.distance_estimator.update_config(cfg.distance)

        if locked and bbox is not None:
            size_px = bbox_size_px(bbox, axis)
            x, y, bw, bh = bbox
            synth = (x, y, size_px, bh)
            roll, pitch, yaw, thr = controller.update(synth, fw, fh, base_throttle=1500)
            rc = (roll, pitch, yaw, thr)
            dist_m = estimator.estimate_distance(size_px)
            full = estimator.compute_following_control(size_px, dt=0.033)
            dist_status = full.status
            pitch_off = full.recommended_pitch_offset
            cv2.putText(
                frame, f"{dist_m:.2f} m", (int(x), max(22, int(y) - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 180), 2, cv2.LINE_AA,
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
            frame, locked=locked, bbox=bbox, size_px=size_px, dist_m=dist_m,
            dist_status=dist_status, pitch_off=pitch_off, rc=rc, fps=fps,
            cfg=cfg, axis=axis, calibrated=calibrated,
        )
        cv2.imshow(win, display)
        key = cv2.waitKey(1) & 0xFF

        if key in (ord("q"), ord("Q")):
            break
        if key == 27:
            if locked:
                locked, bbox = False, None
            else:
                break
        if key in (ord("r"), ord("R")):
            locked, bbox = False, None
            print(" UNLOCKED")
        if key in (ord("a"), ord("A")):
            axis = {"width": "height", "height": "max", "max": "width"}[axis]
            print(f" Axis -> {axis}")
        if key in (ord("s"), ord("S")):
            save_calib(cfg, axis)
            calibrated = True
        if key in (ord("c"), ord("C")):
            if not locked or bbox is None:
                print(" Lock an object first (drag a box), then press C")
            else:
                if run_calibration(cfg, estimator, controller, bbox, axis):
                    calibrated = True
                    save_calib(cfg, axis)

        if ord("0") <= key <= ord("9"):
            new_idx = key - ord("0")
            if new_idx != cam_idx:
                new_cap = open_camera(new_idx)
                if new_cap is not None:
                    cap.release()
                    cap = new_cap
                    cam_idx = new_idx
                    locked, bbox = False, None
                    print(f" Camera {cam_idx}")

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
