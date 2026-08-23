"""
Test: Pinhole distance estimation + PID -> RC mapping pipeline.

Validates the vision->control loop used by T.R.I.V.E.N.I / FPV follow:

  Distance = (Known_Width x Focal_Length) / Pixel_Width

  cx error  -> Yaw PID
  cy error  -> Throttle (altitude) PID
  distance  -> Pitch (follow distance) PID

Run:
  .\\.venv\\Scripts\\python.exe scripts\\test_distance_pid_pipeline.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from config import DistanceConfig, PIDAxisConfig, SystemConfig
from control.fpv_follow import FPVFollowController
from control.pid_controller import PIDController
from estimation.distance_estimator import DistanceEstimator


PASSED = 0
FAILED = 0


def check(name: str, cond: bool, detail: str = "") -> None:
    global PASSED, FAILED
    if cond:
        PASSED += 1
        print(f"  PASS  {name}" + (f"  ({detail})" if detail else ""))
    else:
        FAILED += 1
        print(f"  FAIL  {name}" + (f"  - {detail}" if detail else ""))


# ---------------------------------------------------------------------------
# 1. Pinhole camera distance
# ---------------------------------------------------------------------------

def test_pinhole_distance() -> None:
    print("\n[1] Pinhole distance estimation")
    cfg = DistanceConfig(
        focal_length_px=800.0,
        known_object_width_m=0.45,  # 45 cm (person shoulders)
        desired_distance_m=2.0,
    )
    est = DistanceEstimator(cfg)

    # At exactly the calibration distance: D = (W * f) / w_px
    # If object is 0.45 m wide and appears as 180 px at 2.0 m:
    #   f = (w_px * D) / W = (180 * 2.0) / 0.45 = 800  OK
    dist = est.estimate_distance(180.0)
    check("distance at calibration point", abs(dist - 2.0) < 1e-6, f"{dist:.4f} m")

    # Halve pixel width -> double distance
    dist_far = est.estimate_distance(90.0)
    check("half width -> 2x distance", abs(dist_far - 4.0) < 1e-6, f"{dist_far:.4f} m")

    # Double pixel width -> half distance
    dist_near = est.estimate_distance(360.0)
    check("double width -> 1/2 distance", abs(dist_near - 1.0) < 1e-6, f"{dist_near:.4f} m")

    # Zero / tiny width must not crash or return inf
    dist_min = est.estimate_distance(0.0)
    check("zero width is finite", math.isfinite(dist_min) and dist_min > 0)


def test_focal_length_roundtrip() -> None:
    print("\n[2] Focal length calibration round-trip")
    known_w = 0.30  # m
    known_d = 5.0   # m
    pixel_w = 120.0
    focal = (pixel_w * known_d) / known_w  # expected 2000

    est = DistanceEstimator(
        DistanceConfig(focal_length_px=focal, known_object_width_m=known_w)
    )
    recovered = est.estimate_distance(pixel_w)
    check("calibrate then recover distance", abs(recovered - known_d) < 1e-6, f"{recovered:.4f} m")


# ---------------------------------------------------------------------------
# 2. Distance PID -> pitch direction
# ---------------------------------------------------------------------------

def test_distance_pitch_pid() -> None:
    print("\n[3] Distance -> pitch PID direction")
    cfg = DistanceConfig(
        focal_length_px=800.0,
        known_object_width_m=0.45,
        desired_distance_m=2.0,  # 200 cm
        min_safe_distance_m=0.8,
        max_follow_distance_m=20.0,
        kp=100.0,
        ki=0.0,
        kd=0.0,
        max_pitch_offset=200.0,
    )
    est = DistanceEstimator(cfg)

    # Target farther than desired (small bbox) -> positive pitch offset (fly forward)
    # At 2 m, w_px ~= 180. At ~3 m, w_px ~= 120.
    far = est.compute_following_control(120.0, dt=0.033)
    check("TOO_FAR / farther than setpoint", far.distance_m > cfg.desired_distance_m, f"{far.distance_m:.2f} m")
    check("far -> forward pitch (+)", far.recommended_pitch_offset > 0, f"pitch={far.recommended_pitch_offset:.1f}")

    est.reset()
    # Target closer than desired (large bbox) -> negative pitch offset (fly back)
    near = est.compute_following_control(360.0, dt=0.033)  # ~1 m
    check("closer than setpoint", near.distance_m < cfg.desired_distance_m, f"{near.distance_m:.2f} m")
    check("near -> backward pitch (-)", near.recommended_pitch_offset < 0, f"pitch={near.recommended_pitch_offset:.1f}")

    est.reset()
    # Dangerously close -> safety override (force back)
    danger = est.compute_following_control(900.0, dt=0.033)  # very close
    check("TOO_CLOSE status", danger.status == "TOO_CLOSE", danger.status)
    check("too close -> strong backward", danger.recommended_pitch_offset < 0 and not danger.is_safe)


# ---------------------------------------------------------------------------
# 3. Centroid PID axes (yaw / throttle)
# ---------------------------------------------------------------------------

def test_centroid_pids() -> None:
    print("\n[4] Centroid error -> yaw / throttle PID")
    yaw_pid = PIDController(PIDAxisConfig(kp=100.0, ki=0.0, kd=0.0, max_output=300.0))
    alt_pid = PIDController(PIDAxisConfig(kp=100.0, ki=0.0, kd=0.0, max_output=300.0))

    # nx > 0 -> target right of center -> yaw right (positive output)
    yaw_right = yaw_pid.update(0.5, dt=0.033, deadzone=0.0)
    check("target right -> yaw +", yaw_right.output > 0, f"{yaw_right.output:.1f}")

    yaw_pid.reset()
    yaw_left = yaw_pid.update(-0.5, dt=0.033, deadzone=0.0)
    check("target left -> yaw -", yaw_left.output < 0, f"{yaw_left.output:.1f}")

    # ny > 0 -> target below center -> throttle down (handled as -alt in FPVFollow)
    alt_down = alt_pid.update(0.5, dt=0.033, deadzone=0.0)
    check("target below -> alt PID +", alt_down.output > 0, f"{alt_down.output:.1f}")

    alt_pid.reset()
    alt_up = alt_pid.update(-0.5, dt=0.033, deadzone=0.0)
    check("target above -> alt PID -", alt_up.output < 0, f"{alt_up.output:.1f}")


# ---------------------------------------------------------------------------
# 4. Full FPV follow controller (bbox -> RC)
# ---------------------------------------------------------------------------

def test_fpv_follow_pipeline() -> None:
    print("\n[5] Full FPV follow: bbox -> RC (roll, pitch, yaw, throttle)")
    cfg = SystemConfig()
    cfg.distance.focal_length_px = 800.0
    cfg.distance.known_object_width_m = 0.45
    cfg.distance.desired_distance_m = 2.0
    cfg.distance.min_safe_distance_m = 0.5
    cfg.distance.kp = 80.0
    cfg.distance.ki = 0.0
    cfg.distance.kd = 0.0
    cfg.yaw_pid.kp = 200.0
    cfg.yaw_pid.ki = 0.0
    cfg.yaw_pid.kd = 0.0
    cfg.altitude_pid.kp = 200.0
    cfg.altitude_pid.ki = 0.0
    cfg.altitude_pid.kd = 0.0
    cfg.offsets.deadzone_norm = 0.0
    cfg.prediction.enable_kalman = False

    frame_w, frame_h = 640, 480
    mid = 1500

    # --- Centered, at desired distance ---
    # At 2 m with f=800, W=0.45 -> w_px = (f*W)/D = 180
    ctrl = FPVFollowController(cfg)
    w_px = (cfg.distance.focal_length_px * cfg.distance.known_object_width_m) / cfg.distance.desired_distance_m
    cx, cy = frame_w // 2, frame_h // 2
    bbox = (cx - w_px / 2, cy - 40, w_px, 80.0)

    # Warm up a few frames so slew/filters settle toward setpoint
    for _ in range(8):
        roll, pitch, yaw, thr = ctrl.update(bbox, frame_w, frame_h, base_throttle=mid)

    check("RC channels in range", all(1000 <= v <= 2000 for v in (roll, pitch, yaw, thr)))
    check(
        "centered @ desired dist ~= mid sticks",
        abs(yaw - mid) < 80 and abs(pitch - mid) < 120,
        f"yaw={yaw} pitch={pitch} thr={thr}",
    )
    assert ctrl.last_distance is not None
    check(
        "estimated distance ~= 2.0 m",
        abs(ctrl.last_distance.distance_m - 2.0) < 0.15,
        f"{ctrl.last_distance.distance_m:.2f} m",
    )

    # --- Target on the RIGHT -> yaw should increase ---
    ctrl.reset()
    bbox_right = (480.0, cy - 40, w_px, 80.0)
    for _ in range(10):
        _, _, yaw_r, _ = ctrl.update(bbox_right, frame_w, frame_h, base_throttle=mid)
    check("target RIGHT -> yaw > mid", yaw_r > mid, f"yaw={yaw_r}")

    # --- Target on the LEFT -> yaw should decrease ---
    ctrl.reset()
    bbox_left = (40.0, cy - 40, w_px, 80.0)
    for _ in range(10):
        _, _, yaw_l, _ = ctrl.update(bbox_left, frame_w, frame_h, base_throttle=mid)
    check("target LEFT -> yaw < mid", yaw_l < mid, f"yaw={yaw_l}")

    # --- Target FAR (small bbox) -> pitch forward (project uses pitch_dir=-1 on +error) ---
    # recommended_pitch_offset > 0 when far; FPV applies pitch_dir * offset.
    # With default pitch_dir=-1, forward appears as pitch < mid. Verify distance + offset sign.
    ctrl.reset()
    bbox_far = (cx - 45, cy - 40, 90.0, 80.0)  # ~4 m
    for _ in range(10):
        _, pitch_far, _, _ = ctrl.update(bbox_far, frame_w, frame_h, base_throttle=mid)
    assert ctrl.last_distance is not None
    check("far target distance > desired", ctrl.last_distance.distance_m > 2.5, f"{ctrl.last_distance.distance_m:.2f} m")
    check(
        "far -> pitch moves off mid",
        abs(pitch_far - mid) > 20,
        f"pitch={pitch_far} (offset={ctrl.last_distance.recommended_pitch_offset:.1f})",
    )

    # --- Target HIGH in frame -> throttle should rise (climb) ---
    ctrl.reset()
    bbox_high = (cx - w_px / 2, 20.0, w_px, 80.0)
    for _ in range(10):
        _, _, _, thr_high = ctrl.update(bbox_high, frame_w, frame_h, base_throttle=mid)
    check("target HIGH -> throttle > mid", thr_high > mid, f"thr={thr_high}")

    # --- Target LOW in frame -> throttle should drop ---
    ctrl.reset()
    bbox_low = (cx - w_px / 2, 360.0, w_px, 80.0)
    for _ in range(10):
        _, _, _, thr_low = ctrl.update(bbox_low, frame_w, frame_h, base_throttle=mid)
    check("target LOW -> throttle < mid", thr_low < mid, f"thr={thr_low}")


# ---------------------------------------------------------------------------
# 5. Demo of the formula from the architecture note
# ---------------------------------------------------------------------------

def demo_architecture_numbers() -> None:
    print("\n[6] Architecture demo (200 cm setpoint, 45 cm target)")
    KNOWN_WIDTH_CM = 45.0
    FOCAL_LENGTH = 800.0
    DESIRED_CM = 200.0

    def calculate_distance_cm(pixel_width: float) -> float:
        if pixel_width <= 0:
            return 0.0
        return (KNOWN_WIDTH_CM * FOCAL_LENGTH) / pixel_width

    # Pixel width at desired distance
    w_at_desired = (KNOWN_WIDTH_CM * FOCAL_LENGTH) / DESIRED_CM
    d = calculate_distance_cm(w_at_desired)
    check("demo: at setpoint distance", abs(d - DESIRED_CM) < 1e-6, f"{d:.1f} cm")

    # Example from the note: 300 cm measured vs 200 cm setpoint -> move forward
    w_at_300 = (KNOWN_WIDTH_CM * FOCAL_LENGTH) / 300.0
    d300 = calculate_distance_cm(w_at_300)
    err = d300 - DESIRED_CM
    check("demo: 300 cm reading", abs(d300 - 300.0) < 1e-6, f"{d300:.1f} cm")
    check("demo: error +100 cm -> fly forward", err > 0, f"error={err:.1f} cm")


def main() -> int:
    print("=" * 60)
    print(" Distance + PID pipeline test")
    print("=" * 60)

    test_pinhole_distance()
    test_focal_length_roundtrip()
    test_distance_pitch_pid()
    test_centroid_pids()
    test_fpv_follow_pipeline()
    demo_architecture_numbers()

    print("\n" + "=" * 60)
    print(f" Results: {PASSED} passed, {FAILED} failed")
    print("=" * 60)
    return 0 if FAILED == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
