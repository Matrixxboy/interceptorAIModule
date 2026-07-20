"""Verification & Unit Test Suite for Autonomous FPV Drone Tracking System."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from config import SystemConfig
from control.fpv_follow import FPVFollowController
from control.pid_controller import PIDAxisConfig, PIDController
from estimation.distance_estimator import DistanceEstimator
from safety.failsafe_manager import FailsafeManager
from telemetry.telemetry_logger import TelemetryLogger
from tracking.kalman_filter import BBoxKalmanFilter
from tracking.motion_predictor import MotionPredictor


def test_config() -> None:
    print("[TEST] Testing SystemConfig serialization...")
    cfg = SystemConfig()
    test_file = ROOT / "presets" / "test_preset.json"
    cfg.save_json(test_file)
    assert test_file.exists()

    loaded = SystemConfig.load_json(test_file)
    assert loaded.distance.desired_distance_m == cfg.distance.desired_distance_m
    test_file.unlink()
    print("  -> SystemConfig PASSED")


def test_kalman_and_prediction() -> None:
    print("[TEST] Testing Kalman Filter and Motion Predictor...")
    kf = BBoxKalmanFilter()
    kf.init((100, 100, 50, 50))

    for i in range(10):
        kf.predict(dt=0.033)
        kf.update((100 + i * 5, 100 + i * 2, 50, 50))

    box = kf.get_bbox_xywh()
    assert box[0] > 100

    mp = MotionPredictor()
    traj = mp.update((100, 100, 50, 50), dt=0.033)
    assert traj.aim_cx > 0
    print("  -> Kalman Filter & Motion Predictor PASSED")


def test_distance_estimator() -> None:
    print("[TEST] Testing Vision Distance Estimator...")
    de = DistanceEstimator()
    dist_near = de.estimate_distance(bbox_width_px=200)
    dist_far = de.estimate_distance(bbox_width_px=50)
    assert dist_near < dist_far

    ctrl = de.compute_following_control(bbox_width_px=200)
    assert ctrl.distance_m > 0
    print("  -> Distance Estimator PASSED")


def test_pid_and_controller() -> None:
    print("[TEST] Testing PID Controller & FPV Follow Controller...")
    pid = PIDController(PIDAxisConfig(300.0, 10.0, 20.0, 400.0))
    res = pid.update(error=0.1, dt=0.033)
    assert abs(res.output) > 0

    fc = FPVFollowController()
    roll, pitch, yaw = fc.update((100, 100, 60, 60), frame_w=1280, frame_h=720)
    assert 1000 <= roll <= 2000
    assert 1000 <= pitch <= 2000
    assert 1000 <= yaw <= 2000
    print("  -> PID & FPV Follow Controller PASSED")


def test_failsafe() -> None:
    print("[TEST] Testing Failsafe Manager...")
    fm = FailsafeManager()
    st1 = fm.evaluate(locked=True, confidence=0.9, distance_m=5.0)
    assert st1.is_safe

    st2 = fm.evaluate(locked=False, confidence=0.0, distance_m=None)
    assert st2.is_safe  # unlocked initial state

    fm.trigger_manual_override(True)
    st3 = fm.evaluate(locked=True, confidence=0.9, distance_m=5.0)
    assert not st3.is_safe and st3.override_active
    print("  -> Failsafe Manager PASSED")


def test_telemetry() -> None:
    print("[TEST] Testing Telemetry Logger...")
    tl = TelemetryLogger()
    tl.start_recording()
    tl.log(
        frame_idx=1,
        locked=True,
        confidence=0.85,
        source="yolo",
        error_x=10,
        error_y=-5,
        bbox_xywh=(100, 100, 50, 50),
        distance_m=4.5,
        vx=10.0,
        vy=0.0,
        roll=1500,
        pitch=1520,
        yaw=1480,
        throttle=1000,
        failsafe="System nominal",
    )
    assert len(tl.buffer) == 1

    csv_path = tl.export_csv()
    json_path = tl.export_json()
    assert csv_path.exists() and json_path.exists()
    csv_path.unlink()
    json_path.unlink()
    print("  -> Telemetry Logger PASSED")


if __name__ == "__main__":
    print("=" * 60)
    print(" RUNNING AUTONOMOUS MODULES TEST SUITE")
    print("=" * 60)
    test_config()
    test_kalman_and_prediction()
    test_distance_estimator()
    test_pid_and_controller()
    test_failsafe()
    test_telemetry()
    print("=" * 60)
    print(" ALL TESTS PASSED SUCCESSFULLY!")
    print("=" * 60)
