"""Smoke tests for NN config wiring + Kalman3D + PPN blend."""

from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from config import DetectionConfig, SystemConfig
from control.fpv_follow import FPVFollowController
from detection.hybrid_tracker import HybridYoloLockTracker
from tracking.kalman_filter_3d import KalmanFilter3D
from tracking.ppn_guidance import PPNGuidance


def test_detection_resolved_mode() -> None:
    cfg = DetectionConfig(mode="coco")
    # Without custom weights file, stays coco
    assert cfg.resolved_mode() in ("coco", "custom")
    custom = Path(cfg.custom_weights)
    if custom.is_file():
        assert cfg.resolved_mode() == "custom"
    else:
        assert cfg.resolved_mode() == "coco"
    print("OK detection resolved_mode")


def test_hybrid_detect_every_n() -> None:
    sys_cfg = SystemConfig()
    sys_cfg.detection.detect_every_n = 7
    sys_cfg.detection.detect_every_n_healthy = 21
    hy = HybridYoloLockTracker(det_cfg=sys_cfg.detection, tracker_cfg=sys_cfg.tracker)
    assert hy.yolo_every_n == 7
    assert hy.yolo_every_n_healthy == 21
    sys_cfg.detection.detect_every_n = 3
    hy.apply_detection_config(sys_cfg.detection)
    assert hy.yolo_every_n == 3
    print("OK hybrid detect_every_n")


def test_kalman3d_predict_then_update() -> None:
    kf = KalmanFilter3D(r_pos=1.0)
    kf.init((10.0, 0.0, 0.0))
    kf.predict(0.05)
    # Mild motion — should accept
    p = kf.update((10.2, 0.1, 0.0), depth_sigma_m=0.5, gate_m=5.0)
    assert kf.last_accepted
    assert abs(p[0] - 10.2) < 1.0
    # Huge jump — reject
    kf.predict(0.05)
    p2 = kf.update((50.0, 0.0, 0.0), depth_sigma_m=0.5, gate_m=3.0)
    assert not kf.last_accepted
    assert abs(p2[0] - 50.0) > 5.0  # stayed near predict, not measurement
    print("OK kalman3d gate", p, p2, "innov", kf.last_innovation_norm)


def test_ppn_exposes_omega() -> None:
    ppn = PPNGuidance(N_p=3.0, b=10.0)
    a_P, v_P, psi = ppn.compute_guidance(
        (20.0, 5.0, 0.0),
        (0.0, 2.0, 0.0),
        (8.0, 0.0, 0.0),
        0.033,
    )
    assert a_P.shape == (3,)
    assert ppn.last_omega_norm >= 0.0
    assert math.isfinite(psi)
    print("OK ppn omega", ppn.last_omega_norm, "a_P", a_P)


def test_follow_controller_smoke() -> None:
    cfg = SystemConfig()
    cfg.safety.ppn_enabled = True
    cfg.safety.ppn_gain = 30.0
    ctl = FPVFollowController(cfg)
    # Target offset to the right — should produce yaw deflection over a few frames
    last = (1500, 1500, 1500, 1500)
    for _ in range(8):
        last = ctl.update((900, 340, 100, 80), 1280, 720, base_throttle=1500)
    assert all(1000 <= v <= 2000 for v in last)
    assert ctl.kalman_3d.initialized
    print("OK follow RC", last, "ppn_scale", ctl.last_ppn_scale, "clr", ctl.last_closing_rate)


def main() -> None:
    test_detection_resolved_mode()
    test_hybrid_detect_every_n()
    test_kalman3d_predict_then_update()
    test_ppn_exposes_omega()
    test_follow_controller_smoke()
    print("ALL OK")


if __name__ == "__main__":
    main()
