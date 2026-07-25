"""Smoke test: controller + settings UI work with mount angles set. Run directly."""

from __future__ import annotations

import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from config import SystemConfig  # noqa: E402
from control.fpv_follow import FPVFollowController  # noqa: E402

W, H = 1280, 720

cfg = SystemConfig()
cfg.camera.mount_pitch_deg = 25.0
cfg.camera.mount_roll_deg = -5.0
cfg.camera.stabilize_with_attitude = True

ctl = FPVFollowController(cfg)
ctl.set_vehicle_attitude(2.0, -3.0)

print("-- follow controller with a 25 deg up-tilted, 5 deg rolled camera --")
for label, cyy in (("target at image centre", H / 2), ("target low in frame", H * 0.88)):
    for _ in range(6):
        out = ctl.update((W / 2 - 60, cyy, 120.0, 120.0), W, H, base_throttle=1500)
    b = ctl.last_bearing
    d = ctl.last_distance
    print(
        f"  {label:24s} RPYT={out}  el={b.el_rad * 57.2958:+6.2f}deg  "
        f"az={b.az_rad * 57.2958:+6.2f}deg  los={b.slant_m:5.2f}m  ground={d.distance_m:5.2f}m"
    )

assert ctl.last_bearing is not None and ctl.last_distance is not None
assert ctl.last_distance.slant_m >= ctl.last_distance.distance_m - 1e-6, "ground range must not exceed LOS"

# Config round-trips through JSON (presets must keep mount angles)
tmp = Path(__file__).with_name("_mount_roundtrip.json")
cfg.save_json(tmp)
back = SystemConfig.load_json(tmp)
assert back.camera.mount_pitch_deg == 25.0, back.camera.mount_pitch_deg
assert back.camera.mount_roll_deg == -5.0
assert back.camera.stabilize_with_attitude is True
tmp.unlink()
print("-- preset round-trip keeps mount angles: OK")

# Old presets without the new keys must still load with sane defaults
legacy = Path(__file__).with_name("_legacy.json")
legacy.write_text('{"camera": {"fov_h_deg": 100.0, "camera_index": 2}}', encoding="utf-8")
old = SystemConfig.load_json(legacy)
assert old.camera.fov_h_deg == 100.0 and old.camera.camera_index == 2
assert old.camera.mount_pitch_deg == 0.0 and old.camera.vertical_ref == "level"
legacy.unlink()
print("-- legacy preset without mount keys loads with defaults: OK")

# HUD must survive every mount angle, including straight up / straight down
import numpy as np  # noqa: E402

from core.tracking_worker import TrackingWorkerThread  # noqa: E402


class _HudStub:
    """Minimal stand-in so the HUD renderer can be exercised without a camera."""

    def __init__(self, sys_config, controller):
        self.sys_config = sys_config
        self.controller = controller
        self.active_target = None
        self.active_cam_idx = 0
        self.current_fps = 30.0
        self.is_connected = False


class _Safety:
    is_safe = True
    reason = "OK"


stub = _HudStub(cfg, ctl)
for tilt in (-90.0, -45.0, 0.0, 25.0, 89.0, 90.0):
    for roll in (0.0, -30.0, 95.0, 180.0):
        cfg.camera.mount_pitch_deg = tilt
        cfg.camera.mount_roll_deg = roll
        cfg.camera.vertical_ref = "level"
        frame = np.zeros((H, W, 3), dtype=np.uint8)
        TrackingWorkerThread._render_hud(
            stub, frame, True, (600, 300, 120, 120), 0.87, "scale",
            _Safety(), 1500, 1500, 1500, 1500, 4.2, W, H,
        )
print("-- HUD renders for tilt -90..+90 and roll 0..180: OK")

cfg.camera.mount_pitch_deg = 25.0
cfg.camera.mount_roll_deg = -5.0

from PyQt6.QtWidgets import QApplication  # noqa: E402

from gui.pages.settings_page import SettingsPage  # noqa: E402

app = QApplication.instance() or QApplication([])
page = SettingsPage(cfg)
panel = page.panel_device
assert panel.sp_mount_pitch.value() == 25.0
panel.sp_mount_pitch.setValue(18.0)
assert cfg.camera.mount_pitch_deg == 18.0, cfg.camera.mount_pitch_deg
panel.cmb_vert_ref.setCurrentIndex(1)
assert cfg.camera.vertical_ref == "image", cfg.camera.vertical_ref
panel.load_config(SystemConfig())
assert panel.sp_mount_pitch.value() == 0.0
print("-- settings page mount controls read/write config: OK")

print("\nALL WIRING CHECKS PASSED")
