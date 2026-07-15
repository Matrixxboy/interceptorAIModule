"""Load / save FPV calibration JSON used by main.py and calibration_fpv.py."""

from __future__ import annotations

import json
from copy import deepcopy
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PATH = ROOT / "calibration.json"

DEFAULT_CALIB: dict[str, Any] = {
    "camera_index": 1,
    "frame_width": 1280,
    "frame_height": 720,
    "control_port": "COM5",
    "control_baud": 57600,
    "use_yolo": True,
    "yolo_every_n": 4,
    "tracker_type": "CSRT",
    "min_bbox_area": 250,
    "max_lost_frames": 60,
    "send_hz": 50,
    "mode_on_value": 1900,
    "fpv": {
        "deadzone_norm": 0.018,
        "deadzone_bleed": 0.15,
        "yaw_kp": 340.0,
        "yaw_ki": 45.0,
        "yaw_kd": 60.0,
        "pitch_kp": 310.0,
        "pitch_ki": 40.0,
        "pitch_kd": 55.0,
        "roll_kp": 80.0,
        "roll_ki": 8.0,
        "roll_kd": 15.0,
        "max_yaw": 400.0,
        "max_pitch": 360.0,
        "max_roll": 120.0,
        "i_limit": 0.45,
        "d_filter": 0.35,
        "expo": 0.85,
        "lead_s": 0.14,
        "meas_alpha": 0.50,
        "out_alpha": 0.60,
        "slew_yaw": 1600.0,
        "slew_pitch": 1400.0,
        "slew_roll": 600.0,
        "yaw_dir": 1.0,
        "pitch_dir": -1.0,
        "roll_dir": 1.0,
        "use_roll": False,
        "roll_blend": 0.25,
        "rc_mid": 1500,
        "rc_min": 1000,
        "rc_max": 2000,
    },
}


def default_calibration() -> dict[str, Any]:
    return deepcopy(DEFAULT_CALIB)


def load_calibration(path: Path | str | None = None) -> dict[str, Any]:
    """Load calibration JSON; merge over defaults so new keys always exist."""
    path = Path(path) if path else DEFAULT_PATH
    data = default_calibration()
    if not path.is_file():
        return data
    try:
        with path.open("r", encoding="utf-8") as f:
            loaded = json.load(f)
    except (OSError, json.JSONDecodeError):
        return data

    for key, value in loaded.items():
        if key == "fpv" and isinstance(value, dict):
            data["fpv"].update(value)
        else:
            data[key] = value
    return data


def save_calibration(data: dict[str, Any], path: Path | str | None = None) -> Path:
    path = Path(path) if path else DEFAULT_PATH
    merged = default_calibration()
    for key, value in data.items():
        if key == "fpv" and isinstance(value, dict):
            merged["fpv"].update(value)
        else:
            merged[key] = value
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(merged, f, indent=2)
        f.write("\n")
    return path


def fpv_config_from_dict(fpv: dict[str, Any]):
    """Build FPVFollowConfig from calibration['fpv'] dict."""
    from dataclasses import fields

    from control.fpv_follow import FPVFollowConfig

    base = default_calibration()["fpv"]
    base.update(fpv or {})
    known = {f.name for f in fields(FPVFollowConfig)}
    kwargs = {k: base[k] for k in known if k in base}
    return FPVFollowConfig(**kwargs)
