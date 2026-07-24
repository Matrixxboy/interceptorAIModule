"""Load / save distance calibration (focal + object size + size axis)."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

from config import DistanceConfig, SystemConfig
from paths import BUNDLE_DIR, PRESETS_DIR, ROOT

CALIB_NAME = "distance_calib.json"
MIN_CALIB_DISTANCE_M = 0.05
MAX_CALIB_DISTANCE_M = 30.0
MIN_OBJECT_CM = 1.0
MAX_OBJECT_CM = 250.0


def focal_from_sample(pixel_size: float, known_distance_m: float, known_width_m: float) -> float:
    """Pinhole: F = (pixel_size * distance_m) / object_m."""
    return (max(1.0, float(pixel_size)) * float(known_distance_m)) / max(0.005, float(known_width_m))


def estimate_distance_m(pixel_size: float, focal_px: float, known_width_m: float) -> float:
    """Pinhole: Dist = (object_m * focal_px) / pixel_size."""
    return (max(0.005, float(known_width_m)) * max(1.0, float(focal_px))) / max(1.0, float(pixel_size))


def bbox_size_px(bbox: tuple[float, float, float, float], axis: str) -> float:
    _, _, w, h = bbox
    if axis == "height":
        return max(1.0, float(h))
    if axis == "max":
        return max(1.0, float(w), float(h))
    if axis == "diag":
        return max(1.0, math.hypot(float(w), float(h)))
    return max(1.0, float(w))


def validate_calib_inputs(distance_m: float, object_cm: float) -> str | None:
    """Return an error message, or None if inputs are valid."""
    if not (MIN_CALIB_DISTANCE_M <= distance_m <= MAX_CALIB_DISTANCE_M):
        return f"Distance {distance_m:.3f} m invalid (use {MIN_CALIB_DISTANCE_M}–{MAX_CALIB_DISTANCE_M})."
    if not (MIN_OBJECT_CM <= object_cm <= MAX_OBJECT_CM):
        return f"Object size {object_cm:.1f} cm invalid (use {MIN_OBJECT_CM:.0f}–{MAX_OBJECT_CM:.0f} cm)."
    return None


def calib_search_paths() -> list[Path]:
    """Prefer writable AppData, then project presets, then bundle."""
    return [
        PRESETS_DIR / CALIB_NAME,
        ROOT / "presets" / CALIB_NAME,
        BUNDLE_DIR / "presets" / CALIB_NAME,
    ]


def primary_calib_path() -> Path:
    """Where new calibrations are written."""
    return PRESETS_DIR / CALIB_NAME


def find_existing_calib() -> Path | None:
    for p in calib_search_paths():
        if p.exists():
            return p
    return None


def apply_calib_dict(cfg: SystemConfig, data: dict[str, Any]) -> bool:
    """Apply calib JSON fields onto SystemConfig. Returns True if anything applied."""
    applied = False
    dist = cfg.distance

    if "focal_length_px" in data:
        f = float(data["focal_length_px"])
        if 50.0 <= f <= 20000.0:
            dist.focal_length_px = f
            applied = True

    if "known_object_width_m" in data:
        w = float(data["known_object_width_m"])
        if 0.01 <= w <= 2.5:
            dist.known_object_width_m = w
            applied = True

    if "desired_distance_m" in data:
        d = float(data["desired_distance_m"])
        if 0.05 <= d <= 100.0:
            dist.desired_distance_m = d
            applied = True

    axis = data.get("size_axis")
    if axis in ("width", "height", "max", "diag"):
        dist.size_axis = axis
        applied = True

    if "fov_h_deg" in data and hasattr(cfg, "camera"):
        fov = float(data["fov_h_deg"])
        if 10.0 <= fov <= 180.0:
            cfg.camera.fov_h_deg = fov
            applied = True

    return applied


def load_distance_calib(cfg: SystemConfig, path: Path | None = None) -> dict[str, Any]:
    """Load calib file into cfg. Returns the raw dict (empty if missing)."""
    p = path or find_existing_calib()
    if p is None or not p.exists():
        return {}
    try:
        data = json.loads(p.read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            return {}
        apply_calib_dict(cfg, data)
        return data
    except Exception:
        return {}


def save_distance_calib(cfg: SystemConfig, path: Path | None = None) -> Path:
    """Persist current distance calib. Also mirrors into project presets when writable."""
    data = {
        "focal_length_px": cfg.distance.focal_length_px,
        "known_object_width_m": cfg.distance.known_object_width_m,
        "desired_distance_m": cfg.distance.desired_distance_m,
        "fov_h_deg": cfg.camera.fov_h_deg,
        "size_axis": cfg.distance.size_axis,
    }
    targets = [path] if path is not None else [primary_calib_path()]
    # Keep project copy in sync during development
    project_copy = ROOT / "presets" / CALIB_NAME
    if path is None and project_copy.parent.exists():
        targets.append(project_copy)

    written: Path | None = None
    for t in targets:
        if t is None:
            continue
        try:
            t.parent.mkdir(parents=True, exist_ok=True)
            t.write_text(json.dumps(data, indent=2), encoding="utf-8")
            written = t
        except OSError:
            continue
    if written is None:
        raise OSError("Could not write distance calibration file")
    return written


def is_calibrated(cfg: DistanceConfig | SystemConfig) -> bool:
    """Heuristic: focal was set away from the blank default via a calib file."""
    dist = cfg.distance if isinstance(cfg, SystemConfig) else cfg
    return find_existing_calib() is not None and dist.focal_length_px > 100.0
