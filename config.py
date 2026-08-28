"""Central configuration for FPV MSP visual lock + follow system."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Literal, Sequence

from paths import BUNDLE_DIR, DATA_DIR, LOGS_DIR, MODELS_DIR, PRESETS_DIR, ROOT

# Open-vocab prompts (used when detection.mode = "world")
AERIAL_THREAT_CLASSES: tuple[str, ...] = (
    "drone",
    "quadcopter",
    "UAV",
    "FPV drone",
    "missile",
    "cruise missile",
    "rocket",
    "projectile",
    "aircraft",
    "helicopter"
)

DetectionMode = Literal["world", "coco", "custom"]
TrackerBackend = Literal["bytetrack", "botsort", "iou", "csrt", "kcf"]


@dataclass
class PIDAxisConfig:
    kp: float = 320.0
    ki: float = 40.0
    kd: float = 55.0
    max_output: float = 380.0
    i_limit: float = 0.45
    d_filter: float = 0.35


@dataclass
class DistanceConfig:
    focal_length_px: float = 800.0  # Camera focal length in pixels
    known_object_width_m: float = 0.30  # Physical size of object along size_axis (meters)
    size_axis: str = "max"  # "width" | "height" | "max" | "diag" — which bbox dim → distance
    desired_distance_m: float = 5.0  # Safe nominal follow distance in meters
    min_safe_distance_m: float = 2.0  # Dangerously close threshold
    max_follow_distance_m: float = 25.0  # Maximum follow range
    kp: float = 180.0
    ki: float = 18.0
    kd: float = 30.0
    max_pitch_offset: float = 320.0  # Pitch stick change for distance correction


@dataclass
class PredictionConfig:
    enable_kalman: bool = True
    process_noise_q: float = 1e-2
    measurement_noise_r: float = 1e-1
    lead_time_s: float = 0.12  # Seconds of lead trajectory prediction
    smoothing_factor: float = 0.45


@dataclass
class SafetyConfig:
    min_conf_threshold: float = 0.35
    follow_min_confidence: float = 0.60  # Only follow when detection conf ≥ this
    # Per-axis follow speed (live-tunable). 0 = axis idle, 1 = full configured authority.
    yaw_speed_scale: float = 0.50
    pitch_speed_scale: float = 0.90
    throttle_speed_scale: float = 0.50
    roll_speed_scale: float = 0.25
    # Legacy aliases — kept so older presets still load; mirrored onto per-axis scales.
    follow_speed_scale: float = 0.50
    follow_pitch_scale: float = 0.90
    max_lost_frames: int = 45
    reacquisition_timeout_s: float = 2.5
    max_yaw_rate: float = 1600.0  # µs/s slew limit
    max_climb_rate: float = 1400.0  # µs/s slew limit
    max_descent_rate: float = 1200.0  # µs/s slew limit
    max_pitch_rate: float = 2200.0  # µs/s slew for forward/back distance pitch
    max_forward_speed: float = 350.0  # max pitch µs offset forward
    max_backward_speed: float = 250.0  # max pitch µs offset backward
    max_acceleration: float = 1800.0  # µs/s^2 acceleration limit


@dataclass
class OffsetsConfig:
    horizontal_offset_norm: float = 0.0  # Center offset [-1..1]
    vertical_offset_norm: float = 0.0
    deadzone_norm: float = 0.02
    deadzone_bleed: float = 0.15


@dataclass
class CameraConfig:
    fov_h_deg: float = 90.0
    fov_v_deg: float = 60.0
    frame_width: int = 1280
    frame_height: int = 720
    camera_index: int = 1
    target_fps: float = 60.0  # Cap vision loop (uncapped races to 200–300+ FPS on fast GPUs)
    # --- Mount geometry: lets the camera sit at ANY angle, not just straight ahead ---
    mount_pitch_deg: float = 0.0  # + = tilted UP (typical FPV cruise tilt 15–30°)
    mount_roll_deg: float = 0.0  # + = rotated clockwise in the image
    mount_yaw_deg: float = 0.0  # + = aimed right of the nose
    stabilize_with_attitude: bool = False  # Subtract live FC roll/pitch → gravity-levelled aim
    vertical_ref: Literal["level", "image"] = "level"  # "level" = mount-corrected, "image" = legacy
    desired_elevation_deg: float = 0.0  # Elevation to hold the target at (0 = same height as us)
    use_calibrated_focal: bool = True  # Prefer measured focal length over nominal FOV for angles


@dataclass
class DetectionConfig:
    mode: DetectionMode = "coco"
    model_name: str = "yolo11_fast_precision.onnx"
    model_path: Path = field(default_factory=lambda: MODELS_DIR / "yolo11_fast_precision.onnx")
    custom_weights: Path = field(
        default_factory=lambda: MODELS_DIR / "drone_missile_best.onnx"
    )
    imgsz: int = 416
    conf_threshold: float = 0.25
    iou_threshold: float = 0.45
    max_det: int = 20
    world_classes: Sequence[str] = AERIAL_THREAT_CLASSES
    class_filter: Sequence[int] = (0, 4, 14, 32, 33)
    min_box_area_frac: float = 0.00005
    max_box_area_frac: float = 0.35
    device: str = "auto"  # "auto" | "cuda" | "cpu"
    half: bool = True
    detect_every_n: int = 3
    augment: bool = False


@dataclass
class TrackerConfig:
    backend: TrackerBackend = "bytetrack"
    max_age: int = 45
    min_hits: int = 2
    iou_match_threshold: float = 0.25
    lock_tracker: Literal["csrt", "kcf", "none"] = "none"  # none = faster; scale lock handles size
    reacquire_iou: float = 0.12
    reacquire_max_frames: int = 90
    enable_template_fallback: bool = True
    template_match_threshold: float = 0.45


@dataclass
class AuxChannelsConfig:
    arm_channel: int = 4
    mode_channel: int = 5
    arm_high: int = 1800
    arm_low: int = 1000
    mode_high: int = 1900
    mode_low: int = 1000


@dataclass
class RCControlConfig:
    # Axis channel mappings (Defaults to AETR: 0, 1, 2, 3)
    roll_channel: int = 0
    pitch_channel: int = 1
    throttle_channel: int = 2
    yaw_channel: int = 3
    
    # AUX switch mappings
    lock_channel: int = 6  # 0-based CH7 (AUX3)
    follow_channel: int = 5  # 0-based CH6 (AUX2)
    lock_threshold: int = 1700
    follow_threshold: int = 1700
    
    # Channel Calibrations
    rc_mid: int = 1500
    rc_min: int = 1000
    rc_max: int = 2000
    expo: float = 0.85
    yaw_dir: float = 1.0
    pitch_dir: float = -1.0
    roll_dir: float = 1.0
    use_roll: bool = False
    roll_blend: float = 0.25


@dataclass
class SystemConfig:
    detection: DetectionConfig = field(default_factory=DetectionConfig)
    tracker: TrackerConfig = field(default_factory=TrackerConfig)
    yaw_pid: PIDAxisConfig = field(default_factory=lambda: PIDAxisConfig(340.0, 45.0, 60.0, 400.0))
    altitude_pid: PIDAxisConfig = field(default_factory=lambda: PIDAxisConfig(310.0, 40.0, 55.0, 360.0))
    position_pid: PIDAxisConfig = field(default_factory=lambda: PIDAxisConfig(150.0, 20.0, 30.0, 200.0))
    distance: DistanceConfig = field(default_factory=DistanceConfig)
    prediction: PredictionConfig = field(default_factory=PredictionConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    offsets: OffsetsConfig = field(default_factory=OffsetsConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    aux_channels: AuxChannelsConfig = field(default_factory=AuxChannelsConfig)
    rc_control: RCControlConfig = field(default_factory=RCControlConfig)

    def to_dict(self) -> dict[str, Any]:
        d = asdict(self)

        def convert_paths(obj: Any) -> Any:
            if isinstance(obj, Path):
                return str(obj)
            if isinstance(obj, dict):
                return {k: convert_paths(v) for k, v in obj.items()}
            if isinstance(obj, list):
                return [convert_paths(v) for v in obj]
            return obj

        return convert_paths(d)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> SystemConfig:
        cfg = cls()

        def update_dataclass(target: Any, d: dict[str, Any]) -> None:
            for k, v in d.items():
                if hasattr(target, k):
                    curr = getattr(target, k)
                    if isinstance(v, dict) and hasattr(curr, "__dataclass_fields__"):
                        update_dataclass(curr, v)
                    elif isinstance(curr, Path):
                        setattr(target, k, Path(v))
                    else:
                        setattr(target, k, v)

        update_dataclass(cfg, data)
        # Older presets only had follow_speed_scale / follow_pitch_scale.
        safety_data = data.get("safety") if isinstance(data, dict) else None
        if isinstance(safety_data, dict):
            if "yaw_speed_scale" not in safety_data and "follow_speed_scale" in safety_data:
                cfg.safety.yaw_speed_scale = float(safety_data["follow_speed_scale"])
                cfg.safety.throttle_speed_scale = float(safety_data["follow_speed_scale"])
            if "pitch_speed_scale" not in safety_data and "follow_pitch_scale" in safety_data:
                cfg.safety.pitch_speed_scale = float(safety_data["follow_pitch_scale"])
            # Keep legacy mirrors in sync for any code still reading the old names.
            cfg.safety.follow_speed_scale = cfg.safety.yaw_speed_scale
            cfg.safety.follow_pitch_scale = cfg.safety.pitch_speed_scale
        return cfg

    def save_json(self, filepath: str | Path) -> None:
        p = Path(filepath)
        p.parent.mkdir(parents=True, exist_ok=True)
        with open(p, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, indent=2)

    @classmethod
    def load_json(cls, filepath: str | Path) -> SystemConfig:
        p = Path(filepath)
        if not p.exists():
            return cls()
        with open(p, "r", encoding="utf-8") as f:
            data = json.load(f)
        return cls.from_dict(data)


CONFIG = SystemConfig()
AppConfig = SystemConfig  # Backwards compatibility alias
