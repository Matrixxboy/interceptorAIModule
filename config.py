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
    kp: float = 120.0
    ki: float = 15.0
    kd: float = 25.0
    max_pitch_offset: float = 200.0  # Pitch stick change for distance correction


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
    max_lost_frames: int = 45
    reacquisition_timeout_s: float = 2.5
    max_yaw_rate: float = 1600.0  # µs/s slew limit
    max_climb_rate: float = 1400.0  # µs/s slew limit
    max_descent_rate: float = 1200.0  # µs/s slew limit
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


@dataclass
class DetectionConfig:
    mode: DetectionMode = "coco"
    model_name: str = "yolov8n.pt"
    model_path: Path = field(default_factory=lambda: MODELS_DIR / "yolov8n.pt")
    custom_weights: Path = field(
        default_factory=lambda: MODELS_DIR / "drone_missile_best.pt"
    )
    imgsz: int = 640
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
    lock_tracker: Literal["csrt", "kcf", "none"] = "csrt"
    reacquire_iou: float = 0.12
    reacquire_max_frames: int = 90
    enable_template_fallback: bool = True
    template_match_threshold: float = 0.45


@dataclass
class DeviceConfig:
    theme: str = "dark"
    ui_scale: float = 1.0


@dataclass
class AuxChannelsConfig:
    arm_channel: int = 4
    mode_channel: int = 5
    arm_high: int = 1800
    arm_low: int = 1000
    mode_high: int = 1900
    mode_low: int = 1000


@dataclass
class JoystickChannelConfig:
    name: str = ""
    axis: int = -1
    is_button: bool = False
    inverted: bool = False
    min_val: int = 1000
    center_val: int = 1500
    max_val: int = 2000
    # MSP / INAV RC channel index (0-based). CH5=4 (AUX1), CH6=5 (AUX2), …
    rc_channel: int = -1


@dataclass
class JoystickConfig:
    enabled: bool = False
    device_name: str = ""
    deadzone: float = 0.05
    roll: JoystickChannelConfig = field(default_factory=lambda: JoystickChannelConfig(name="Roll", axis=0))
    pitch: JoystickChannelConfig = field(default_factory=lambda: JoystickChannelConfig(name="Pitch", axis=1, inverted=False))
    throttle: JoystickChannelConfig = field(default_factory=lambda: JoystickChannelConfig(name="Throttle", axis=2))
    yaw: JoystickChannelConfig = field(default_factory=lambda: JoystickChannelConfig(name="Yaw", axis=3))

    aux_channels: list[JoystickChannelConfig] = field(default_factory=lambda: [
        JoystickChannelConfig(
            name="Arm", axis=0, is_button=True, rc_channel=4,
            min_val=1000, center_val=1000, max_val=1800,
        ),
        JoystickChannelConfig(
            name="Flight Mode", axis=1, is_button=True, rc_channel=5,
            min_val=1000, center_val=1000, max_val=1900,
        ),
    ])


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
    device: DeviceConfig = field(default_factory=DeviceConfig)
    aux_channels: AuxChannelsConfig = field(default_factory=AuxChannelsConfig)
    joystick: JoystickConfig = field(default_factory=JoystickConfig)

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
                    elif isinstance(v, list) and k == "aux_channels":
                        parsed_list = []
                        for item in v:
                            if isinstance(item, dict):
                                ch = JoystickChannelConfig()
                                update_dataclass(ch, item)
                                parsed_list.append(ch)
                            else:
                                parsed_list.append(item)
                        setattr(target, k, parsed_list)
                    elif isinstance(curr, Path):
                        setattr(target, k, Path(v))
                    else:
                        setattr(target, k, v)

        update_dataclass(cfg, data)
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
