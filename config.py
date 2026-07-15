"""Central configuration for FPV MSP visual lock + follow."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal, Sequence

ROOT = Path(__file__).resolve().parent
MODELS_DIR = ROOT / "models"


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
    "helicopter",
    "person",
)

DetectionMode = Literal["world", "coco", "custom"]
TrackerBackend = Literal["bytetrack", "botsort", "iou", "csrt", "kcf"]


@dataclass(frozen=True)
class DetectionConfig:
    # coco + yolov8n.pt is the reliable default; use world/custom when ready
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
    # COCO: person=0, airplane=4, bird=14, sports ball=32, kite=33
    class_filter: Sequence[int] = (0, 4, 14, 32, 33)
    min_box_area_frac: float = 0.00005
    max_box_area_frac: float = 0.35
    device: str = "auto"  # "auto" | "cuda" | "cpu"
    half: bool = True
    detect_every_n: int = 3
    augment: bool = False


@dataclass(frozen=True)
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


@dataclass(frozen=True)
class AppConfig:
    detection: DetectionConfig = field(default_factory=DetectionConfig)
    tracker: TrackerConfig = field(default_factory=TrackerConfig)


CONFIG = AppConfig()
