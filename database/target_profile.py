"""Dynamic in-memory target profile and timeline models."""

from __future__ import annotations

import time
import uuid
from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any


class TargetStatus(str, Enum):
    DETECTED = "detected"
    LOCKED = "locked"
    TRACKING = "tracking"
    LOST = "lost"
    REACQUIRED = "reacquired"
    FINISHED = "finished"
    ARCHIVED = "archived"


@dataclass
class TimelineEvent:
    timestamp: float
    event: str
    confidence: float = 0.0
    camera_position: str = ""
    drone_state: str = ""
    system_response: str = ""
    metadata: dict[str, Any] = field(default_factory=dict)

    @property
    def time_str(self) -> str:
        from datetime import datetime

        return datetime.fromtimestamp(self.timestamp).strftime("%H:%M:%S.%f")[:-3]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class TargetProfile:
    """Adaptive profile for a tracked target."""

    target_id: str = field(default_factory=lambda: uuid.uuid4().hex[:12].upper())
    label: str = "unknown"
    status: TargetStatus = TargetStatus.DETECTED

    detection_time: float = field(default_factory=time.time)
    lock_time: float | None = None
    finish_time: float | None = None

    confidence: float = 0.0
    confidence_history: list[float] = field(default_factory=list)
    bbox_history: list[tuple[float, float, float, float]] = field(default_factory=list)
    velocity_history: list[tuple[float, float]] = field(default_factory=list)
    motion_vectors: list[tuple[float, float]] = field(default_factory=list)

    last_bbox: tuple[float, float, float, float] | None = None
    last_position: tuple[float, float] | None = None
    velocity: tuple[float, float] = (0.0, 0.0)
    direction_deg: float = 0.0
    distance_m: float = 0.0
    object_width_px: float = 0.0
    object_height_px: float = 0.0

    color_signature: list[float] = field(default_factory=list)
    feature_embedding: list[float] = field(default_factory=list)
    tracking_source: str = "none"
    frames_processed: int = 0

    timeline: list[TimelineEvent] = field(default_factory=list)
    image_paths: dict[str, str] = field(default_factory=dict)
    drone_commands: list[dict[str, Any]] = field(default_factory=list)

    def add_event(
        self,
        event: str,
        confidence: float = 0.0,
        camera_position: str = "",
        drone_state: str = "",
        system_response: str = "",
        **metadata: Any,
    ) -> None:
        self.timeline.append(
            TimelineEvent(
                timestamp=time.time(),
                event=event,
                confidence=confidence,
                camera_position=camera_position,
                drone_state=drone_state,
                system_response=system_response,
                metadata=metadata,
            )
        )

    def update_frame(
        self,
        bbox_xywh: tuple[float, float, float, float],
        confidence: float,
        source: str,
        vx: float = 0.0,
        vy: float = 0.0,
        distance_m: float = 0.0,
        color_hist: list[float] | None = None,
    ) -> None:
        self.frames_processed += 1
        self.confidence = confidence
        self.tracking_source = source
        self.last_bbox = bbox_xywh
        self.distance_m = distance_m

        x, y, w, h = bbox_xywh
        self.last_position = (x + w * 0.5, y + h * 0.5)
        self.object_width_px = w
        self.object_height_px = h
        self.velocity = (vx, vy)

        import math

        if abs(vx) > 0.1 or abs(vy) > 0.1:
            self.direction_deg = math.degrees(math.atan2(vy, vx))

        self.confidence_history.append(confidence)
        self.bbox_history.append(bbox_xywh)
        self.velocity_history.append((vx, vy))
        self.motion_vectors.append((vx, vy))

        if color_hist:
            self.color_signature = list(color_hist)

        if len(self.confidence_history) > 2000:
            self.confidence_history = self.confidence_history[-1000:]
            self.bbox_history = self.bbox_history[-1000:]
            self.velocity_history = self.velocity_history[-1000:]
            self.motion_vectors = self.motion_vectors[-1000:]

    @property
    def tracking_duration_s(self) -> float:
        end = self.finish_time or time.time()
        start = self.lock_time or self.detection_time
        return max(0.0, end - start)

    def to_dict(self) -> dict[str, Any]:
        d = asdict(self)
        d["status"] = self.status.value
        d["timeline"] = [e.to_dict() for e in self.timeline]
        return d

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> TargetProfile:
        data = dict(data)
        data["status"] = TargetStatus(data.get("status", "detected"))
        timeline_raw = data.pop("timeline", [])
        profile = cls(**{k: v for k, v in data.items() if k in cls.__dataclass_fields__})
        profile.timeline = [TimelineEvent(**e) for e in timeline_raw]
        return profile
