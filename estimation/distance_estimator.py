"""Vision-Based Distance Estimator and Following Distance Controller."""

from __future__ import annotations

from dataclasses import dataclass

from config import DistanceConfig
from estimation.distance_calib import bbox_size_px, estimate_distance_m


@dataclass
class DistanceEstimate:
    distance_m: float
    distance_error_m: float
    status: str  # "TOO_CLOSE" | "NOMINAL" | "TOO_FAR" | "SAFE"
    recommended_pitch_offset: float
    is_safe: bool


class DistanceEstimator:
    def __init__(self, cfg: DistanceConfig | None = None) -> None:
        self.cfg = cfg or DistanceConfig()
        self._integral_err = 0.0
        self._prev_err = 0.0

    def update_config(self, cfg: DistanceConfig) -> None:
        self.cfg = cfg

    def reset(self) -> None:
        self._integral_err = 0.0
        self._prev_err = 0.0

    def estimate_distance(self, bbox_width_px: float) -> float:
        """Pinhole distance from a single pixel size along the calibrated axis."""
        return estimate_distance_m(
            bbox_width_px,
            self.cfg.focal_length_px,
            self.cfg.known_object_width_m,
        )

    def estimate_distance_from_bbox(self, bbox_xywh: tuple[float, float, float, float]) -> float:
        size_px = bbox_size_px(bbox_xywh, getattr(self.cfg, "size_axis", "width") or "width")
        return self.estimate_distance(size_px)

    def compute_following_control(
        self,
        bbox_width_px: float,
        dt: float = 0.033,
    ) -> DistanceEstimate:
        c = self.cfg
        dist_m = self.estimate_distance(bbox_width_px)
        dt = max(1e-4, float(dt))

        # Error: Positive means target is further than desired distance -> drone should move forward
        dist_err = dist_m - c.desired_distance_m

        if dist_m < c.min_safe_distance_m:
            status = "TOO_CLOSE"
            is_safe = False
        elif dist_m > c.max_follow_distance_m:
            status = "TOO_FAR"
            is_safe = True
        else:
            status = "NOMINAL"
            is_safe = True

        self._integral_err += dist_err * dt
        self._integral_err = max(-2.0, min(2.0, self._integral_err))

        d_err = (dist_err - self._prev_err) / dt
        self._prev_err = dist_err

        pitch_offset = (c.kp * dist_err) + (c.ki * self._integral_err) + (c.kd * d_err)

        if dist_m < c.min_safe_distance_m:
            overclose_ratio = (c.min_safe_distance_m - dist_m) / max(0.1, c.min_safe_distance_m)
            pitch_offset = -abs(c.max_pitch_offset) * (1.0 + overclose_ratio)

        pitch_offset = max(-c.max_pitch_offset, min(c.max_pitch_offset, pitch_offset))

        return DistanceEstimate(
            distance_m=dist_m,
            distance_error_m=dist_err,
            status=status,
            recommended_pitch_offset=pitch_offset,
            is_safe=is_safe,
        )
