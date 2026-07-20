"""Vision-Based Distance Estimator and Following Distance Controller."""

from __future__ import annotations

from dataclasses import dataclass

from config import DistanceConfig


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
        w_px = max(1.0, float(bbox_width_px))
        f = max(100.0, float(self.cfg.focal_length_px))
        w_m = max(0.01, float(self.cfg.known_object_width_m))
        return (f * w_m) / w_px

    def compute_following_control(
        self,
        bbox_width_px: float,
        dt: float = 0.033,
    ) -> DistanceEstimate:
        c = self.cfg
        dist_m = self.estimate_distance(bbox_width_px)
        dt = max(1e-4, float(dt))

        # Error: Positive means target is further than desired distance -> drone should move forward (pitch down/forward)
        dist_err = dist_m - c.desired_distance_m

        # Safety status check
        if dist_m < c.min_safe_distance_m:
            status = "TOO_CLOSE"
            is_safe = False
        elif dist_m > c.max_follow_distance_m:
            status = "TOO_FAR"
            is_safe = True
        else:
            status = "NOMINAL"
            is_safe = True

        # PID distance loop for forward/backward pitch offset
        self._integral_err += dist_err * dt
        self._integral_err = max(-2.0, min(2.0, self._integral_err))

        d_err = (dist_err - self._prev_err) / dt
        self._prev_err = dist_err

        pitch_offset = (c.kp * dist_err) + (c.ki * self._integral_err) + (c.kd * d_err)

        # Safety override: if too close, force backward pitch offset
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
