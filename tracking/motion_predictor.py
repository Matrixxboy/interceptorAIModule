"""Motion Predictor and Target Trajectory Lead Estimator."""

from __future__ import annotations

from dataclasses import dataclass

from config import PredictionConfig
from tracking.kalman_filter import BBoxKalmanFilter


@dataclass
class TrajectoryEstimate:
    smoothed_bbox: tuple[float, float, float, float]
    predicted_bbox: tuple[float, float, float, float]
    aim_cx: float
    aim_cy: float
    vx_px_s: float
    vy_px_s: float
    speed_px_s: float


class MotionPredictor:
    def __init__(self, cfg: PredictionConfig | None = None) -> None:
        self.cfg = cfg or PredictionConfig()
        self.kalman = BBoxKalmanFilter(
            q_var=self.cfg.process_noise_q,
            r_var=self.cfg.measurement_noise_r,
        )
        self.reset()

    def update_config(self, cfg: PredictionConfig) -> None:
        self.cfg = cfg
        self.kalman.q_var = cfg.process_noise_q
        self.kalman.r_var = cfg.measurement_noise_r

    def reset(self) -> None:
        self.kalman = BBoxKalmanFilter(
            q_var=self.cfg.process_noise_q,
            r_var=self.cfg.measurement_noise_r,
        )
        self._last_cx: float | None = None
        self._last_cy: float | None = None

    def update(
        self,
        bbox_xywh: tuple[float, float, float, float] | None,
        dt: float,
    ) -> TrajectoryEstimate:
        lead_s = max(0.0, float(self.cfg.lead_time_s))

        if bbox_xywh is not None:
            if not self.kalman.initialized:
                self.kalman.init(bbox_xywh)
            else:
                self.kalman.predict(dt)
                self.kalman.update(bbox_xywh)
        else:
            if self.kalman.initialized:
                self.kalman.predict(dt)

        if not self.kalman.initialized:
            empty = (0.0, 0.0, 0.0, 0.0)
            return TrajectoryEstimate(
                smoothed_bbox=empty,
                predicted_bbox=empty,
                aim_cx=0.0,
                aim_cy=0.0,
                vx_px_s=0.0,
                vy_px_s=0.0,
                speed_px_s=0.0,
            )

        cur_box = self.kalman.get_bbox_xywh()
        future_box = self.kalman.predict_future(lead_s)
        vx, vy = self.kalman.get_velocity()
        speed = float((vx ** 2 + vy ** 2) ** 0.5)

        aim_cx = cur_box[0] + cur_box[2] * 0.5 + vx * lead_s
        aim_cy = cur_box[1] + cur_box[3] * 0.5 + vy * lead_s

        return TrajectoryEstimate(
            smoothed_bbox=cur_box,
            predicted_bbox=future_box,
            aim_cx=aim_cx,
            aim_cy=aim_cy,
            vx_px_s=vx,
            vy_px_s=vy,
            speed_px_s=speed,
        )
