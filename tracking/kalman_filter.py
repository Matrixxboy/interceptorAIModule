"""8D Constant-Velocity Kalman Filter for Bounding Box Tracking & Motion Prediction."""

from __future__ import annotations

import numpy as np


class BBoxKalmanFilter:
    """
    State vector: [cx, cy, w, h, vx, vy, vw, vh]^T
    Measurement vector: [cx, cy, w, h]^T
    """

    def __init__(
        self,
        q_var: float = 1e-2,
        r_var: float = 1e-1,
    ) -> None:
        self.q_var = q_var
        self.r_var = r_var
        self.initialized = False

        # State vector [8x1]
        self.x = np.zeros((8, 1), dtype=np.float64)

        # State covariance matrix [8x8]
        self.P = np.eye(8, dtype=np.float64) * 10.0

        # Measurement matrix [4x8]
        self.H = np.zeros((4, 8), dtype=np.float64)
        for i in range(4):
            self.H[i, i] = 1.0

        self._build_matrices(dt=0.033)

    def _build_matrices(self, dt: float) -> None:
        dt = max(1e-4, float(dt))
        # Transition matrix F [8x8]
        self.F = np.eye(8, dtype=np.float64)
        for i in range(4):
            self.F[i, i + 4] = dt

        # Process noise covariance Q [8x8]
        q_pos = self.q_var * (dt ** 2) / 2.0
        q_vel = self.q_var * dt
        self.Q = np.eye(8, dtype=np.float64)
        for i in range(4):
            self.Q[i, i] = q_pos
            self.Q[i + 4, i + 4] = q_vel

        # Measurement noise covariance R [4x4]
        self.R = np.eye(4, dtype=np.float64) * self.r_var

    def init(self, bbox_xywh: tuple[float, float, float, float]) -> None:
        x, y, w, h = bbox_xywh
        cx = x + w * 0.5
        cy = y + h * 0.5
        self.x = np.array([[cx], [cy], [w], [h], [0.0], [0.0], [0.0], [0.0]], dtype=np.float64)
        self.P = np.eye(8, dtype=np.float64) * 10.0
        self.initialized = True

    def predict(self, dt: float = 0.033) -> tuple[float, float, float, float]:
        if not self.initialized:
            return 0.0, 0.0, 0.0, 0.0

        self._build_matrices(dt)

        # State prediction: x' = F * x
        self.x = self.F @ self.x

        # Covariance prediction: P' = F * P * F^T + Q
        self.P = self.F @ self.P @ self.F.T + self.Q

        return self.get_bbox_xywh()

    def update(self, bbox_xywh: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
        x, y, w, h = bbox_xywh
        cx = x + w * 0.5
        cy = y + h * 0.5

        if not self.initialized:
            self.init(bbox_xywh)
            return bbox_xywh

        z = np.array([[cx], [cy], [w], [h]], dtype=np.float64)
        y_innov = z - self.x[:4]
        S = self.P[:4, :4] + self.R
        K = np.linalg.solve(S, self.P[:4, :]).T

        self.x = self.x + K @ y_innov
        self.P = self.P - K @ self.P[:4, :]

        return self.get_bbox_xywh()

    def get_bbox_xywh(self) -> tuple[float, float, float, float]:
        cx, cy, w, h = self.x[0, 0], self.x[1, 0], self.x[2, 0], self.x[3, 0]
        w = max(1.0, w)
        h = max(1.0, h)
        x = cx - w * 0.5
        y = cy - h * 0.5
        return float(x), float(y), float(w), float(h)

    def predict_future(self, lead_s: float) -> tuple[float, float, float, float]:
        if not self.initialized:
            return 0.0, 0.0, 0.0, 0.0
        cx = self.x[0, 0] + self.x[4, 0] * lead_s
        cy = self.x[1, 0] + self.x[5, 0] * lead_s
        w = max(1.0, self.x[2, 0] + self.x[6, 0] * lead_s)
        h = max(1.0, self.x[3, 0] + self.x[7, 0] * lead_s)
        x = cx - w * 0.5
        y = cy - h * 0.5
        return float(x), float(y), float(w), float(h)

    def get_velocity(self) -> tuple[float, float]:
        if not self.initialized:
            return 0.0, 0.0
        return float(self.x[4, 0]), float(self.x[5, 0])
