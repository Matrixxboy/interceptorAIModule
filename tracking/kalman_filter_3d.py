"""3D Constant-Acceleration Kalman Filter for Target Tracking in World Frame."""

from __future__ import annotations

import numpy as np


class KalmanFilter3D:
    """
    State vector: [x, y, z, vx, vy, vz, ax, ay, az]^T
    Measurement vector: [x, y, z]^T
    """

    def __init__(
        self,
        q_pos: float = 1e-2,
        q_vel: float = 1e-1,
        q_acc: float = 1.0,
        r_pos: float = 1.0,
    ) -> None:
        self.q_pos = q_pos
        self.q_vel = q_vel
        self.q_acc = q_acc
        self.r_pos = r_pos
        self.initialized = False
        self.last_innovation_norm: float = 0.0
        self.last_accepted: bool = False

        # State vector [9x1]
        self.x = np.zeros((9, 1), dtype=np.float64)

        # State covariance matrix [9x9]
        self.P = np.eye(9, dtype=np.float64) * 10.0

        # Measurement matrix [3x9]
        self.H = np.zeros((3, 9), dtype=np.float64)
        for i in range(3):
            self.H[i, i] = 1.0

        self._build_matrices(dt=0.033)

    def _build_matrices(self, dt: float) -> None:
        dt = max(1e-4, float(dt))
        # Transition matrix F [9x9]
        self.F = np.eye(9, dtype=np.float64)
        for i in range(3):
            self.F[i, i + 3] = dt
            self.F[i, i + 6] = 0.5 * (dt ** 2)
            self.F[i + 3, i + 6] = dt

        # Process noise covariance Q [9x9]
        self.Q = np.eye(9, dtype=np.float64)
        for i in range(3):
            self.Q[i, i] = self.q_pos
            self.Q[i + 3, i + 3] = self.q_vel
            self.Q[i + 6, i + 6] = self.q_acc

        # Measurement noise covariance R [3x3] — base; update() may inflate depth axis
        self.R = np.eye(3, dtype=np.float64) * self.r_pos

    def init(self, pos_3d: tuple[float, float, float]) -> None:
        self.x = np.zeros((9, 1), dtype=np.float64)
        self.x[0, 0] = pos_3d[0]
        self.x[1, 0] = pos_3d[1]
        self.x[2, 0] = pos_3d[2]
        self.P = np.eye(9, dtype=np.float64) * 10.0
        self.initialized = True
        self.last_innovation_norm = 0.0
        self.last_accepted = True

    def predict(self, dt: float = 0.033) -> tuple[float, float, float]:
        if not self.initialized:
            return 0.0, 0.0, 0.0

        self._build_matrices(dt)

        # State prediction: x' = F * x
        self.x = self.F @ self.x

        # Covariance prediction: P' = F * P * F^T + Q
        self.P = self.F @ self.P @ self.F.T + self.Q

        return self.get_position()

    def update(
        self,
        pos_3d: tuple[float, float, float],
        *,
        depth_sigma_m: float | None = None,
        gate_m: float | None = None,
    ) -> tuple[float, float, float]:
        """
        Measurement update. Prefer calling predict(dt) first each frame.

        depth_sigma_m: extra 1σ noise on the range/depth axis (index 0 ≈ forward).
        gate_m: if set, reject updates whose innovation norm exceeds this (keep predict).
        """
        if not self.initialized:
            self.init(pos_3d)
            return pos_3d

        # Inflate R with depth uncertainty (monocular range is the noisy axis)
        R = self.R.copy()
        if depth_sigma_m is not None and depth_sigma_m > 0.0:
            # Apply extra variance primarily along x (LOS / forward in camera-world)
            extra = float(depth_sigma_m) ** 2
            R[0, 0] = max(R[0, 0], self.r_pos + extra)
            R[1, 1] = max(R[1, 1], self.r_pos + 0.25 * extra)
            R[2, 2] = max(R[2, 2], self.r_pos + 0.25 * extra)

        z = np.array([[pos_3d[0]], [pos_3d[1]], [pos_3d[2]]], dtype=np.float64)
        y_innov = z - self.H @ self.x
        innov_norm = float(np.linalg.norm(y_innov))
        self.last_innovation_norm = innov_norm

        if gate_m is not None and gate_m > 0.0 and innov_norm > float(gate_m):
            # Reject outlier — keep predicted state
            self.last_accepted = False
            return self.get_position()

        S = self.H @ self.P @ self.H.T + R
        K = np.linalg.solve(S.T, (self.P @ self.H.T).T).T

        self.x = self.x + K @ y_innov
        self.P = self.P - K @ self.H @ self.P
        self.last_accepted = True

        return self.get_position()

    def get_position(self) -> tuple[float, float, float]:
        return float(self.x[0, 0]), float(self.x[1, 0]), float(self.x[2, 0])

    def get_velocity(self) -> tuple[float, float, float]:
        if not self.initialized:
            return 0.0, 0.0, 0.0
        return float(self.x[3, 0]), float(self.x[4, 0]), float(self.x[5, 0])

    def get_acceleration(self) -> tuple[float, float, float]:
        if not self.initialized:
            return 0.0, 0.0, 0.0
        return float(self.x[6, 0]), float(self.x[7, 0]), float(self.x[8, 0])

    def reset(self) -> None:
        self.initialized = False
        self.x[:] = 0.0
        self.P = np.eye(9, dtype=np.float64) * 10.0
        self.last_innovation_norm = 0.0
        self.last_accepted = False
