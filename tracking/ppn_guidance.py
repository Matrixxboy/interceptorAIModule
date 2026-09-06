"""Pure Proportional Navigation (PPN) Guidance Law in 3D.

Implements the guidance law from the thesis (Equations 28-32):
  a_P  = N_p * (v_I × ω)                   -- Eq 28
  ω    = (r × v_rel) / |r|²                -- Eq 29
  v_P  = |v_P0| * (v_P_prev + dt*a_P) / |…| -- Eq 30 (constant-speed integration)
  v_P0 = b * (r̂) + v_t                     -- Eq 31
  ψ_ref = atan2(v_Py, v_Px)                -- Eq 32
"""

import numpy as np


class PPNGuidance:
    def __init__(self, N_p: float = 3.0, b: float = 10.0, C_b: float = 1.0):
        """
        Args:
            N_p: Navigation constant (typically 3–5). Higher = more aggressive turning.
            b:   Closing speed magnitude in m/s towards the target.
            C_b: Altitude scaling factor for the closing velocity (≥1 dampens vertical).
        """
        self.N_p = N_p
        self.b = b
        self.C_b = C_b
        self.v_P_prev: np.ndarray | None = None

    def reset(self) -> None:
        self.v_P_prev = None

    def compute_guidance(
        self,
        p_w_t: tuple[float, float, float],
        v_w_t: tuple[float, float, float],
        v_w_I: tuple[float, float, float],
        dt: float,
    ) -> tuple[np.ndarray, np.ndarray, float]:
        """
        Computes PPN acceleration, velocity reference, and yaw reference.

        Args:
            p_w_t: Target position relative to interceptor (world frame, meters).
            v_w_t: Target velocity (world frame, m/s).
            v_w_I: Current interceptor velocity (world frame, m/s).
            dt:    Time step (seconds).

        Returns:
            a_P:      Commanded PPN acceleration vector (3D, m/s²).
            v_P:      Commanded velocity reference vector (3D, m/s).
            psi_ref:  Commanded heading angle (radians).
        """
        r = np.array(p_w_t, dtype=np.float64)
        v_t = np.array(v_w_t, dtype=np.float64)
        v_I = np.array(v_w_I, dtype=np.float64)

        r_norm_sq = float(np.dot(r, r))
        if r_norm_sq < 1e-6:
            return np.zeros(3), np.zeros(3), 0.0

        r_norm = np.sqrt(r_norm_sq)

        # Relative velocity (target w.r.t. interceptor)
        v_rel = v_t - v_I

        # Eq 29: Line-of-sight angular rate
        omega = np.cross(r, v_rel) / r_norm_sq

        # Eq 28: PPN acceleration command
        a_P = self.N_p * np.cross(v_I, omega)

        # Eq 31: Initial velocity reference (closing speed towards target + target drift)
        r_hat = r / r_norm
        b_vec = np.array([self.b, self.b, self.b / max(0.1, self.C_b)])
        v_P0 = b_vec * r_hat + v_t

        # Eq 30: Integrate acceleration, but keep speed magnitude constant
        V_P_mag = float(np.linalg.norm(v_P0))
        if V_P_mag < 1e-6:
            V_P_mag = self.b  # fallback

        if self.v_P_prev is None:
            self.v_P_prev = v_P0.copy()

        v_P_next = self.v_P_prev + dt * a_P
        v_P_next_norm = float(np.linalg.norm(v_P_next))

        if v_P_next_norm > 1e-6:
            v_P = V_P_mag * (v_P_next / v_P_next_norm)
        else:
            v_P = v_P0

        self.v_P_prev = v_P.copy()

        # Eq 32: Yaw reference (heading towards the target)
        v_t_2d = np.array(v_w_t[:2])
        if np.linalg.norm(v_t_2d) > 0.01:
            psi_ref = float(np.arctan2(v_P[1], v_P[0]))
        else:
            psi_ref = float(np.arctan2(r[1], r[0]))

        return a_P, v_P, psi_ref
