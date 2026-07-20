"""Multi-axis PID Controller with Exponential Shaping, Deadzones, Anti-Windup, and Low-Pass Filtered Derivative."""

from __future__ import annotations

from dataclasses import dataclass

from config import PIDAxisConfig


def clamp(val: float, lo: float, hi: float) -> float:
    return lo if val < lo else hi if val > hi else val


@dataclass
class PIDResult:
    output: float
    p_term: float
    i_term: float
    d_term: float


class PIDController:
    def __init__(self, cfg: PIDAxisConfig | None = None) -> None:
        self.cfg = cfg or PIDAxisConfig()
        self.reset()

    def update_config(self, cfg: PIDAxisConfig) -> None:
        self.cfg = cfg

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = 0.0
        self.d_filter_val = 0.0
        self.has_prev = False

    def shape_error(self, error: float, deadzone: float = 0.02, bleed: float = 0.15, expo: float = 0.85) -> float:
        abs_e = abs(error)
        if abs_e < deadzone:
            return error * bleed
        sign = 1.0 if error >= 0.0 else -1.0
        remapped = (abs_e - deadzone) / max(1e-6, 1.0 - deadzone)
        remapped = clamp(remapped, 0.0, 1.0)
        return sign * (remapped ** expo)

    def update(
        self,
        error: float,
        dt: float,
        deadzone: float = 0.02,
        expo: float = 0.85,
    ) -> PIDResult:
        c = self.cfg
        dt = max(1e-4, float(dt))

        shaped = self.shape_error(error, deadzone=deadzone, expo=expo)

        # Anti-windup integral logic
        if abs(error) < deadzone:
            self.integral *= 0.92
        else:
            self.integral = clamp(self.integral + shaped * dt, -c.i_limit, c.i_limit)

        # Low-pass derivative
        if not self.has_prev:
            self.prev_error = shaped
            self.has_prev = True
            raw_d = 0.0
        else:
            raw_d = (shaped - self.prev_error) / dt

        self.d_filter_val = c.d_filter * raw_d + (1.0 - c.d_filter) * self.d_filter_val
        self.prev_error = shaped

        p_term = c.kp * shaped
        i_term = c.ki * self.integral
        d_term = c.kd * self.d_filter_val

        out = p_term + i_term + d_term
        out_clamped = clamp(out, -c.max_output, c.max_output)

        return PIDResult(
            output=out_clamped,
            p_term=p_term,
            i_term=i_term,
            d_term=d_term,
        )
