"""
FPV visual-servo follow controller.

For a fixed forward camera on an FPV / interceptor drone:
  - Horizontal error  → YAW  (turn to face target)
  - Vertical error    → PITCH (nose toward target)
  - Roll is optional and light (strafe) — default off for pure aim

Uses normalized image error, lead prediction, filtered D-term,
progressive authority, and slew limiting so sticks track cleanly.
"""

from __future__ import annotations

import time
from dataclasses import dataclass


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


@dataclass
class FPVFollowConfig:
    # Soft deadzone in normalized coords (|error| / half-frame)
    deadzone_norm: float = 0.02
    # Keep a tiny residual inside deadzone so sticks don't "stick" off-center
    deadzone_bleed: float = 0.15

    # PID on normalized error [-1..1] → stick offset (µs)
    yaw_kp: float = 320.0
    yaw_ki: float = 40.0
    yaw_kd: float = 55.0

    pitch_kp: float = 300.0
    pitch_ki: float = 35.0
    pitch_kd: float = 50.0

    roll_kp: float = 80.0
    roll_ki: float = 8.0
    roll_kd: float = 15.0

    # Max stick offsets from 1500
    max_yaw: float = 380.0
    max_pitch: float = 350.0
    max_roll: float = 120.0

    i_limit: float = 0.45  # on normalized integral
    d_filter: float = 0.35  # EMA on derivative (higher = less lag)

    # Progressive gain: |e|^expo — <1 soft near center, >1 aggressive far
    expo: float = 0.85

    # Lead time (seconds) using image-plane velocity
    lead_s: float = 0.12

    # Measurement EMA (center px) — reduces tracker jitter before PID
    meas_alpha: float = 0.45

    # Stick output EMA + slew (µs/s)
    out_alpha: float = 0.55
    slew_yaw: float = 1400.0
    slew_pitch: float = 1200.0
    slew_roll: float = 600.0

    # Axis directions (flip if your FC/camera mapping is reversed)
    yaw_dir: float = 1.0
    pitch_dir: float = -1.0  # image y down → pitch up when target above
    roll_dir: float = 1.0

    use_roll: bool = False  # FPV aim: yaw+pitch only by default
    roll_blend: float = 0.25  # if use_roll, fraction of horizontal into roll

    rc_mid: int = 1500
    rc_min: int = 1000
    rc_max: int = 2000


class FPVFollowController:
    def __init__(self, cfg: FPVFollowConfig | None = None) -> None:
        self.cfg = cfg or FPVFollowConfig()
        self.reset()

    def reset(self) -> None:
        c = self.cfg
        self._ix = 0.0
        self._iy = 0.0
        self._prev_nx = 0.0
        self._prev_ny = 0.0
        self._dn_x = 0.0
        self._dn_y = 0.0
        self._has_prev = False
        self._smoothed_cx: float | None = None
        self._smoothed_cy: float | None = None
        self._vx = 0.0
        self._vy = 0.0
        self._out_yaw = 0.0
        self._out_pitch = 0.0
        self._out_roll = 0.0
        self._cmd_roll = float(c.rc_mid)
        self._cmd_pitch = float(c.rc_mid)
        self._cmd_yaw = float(c.rc_mid)
        self._t: float | None = None

    def _shape(self, n: float) -> float:
        """Soft deadzone + expo curve on normalized error."""
        c = self.cfg
        a = abs(n)
        if a < c.deadzone_norm:
            return n * c.deadzone_bleed
        # Remap outside deadzone to full range, then expo
        sign = 1.0 if n >= 0 else -1.0
        remapped = (a - c.deadzone_norm) / max(1e-6, 1.0 - c.deadzone_norm)
        remapped = _clamp(remapped, 0.0, 1.0)
        return sign * (remapped ** c.expo)

    def _axis_pid(
        self,
        n: float,
        integral: float,
        prev_n: float,
        dn_filt: float,
        kp: float,
        ki: float,
        kd: float,
        dt: float,
        direction: float,
        max_out: float,
    ) -> tuple[float, float, float]:
        c = self.cfg
        shaped = self._shape(n)

        # Integrate shaped error; bleed when near center
        if abs(n) < c.deadzone_norm:
            integral *= 0.92
        else:
            integral = _clamp(integral + shaped * dt, -c.i_limit, c.i_limit)

        raw_d = (n - prev_n) / dt if dt > 1e-4 else 0.0
        dn = c.d_filter * raw_d + (1.0 - c.d_filter) * dn_filt

        out = direction * (kp * shaped + ki * integral + kd * dn)
        out = _clamp(out, -max_out, max_out)
        return out, integral, dn

    def update(
        self,
        obj_cx: float,
        obj_cy: float,
        frame_w: int,
        frame_h: int,
    ) -> tuple[int, int, int]:
        """
        Returns (roll, pitch, yaw) RC µs.
        obj_cx/cy = tracked target center in pixels.
        """
        c = self.cfg
        now = time.perf_counter()
        if self._t is None:
            dt = 1.0 / 50.0
        else:
            dt = _clamp(now - self._t, 0.001, 0.08)
        self._t = now

        # Smooth measurement + estimate image velocity (px/s)
        if self._smoothed_cx is None:
            self._smoothed_cx = float(obj_cx)
            self._smoothed_cy = float(obj_cy)
            self._vx = 0.0
            self._vy = 0.0
        else:
            prev_x, prev_y = self._smoothed_cx, self._smoothed_cy
            a = c.meas_alpha
            self._smoothed_cx = a * float(obj_cx) + (1.0 - a) * prev_x
            self._smoothed_cy = a * float(obj_cy) + (1.0 - a) * prev_y
            self._vx = (self._smoothed_cx - prev_x) / dt
            self._vy = (self._smoothed_cy - prev_y) / dt

        half_w = max(1.0, frame_w * 0.5)
        half_h = max(1.0, frame_h * 0.5)
        frame_cx = frame_w * 0.5
        frame_cy = frame_h * 0.5

        # Lead aim point in pixels
        aim_x = self._smoothed_cx + self._vx * c.lead_s
        aim_y = self._smoothed_cy + self._vy * c.lead_s

        nx = _clamp((aim_x - frame_cx) / half_w, -1.0, 1.0)
        ny = _clamp((aim_y - frame_cy) / half_h, -1.0, 1.0)

        if not self._has_prev:
            self._prev_nx, self._prev_ny = nx, ny
            self._has_prev = True

        yaw_off, self._ix, self._dn_x = self._axis_pid(
            nx, self._ix, self._prev_nx, self._dn_x,
            c.yaw_kp, c.yaw_ki, c.yaw_kd, dt, c.yaw_dir, c.max_yaw,
        )
        pitch_off, self._iy, self._dn_y = self._axis_pid(
            ny, self._iy, self._prev_ny, self._dn_y,
            c.pitch_kp, c.pitch_ki, c.pitch_kd, dt, c.pitch_dir, c.max_pitch,
        )

        if c.use_roll:
            roll_off, _, _ = self._axis_pid(
                nx, 0.0, self._prev_nx, 0.0,
                c.roll_kp * c.roll_blend, 0.0, c.roll_kd * c.roll_blend,
                dt, c.roll_dir, c.max_roll,
            )
        else:
            roll_off = 0.0

        self._prev_nx, self._prev_ny = nx, ny

        # Output EMA
        oa = c.out_alpha
        self._out_yaw = oa * yaw_off + (1.0 - oa) * self._out_yaw
        self._out_pitch = oa * pitch_off + (1.0 - oa) * self._out_pitch
        self._out_roll = oa * roll_off + (1.0 - oa) * self._out_roll

        target_yaw = c.rc_mid + self._out_yaw
        target_pitch = c.rc_mid + self._out_pitch
        target_roll = c.rc_mid + self._out_roll

        def slew(cur: float, tgt: float, rate: float) -> float:
            step = rate * dt
            return cur + _clamp(tgt - cur, -step, step)

        self._cmd_yaw = slew(self._cmd_yaw, target_yaw, c.slew_yaw)
        self._cmd_pitch = slew(self._cmd_pitch, target_pitch, c.slew_pitch)
        self._cmd_roll = slew(self._cmd_roll, target_roll, c.slew_roll)

        roll = int(_clamp(self._cmd_roll, c.rc_min, c.rc_max))
        pitch = int(_clamp(self._cmd_pitch, c.rc_min, c.rc_max))
        yaw = int(_clamp(self._cmd_yaw, c.rc_min, c.rc_max))
        return roll, pitch, yaw

    def fade_to_mid(self, factor: float = 0.88) -> tuple[int, int, int]:
        """Gently return sticks to mid (track loss / disable)."""
        c = self.cfg
        self._cmd_roll = c.rc_mid + (self._cmd_roll - c.rc_mid) * factor
        self._cmd_pitch = c.rc_mid + (self._cmd_pitch - c.rc_mid) * factor
        self._cmd_yaw = c.rc_mid + (self._cmd_yaw - c.rc_mid) * factor
        self._out_roll *= factor
        self._out_pitch *= factor
        self._out_yaw *= factor
        self._ix *= factor
        self._iy *= factor
        return (
            int(_clamp(self._cmd_roll, c.rc_min, c.rc_max)),
            int(_clamp(self._cmd_pitch, c.rc_min, c.rc_max)),
            int(_clamp(self._cmd_yaw, c.rc_min, c.rc_max)),
        )
