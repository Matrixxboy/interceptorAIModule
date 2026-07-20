"""FPV visual-servo follow controller supporting 4-axis control (Yaw, Altitude/Pitch, Distance, Roll)."""

from __future__ import annotations

import time
from dataclasses import dataclass

from config import SystemConfig
from control.pid_controller import PIDController
from estimation.distance_estimator import DistanceEstimate, DistanceEstimator
from tracking.motion_predictor import MotionPredictor, TrajectoryEstimate


def clamp(val: float, lo: float, hi: float) -> float:
    return lo if val < lo else hi if val > hi else val


@dataclass
class FPVFollowConfig:
    deadzone_norm: float = 0.02
    deadzone_bleed: float = 0.15
    yaw_kp: float = 340.0
    yaw_ki: float = 45.0
    yaw_kd: float = 60.0
    pitch_kp: float = 310.0
    pitch_ki: float = 40.0
    pitch_kd: float = 55.0
    roll_kp: float = 80.0
    roll_ki: float = 8.0
    roll_kd: float = 15.0
    max_yaw: float = 400.0
    max_pitch: float = 360.0
    max_roll: float = 120.0
    i_limit: float = 0.45
    d_filter: float = 0.35
    expo: float = 0.85
    lead_s: float = 0.12
    meas_alpha: float = 0.45
    out_alpha: float = 0.55
    slew_yaw: float = 1600.0
    slew_pitch: float = 1400.0
    slew_roll: float = 600.0
    yaw_dir: float = 1.0
    pitch_dir: float = -1.0
    roll_dir: float = 1.0
    use_roll: bool = False
    roll_blend: float = 0.25
    rc_mid: int = 1500
    rc_min: int = 1000
    rc_max: int = 2000


class FPVFollowController:
    def __init__(self, sys_cfg: SystemConfig | None = None) -> None:
        self.sys_cfg = sys_cfg or SystemConfig()
        self.cfg = FPVFollowConfig(
            yaw_kp=self.sys_cfg.yaw_pid.kp,
            yaw_ki=self.sys_cfg.yaw_pid.ki,
            yaw_kd=self.sys_cfg.yaw_pid.kd,
            pitch_kp=self.sys_cfg.altitude_pid.kp,
            pitch_ki=self.sys_cfg.altitude_pid.ki,
            pitch_kd=self.sys_cfg.altitude_pid.kd,
            max_yaw=self.sys_cfg.yaw_pid.max_output,
            max_pitch=self.sys_cfg.altitude_pid.max_output,
            deadzone_norm=self.sys_cfg.offsets.deadzone_norm,
            lead_s=self.sys_cfg.prediction.lead_time_s,
        )

        self.yaw_pid = PIDController(self.sys_cfg.yaw_pid)
        self.altitude_pid = PIDController(self.sys_cfg.altitude_pid)
        self.distance_estimator = DistanceEstimator(self.sys_cfg.distance)
        self.motion_predictor = MotionPredictor(self.sys_cfg.prediction)

        self.reset()

    def update_sys_config(self, sys_cfg: SystemConfig) -> None:
        self.sys_cfg = sys_cfg
        self.yaw_pid.update_config(sys_cfg.yaw_pid)
        self.altitude_pid.update_config(sys_cfg.altitude_pid)
        self.distance_estimator.update_config(sys_cfg.distance)
        self.motion_predictor.update_config(sys_cfg.prediction)
        self.cfg.deadzone_norm = sys_cfg.offsets.deadzone_norm
        self.cfg.lead_s = sys_cfg.prediction.lead_time_s

    def reset(self) -> None:
        c = self.cfg
        self.yaw_pid.reset()
        self.altitude_pid.reset()
        self.distance_estimator.reset()
        self.motion_predictor.reset()

        self._cmd_roll = float(c.rc_mid)
        self._cmd_pitch = float(c.rc_mid)
        self._cmd_yaw = float(c.rc_mid)
        self._t: float | None = None

        self.last_trajectory: TrajectoryEstimate | None = None
        self.last_distance: DistanceEstimate | None = None

    def update(
        self,
        bbox_xywh: tuple[float, float, float, float] | None,
        frame_w: int,
        frame_h: int,
    ) -> tuple[int, int, int]:
        c = self.cfg
        now = time.perf_counter()
        dt = 0.033 if self._t is None else clamp(now - self._t, 0.001, 0.1)
        self._t = now

        # Update motion prediction & trajectory estimation
        traj = self.motion_predictor.update(bbox_xywh, dt)
        self.last_trajectory = traj

        if bbox_xywh is None and not self.motion_predictor.kalman.initialized:
            return self.fade_to_mid()

        # Update distance estimation using bbox width
        curr_w_px = traj.smoothed_bbox[2] if traj.smoothed_bbox[2] > 0 else (bbox_xywh[2] if bbox_xywh else 50.0)
        dist_est = self.distance_estimator.compute_following_control(curr_w_px, dt)
        self.last_distance = dist_est

        half_w = max(1.0, frame_w * 0.5)
        half_h = max(1.0, frame_h * 0.5)
        frame_cx = frame_w * 0.5 + self.sys_cfg.offsets.horizontal_offset_norm * half_w
        frame_cy = frame_h * 0.5 + self.sys_cfg.offsets.vertical_offset_norm * half_h

        # Aim point normalized relative to offset frame center
        nx = clamp((traj.aim_cx - frame_cx) / half_w, -1.0, 1.0)
        ny = clamp((traj.aim_cy - frame_cy) / half_h, -1.0, 1.0)

        # PID calculations
        yaw_res = self.yaw_pid.update(nx, dt, deadzone=c.deadzone_norm, expo=c.expo)
        alt_res = self.altitude_pid.update(ny, dt, deadzone=c.deadzone_norm, expo=c.expo)

        yaw_off = c.yaw_dir * yaw_res.output
        pitch_off = c.pitch_dir * alt_res.output + dist_est.recommended_pitch_offset
        roll_off = 0.0

        target_yaw = c.rc_mid + yaw_off
        target_pitch = c.rc_mid + pitch_off
        target_roll = c.rc_mid + roll_off

        def slew(cur: float, tgt: float, rate: float) -> float:
            step = rate * dt
            return cur + clamp(tgt - cur, -step, step)

        self._cmd_yaw = slew(self._cmd_yaw, target_yaw, self.sys_cfg.safety.max_yaw_rate)
        self._cmd_pitch = slew(self._cmd_pitch, target_pitch, self.sys_cfg.safety.max_climb_rate)
        self._cmd_roll = slew(self._cmd_roll, target_roll, c.slew_roll)

        roll = int(clamp(self._cmd_roll, c.rc_min, c.rc_max))
        pitch = int(clamp(self._cmd_pitch, c.rc_min, c.rc_max))
        yaw = int(clamp(self._cmd_yaw, c.rc_min, c.rc_max))
        return roll, pitch, yaw

    def fade_to_mid(self, factor: float = 0.88) -> tuple[int, int, int]:
        c = self.cfg
        self._cmd_roll = c.rc_mid + (self._cmd_roll - c.rc_mid) * factor
        self._cmd_pitch = c.rc_mid + (self._cmd_pitch - c.rc_mid) * factor
        self._cmd_yaw = c.rc_mid + (self._cmd_yaw - c.rc_mid) * factor
        return (
            int(clamp(self._cmd_roll, c.rc_min, c.rc_max)),
            int(clamp(self._cmd_pitch, c.rc_min, c.rc_max)),
            int(clamp(self._cmd_yaw, c.rc_min, c.rc_max)),
        )
