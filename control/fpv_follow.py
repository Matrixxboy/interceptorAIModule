"""FPV visual-servo interceptor controller — 4-axis (Yaw, Altitude/Pitch, Distance, Roll).

This controller steers the drone towards the predicted interception point of a
moving target using:
  1.  A 3D Kalman filter (position + velocity + acceleration in world frame).
  2.  A 3D quadratic interception solver (time-to-impact & collision point).
  3.  Pure Proportional Navigation (PPN) for commanded acceleration.
  4.  Existing PID loops for yaw & altitude stabilization.
"""

from __future__ import annotations

import math
import time
import numpy as np
from dataclasses import dataclass

from config import SystemConfig
from control.pid_controller import PIDController
from estimation.distance_estimator import DistanceEstimate, DistanceEstimator
from tracking.motion_predictor import MotionPredictor, TrajectoryEstimate
from tracking.interception_calc import InterceptionCalculator
from tracking.kalman_filter_3d import KalmanFilter3D
from tracking.ppn_guidance import PPNGuidance
from vision.camera_geometry import TargetBearing, solve_bearing, pixel_to_world, world_to_pixel


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

        # Interceptor speed in m/s (derived from px/s config ÷ 100 as approximation)
        self._interceptor_speed_ms = getattr(
            self.sys_cfg.prediction, "interceptor_speed_px_s", 1200.0
        ) / 100.0
        self.interception_calc = InterceptionCalculator(
            interceptor_speed=self._interceptor_speed_ms
        )

        # 3D world-frame tracker + PPN guidance
        # High r_pos dampens noisy depth estimates; low q_acc prevents false acceleration
        self.kalman_3d = KalmanFilter3D(
            q_pos=1e-2, q_vel=5e-2, q_acc=0.5, r_pos=5.0
        )
        self.ppn = PPNGuidance(N_p=3.0, b=self._interceptor_speed_ms, C_b=1.0)

        # Live airframe attitude (only used when camera stabilization is enabled)
        self.vehicle_roll_deg = 0.0
        self.vehicle_pitch_deg = 0.0

        self.reset()

    def update_sys_config(self, sys_cfg: SystemConfig) -> None:
        self.sys_cfg = sys_cfg
        self.yaw_pid.update_config(sys_cfg.yaw_pid)
        self.altitude_pid.update_config(sys_cfg.altitude_pid)
        self.distance_estimator.update_config(sys_cfg.distance)
        self.motion_predictor.update_config(sys_cfg.prediction)
        self._interceptor_speed_ms = getattr(
            sys_cfg.prediction, "interceptor_speed_px_s", 1200.0
        ) / 100.0
        self.interception_calc.interceptor_speed = self._interceptor_speed_ms
        self.ppn.b = self._interceptor_speed_ms
        self.cfg.deadzone_norm = sys_cfg.offsets.deadzone_norm
        self.cfg.lead_s = sys_cfg.prediction.lead_time_s

    def reset(self) -> None:
        c = self.cfg
        self.yaw_pid.reset()
        self.altitude_pid.reset()
        self.distance_estimator.reset()
        self.motion_predictor.reset()
        self.ppn.reset()

        self._cmd_roll = float(c.rc_mid)
        self._cmd_pitch = float(c.rc_mid)
        self._cmd_yaw = float(c.rc_mid)
        self._cmd_throttle = 1500.0
        self._t: float | None = None

        # --- Exposed state for HUD ---
        self.last_trajectory: TrajectoryEstimate | None = None
        self.last_distance: DistanceEstimate | None = None
        self.last_bearing: TargetBearing | None = None
        self.last_intercept_pt: tuple[float, float] | None = None
        self.last_intercept_3d: tuple[float, float, float] | None = None
        self.last_t_intercept: float | None = None
        self.last_kalman_3d_pos: tuple[float, float, float] | None = None
        self.last_kalman_3d_vel: tuple[float, float, float] | None = None
        self.last_a_P: np.ndarray | None = None
        self.last_v_P: np.ndarray | None = None
        self.last_closing_rate: float = 0.0

    def set_vehicle_attitude(self, roll_deg: float, pitch_deg: float) -> None:
        """Feed FC attitude so a fixed camera can be levelled against gravity."""
        self.vehicle_roll_deg = float(roll_deg)
        self.vehicle_pitch_deg = float(pitch_deg)

    def update(
        self,
        bbox_xywh: tuple[float, float, float, float] | None,
        frame_w: int,
        frame_h: int,
        base_throttle: int = 1500,
    ) -> tuple[int, int, int, int]:
        c = self.cfg
        now = time.perf_counter()
        dt = 0.033 if self._t is None else clamp(now - self._t, 0.001, 0.1)
        self._t = now

        # Update motion prediction & trajectory estimation
        traj = self.motion_predictor.update(bbox_xywh, dt)
        self.last_trajectory = traj

        if bbox_xywh is None and not self.motion_predictor.kalman.initialized:
            return self.fade_to_mid(base_throttle=base_throttle)

        # Distance from calibrated axis (width / height / max / diag)
        from estimation.distance_calib import bbox_size_px

        if traj.smoothed_bbox[2] > 0 and traj.smoothed_bbox[3] > 0:
            size_src = traj.smoothed_bbox
        elif bbox_xywh is not None:
            size_src = bbox_xywh
        else:
            size_src = (0.0, 0.0, 50.0, 50.0)
        axis = getattr(self.sys_cfg.distance, "size_axis", "width") or "width"
        size_px = bbox_size_px(size_src, axis)
        slant_m = self.distance_estimator.estimate_distance(size_px)

        # ── Current target pixel centre ──
        if bbox_xywh is not None:
            box_cx = float(bbox_xywh[0]) + float(bbox_xywh[2]) * 0.5
            box_cy = float(bbox_xywh[1]) + float(bbox_xywh[3]) * 0.5
        else:
            sb = traj.smoothed_bbox
            box_cx = sb[0] + sb[2] * 0.5
            box_cy = sb[1] + sb[3] * 0.5

        # ── 3D Kalman Filter ──
        p_w_t_raw = pixel_to_world(
            box_cx, box_cy, slant_m, frame_w, frame_h,
            self.sys_cfg.camera, self.vehicle_roll_deg, self.vehicle_pitch_deg, 0.0,
            self.sys_cfg.distance.focal_length_px,
        )

        if bbox_xywh is not None:
            p_w_t = self.kalman_3d.update(p_w_t_raw)
        else:
            p_w_t = self.kalman_3d.predict(dt)

        v_w_t = self.kalman_3d.get_velocity()
        self.last_kalman_3d_pos = p_w_t
        self.last_kalman_3d_vel = v_w_t

        # Velocity confidence: only trust the 3D velocity when it's been
        # consistent for several frames. Low-confidence = don't let PPN fight the PID.
        tgt_speed = math.sqrt(v_w_t[0]**2 + v_w_t[1]**2 + v_w_t[2]**2)
        vel_confident = tgt_speed > 0.3 and self.kalman_3d.initialized

        # ── 3D Interception point (quadratic solver) ──
        if vel_confident:
            intercept_3d, t_int = self.interception_calc.calculate_interception_3d(
                p_w_t, v_w_t, interceptor_speed_ms=self._interceptor_speed_ms,
            )
        else:
            intercept_3d, t_int = None, None

        if intercept_3d is not None:
            self.last_intercept_3d = intercept_3d
            self.last_t_intercept = t_int
        else:
            self.last_intercept_3d = p_w_t
            self.last_t_intercept = None
            intercept_3d = p_w_t

        # ── PPN Guidance (only when velocity is trusted) ──
        ppn_active = vel_confident
        if ppn_active:
            if self.last_v_P is not None:
                v_w_I = (float(self.last_v_P[0]),
                         float(self.last_v_P[1]),
                         float(self.last_v_P[2]))
            else:
                r = np.array(p_w_t)
                r_norm = float(np.linalg.norm(r))
                if r_norm > 1e-3:
                    r_hat = r / r_norm
                    v_w_I = tuple(float(x) for x in r_hat * self._interceptor_speed_ms)
                else:
                    v_w_I = (1.0, 0.0, 0.0)

            a_P, v_P, psi_ref = self.ppn.compute_guidance(p_w_t, v_w_t, v_w_I, dt)
            self.last_a_P = a_P
            self.last_v_P = v_P
        else:
            a_P = np.zeros(3)
            self.last_a_P = a_P
            # Reset PPN state when not confident so it doesn't accumulate stale velocity
            self.ppn.reset()
            self.last_v_P = None

        # Closing rate
        r_vec = np.array(p_w_t)
        r_norm = float(np.linalg.norm(r_vec))
        if r_norm > 1e-3 and self.last_v_P is not None:
            r_hat = r_vec / r_norm
            v_rel = np.array(v_w_t) - np.array(
                (float(self.last_v_P[0]), float(self.last_v_P[1]), float(self.last_v_P[2]))
            ) if self.last_v_P is not None else np.array(v_w_t)
            self.last_closing_rate = -float(np.dot(v_rel, r_hat))
        else:
            self.last_closing_rate = 0.0

        # ── Project intercept point to 2D for HUD only ──
        intercept_pixel = world_to_pixel(
            intercept_3d, frame_w, frame_h,
            self.sys_cfg.camera, self.vehicle_roll_deg, self.vehicle_pitch_deg, 0.0,
            self.sys_cfg.distance.focal_length_px,
        )
        if intercept_pixel is not None:
            self.last_intercept_pt = (intercept_pixel[0], intercept_pixel[1])
        else:
            self.last_intercept_pt = None

        # ═══════════════════════════════════════════════════════════════════
        # PID ALWAYS STEERS TOWARD THE REAL TARGET — never the intercept pt
        # ═══════════════════════════════════════════════════════════════════
        bearing = solve_bearing(
            box_cx, box_cy, frame_w, frame_h,
            self.sys_cfg.camera, self.sys_cfg.offsets,
            slant_m=slant_m,
            vehicle_roll_deg=self.vehicle_roll_deg,
            vehicle_pitch_deg=self.vehicle_pitch_deg,
            calibrated_focal_px=self.sys_cfg.distance.focal_length_px,
        )
        self.last_bearing = bearing

        dist_est = self.distance_estimator.compute_following_control(
            size_px, dt, distance_m=bearing.ground_m,
        )
        self.last_distance = dist_est

        nx = bearing.nx
        ny = bearing.ny

        # ── PID ──
        yaw_res = self.yaw_pid.update(nx, dt, deadzone=c.deadzone_norm, expo=c.expo)
        alt_res = self.altitude_pid.update(ny, dt, deadzone=c.deadzone_norm, expo=c.expo)

        speed_scale = clamp(float(getattr(self.sys_cfg.safety, "follow_speed_scale", 0.50)), 0.05, 1.0)
        pitch_scale = clamp(float(getattr(self.sys_cfg.safety, "follow_pitch_scale", 0.90)), 0.05, 1.5)

        # ── PPN correction (ADDITIVE only, never replaces PID) ──
        ppn_gain = 30.0  # µs per (m/s²) — conservative to not overpower PID
        ppn_pitch    = float(a_P[0]) * pitch_scale * ppn_gain
        ppn_roll     = float(a_P[1]) * pitch_scale * ppn_gain
        ppn_throttle = -float(a_P[2]) * speed_scale * ppn_gain

        # Yaw: PID drives yaw toward the real target center
        yaw_off = c.yaw_dir * yaw_res.output * speed_scale

        # Pitch: distance PID + PPN forward/backward correction
        raw_pitch = float(dist_est.recommended_pitch_offset) * pitch_scale + ppn_pitch
        fwd_cap = float(getattr(self.sys_cfg.safety, "max_forward_speed", 350.0))
        back_cap = float(getattr(self.sys_cfg.safety, "max_backward_speed", 250.0))
        raw_pitch = clamp(raw_pitch, -back_cap, fwd_cap)
        pitch_off = c.pitch_dir * raw_pitch

        # Throttle: altitude PID + PPN vertical correction
        throttle_off = -alt_res.output * speed_scale + ppn_throttle

        # Roll: PPN lateral correction only
        roll_off = ppn_roll

        target_yaw = c.rc_mid + yaw_off
        target_pitch = c.rc_mid + pitch_off
        target_roll = c.rc_mid + roll_off
        target_throttle = float(base_throttle) + throttle_off

        def slew(cur: float, tgt: float, rate: float) -> float:
            step = rate * dt
            return cur + clamp(tgt - cur, -step, step)

        # Scale slew rates gently — keep a floor so small speed_scale still moves
        slew_scale = max(0.45, speed_scale)
        pitch_slew = float(getattr(self.sys_cfg.safety, "max_pitch_rate", 2200.0)) * max(0.55, pitch_scale)
        self._cmd_yaw = slew(self._cmd_yaw, target_yaw, self.sys_cfg.safety.max_yaw_rate * slew_scale)
        self._cmd_pitch = slew(self._cmd_pitch, target_pitch, pitch_slew)
        self._cmd_roll = slew(self._cmd_roll, target_roll, c.slew_roll * slew_scale)
        self._cmd_throttle = slew(
            getattr(self, "_cmd_throttle", float(base_throttle)),
            target_throttle,
            1200.0 * slew_scale,
        )

        roll = int(clamp(self._cmd_roll, float(c.rc_min), float(c.rc_max)))
        pitch = int(clamp(self._cmd_pitch, float(c.rc_min), float(c.rc_max)))
        yaw = int(clamp(self._cmd_yaw, float(c.rc_min), float(c.rc_max)))
        throttle = int(clamp(self._cmd_throttle, float(c.rc_min), float(c.rc_max)))

        return roll, pitch, yaw, throttle

    def fade_to_mid(self, factor: float = 0.88, base_throttle: int = 1500) -> tuple[int, int, int, int]:
        c = self.cfg
        self._cmd_roll = c.rc_mid + (self._cmd_roll - c.rc_mid) * factor
        self._cmd_pitch = c.rc_mid + (self._cmd_pitch - c.rc_mid) * factor
        self._cmd_yaw = c.rc_mid + (self._cmd_yaw - c.rc_mid) * factor
        cur_thr = getattr(self, '_cmd_throttle', float(base_throttle))
        self._cmd_throttle = float(base_throttle) + (cur_thr - float(base_throttle)) * factor
        return (
            int(clamp(self._cmd_roll, float(c.rc_min), float(c.rc_max))),
            int(clamp(self._cmd_pitch, float(c.rc_min), float(c.rc_max))),
            int(clamp(self._cmd_yaw, float(c.rc_min), float(c.rc_max))),
            int(clamp(self._cmd_throttle, float(c.rc_min), float(c.rc_max))),
        )
