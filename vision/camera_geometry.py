"""Camera mount geometry — turns pixel positions into true target bearings.

An FPV camera is rarely bolted on looking dead straight ahead: it is usually
tilted up (10–40° for forward flight), sometimes rotated or offset sideways.
Without compensation the controller assumes "target at image centre == target
dead ahead", so an up-tilted camera makes the drone hold the target well above
itself and mis-reads the follow range.

This module converts a pixel to a bearing in the drone body frame (and, when
attitude telemetry is available, in a gravity-levelled frame) for any mount
angle.

Frames
------
Camera: +X right, +Y down, +Z along the optical axis.
Body:   +X forward (nose), +Y right, +Z down.

Angle conventions (all degrees in config, radians internally)
------------------------------------------------------------
mount_pitch_deg  > 0  camera tilted UP (nose-up look)
mount_roll_deg   > 0  camera rotated clockwise seen from behind the lens, i.e.
                      its right edge dropped. The picture then looks rotated
                      counter-clockwise, so the horizon rises to the right.
mount_yaw_deg    > 0  camera aimed to the right of the nose
azimuth          > 0  target is to the right
elevation        > 0  target is above the reference plane
"""

from __future__ import annotations

import math
from dataclasses import dataclass

Vec3 = tuple[float, float, float]


def clamp(val: float, lo: float, hi: float) -> float:
    return lo if val < lo else hi if val > hi else val


def focal_px_from_fov(dim_px: float, fov_deg: float) -> float:
    """Pinhole focal length in pixels for a sensor dimension and its FOV."""
    fov = clamp(float(fov_deg), 1.0, 179.0)
    return (max(1.0, float(dim_px)) * 0.5) / math.tan(math.radians(fov) * 0.5)


def fov_deg_from_focal(dim_px: float, focal_px: float) -> float:
    """Inverse of :func:`focal_px_from_fov`."""
    return math.degrees(2.0 * math.atan((max(1.0, float(dim_px)) * 0.5) / max(1.0, float(focal_px))))


def _rot_y(v: Vec3, ang: float) -> Vec3:
    """Rotate about body +Y. Positive angle pitches a forward vector UP."""
    c, s = math.cos(ang), math.sin(ang)
    x, y, z = v
    return (x * c + z * s, y, -x * s + z * c)


def _rot_z(v: Vec3, ang: float) -> Vec3:
    """Rotate about body +Z (down). Positive angle yaws a forward vector RIGHT."""
    c, s = math.cos(ang), math.sin(ang)
    x, y, z = v
    return (x * c - y * s, x * s + y * c, z)


def _rot_x(v: Vec3, ang: float) -> Vec3:
    """Rotate about body +X (forward). Positive angle rolls right-wing DOWN."""
    c, s = math.cos(ang), math.sin(ang)
    x, y, z = v
    return (x, y * c - z * s, y * s + z * c)


def _normalize(v: Vec3) -> Vec3:
    x, y, z = v
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-9:
        return (1.0, 0.0, 0.0)
    return (x / n, y / n, z / n)


def _bearing(v: Vec3) -> tuple[float, float]:
    """Azimuth (right +) and elevation (up +) of a direction vector."""
    x, y, z = v
    az = math.atan2(y, x)
    el = math.atan2(-z, math.hypot(x, y))
    return az, el


@dataclass
class TargetBearing:
    """Where the target really is, relative to the airframe."""

    az_rad: float  # Azimuth used for control (right +)
    el_rad: float  # Elevation used for control (up +)
    az_body_rad: float  # Azimuth in body frame (ignores vehicle attitude)
    el_body_rad: float
    az_err_rad: float  # Azimuth error after aim offsets
    el_err_rad: float  # Elevation error after aim offsets / desired elevation
    nx: float  # Normalized yaw error for the PID  [-1..1]
    ny: float  # Normalized vertical error for the PID [-1..1] (+ = target low)
    slant_m: float  # Line-of-sight range from the pinhole size estimate
    ground_m: float  # Horizontal component of the range — what "follow distance" means
    vertical_m: float  # Vertical separation (+ = target above us)
    half_fov_h_rad: float
    half_fov_v_rad: float
    levelled: bool  # True when vehicle attitude was compensated


def resolve_focal_px(
    cam_cfg,
    frame_w: int,
    frame_h: int,
    calibrated_focal_px: float | None = None,
) -> tuple[float, float]:
    """Pick the focal lengths (fx, fy) used for angle maths.

    The distance calibration produces a real measured focal length; prefer it
    over the nominal FOV numbers when it is physically plausible, since then
    angles and ranges come from the same optical model. Square pixels are
    assumed for the calibrated path.
    """
    use_calib = bool(getattr(cam_cfg, "use_calibrated_focal", True))
    if use_calib and calibrated_focal_px:
        f = float(calibrated_focal_px)
        implied_fov = fov_deg_from_focal(frame_w, f)
        if 20.0 <= implied_fov <= 170.0:
            return f, f
    fx = focal_px_from_fov(frame_w, getattr(cam_cfg, "fov_h_deg", 90.0))
    fy = focal_px_from_fov(frame_h, getattr(cam_cfg, "fov_v_deg", 60.0))
    return fx, fy


def solve_bearing(
    box_cx: float,
    box_cy: float,
    frame_w: int,
    frame_h: int,
    cam_cfg,
    offsets_cfg=None,
    slant_m: float = 0.0,
    vehicle_roll_deg: float = 0.0,
    vehicle_pitch_deg: float = 0.0,
    calibrated_focal_px: float | None = None,
) -> TargetBearing:
    """Convert a target pixel into a mount-corrected bearing + range split."""
    fx, fy = resolve_focal_px(cam_cfg, frame_w, frame_h, calibrated_focal_px)
    half_fov_h = math.atan((max(1.0, frame_w) * 0.5) / fx)
    half_fov_v = math.atan((max(1.0, frame_h) * 0.5) / fy)

    dx = float(box_cx) - float(frame_w) * 0.5
    dy = float(box_cy) - float(frame_h) * 0.5

    # 1) Pixels → normalized image plane. Roll is a pure rotation here (exact even
    #    when fx != fy), so undo it now to align the image axes with the airframe.
    x, y = dx / fx, dy / fy
    mount_roll = math.radians(float(getattr(cam_cfg, "mount_roll_deg", 0.0)))
    if abs(mount_roll) > 1e-6:
        cr, sr = math.cos(mount_roll), math.sin(mount_roll)
        x, y = x * cr - y * sr, x * sr + y * cr

    # 2) Ray in a body-aligned frame (X fwd, Y right, Z down)
    ray = _normalize((1.0, x, y))

    # 3) Apply the mount tilt and sideways aim
    mount_pitch = math.radians(float(getattr(cam_cfg, "mount_pitch_deg", 0.0)))
    mount_yaw = math.radians(float(getattr(cam_cfg, "mount_yaw_deg", 0.0)))
    ray_body = _rot_z(_rot_y(ray, mount_pitch), mount_yaw)
    az_body, el_body = _bearing(ray_body)

    # 4) Optionally remove the airframe's own attitude to get gravity-levelled angles
    stabilize = bool(getattr(cam_cfg, "stabilize_with_attitude", False))
    if stabilize:
        ray_ctl = _rot_y(
            _rot_x(ray_body, math.radians(float(vehicle_roll_deg))),
            math.radians(float(vehicle_pitch_deg)),
        )
        az_ctl, el_ctl = _bearing(ray_ctl)
    else:
        az_ctl, el_ctl = az_body, el_body

    # 5) Aim point offsets — kept in the same units as the old pixel-normalized ones
    h_off = float(getattr(offsets_cfg, "horizontal_offset_norm", 0.0)) if offsets_cfg else 0.0
    v_off = float(getattr(offsets_cfg, "vertical_offset_norm", 0.0)) if offsets_cfg else 0.0
    az_target = h_off * half_fov_h
    el_target = math.radians(float(getattr(cam_cfg, "desired_elevation_deg", 0.0))) - v_off * half_fov_v

    az_err = az_ctl - az_target
    nx = clamp(az_err / max(1e-6, half_fov_h), -1.0, 1.0)

    # Vertical reference: "level" holds the target at a true elevation (mount-corrected),
    # "image" reproduces the legacy behaviour of parking it at the image centre.
    vertical_ref = str(getattr(cam_cfg, "vertical_ref", "level") or "level").lower()
    if vertical_ref.startswith("image"):
        el_err = 0.0
        ny = clamp((float(box_cy) - (float(frame_h) * 0.5 + v_off * frame_h * 0.5)) / max(1.0, frame_h * 0.5), -1.0, 1.0)
    else:
        el_err = el_ctl - el_target
        ny = clamp(-el_err / max(1e-6, half_fov_v), -1.0, 1.0)

    # 6) Split the line-of-sight range into horizontal / vertical parts.
    #    "Follow distance" is a horizontal distance, so a tilted camera must not
    #    feed raw slant range into the distance controller.
    slant = max(0.0, float(slant_m))
    ground = slant * math.cos(el_ctl)
    vertical = slant * math.sin(el_ctl)

    return TargetBearing(
        az_rad=az_ctl,
        el_rad=el_ctl,
        az_body_rad=az_body,
        el_body_rad=el_body,
        az_err_rad=az_err,
        el_err_rad=el_err,
        nx=nx,
        ny=ny,
        slant_m=slant,
        ground_m=ground,
        vertical_m=vertical,
        half_fov_h_rad=half_fov_h,
        half_fov_v_rad=half_fov_v,
        levelled=stabilize,
    )


def level_reference_line(
    frame_w: int,
    frame_h: int,
    cam_cfg,
    offsets_cfg=None,
    vehicle_roll_deg: float = 0.0,
    vehicle_pitch_deg: float = 0.0,
    calibrated_focal_px: float | None = None,
) -> tuple[int, float]:
    """Image row (and tilt in degrees) where the aim elevation projects.

    Used by the HUD so the operator can see where "level with the drone" sits in
    a tilted camera's picture. The returned tilt is positive when the line's
    right-hand end should be drawn *lower* on screen.
    """
    _, fy = resolve_focal_px(cam_cfg, frame_w, frame_h, calibrated_focal_px)
    half_fov_v = math.atan((max(1.0, frame_h) * 0.5) / fy)

    v_off = float(getattr(offsets_cfg, "vertical_offset_norm", 0.0)) if offsets_cfg else 0.0
    el_target = math.radians(float(getattr(cam_cfg, "desired_elevation_deg", 0.0))) - v_off * half_fov_v

    tilt = float(getattr(cam_cfg, "mount_pitch_deg", 0.0))
    roll = float(getattr(cam_cfg, "mount_roll_deg", 0.0))
    if bool(getattr(cam_cfg, "stabilize_with_attitude", False)):
        tilt += float(vehicle_pitch_deg)
        roll += float(vehicle_roll_deg)

    # el = tilt - atan(dy / fy)  ->  dy = fy * tan(tilt - el_target)
    ang = clamp(math.radians(tilt) - el_target, -1.45, 1.45)
    row = frame_h * 0.5 + fy * math.tan(ang)
    return int(round(row)), -roll
