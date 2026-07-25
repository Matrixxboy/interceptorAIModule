"""Sanity checks for mount-angle geometry — run directly: python scripts/check_camera_geometry.py"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from config import CameraConfig, OffsetsConfig  # noqa: E402
from vision.camera_geometry import level_reference_line, solve_bearing  # noqa: E402

W, H = 1280, 720
FAILS: list[str] = []


def check(name: str, got: float, want: float, tol: float = 0.05) -> None:
    ok = abs(got - want) <= tol
    print(f"{'PASS' if ok else 'FAIL'}  {name}: got {got:+.3f} want {want:+.3f}")
    if not ok:
        FAILS.append(name)


def cam(**kw) -> CameraConfig:
    c = CameraConfig(fov_h_deg=90.0, fov_v_deg=60.0, use_calibrated_focal=False)
    for k, v in kw.items():
        setattr(c, k, v)
    return c


off = OffsetsConfig()

print("\n-- straight, level camera: centre pixel is dead ahead --")
b = solve_bearing(W / 2, H / 2, W, H, cam(), off, slant_m=10.0)
check("azimuth deg", math.degrees(b.az_rad), 0.0)
check("elevation deg", math.degrees(b.el_rad), 0.0)
check("nx", b.nx, 0.0)
check("ny", b.ny, 0.0)
check("ground range m", b.ground_m, 10.0)

print("\n-- camera tilted UP 25 deg: centre pixel is 25 deg above us --")
c25 = cam(mount_pitch_deg=25.0)
b = solve_bearing(W / 2, H / 2, W, H, c25, off, slant_m=10.0)
check("elevation deg", math.degrees(b.el_rad), 25.0)
check("ny sign (negative = climb)", -1.0 if b.ny < 0 else 1.0, -1.0)
check("ground range m", b.ground_m, 10.0 * math.cos(math.radians(25.0)))
check("vertical sep m", b.vertical_m, 10.0 * math.sin(math.radians(25.0)))

print("\n-- same camera, target on the aim line: no climb demand --")
row, tilt = level_reference_line(W, H, c25, off)
b = solve_bearing(W / 2, row, W, H, c25, off, slant_m=10.0)
check("elevation deg", math.degrees(b.el_rad), 0.0, tol=0.2)
check("ny", b.ny, 0.0, tol=0.01)
check("ground == slant", b.ground_m, 10.0, tol=0.02)
print(f"      aim row = {row}px (image centre is {H // 2}px), line tilt {tilt:+.1f} deg")

print("\n-- 90 deg down-looking camera: centre pixel is straight below --")
b = solve_bearing(W / 2, H / 2, W, H, cam(mount_pitch_deg=-90.0), off, slant_m=8.0)
check("elevation deg", math.degrees(b.el_rad), -90.0)
check("ground range m", b.ground_m, 0.0, tol=0.02)
check("vertical sep m", b.vertical_m, -8.0, tol=0.02)

print("\n-- horizontal aim: right of centre yaws right, tilt must not corrupt it --")
b = solve_bearing(W / 2 + 200, H / 2, W, H, cam(), off, slant_m=10.0)
az_flat = math.degrees(b.az_rad)
check("azimuth sign", 1.0 if az_flat > 0 else -1.0, 1.0)
b_t = solve_bearing(W / 2 + 200, H / 2, W, H, c25, off, slant_m=10.0)
check("azimuth still right with 25 deg tilt", 1.0 if b_t.az_rad > 0 else -1.0, 1.0)

print("\n-- camera rolled 90 deg: image-right is really straight down --")
b = solve_bearing(W / 2 + 200, H / 2, W, H, cam(mount_roll_deg=90.0), off, slant_m=10.0)
check("azimuth deg (no sideways error)", math.degrees(b.az_rad), 0.0, tol=0.2)
check("elevation sign (target below)", -1.0 if b.el_rad < 0 else 1.0, -1.0)

print("\n-- sideways mount yaw 30 deg: centre pixel is 30 deg right --")
b = solve_bearing(W / 2, H / 2, W, H, cam(mount_yaw_deg=30.0), off, slant_m=10.0)
check("azimuth deg", math.degrees(b.az_rad), 30.0)

print("\n-- attitude levelling: drone pitched up 15 deg cancels 15 deg of tilt --")
c = cam(mount_pitch_deg=15.0, stabilize_with_attitude=True)
b = solve_bearing(W / 2, H / 2, W, H, c, off, slant_m=10.0, vehicle_pitch_deg=-15.0)
check("levelled elevation deg", math.degrees(b.el_rad), 0.0, tol=0.2)

print("\n-- legacy image-centre mode still parks the target mid-frame --")
c = cam(mount_pitch_deg=25.0, vertical_ref="image")
b = solve_bearing(W / 2, H / 2, W, H, c, off, slant_m=10.0)
check("ny at image centre", b.ny, 0.0)

print("\n" + ("ALL CHECKS PASSED" if not FAILS else f"FAILURES: {FAILS}"))
sys.exit(1 if FAILS else 0)
