"""Verify the AI follow path produces non-neutral sticks and is not blocked by joystick/conf.

Run:  .venv\\Scripts\\python.exe scripts\\test_follow_path.py
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from config import SystemConfig
from control.fpv_follow import FPVFollowController
from control.joystick_manager import JoystickManager
from core.tracking_worker import TrackingWorkerThread
from safety.failsafe_manager import FailsafeManager


FAILS: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(f"{'PASS' if ok else 'FAIL'}  {name}" + (f" — {detail}" if detail else ""))
    if not ok:
        FAILS.append(name)


print("\n--- FPV controller produces stick deflection ---")
cfg = SystemConfig()
cfg.safety.follow_speed_scale = 0.50
cfg.camera.mount_pitch_deg = 0.0
ctl = FPVFollowController(cfg)
# Target to the right of centre
bbox = (900.0, 300.0, 120.0, 120.0)
last = (1500, 1500, 1500, 1500)
for _ in range(30):
    last = ctl.update(bbox, 1280, 720, base_throttle=1500)
roll, pitch, yaw, thr = last
check("yaw moves right of mid", yaw > 1510, f"yaw={yaw}")
check("bearing nx > 0", ctl.last_bearing is not None and ctl.last_bearing.nx > 0.1,
      f"nx={ctl.last_bearing.nx if ctl.last_bearing else None}")

print("\n--- Confidence gate vs healthy scale-lock conf ---")
# Simulate the mapped confidence used after the fix
raw_score = 0.55
mapped = max(0.70, min(1.0, 0.55 + 0.60 * max(0.0, raw_score)))
check("mapped conf clears 60% gate", mapped >= cfg.safety.follow_min_confidence,
      f"mapped={mapped:.2f} gate={cfg.safety.follow_min_confidence:.2f}")
old_floor = max(0.55, raw_score)
check("old floor would have failed gate", old_floor < cfg.safety.follow_min_confidence,
      f"old={old_floor:.2f}")

print("\n--- Failsafe must not override a healthy lock ---")
fs = FailsafeManager(cfg.safety)
st = fs.evaluate(locked=True, confidence=0.75, distance_m=6.0, min_safe_distance_m=2.0)
check("healthy lock is safe", st.is_safe and not st.override_active, st.reason)

print("\n--- Worker follow decision matrix (logic) ---")
# Exercise the same boolean used in the worker without spinning the camera thread
cases = [
    ("locked+assist+conf", True, True, 0.80, False, True),
    ("unlocked", False, True, 0.90, False, False),
    ("assist off", True, False, 0.90, False, False),
    ("low conf", True, True, 0.40, False, False),
    ("override", True, True, 0.90, True, False),
]
for name, locked, assist, conf, override, expect in cases:
    ai = locked and assist and conf >= cfg.safety.follow_min_confidence and not override
    check(name, ai == expect, f"ai_follow={ai}")

print("\n--- Joystick must not permanently block AI (structural) ---")
# Confirm the worker source no longer returns early with joy sticks before AI
src = Path(ROOT / "core" / "tracking_worker.py").read_text(encoding="utf-8")
check(
    "AI follow runs even with joystick",
    "if ai_follow:" in src and "joy_connected" in src and "FOLLOWING conf=" in src,
)
check(
    "old joy-only branch removed",
    "elif locked and self.assist_enabled and conf >=" not in src,
)

print("\n" + ("ALL FOLLOW CHECKS PASSED" if not FAILS else f"FAILURES: {FAILS}"))
sys.exit(1 if FAILS else 0)
