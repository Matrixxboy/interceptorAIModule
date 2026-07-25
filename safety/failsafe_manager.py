"""Safety Manager & Failsafe Controller for Autonomous Drone Operations."""

from __future__ import annotations

import time
from dataclasses import dataclass

from config import SafetyConfig


@dataclass
class SafetyState:
    is_safe: bool
    failsafe_active: bool
    override_active: bool
    allow_forward_motion: bool
    reason: str


class FailsafeManager:
    def __init__(self, cfg: SafetyConfig | None = None) -> None:
        self.cfg = cfg or SafetyConfig()
        self.manual_override = False
        self._lost_frames = 0
        self._last_lock_time = time.time()

    def update_config(self, cfg: SafetyConfig) -> None:
        self.cfg = cfg

    def trigger_manual_override(self, active: bool = True) -> None:
        self.manual_override = active

    def reset(self) -> None:
        self.manual_override = False
        self._lost_frames = 0
        self._last_lock_time = time.time()

    def evaluate(
        self,
        locked: bool,
        confidence: float,
        distance_m: float | None = None,
        min_safe_distance_m: float | None = None,
    ) -> SafetyState:
        c = self.cfg

        if self.manual_override:
            return SafetyState(
                is_safe=False,
                failsafe_active=True,
                override_active=True,
                allow_forward_motion=False,
                reason="Manual override engaged by operator.",
            )

        if not locked:
            self._lost_frames += 1
            if self._lost_frames > c.max_lost_frames:
                return SafetyState(
                    is_safe=False,
                    failsafe_active=True,
                    override_active=False,
                    allow_forward_motion=False,
                    reason=f"Target lost for > {c.max_lost_frames} frames.",
                )
            return SafetyState(
                is_safe=True,
                failsafe_active=False,
                override_active=False,
                allow_forward_motion=False,
                reason="No active target lock.",
            )

        # Locked checks
        if confidence < c.min_conf_threshold:
            self._lost_frames += 1
            if self._lost_frames > c.max_lost_frames:
                return SafetyState(
                    is_safe=False,
                    failsafe_active=True,
                    override_active=False,
                    allow_forward_motion=False,
                    reason=f"Detection confidence ({confidence:.2f}) below threshold ({c.min_conf_threshold:.2f}).",
                )

        self._lost_frames = 0
        self._last_lock_time = time.time()

        # Distance safety — only hard-fail when well inside min_safe (avoids false FC-style alarms
        # from noisy uncalibrated distance estimates).
        allow_forward = True
        if distance_m is not None and locked:
            min_safe = float(min_safe_distance_m) if min_safe_distance_m is not None else 2.0
            min_safe = max(0.2, min_safe)
            if distance_m < min_safe * 0.35:
                return SafetyState(
                    is_safe=False,
                    failsafe_active=True,
                    override_active=False,
                    allow_forward_motion=False,
                    reason=f"Proximity alert! Distance ({distance_m:.2f}m) below safe minimum.",
                )
            elif distance_m < min_safe:
                allow_forward = False

        return SafetyState(
            is_safe=True,
            failsafe_active=False,
            override_active=False,
            allow_forward_motion=allow_forward,
            reason="System nominal.",
        )
