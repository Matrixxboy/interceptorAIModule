"""Manager for translating RC channel inputs into Target Tracking States."""

from __future__ import annotations

from config import SystemConfig


class RCManager:
    def __init__(self, cfg: SystemConfig) -> None:
        self.cfg = cfg
        self.lock_active = False
        self.follow_active = False

    def update_config(self, cfg: SystemConfig) -> None:
        self.cfg = cfg

    def parse_channels(self, channels: list[int]) -> tuple[bool, bool]:
        """
        Parses the raw RC channels (list of 16 PWM values) and returns
        the current state of the Lock and Follow switches.
        """
        lock_ch_idx = self.cfg.rc_control.lock_channel
        follow_ch_idx = self.cfg.rc_control.follow_channel

        if 0 <= lock_ch_idx < len(channels):
            self.lock_active = channels[lock_ch_idx] >= self.cfg.rc_control.lock_threshold
        else:
            self.lock_active = False

        if 0 <= follow_ch_idx < len(channels):
            self.follow_active = channels[follow_ch_idx] >= self.cfg.rc_control.follow_threshold
        else:
            self.follow_active = False

        return self.lock_active, self.follow_active
