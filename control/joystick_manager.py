"""Manager for reading USB joysticks/gamepads for RC override."""

from __future__ import annotations

import os
from dataclasses import dataclass, field

os.environ["SDL_VIDEODRIVER"] = "dummy"

try:
    import pygame

    _HAS_PYGAME = True
except ImportError:
    pygame = None  # type: ignore
    _HAS_PYGAME = False

from PyQt6.QtCore import QObject, pyqtSignal

from config import SystemConfig


def clamp(val: float, lo: float, hi: float) -> float:
    return lo if val < lo else hi if val > hi else val


@dataclass
class JoystickState:
    connected: bool = False
    device_name: str = ""
    roll_raw: float = 0.0
    pitch_raw: float = 0.0
    yaw_raw: float = 0.0
    throttle_raw: float = -1.0
    roll_pwm: int = 1500
    pitch_pwm: int = 1500
    yaw_pwm: int = 1500
    throttle_pwm: int = 1000
    aux_pwm: dict = field(default_factory=dict)


class JoystickManager(QObject):
    state_updated = pyqtSignal(object)

    def __init__(self, sys_cfg: SystemConfig | None = None, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.sys_cfg = sys_cfg or SystemConfig()
        self.joystick = None
        self.state = JoystickState()
        self.active_joystick_id = -1
        self.available = _HAS_PYGAME

        if _HAS_PYGAME:
            pygame.display.init()
            pygame.joystick.init()
            self.scan_devices()

    def update_config(self, cfg: SystemConfig) -> None:
        self.sys_cfg = cfg
        self.scan_devices()

    def scan_devices(self) -> list[str]:
        if not _HAS_PYGAME:
            return []

        pygame.joystick.quit()
        pygame.joystick.init()

        count = pygame.joystick.get_count()
        devices: list[str] = []
        target_name = self.sys_cfg.joystick.device_name
        best_id = -1

        for i in range(count):
            try:
                joy = pygame.joystick.Joystick(i)
                joy.init()
                name = joy.get_name()
                devices.append(name)
                if target_name and target_name == name:
                    best_id = i
                elif best_id == -1:
                    best_id = i
            except Exception:
                pass

        if best_id >= 0 and self.sys_cfg.joystick.enabled:
            if self.active_joystick_id != best_id or self.joystick is None:
                self.joystick = pygame.joystick.Joystick(best_id)
                self.joystick.init()
                self.active_joystick_id = best_id
                self.state.connected = True
                self.state.device_name = self.joystick.get_name()
        else:
            self.joystick = None
            self.active_joystick_id = -1
            self.state.connected = False
            self.state.device_name = ""

        return devices

    def _map_to_pwm(self, raw_val: float, ch_cfg) -> int:
        deadzone = self.sys_cfg.joystick.deadzone
        if abs(raw_val) < deadzone:
            raw_val = 0.0
        if ch_cfg.inverted:
            raw_val = -raw_val
        if raw_val >= 0:
            pwm = ch_cfg.center_val + raw_val * (ch_cfg.max_val - ch_cfg.center_val)
        else:
            pwm = ch_cfg.center_val + raw_val * (ch_cfg.center_val - ch_cfg.min_val)
        return int(clamp(pwm, 800, 2200))

    def poll(self) -> JoystickState:
        if not _HAS_PYGAME:
            self.state.connected = False
            self.state.aux_pwm.clear()
            self.state_updated.emit(self.state)
            return self.state

        pygame.event.pump()

        if self.joystick and self.joystick.get_init():
            cfg = self.sys_cfg.joystick
            axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]

            def get_axis(idx: int) -> float:
                return axes[idx] if 0 <= idx < len(axes) else 0.0

            def get_button(idx: int) -> bool:
                if 0 <= idx < self.joystick.get_numbuttons():
                    return bool(self.joystick.get_button(idx))
                return False

            self.state.roll_raw = get_axis(cfg.roll.axis)
            self.state.pitch_raw = get_axis(cfg.pitch.axis)
            self.state.throttle_raw = get_axis(cfg.throttle.axis)
            self.state.yaw_raw = get_axis(cfg.yaw.axis)

            self.state.roll_pwm = self._map_to_pwm(self.state.roll_raw, cfg.roll)
            self.state.pitch_pwm = self._map_to_pwm(self.state.pitch_raw, cfg.pitch)
            self.state.throttle_pwm = self._map_to_pwm(self.state.throttle_raw, cfg.throttle)
            self.state.yaw_pwm = self._map_to_pwm(self.state.yaw_raw, cfg.yaw)

            # One entry per AUX row — keyed by stable index so duplicate names
            # cannot overwrite each other or share the wrong PWM.
            self.state.aux_pwm.clear()
            for i, aux in enumerate(cfg.aux_channels):
                if aux.is_button:
                    pressed = get_button(max(0, aux.axis))
                    if pressed ^ aux.inverted:
                        pwm = aux.max_val
                    else:
                        pwm = aux.min_val
                else:
                    raw = get_axis(max(0, aux.axis))
                    pwm = self._map_to_pwm(raw, aux)

                pwm = int(clamp(pwm, 800, 2200))
                self.state.aux_pwm[f"#{i}"] = pwm
                # Convenience alias only when the name is unique in the list
                name = (aux.name or "").strip()
                if name and sum(1 for a in cfg.aux_channels if (a.name or "").strip() == name) == 1:
                    self.state.aux_pwm[name] = pwm

            self.state.connected = True
        else:
            self.state.connected = False
            self.state.roll_pwm = 1500
            self.state.pitch_pwm = 1500
            self.state.yaw_pwm = 1500
            self.state.throttle_pwm = 1000
            self.state.aux_pwm.clear()

        self.state_updated.emit(self.state)
        return self.state
