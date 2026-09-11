"""Real-time tracking worker thread with target database integration."""

from __future__ import annotations

import math
import time

import cv2
import numpy as np
import serial
from PyQt6.QtCore import QObject, QThread, pyqtSignal

from pathlib import Path

from config import SystemConfig
from control.fpv_follow import FPVFollowController
from control.msp_link import is_capture_card_port
from plugins.flight_controllers.mavlink_controller import MAVLinkController
from plugins.flight_controllers.msp_controller import MSPController
from database.target_profile import TargetProfile, TargetStatus
from database.target_store import TargetStore
from detection.hybrid_tracker import HybridYoloLockTracker
from control.joystick_manager import JoystickManager
from estimation.distance_calib import bbox_size_px
from sys_logging.system_logger import LogCategory, LogSeverity, SystemLogger
from safety.failsafe_manager import FailsafeManager
from telemetry.telemetry_logger import TelemetryLogger, TelemetryRecord
from vision.camera_geometry import level_reference_line


_CAPTURE_HINT = ("uhd", "capture", "hdmi", "usb3", "video grab", "video input")
# Goggles HDMI → this USB capture card. Open by name, never by a guessed index.
_GOGGLES_CAPTURE = "USB3.0 UHD"
_device_name_cache: list[str] = []
_FEED_BG_PATH = Path(__file__).resolve().parents[1] / "public" / "basebg.jpg"
_feed_bg_src: np.ndarray | None = None
_feed_bg_sized: dict[tuple[int, int], np.ndarray] = {}

_HUD_WHITE = (232, 234, 232)
_HUD_INFO = (199, 135, 98)
_HUD_OK = (131, 175, 95)
_HUD_ERR = (82, 74, 184)
_HUD_MUTE = (176, 180, 176)
_HUD_WARN = (90, 154, 184)
_HUD_LOCK = (70, 255, 90)  # BGR tactical green — lock box / seeker
_HUD_FONT = cv2.FONT_HERSHEY_SIMPLEX


def _load_feed_bg() -> np.ndarray | None:
    global _feed_bg_src
    if _feed_bg_src is not None:
        return _feed_bg_src
    img = cv2.imread(str(_FEED_BG_PATH))
    if img is None or img.size == 0:
        return None
    _feed_bg_src = img
    return _feed_bg_src


def default_feed_frame(width: int, height: int) -> np.ndarray:
    """Front-facing landscape placeholder used when HDMI is dark or missing."""
    w = max(320, int(width))
    h = max(180, int(height))
    key = (w, h)
    cached = _feed_bg_sized.get(key)
    if cached is not None:
        return cached.copy()
    src = _load_feed_bg()
    if src is None:
        frame = np.zeros((h, w, 3), dtype=np.uint8)
    else:
        frame = cv2.resize(src, (w, h), interpolation=cv2.INTER_AREA)
    _feed_bg_sized[key] = frame
    return frame.copy()


def _hud_panel(frame: np.ndarray, x: int, y: int, pw: int, ph: int) -> None:
    """Dark matte HUD plate — no bloom."""
    fh, fw = frame.shape[:2]
    x = int(np.clip(x, 0, max(0, fw - 1)))
    y = int(np.clip(y, 0, max(0, fh - 1)))
    x2 = int(np.clip(x + pw, 0, fw))
    y2 = int(np.clip(y + ph, 0, fh))
    roi = frame[y:y2, x:x2]
    if roi.size == 0:
        return
    overlay = roi.copy()
    overlay[:] = (28, 22, 18)
    cv2.addWeighted(overlay, 0.62, roi, 0.38, 0, roi)
    cv2.rectangle(frame, (x, y), (x2 - 1, y2 - 1), (58, 49, 44), 1)


def _hud_text(frame: np.ndarray, text: str, org: tuple[int, int], color: tuple[int, int, int],
              scale: float = 0.62, thick: int = 2) -> None:
    x, y = int(org[0]), int(org[1])
    outline = max(3, thick + 2)
    cv2.putText(frame, text, (x, y), _HUD_FONT, scale, (12, 10, 8), outline, cv2.LINE_AA)
    cv2.putText(frame, text, (x, y), _HUD_FONT, scale, color, thick, cv2.LINE_AA)


def _draw_corner_brackets(
    frame: np.ndarray,
    x: int,
    y: int,
    bw: int,
    bh: int,
    color: tuple[int, int, int],
    thickness: int = 2,
    tick: int | None = None,
) -> None:
    """Open targeting brackets — like a seeker lock box, not a filled card."""
    if bw < 8 or bh < 8:
        return
    tick = int(tick if tick is not None else max(18, min(bw, bh) * 0.34))
    tick = max(12, min(tick, bw // 2, bh // 2))
    x2, y2 = x + bw, y + bh
    t = max(2, int(thickness))
    cv2.line(frame, (x, y), (x + tick, y), color, t)
    cv2.line(frame, (x, y), (x, y + tick), color, t)
    cv2.line(frame, (x2, y), (x2 - tick, y), color, t)
    cv2.line(frame, (x2, y), (x2, y + tick), color, t)
    cv2.line(frame, (x, y2), (x + tick, y2), color, t)
    cv2.line(frame, (x, y2), (x, y2 - tick), color, t)
    cv2.line(frame, (x2, y2), (x2 - tick, y2), color, t)
    cv2.line(frame, (x2, y2), (x2, y2 - tick), color, t)


def _draw_scope_crosshair(
    frame: np.ndarray,
    cx: int,
    cy: int,
    w: int,
    h: int,
    color: tuple[int, int, int],
    thickness: int = 2,
) -> None:
    arm = int(min(w, h) * 0.38)
    gap = max(14, int(min(w, h) * 0.028))
    t = max(2, thickness)
    cv2.line(frame, (cx - arm, cy), (cx - gap, cy), color, t)
    cv2.line(frame, (cx + gap, cy), (cx + arm, cy), color, t)
    cv2.line(frame, (cx, cy - arm), (cx, cy - gap), color, t)
    cv2.line(frame, (cx, cy + gap), (cx, cy + arm), color, t)
    cv2.circle(frame, (cx, cy), 3, color, -1)
    tick = max(8, gap // 2)
    for dx in (-arm // 2, arm // 2):
        cv2.line(frame, (cx + dx, cy - tick), (cx + dx, cy + tick), color, 1)
    for dy in (-arm // 2, arm // 2):
        cv2.line(frame, (cx - tick, cy + dy), (cx + tick, cy + dy), color, 1)


def _paste_seeker_view(frame: np.ndarray, crop: np.ndarray, margin: int = 10) -> None:
    """PIP zoom, aspect-correct, pinned to the bottom-right of the video."""
    if crop is None or crop.size == 0:
        return
    fh, fw = frame.shape[:2]
    ch, cw = crop.shape[:2]
    if ch < 4 or cw < 4:
        return

    max_side = max(140, min(220, int(min(fw, fh) * 0.20)))
    scale = min(max_side / float(cw), max_side / float(ch))
    nw = max(8, int(round(cw * scale)))
    nh = max(8, int(round(ch * scale)))
    zoomed = cv2.resize(crop, (nw, nh), interpolation=cv2.INTER_LINEAR)

    x = max(2, fw - nw - margin)
    y = max(2, fh - nh - margin)
    frame[y:y + nh, x:x + nw] = zoomed
    cv2.rectangle(frame, (x - 1, y - 1), (x + nw, y + nh), _HUD_LOCK, 2)
    inset = max(6, min(12, nw // 16, nh // 16))
    _draw_corner_brackets(
        frame, x + inset, y + inset, nw - 2 * inset, nh - 2 * inset,
        _HUD_LOCK, 2, tick=max(10, min(16, nw // 8)),
    )
    scx, scy = x + nw // 2, y + nh // 2
    cv2.line(frame, (scx - 10, scy), (scx + 10, scy), _HUD_LOCK, 1)
    cv2.line(frame, (scx, scy - 10), (scx, scy + 10), _HUD_LOCK, 1)
    _hud_text(frame, "SEEKER VIEW", (x + 6, y + 18), _HUD_LOCK, 0.50, 2)


def _hud_measure(
    lines: list[str],
    scale: float = 0.60,
    pad: int = 10,
    line_gap: int = 8,
) -> tuple[int, int]:
    lines = [ln for ln in lines if ln]
    if not lines:
        return 0, 0
    sizes = [cv2.getTextSize(t, _HUD_FONT, scale, 1)[0] for t in lines]
    tw = max(s[0] for s in sizes)
    th = sum(s[1] for s in sizes) + line_gap * (len(lines) - 1)
    return tw + pad * 2, th + pad * 2


def _hud_block(
    frame: np.ndarray,
    lines: list[str],
    x: int,
    y: int,
    colors: list[tuple[int, int, int]] | None = None,
    scale: float = 0.60,
    pad: int = 10,
    line_gap: int = 8,
) -> tuple[int, int]:
    lines = [ln for ln in lines if ln]
    if not lines:
        return 0, 0
    sizes = [cv2.getTextSize(t, _HUD_FONT, scale, 1)[0] for t in lines]
    tw = max(s[0] for s in sizes)
    th = sum(s[1] for s in sizes) + line_gap * (len(lines) - 1)
    pw, ph = tw + pad * 2, th + pad * 2
    fh, fw = frame.shape[:2]
    x = int(np.clip(x, 8, max(8, fw - pw - 8)))
    y = int(np.clip(y, 8, max(8, fh - ph - 8)))
    _hud_panel(frame, x, y, pw, ph)
    cy = y + pad
    for i, t in enumerate(lines):
        baseline = cy + sizes[i][1]
        col = colors[i] if colors and i < len(colors) else _HUD_WHITE
        _hud_text(frame, t, (x + pad, baseline), col, scale)
        cy = baseline + line_gap
    return pw, ph


def _wrap_status(text: str, max_chars: int = 36) -> list[str]:
    raw = " ".join((text or "").split())
    if not raw:
        return ["No camera feed"]

    chunks: list[str] = []
    for part in raw.split(" — "):
        part = part.strip()
        if not part:
            continue
        if len(part) <= max_chars:
            chunks.append(part)
            continue
        words = part.replace(",", ", ").split()
        cur = ""
        for word in words:
            trial = f"{cur} {word}".strip()
            if len(trial) > max_chars and cur:
                chunks.append(cur)
                cur = word
            else:
                cur = trial
        if cur:
            chunks.append(cur)
    return chunks[:4] or ["No camera feed"]


def list_camera_devices(max_test: int = 6) -> list[tuple[str, str]]:
    """List video devices without opening them.

    Each item is (directshow_name, label). Opening DirectShow here steals the
    live capture and resets the USB hub, which drops the telemetry COM port.
    USB capture cards are listed first.
    """
    # Do not spawn PowerShell here. That call blocked the UI for several
    # seconds and the tracking thread. The goggles card has a fixed name.
    names = list(_device_name_cache) or [_GOGGLES_CAPTURE]
    if _GOGGLES_CAPTURE not in names:
        names.insert(0, _GOGGLES_CAPTURE)
    if not names:
        return [(_GOGGLES_CAPTURE, f"USB capture — {_GOGGLES_CAPTURE}")]
    ranked = sorted(names, key=lambda name: (0 if _is_capture_card(name) else 1, name.lower()))
    devices: list[tuple[str, str]] = []
    for name in ranked[:max_test]:
        label = f"USB capture — {name}" if _is_capture_card(name) else name
        devices.append((name, label))
    return devices


def _is_capture_card(name: str) -> bool:
    low = name.lower()
    return any(hint in low for hint in _CAPTURE_HINT)


def _windows_video_names(max_test: int) -> list[str]:
    """Friendly names of video-capture devices. Does not open the capture pin."""
    if not hasattr(cv2, "CAP_DSHOW"):
        return []
    try:
        import subprocess

        script = (
            "$ErrorActionPreference='SilentlyContinue'; "
            "Get-PnpDevice -Class Camera -Status OK | "
            "Where-Object { $_.FriendlyName -and $_.FriendlyName -notmatch 'Audio|Proxy' } | "
            "Select-Object -ExpandProperty FriendlyName"
        )
        proc = subprocess.run(
            ["powershell", "-NoProfile", "-NonInteractive", "-Command", script],
            capture_output=True,
            text=True,
            timeout=15,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        )
    except Exception:
        return []
    names: list[str] = []
    for line in (proc.stdout or "").splitlines():
        name = line.strip()
        if name and name not in names:
            names.append(name)
        if len(names) >= max_test:
            break
    if names:
        _device_name_cache.clear()
        _device_name_cache.extend(names)
    return names or list(_device_name_cache)


class TrackingWorkerThread(QThread):
    frame_processed = pyqtSignal(np.ndarray, object)
    target_changed = pyqtSignal(object)
    fps_updated = pyqtSignal(float)
    follow_changed = pyqtSignal(bool)

    def __init__(
        self,
        sys_config: SystemConfig,
        target_store: TargetStore | None = None,
        joystick_mgr: JoystickManager | None = None,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self.sys_config = sys_config
        self.target_store = target_store or TargetStore()
        self.joystick_mgr = joystick_mgr
        self.sys_log = SystemLogger()
        self.running = False
        self.assist_enabled = False
        self._joy_follow_level = False
        self._joy_follow_seen = False
        self.arm_requested = False
        self.gui_arm_requested = False
        self.mode_requested = False

        self.fc = MAVLinkController()
        self.is_connected = False
        self.port_name = ""
        self.baud_rate = 115200

        self.requested_cam_idx: int | str | None = _GOGGLES_CAPTURE
        self.active_cam_idx: int = self.sys_config.camera.camera_index
        self.active_cam_name: str = ""
        self.camera_status: str = f"Opening {_GOGGLES_CAPTURE}…"
        self.camera_live: bool = False

        self.hybrid = HybridYoloLockTracker(
            det_cfg=self.sys_config.detection,
            tracker_cfg=self.sys_config.tracker,
        )
        self.controller = FPVFollowController(self.sys_config)
        self.failsafe = FailsafeManager(self.sys_config.safety)
        self.logger = TelemetryLogger()

        self.pending_roi: tuple[int, int, int, int] | None = None
        self.pending_auto_lock = False
        self.frame_count = 0
        self.active_target: TargetProfile | None = None
        self._was_locked = False
        self._fps_window: list[float] = []
        self.throttle_value: int = 1000
        self.flight_mode: str = "ANGLE"
        self.follow_status: str = "IDLE"  # Why sticks are / aren't AI-driven (HUD)
        self._joy_lock_state: bool = False
        self._joy_follow_level: bool = False

    def adjust_throttle(self, delta: int) -> int:
        self.throttle_value = max(1000, min(2000, self.throttle_value + delta))
        self.sys_log.log(
            LogCategory.DRONE,
            f"Throttle updated: {self.throttle_value} µs",
            module="Manual Control",
        )
        return self.throttle_value

    def set_throttle(self, value: int) -> int:
        self.throttle_value = max(1000, min(2000, value))
        self.sys_log.log(
            LogCategory.DRONE,
            f"Throttle set: {self.throttle_value} µs",
            module="Manual Control",
        )
        return self.throttle_value

    def toggle_flight_mode(self) -> str:
        self.flight_mode = "ACRO" if self.flight_mode == "ANGLE" else "ANGLE"
        if hasattr(self, 'fc') and hasattr(self.fc, 'set_flight_mode'):
            self.fc.set_flight_mode(self.flight_mode)
        ch6_val = 1900 if self.flight_mode == "ANGLE" else 1000
        self.sys_log.log(
            LogCategory.DRONE,
            f"Flight mode toggled to: {self.flight_mode} (Ch6 = {ch6_val} µs)",
            module="Manual Control",
        )
        return self.flight_mode

    def set_follow(self, enabled: bool, source: str = "gui") -> None:
        """AI may drive sticks only after an explicit Follow. Lock alone does nothing."""
        enabled = bool(enabled)
        changed = self.assist_enabled != enabled
        self.assist_enabled = enabled
        self.follow_status = "FOLLOW ON" if enabled else "FOLLOW OFF"
        if not enabled:
            self.controller.fade_to_mid(base_throttle=self.throttle_value)
        if changed:
            self.follow_changed.emit(enabled)
            self.sys_log.log(
                LogCategory.DRONE,
                f"Follow {'enabled' if enabled else 'disabled'} ({source})",
                module="Manual Control",
            )

    def arm_drone(self) -> bool:
        self.gui_arm_requested = True
        self.force_disarm = False
        self.arm_requested = True
        self.sys_log.log(LogCategory.DRONE, "ARM requested (GUI)", module="Manual Control")
        return True

    def disarm_drone(self) -> bool:
        self.gui_arm_requested = False
        self.arm_requested = False
        self.force_disarm = True
        self.throttle_value = 1000  # Reset throttle to safety min on disarm
        self.sys_log.log(LogCategory.DRONE, "DISARM requested (GUI Override) - Throttle reset to 1000", module="Manual Control")
        return False

    @property
    def current_fps(self) -> float:
        if not self._fps_window:
            return 0.0
        return sum(self._fps_window) / len(self._fps_window)

    def switch_camera(self, cam_index: int | str) -> None:
        """Switch video source. Do not reopen the HDMI card — that resets USB."""
        name = str(cam_index).strip()
        if (
            self.active_cam_name
            and _is_capture_card(self.active_cam_name)
            and (not name.isdigit() or _is_capture_card(name))
        ):
            return
        self.requested_cam_idx = cam_index

    def connect_serial(self, port_name: str, baud_rate: int = 115200) -> tuple[bool, str]:
        port_name = (port_name or "").strip()
        if not port_name:
            return False, "No COM port selected."
        if is_capture_card_port(port_name):
            return False, (
                f"{port_name} is the HDMI capture card, not telemetry.\n"
                "Opening it stops the goggles feed. Pick the STM / CP210 / CH340 COM port."
            )

        self.disconnect_serial()

        # Attempt 1: Try MSPController (for Betaflight / INAV / MSP)
        try:
            aux = self.sys_config.aux_channels
            msp = MSPController(
                port=port_name,
                baudrate=baud_rate,
                arm_channel=aux.arm_channel,
                mode_channel=aux.mode_channel,
                arm_high=aux.arm_high,
                arm_low=aux.arm_low,
                mode_high=aux.mode_high,
                mode_low=aux.mode_low,
            )
            if msp.connect():
                self.fc = msp
                self.is_connected = True
                self.port_name = port_name
                self.baud_rate = baud_rate
                self.sys_config.device.serial_port = port_name
                self.sys_config.device.baud_rate = baud_rate
                self.sys_log.log(
                    LogCategory.DRONE,
                    f"Connected to {port_name} @ {baud_rate} (MSP)",
                    module="MSP Link",
                )
                return True, f"Connected to {port_name} @ {baud_rate} (MSP)"
        except Exception as e:
            self.sys_log.log(
                LogCategory.DRONE,
                f"MSP connection attempt on {port_name} failed: {e}",
                severity=LogSeverity.WARNING,
                module="FC Link",
            )

        # Attempt 2: Try MAVLinkController (for ArduPilot / PX4 / MAVLink)
        try:
            aux = self.sys_config.aux_channels
            mav = MAVLinkController(connection_string=port_name, baudrate=baud_rate)
            if mav.connect():
                if hasattr(mav, "update_aux_config"):
                    mav.update_aux_config(
                        arm_channel=aux.arm_channel,
                        mode_channel=aux.mode_channel,
                        arm_high=aux.arm_high,
                        arm_low=aux.arm_low,
                        mode_high=aux.mode_high,
                        mode_low=aux.mode_low,
                    )
                self.fc = mav
                self.is_connected = True
                self.port_name = port_name
                self.baud_rate = baud_rate
                self.sys_config.device.serial_port = port_name
                self.sys_config.device.baud_rate = baud_rate
                self.sys_log.log(
                    LogCategory.DRONE,
                    f"Connected to {port_name} @ {baud_rate} (MAVLink)",
                    module="MAVLink Link",
                )
                return True, f"Connected to {port_name} @ {baud_rate} (MAVLink)"
        except Exception as e:
            self.sys_log.log(
                LogCategory.DRONE,
                f"MAVLink connection attempt on {port_name} failed: {e}",
                severity=LogSeverity.ERROR,
                module="FC Link",
            )

        self.is_connected = False
        return False, f"Could not connect to flight controller on {port_name}"

    def disconnect_serial(self) -> None:
        if hasattr(self, 'fc') and self.fc.is_connected():
            self.fc.disconnect()
        self.is_connected = False

    def update_config(self, cfg: SystemConfig) -> None:
        self.sys_config = cfg
        self.controller.update_sys_config(cfg)
        self.failsafe.update_config(cfg.safety)
        if hasattr(self, "hybrid") and hasattr(self.hybrid, "apply_detection_config"):
            self.hybrid.apply_detection_config(cfg.detection)
            self.hybrid.tcfg = cfg.tracker
        # Keep MSP ARM/Mode channel mapping in sync with Settings → AUX
        if hasattr(self, "fc") and hasattr(self.fc, "update_aux_config"):
            aux = cfg.aux_channels
            self.fc.update_aux_config(
                arm_channel=aux.arm_channel,
                mode_channel=aux.mode_channel,
                arm_high=aux.arm_high,
                arm_low=aux.arm_low,
                mode_high=aux.mode_high,
                mode_low=aux.mode_low,
            )

    def set_roi_lock(self, x: int, y: int, w: int, h: int) -> None:
        self.pending_roi = (x, y, w, h)

    def trigger_auto_lock(self) -> None:
        self.pending_auto_lock = True

    def set_target_type(self, kind: str) -> None:
        kind = (kind or "auto").lower()
        if kind not in ("auto", "aerial", "ground"):
            kind = "auto"
        self.sys_config.detection.target_type = kind  # type: ignore[assignment]
        if hasattr(self, "hybrid") and hasattr(self.hybrid, "apply_detection_config"):
            self.hybrid.apply_detection_config(self.sys_config.detection)

    def reset_lock(self) -> None:
        if self.active_target:
            self.target_store.finish_target(self.active_target)
            self.sys_log.log(
                LogCategory.TRACKING,
                f"Target {self.active_target.target_id} session finished",
                target_id=self.active_target.target_id,
            )
            self.active_target = None
            self.target_changed.emit(None)
        self.hybrid.reset()
        self.controller.reset()
        self.failsafe.reset()
        self._was_locked = False

    def _start_target_lock(
        self,
        frame: np.ndarray,
        bbox: tuple[int, int, int, int],
        source: str,
        label: str = "unknown",
    ) -> None:
        if self.active_target:
            self.target_store.finish_target(self.active_target)
        profile = self.target_store.create_target(label=label)
        profile.confidence = self.hybrid._conf if hasattr(self.hybrid, "_conf") else 0.0
        self.target_store.lock_target(profile, frame, bbox, source=source)
        profile.add_event("Features Generated", system_response="Color histogram + optical flow initialized")
        profile.add_event("Tracking Started", system_response="Hybrid tracker active")
        self.active_target = profile
        self.target_changed.emit(profile)
        width_m = float(getattr(self.hybrid, "known_width_m", 0.30) or 0.30)
        self.controller.distance_estimator.set_target_width_m(width_m)
        self.sys_log.log(
            LogCategory.TRACKING,
            f"Target locked: {profile.target_id} via {source} family={getattr(self.hybrid, 'family', '')} width={width_m:.2f}m",
            target_id=profile.target_id,
        )

    def _open_camera(self, cam_idx: int | str) -> cv2.VideoCapture | None:
        """Open only the HDMI grabber. Never scan other USB cameras.

        Opening webcam / IR / extra capture indexes on a shared hub resets the
        USB3.0 UHD card and kills the goggles feed when telemetry is plugged in.
        """
        name = str(cam_idx).strip()
        if name.isdigit():
            index = int(name)
        else:
            index = int(self.sys_config.camera.camera_index)
        if index <= 0:
            index = 1

        label = _GOGGLES_CAPTURE
        self.camera_status = f"Opening {label}…"
        cap = cv2.VideoCapture(index, cv2.CAP_MSMF)
        if not cap.isOpened() and index != 1:
            cap.release()
            index = 1
            cap = cv2.VideoCapture(index, cv2.CAP_MSMF)
        if not cap.isOpened():
            cap.release()
            self.camera_status = f"{label} — close OBS, then restart (card is in use)"
            self.sys_log.log(
                LogCategory.CAMERA,
                f"Could not open USB capture index {index} ({label})",
                severity=LogSeverity.ERROR,
            )
            return None
        try:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.sys_config.camera.frame_width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.sys_config.camera.frame_height)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        self.active_cam_idx = index
        self.active_cam_name = label
        self.sys_config.camera.camera_index = index
        self.camera_status = f"{label} — waiting for goggles HDMI"
        self.sys_log.log(
            LogCategory.CAMERA,
            f"Opened USB capture card {label} (MSMF index {index})",
        )
        return cap

    def run(self) -> None:
        self.running = True
        if hasattr(cv2, "setLogLevel"):
            cv2.setLogLevel(cv2.LOG_LEVEL_SILENT)

        # Open the HDMI capture card by name. Do not call PowerShell or
        # VideoCapture(index) here — both freeze the UI and drop the radio.
        source = self.requested_cam_idx if self.requested_cam_idx is not None else _GOGGLES_CAPTURE
        if str(source).isdigit():
            source = _GOGGLES_CAPTURE
        cap = self._open_camera(source)
        self.requested_cam_idx = None

        self.sys_log.log(LogCategory.SYSTEM, "Tracking pipeline started", module="Worker")
        
        # Load YOLO model into VRAM on startup so the first lock is instant
        try:
            self.sys_log.log(LogCategory.SYSTEM, "Pre-loading YOLO weights into VRAM...", module="Worker")
            self.hybrid.ensure_detector()
            self.sys_log.log(LogCategory.SYSTEM, "YOLO loaded into memory successfully.", module="Worker")
        except Exception as e:
            self.sys_log.log(LogCategory.SYSTEM, f"Failed to pre-load YOLO: {e}", severity=LogSeverity.ERROR, module="Worker")

        last_time = time.time()
        last_msp_send = 0.0
        msp_interval = 1.0 / 50.0
        last_att_poll = 0.0
        att_interval = 1.0 / 10.0
        failed_reads = 0

        while self.running:
            frame_start = time.time()

            if self.requested_cam_idx is not None:
                new_idx = self.requested_cam_idx
                self.requested_cam_idx = None
                # Closing a USB capture resets the hub and drops the telemetry radio.
                same = str(new_idx) in (str(self.active_cam_idx), self.active_cam_name)
                already = cap is not None and cap.isOpened() and same
                if not already:
                    if cap is not None and cap.isOpened():
                        cap.release()
                    cap = self._open_camera(new_idx)
                    failed_reads = 0
                    self.sys_log.log(LogCategory.CAMERA, f"Opened USB video: {new_idx}")

            ok = False
            frame = None
            if cap is not None and cap.isOpened():
                ok, frame = cap.read()
                if ok and frame is not None:
                    failed_reads = 0
                    self.camera_status = self.active_cam_name or "USB capture"
                else:
                    # No HDMI picture. Keep the device open so COM stays up.
                    failed_reads += 1

            if not ok or frame is None:
                # Black HDMI is normal (goggles off). Do not reopen — that
                # scans USB and drops telemetry + the capture card together.
                if self.active_cam_name:
                    self.camera_status = f"{self.active_cam_name} — waiting for goggles HDMI"
                self.camera_live = False
                fw = int(getattr(self.sys_config.camera, "frame_width", 1280) or 1280)
                fh = int(getattr(self.sys_config.camera, "frame_height", 720) or 720)
                frame = default_feed_frame(fw, fh)
                time.sleep(0.2)
            else:
                self.camera_live = True

            self.frame_count += 1
            now = time.time()
            dt = max(0.001, now - last_time)
            last_time = now

            if ok and frame is not None:
                # Apply Gaussian Blur to reduce noise and enhance tracking/detection visibility
                frame = cv2.GaussianBlur(frame, (3, 3), 0)

            h, w = frame.shape[:2]
            cx, cy = w // 2, h // 2

            try:
                if self.pending_roi is not None:
                    rx, ry, rw, rh = self.pending_roi
                    self.hybrid.lock_xywh(frame, (rx, ry, rw, rh), label="manual")
                    self.controller.reset()
                    self._start_target_lock(frame, (rx, ry, rw, rh), "manual", "manual")
                    self.pending_roi = None

                if self.pending_auto_lock:
                    best = self.hybrid.lock_best(frame)
                    if best and self.hybrid._bbox:
                        self._start_target_lock(
                            frame, self.hybrid._bbox, "yolo", self.hybrid._label or "yolo"
                        )
                    self.controller.reset()
                    self.pending_auto_lock = False

                dets = []
                locked = False
                bbox = None
                conf = 0.0
                source = "none"

                if ok and self.hybrid.locked:
                    res = self.hybrid.update(frame)
                    dets = res.detections
                    if res.ok and res.bbox_xywh is not None:
                        locked = True
                        bbox = res.bbox_xywh
                        conf = res.conf
                        source = res.source

                # Local ref — UI unlock can clear self.active_target mid-frame
                target = self.active_target
                if locked and target is not None:
                    vx, vy = self.controller.motion_predictor.kalman.get_velocity()
                    color_hist = None
                    # Hist flatten is expensive — only every few frames
                    if self.frame_count % 10 == 0 and self.hybrid._target_hist is not None:
                        color_hist = self.hybrid._target_hist.flatten().tolist()[:64]
                    dist_m = 0.0
                    if self.controller.last_distance:
                        dist_m = self.controller.last_distance.distance_m
                    target.update_frame(
                        bbox_xywh=bbox,
                        confidence=conf,
                        source=source,
                        vx=vx,
                        vy=vy,
                        distance_m=dist_m,
                        color_hist=color_hist,
                    )
                    if self.frame_count % 3 == 0:
                        self.target_store.update_active(target, frame, bbox_xywh=bbox)
                    if self.assist_enabled and not self._was_locked:
                        target.add_event(
                            "Drone Following",
                            confidence=conf,
                            drone_state="assist_enabled",
                            system_response="FPV follow controller active",
                        )
                elif self._was_locked and not locked and target is not None:
                    self.target_store.mark_lost(target)
                    self.sys_log.log(
                        LogCategory.TRACKING,
                        f"Target {target.target_id} temporarily lost",
                        severity=LogSeverity.WARNING,
                        target_id=target.target_id,
                    )
                elif not self._was_locked and locked and target is not None:
                    if target.status == TargetStatus.LOST:
                        self.target_store.mark_reacquired(target)

                self._was_locked = locked

                # Attitude only — never get_telemetry() here. That call blocks the
                # MSP UART (several round-trips) and overwrites fc._armed from
                # STATUS, which then sends an explicit disarm packet mid-air.
                if (
                    self.sys_config.camera.stabilize_with_attitude
                    and self.is_connected
                    and now - last_att_poll >= att_interval
                    and hasattr(self, "fc")
                    and hasattr(self.fc, "get_attitude")
                ):
                    last_att_poll = now
                    try:
                        att = self.fc.get_attitude() or {}
                        if att:
                            self.controller.set_vehicle_attitude(
                                float(att.get("roll_deg", 0.0)),
                                float(att.get("pitch_deg", 0.0)),
                                float(att.get("yaw_deg", 0.0)),
                            )
                    except Exception:
                        pass

                dist_m = 0.0
                if self.controller.last_distance:
                    dist_m = self.controller.last_distance.distance_m

                safety_state = self.failsafe.evaluate(
                    locked,
                    conf,
                    dist_m if locked else None,
                    min_safe_distance_m=self.sys_config.distance.min_safe_distance_m,
                )

                roll, pitch, yaw, throttle = 1500, 1500, 1500, self.throttle_value
                channel_overrides: dict[int, int] = {}
                joy_wants_arm = False
                joy_connected = False
                js_roll = js_pitch = js_yaw = js_thr = None

                # Always poll joystick in the control loop (Joystick page is not required)
                if self.sys_config.joystick.enabled and self.joystick_mgr is not None:
                    self.joystick_mgr.poll()

                if self.sys_config.joystick.enabled and self.joystick_mgr and self.joystick_mgr.state.connected:
                    joy_connected = True
                    js = self.joystick_mgr.state
                    js_roll, js_pitch, js_yaw, js_thr = js.roll_pwm, js.pitch_pwm, js.yaw_pwm, js.throttle_pwm

                    used_rc: set[int] = set()
                    for i, aux_cfg in enumerate(self.sys_config.joystick.aux_channels):
                        rc = int(aux_cfg.rc_channel)
                        if rc < 0 or rc > 15 or rc in used_rc:
                            continue
                        used_rc.add(rc)
                        pwm = js.aux_pwm.get(
                            f"#{i}",
                            js.aux_pwm.get(aux_cfg.name, aux_cfg.center_val),
                        )
                        channel_overrides[rc] = int(pwm)

                    aux_map = self.sys_config.aux_channels
                    lock_ch = int(getattr(aux_map, "lock_channel", 4))
                    follow_ch = int(getattr(aux_map, "follow_channel", 5))
                    lock_high = int(getattr(aux_map, "lock_high", 1900))
                    lock_low = int(getattr(aux_map, "lock_low", 1000))
                    follow_high = int(getattr(aux_map, "follow_high", 1900))
                    follow_low = int(getattr(aux_map, "follow_low", 1000))

                    lock_pwm = channel_overrides.get(lock_ch)
                    follow_pwm = channel_overrides.get(follow_ch)
                    follow_is_button = False
                    for i, aux_cfg in enumerate(self.sys_config.joystick.aux_channels):
                        n = (aux_cfg.name or "").lower()
                        rc = int(aux_cfg.rc_channel)
                        if rc == follow_ch or "follow" in n:
                            follow_is_button = bool(aux_cfg.is_button)
                            if follow_pwm is None:
                                follow_pwm = js.aux_pwm.get(f"#{i}", js.aux_pwm.get(aux_cfg.name))
                        if lock_pwm is None and (rc == lock_ch or "lock" in n):
                            lock_pwm = js.aux_pwm.get(f"#{i}", js.aux_pwm.get(aux_cfg.name))

                    # Lock / Follow are GCS-only. Do not send them as INAV flight modes.
                    for i, aux_cfg in enumerate(self.sys_config.joystick.aux_channels):
                        n = (aux_cfg.name or "").lower()
                        if "lock" in n or "follow" in n:
                            rc = int(aux_cfg.rc_channel)
                            if rc >= 0:
                                channel_overrides.pop(rc, None)
                    channel_overrides.pop(lock_ch, None)
                    channel_overrides.pop(follow_ch, None)

                    # Resolve Arm switch PWM from FC arm channel or named "Arm" AUX
                    arm_ch = int(self.sys_config.aux_channels.arm_channel)
                    arm_pwm = channel_overrides.get(arm_ch)
                    if arm_pwm is None:
                        arm_pwm = js.aux_pwm.get("Arm")
                    if arm_pwm is None:
                        # Fall back: any AUX whose name contains "arm"
                        for i, aux_cfg in enumerate(self.sys_config.joystick.aux_channels):
                            if "arm" in (aux_cfg.name or "").lower():
                                arm_pwm = js.aux_pwm.get(f"#{i}")
                                if arm_pwm is not None and int(aux_cfg.rc_channel) >= 0:
                                    channel_overrides[int(aux_cfg.rc_channel)] = int(arm_pwm)
                                break
                    
                    joy_wants_arm = False
                    if arm_pwm is not None:
                        pwm_val = int(arm_pwm)
                        if pwm_val > 1600:
                            joy_wants_arm = True
                        elif pwm_val < 1400:
                            joy_wants_arm = False

                    # Do not let a noisy/released AUX write ARM low while latched.
                    # make_rc_channels already forces the ARM channel from arm_requested.
                    channel_overrides.pop(arm_ch, None)

                    # Latch ARM until an explicit disarm (GUI / X). A released or
                    # glitchy stick must not drop CH7 (ARM) in flight.
                    if getattr(self, "force_disarm", False):
                        self.arm_requested = False
                        self.gui_arm_requested = False
                        if not joy_wants_arm:
                            self.force_disarm = False
                    elif joy_wants_arm or self.gui_arm_requested or self.arm_requested:
                        self.arm_requested = True
                        if joy_wants_arm:
                            self.gui_arm_requested = True

                    # Mode switch → flight mode
                    mode_ch = int(self.sys_config.aux_channels.mode_channel)
                    mode_pwm = channel_overrides.get(mode_ch, js.aux_pwm.get("Flight Mode"))
                    if mode_pwm is not None:
                        if int(mode_pwm) >= int(self.sys_config.aux_channels.mode_high) - 50:
                            self.flight_mode = "ANGLE"
                            if hasattr(self.fc, "set_flight_mode"):
                                self.fc.set_flight_mode("ANGLE")
                        elif int(mode_pwm) <= int(self.sys_config.aux_channels.mode_low) + 50:
                            self.flight_mode = "ACRO"
                            if hasattr(self.fc, "set_flight_mode"):
                                self.fc.set_flight_mode("ACRO")

                    # Lock switch (Settings → AUX lock channel / PWM)
                    if lock_pwm is None:
                        lock_pwm = js.aux_pwm.get("Lock")
                    joy_wants_lock = self._joy_lock_state
                    if lock_pwm is not None:
                        pwm_val = int(lock_pwm)
                        if pwm_val >= lock_high - 50:
                            joy_wants_lock = True
                        elif pwm_val <= lock_low + 50:
                            joy_wants_lock = False

                    if joy_wants_lock and not self._joy_lock_state:
                        self.trigger_auto_lock()
                    elif not joy_wants_lock and self._joy_lock_state:
                        self.reset_lock()
                    self._joy_lock_state = joy_wants_lock

                    # Follow AUX (Settings → AUX follow channel / PWM):
                    #   button  — press toggles Follow / Unfollow
                    #   switch  — high follows, low unfollows
                    if follow_pwm is None:
                        follow_pwm = js.aux_pwm.get("Follow")
                    if follow_pwm is not None:
                        high = int(follow_pwm) >= follow_high - 50
                        low = int(follow_pwm) <= follow_low + 50
                        if high or low:
                            if not self._joy_follow_seen:
                                self._joy_follow_level = high
                                self._joy_follow_seen = True
                            elif high != self._joy_follow_level:
                                self._joy_follow_level = high
                                if follow_is_button:
                                    if high:
                                        self.set_follow(not self.assist_enabled, source="joystick")
                                else:
                                    self.set_follow(high, source="joystick")
                else:
                    self.arm_requested = bool(self.gui_arm_requested)

                follow_min = float(self.sys_config.safety.follow_min_confidence)
                ai_follow = (
                    locked
                    and self.assist_enabled
                    and conf >= follow_min
                    and not safety_state.override_active
                )

                if ai_follow:
                    roll, pitch, yaw, throttle = self.controller.update(
                        bbox, w, h, base_throttle=self.throttle_value
                    )
                    # Hold / reverse-only when inside the soft proximity band
                    if not safety_state.allow_forward_motion:
                        mid = 1500
                        if self.controller.cfg.pitch_dir < 0:
                            pitch = max(int(pitch), mid)
                        else:
                            pitch = min(int(pitch), mid)
                    # Safety: large stick deflection from the remote overrides that axis
                    if joy_connected and js_roll is not None:
                        def _stick_or_ai(joy_v: int, ai_v: int, mid: int = 1500, dz: int = 70) -> int:
                            return int(joy_v) if abs(int(joy_v) - mid) > dz else int(ai_v)

                        roll = _stick_or_ai(js_roll, roll)
                        pitch = _stick_or_ai(js_pitch, pitch)
                        yaw = _stick_or_ai(js_yaw, yaw)
                        # Throttle: prefer pilot stick when raised above the GUI base
                        if abs(int(js_thr) - int(self.throttle_value)) > 80:
                            throttle = int(js_thr)
                    self.follow_status = f"FOLLOWING conf={conf * 100:.0f}%"
                elif joy_connected and js_roll is not None:
                    # Manual sticks from the remote; AUX/arm already applied above
                    roll, pitch, yaw, throttle = int(js_roll), int(js_pitch), int(js_yaw), int(js_thr)
                    self.controller.fade_to_mid(base_throttle=self.throttle_value)
                    if not locked:
                        self.follow_status = "MANUAL (no lock)"
                    elif not self.assist_enabled:
                        self.follow_status = "MANUAL (Follow OFF)"
                    elif conf < follow_min:
                        self.follow_status = f"MANUAL (conf {conf * 100:.0f}% < {follow_min * 100:.0f}%)"
                    elif safety_state.override_active:
                        self.follow_status = "MANUAL (override)"
                    else:
                        self.follow_status = "MANUAL"
                else:
                    roll, pitch, yaw, throttle = self.controller.fade_to_mid(
                        base_throttle=self.throttle_value
                    )
                    if not locked:
                        self.follow_status = "IDLE (no lock)"
                    elif not self.assist_enabled:
                        self.follow_status = "IDLE (Follow OFF — press Follow)"
                    elif conf < follow_min:
                        self.follow_status = f"IDLE (conf {conf * 100:.0f}% < {follow_min * 100:.0f}%)"
                    elif safety_state.override_active:
                        self.follow_status = "IDLE (override)"
                    else:
                        self.follow_status = "IDLE"

                if now - last_msp_send >= msp_interval:
                    last_msp_send = now
                    if self.is_connected and hasattr(self, "fc") and self.fc.is_connected():
                        if hasattr(self.fc, "set_channel_overrides"):
                            self.fc.set_channel_overrides(channel_overrides)
                        if hasattr(self.fc, "set_flight_mode"):
                            self.fc.set_flight_mode(self.flight_mode)
                        # Do not call fc.disarm()/arm() here. Those send a one-shot
                        # packet with throttle=1000, and disarm() drops the ARM channel.
                        # send_control already writes ARM from the latched flag.
                        if hasattr(self.fc, "_armed"):
                            self.fc._armed = bool(self.arm_requested)

                        self.fc.send_control(roll=roll, pitch=pitch, yaw=yaw, throttle=throttle)
                        if self.frame_count % 30 == 0:
                            self.sys_log.log(
                                LogCategory.DRONE,
                                f"RC -> R:{roll} P:{pitch} Y:{yaw} T:{throttle} | "
                                f"ARM:{'YES' if self.arm_requested else 'NO'} "
                                f"(gui={self.gui_arm_requested} joy={joy_wants_arm}) | AUX:{channel_overrides}",
                                module="FC Link",
                            )

                self._render_hud(
                    frame, locked, bbox, conf, source, safety_state,
                    roll, pitch, yaw, throttle, dist_m, w, h,
                )

                err_x = 0.0
                err_y = 0.0
                if bbox is not None:
                    err_x = (bbox[0] + bbox[2] * 0.5) - cx
                    err_y = (bbox[1] + bbox[3] * 0.5) - cy

                rec = self.logger.log(
                    frame_idx=self.frame_count,
                    locked=locked,
                    confidence=conf,
                    source=source,
                    error_x=err_x,
                    error_y=err_y,
                    bbox_xywh=bbox,
                    distance_m=dist_m,
                    vx=self.controller.motion_predictor.kalman.get_velocity()[0],
                    vy=self.controller.motion_predictor.kalman.get_velocity()[1],
                    roll=roll,
                    pitch=pitch,
                    yaw=yaw,
                    throttle=self.throttle_value,
                    failsafe=safety_state.reason,
                )

                frame_ms = (time.time() - frame_start) * 1000
                self._fps_window.append(1.0 / max(dt, 0.001))
                if len(self._fps_window) > 30:
                    self._fps_window.pop(0)
                fps = self.current_fps
                self.fps_updated.emit(fps)

                if self.frame_count % 60 == 0:
                    self.sys_log.log(
                        LogCategory.PERFORMANCE,
                        f"Pipeline running @ {fps:.1f} FPS",
                        fps=fps,
                        latency_ms=frame_ms,
                    )

                if ok or self.frame_count % 8 == 0:
                    self.frame_processed.emit(frame.copy(), rec)
            except Exception as exc:
                self.sys_log.log(
                    LogCategory.SYSTEM,
                    f"Frame processing error (continuing): {exc}",
                    severity=LogSeverity.ERROR,
                    module="TrackingWorker",
                )

            # Pace the loop — without this a fast machine (RTX) spins at 150–300+ FPS,
            # floods the Qt UI, and makes PID/dt jittery. MSP stays on its own 50 Hz timer.
            target_fps = float(getattr(self.sys_config.camera, "target_fps", 60.0) or 60.0)
            target_fps = max(15.0, min(120.0, target_fps))
            frame_budget = 1.0 / target_fps
            elapsed = time.time() - frame_start
            if elapsed < frame_budget:
                time.sleep(frame_budget - elapsed)

        self.disconnect_serial()
        if cap is not None and cap.isOpened():
            cap.release()

    def _render_hud(
        self,
        frame: np.ndarray,
        locked: bool,
        bbox: tuple[int, int, int, int] | None,
        conf: float,
        source: str,
        safety: object,
        roll: int,
        pitch: int,
        yaw: int,
        throttle: int,
        dist_m: float,
        w: int,
        h: int,
    ) -> None:
        cx, cy = w // 2, h // 2
        lock_color = _HUD_LOCK if locked else _HUD_WHITE
        _draw_scope_crosshair(frame, cx, cy, w, h, lock_color, 2)

        dz_px_x = int(w * 0.5 * self.sys_config.offsets.deadzone_norm)
        dz_px_y = int(h * 0.5 * self.sys_config.offsets.deadzone_norm)
        cv2.rectangle(frame, (cx - dz_px_x, cy - dz_px_y), (cx + dz_px_x, cy + dz_px_y), _HUD_WARN, 1)

        # ── Aim reference line ──
        cam = self.sys_config.camera
        mount_active = (
            abs(cam.mount_pitch_deg) > 0.5
            or abs(cam.mount_roll_deg) > 0.5
            or abs(cam.mount_yaw_deg) > 0.5
            or cam.stabilize_with_attitude
        )
        if mount_active and str(getattr(cam, "vertical_ref", "level")).startswith("level"):
            row, tilt_deg = level_reference_line(
                w, h, cam, self.sys_config.offsets,
                vehicle_roll_deg=self.controller.vehicle_roll_deg,
                vehicle_pitch_deg=self.controller.vehicle_pitch_deg,
                calibrated_focal_px=self.sys_config.distance.focal_length_px,
            )
            if -h < row < 2 * h:
                half_span = int(w * 0.32)
                dy_tilt = int(math.tan(math.radians(max(-80.0, min(80.0, tilt_deg)))) * half_span)
                p1 = (cx - half_span, int(np.clip(row - dy_tilt, -5, h + 5)))
                p2 = (cx + half_span, int(np.clip(row + dy_tilt, -5, h + 5)))
                cv2.line(frame, p1, p2, _HUD_INFO, 1, cv2.LINE_AA)
                aim_label = (
                    f"AIM {cam.desired_elevation_deg:+.0f}deg  TILT {cam.mount_pitch_deg:+.0f}deg"
                    + ("  LVL" if cam.stabilize_with_attitude else "")
                )
                aim_y = int(np.clip(int(row) - 12, 58, h - 96))
                _hud_text(frame, aim_label, (cx - half_span, aim_y), _HUD_INFO, 0.62, 2)

        # ── Interceptor telemetry (always visible) ──
        ctrl = self.controller
        tti_text = "---"
        closing_rate = 0.0
        tgt_speed = 0.0

        if hasattr(ctrl, "last_t_intercept") and ctrl.last_t_intercept is not None:
            tti_text = f"{ctrl.last_t_intercept:.1f}s"
        if hasattr(ctrl, "last_closing_rate"):
            closing_rate = ctrl.last_closing_rate
        if hasattr(ctrl, "last_kalman_3d_vel") and ctrl.last_kalman_3d_vel is not None:
            vt = ctrl.last_kalman_3d_vel
            tgt_speed = math.sqrt(vt[0]**2 + vt[1]**2 + vt[2]**2)

        # Interceptor gauge and PPN block only while Follow is on.
        if self.assist_enabled:
            gauge_x = w - 45
            gauge_top = 80
            gauge_bot = h - 100
            gauge_h = gauge_bot - gauge_top
            gauge_w = 18

            cv2.rectangle(frame, (gauge_x, gauge_top), (gauge_x + gauge_w, gauge_bot), (24, 19, 16), -1)
            cv2.rectangle(frame, (gauge_x, gauge_top), (gauge_x + gauge_w, gauge_bot), (58, 49, 44), 1)

            max_closing = 20.0
            fill_frac = max(0.0, min(1.0, closing_rate / max_closing))
            fill_h = int(gauge_h * fill_frac)
            if fill_h > 0:
                bar_color = (131, 175, 95) if closing_rate > 0 else (82, 74, 184)
                cv2.rectangle(
                    frame,
                    (gauge_x + 1, gauge_bot - fill_h),
                    (gauge_x + gauge_w - 1, gauge_bot),
                    bar_color, -1,
                )

            for tick_val in range(0, int(max_closing) + 1, 5):
                tick_y = gauge_bot - int(gauge_h * tick_val / max_closing)
                cv2.line(frame, (gauge_x - 4, tick_y), (gauge_x, tick_y), (180, 180, 180), 1)

            _hud_block(
                frame,
                [
                    "INTERCEPTOR",
                    f"TTI    {tti_text}",
                    f"TGT    {tgt_speed:.1f} m/s",
                    f"CLR    {closing_rate:+.1f} m/s",
                ],
                w - 250,
                10,
                [
                    _HUD_INFO,
                    _HUD_MUTE if tti_text == "---" else _HUD_ERR,
                    _HUD_INFO,
                    _HUD_OK if closing_rate > 0 else _HUD_ERR,
                ],
            )

        seeker_crop: np.ndarray | None = None

        # ═══════════════════════════════════════════════════════
        # TARGET LOCK BOX + INTERCEPTION VISUALS
        # ═══════════════════════════════════════════════════════
        if locked and bbox is not None:
            bx, by, bw, bh = [int(v) for v in bbox]
            obj_cx, obj_cy = bx + bw // 2, by + bh // 2
            axis = getattr(self.sys_config.distance, "size_axis", "max") or "max"
            size_px = bbox_size_px((float(bx), float(by), float(bw), float(bh)), axis)

            # Capture seeker crop before drawing lock graphics on the main frame.
            pad = max(12, int(max(bw, bh) * 0.45))
            x0 = max(0, bx - pad)
            y0 = max(0, by - pad)
            x1 = min(w, bx + bw + pad)
            y1 = min(h, by + bh + pad)
            seeker_crop = frame[y0:y1, x0:x1].copy() if y1 > y0 and x1 > x0 else None

            # Outer + inner targeting brackets (visible lock box)
            outer = max(10, int(min(bw, bh) * 0.14))
            _draw_corner_brackets(
                frame, bx - outer, by - outer, bw + 2 * outer, bh + 2 * outer,
                _HUD_LOCK, thickness=3, tick=max(22, int(min(bw, bh) * 0.38)),
            )
            _draw_corner_brackets(frame, bx, by, bw, bh, _HUD_LOCK, thickness=2)
            cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), _HUD_LOCK, 1)
            cv2.circle(frame, (obj_cx, obj_cy), 4, _HUD_LOCK, -1)
            cv2.circle(frame, (obj_cx, obj_cy), 10, _HUD_LOCK, 1)

            # Measured axis highlight
            if axis == "height":
                cv2.line(frame, (obj_cx, by), (obj_cx, by + bh), _HUD_LOCK, 2)
            elif axis == "diag":
                cv2.line(frame, (bx, by), (bx + bw, by + bh), _HUD_LOCK, 2)
            else:
                if axis == "width" or bw >= bh:
                    cv2.line(frame, (bx, obj_cy), (bx + bw, obj_cy), _HUD_LOCK, 2)
                else:
                    cv2.line(frame, (obj_cx, by), (obj_cx, by + bh), _HUD_LOCK, 2)

            # Interception point + steering lines only while Follow is on
            has_intercept = (
                self.assist_enabled
                and hasattr(ctrl, "last_intercept_pt")
                and ctrl.last_intercept_pt is not None
            )
            if has_intercept:
                int_x = int(ctrl.last_intercept_pt[0])
                int_y = int(ctrl.last_intercept_pt[1])
                # Predicted intercept diamond
                diamond_sz = 8
                pts = np.array([
                    [int_x, int_y - diamond_sz],
                    [int_x + diamond_sz, int_y],
                    [int_x, int_y + diamond_sz],
                    [int_x - diamond_sz, int_y],
                ], dtype=np.int32)
                cv2.polylines(frame, [pts], True, (82, 74, 184), 1, cv2.LINE_AA)
                cv2.circle(frame, (int_x, int_y), 2, (82, 74, 184), -1)

                # Steering line (crosshair → intercept)
                cv2.line(frame, (cx, cy), (int_x, int_y), (82, 74, 184), 1, cv2.LINE_AA)
                # Predicted path (target → intercept)
                cv2.line(frame, (obj_cx, obj_cy), (int_x, int_y), (199, 135, 98), 1, cv2.LINE_AA)

                # TTI label near intercept point
                if hasattr(ctrl, "last_t_intercept") and ctrl.last_t_intercept is not None:
                    _hud_text(frame, f"{ctrl.last_t_intercept:.1f}s",
                              (int_x + 14, int_y - 6), _HUD_ERR, 0.62, 2)
            else:
                cv2.line(frame, (cx, cy), (obj_cx, obj_cy), _HUD_LOCK, 2)

            tid = self.active_target.target_id if self.active_target else "---"
            lock_y = max(64, min(by - 16, h - 140))
            size_y = min(h - 110, by + bh + 26)
            dist_y = min(h - 86, by + bh + 52)
            _hud_text(frame, f"LOCK  [{tid}]  {source.upper()}  {conf * 100:.0f}%",
                      (bx, lock_y), _HUD_LOCK, 0.72, 2)
            _hud_text(frame, f"W {bw}px   H {bh}px   SIZE {size_px:.0f}px ({axis})",
                      (bx, size_y), _HUD_WHITE, 0.58, 2)
            _hud_text(frame, f"DIST  {dist_m:.2f} m", (bx, dist_y), _HUD_LOCK, 0.70, 2)

            brg = self.controller.last_bearing
            if brg is not None and mount_active:
                _hud_text(
                    frame,
                    f"AZ {math.degrees(brg.az_rad):+.1f}  EL {math.degrees(brg.el_rad):+.1f}  "
                    f"LOS {brg.slant_m:.2f}m  dALT {brg.vertical_m:+.2f}m",
                    (bx, min(h - 64, by + bh + 78)),
                    _HUD_INFO,
                    0.56,
                    2,
                )

        # ═══════════════════════════════════════════════════════
        # STATUS / CAMERA / INSTRUMENT STRIP
        # ═══════════════════════════════════════════════════════
        reason = str(getattr(safety, "reason", "—") or "—")
        status_lines = [f"STATUS  {reason}"] if len(reason) <= 44 else ["STATUS", reason]
        header_color = _HUD_OK if getattr(safety, "is_safe", False) else _HUD_ERR
        _hud_block(frame, status_lines, 12, 10, [header_color] * len(status_lines), scale=0.62)

        if not getattr(self, "camera_live", True):
            cam_lines = _wrap_status(self.camera_status or "No camera feed")
            pw, ph = _hud_measure(cam_lines, scale=0.64)
            _hud_block(
                frame,
                cam_lines,
                (w - pw) // 2,
                (h - ph) // 2 - 8,
                [_HUD_WARN] * len(cam_lines),
                scale=0.64,
            )

        cam_name = self.active_cam_name or str(self.active_cam_idx)
        mode = "FOLLOW" if self.assist_enabled else "LOCKED ONLY"
        serial = "OK" if self.is_connected else "OFF"
        ctrl_txt = str(getattr(self, "follow_status", "IDLE") or "IDLE")
        assist = "ON" if self.assist_enabled else "OFF"
        follow_color = _HUD_OK if ctrl_txt.startswith("FOLLOWING") else _HUD_INFO
        bottom = [
            f"CTRL  {ctrl_txt}    ASSIST {assist}",
            f"CAM  {cam_name}    FPS {self.current_fps:.0f}    {mode}    SERIAL {serial}",
        ]
        _, bph = _hud_measure(bottom, scale=0.60)
        _hud_block(frame, bottom, 12, h - bph - 12, [follow_color, _HUD_WHITE], scale=0.60)

        if seeker_crop is not None:
            _paste_seeker_view(frame, seeker_crop, margin=10)

    def stop(self) -> None:
        self.running = False
        self.wait()
