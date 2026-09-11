"""Real-time tracking worker thread with target database integration."""

from __future__ import annotations

import math
import time

import cv2
import numpy as np
import serial
from PyQt6.QtCore import QObject, QThread, pyqtSignal

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
        """Switch video source. Name opens that USB device only — no hub probe."""
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
        self.sys_log.log(
            LogCategory.TRACKING,
            f"Target locked: {profile.target_id} via {source}",
            target_id=profile.target_id,
        )

    def _try_msmf(self, index: int) -> cv2.VideoCapture | None:
        cap = cv2.VideoCapture(index, cv2.CAP_MSMF)
        if not cap.isOpened():
            cap.release()
            return None
        try:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.sys_config.camera.frame_width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.sys_config.camera.frame_height)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        return cap

    def _capture_score(self, cap: cv2.VideoCapture) -> int:
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
        if w < 640 or h < 360:
            return 0
        score = w * h
        # HDMI grabbers are 720p/1080p; laptop webcams are usually smaller.
        if w >= 1280 and h >= 720:
            score += 10_000_000
        return score

    def _open_camera(self, cam_idx: int | str) -> cv2.VideoCapture | None:
        """Open the goggles HDMI card via Media Foundation.

        Do not assume MSMF index 1. Plugging in telemetry shifts Windows
        camera indices, so index 1 can become a webcam / IR cam. Pick the
        HDMI-sized device instead. DirectShow-by-name fails on this card.
        """
        name = str(cam_idx).strip()
        want_capture = (not name.isdigit()) or _is_capture_card(name)
        preferred = int(self.sys_config.camera.camera_index)
        if name.isdigit():
            preferred = int(name)

        self.camera_status = f"Opening {_GOGGLES_CAPTURE if want_capture else name}…"
        order: list[int] = []
        for i in (preferred, 1, 2, 3, 4, 5):
            if 1 <= i <= 9 and i not in order:
                order.append(i)

        best_cap: cv2.VideoCapture | None = None
        best_idx = preferred
        best_score = -1
        for index in order:
            cap = self._try_msmf(index)
            if cap is None:
                continue
            score = self._capture_score(cap)
            if want_capture and score >= 10_000_000:
                if best_cap is not None:
                    best_cap.release()
                best_cap, best_idx, best_score = cap, index, score
                break
            if score > best_score:
                if best_cap is not None:
                    best_cap.release()
                best_cap, best_idx, best_score = cap, index, score
            else:
                cap.release()

        # Index 0 is usually the laptop webcam. Opening it can reset USB, so
        # only try it if no other video device looked like an HDMI grabber.
        if best_cap is None or (want_capture and best_score < 10_000_000):
            cap0 = self._try_msmf(0)
            if cap0 is not None:
                score0 = self._capture_score(cap0)
                if score0 > best_score:
                    if best_cap is not None:
                        best_cap.release()
                    best_cap, best_idx, best_score = cap0, 0, score0
                else:
                    cap0.release()

        if best_cap is None:
            self.camera_status = f"{_GOGGLES_CAPTURE} — close OBS, then restart (card is in use)"
            self.sys_log.log(
                LogCategory.CAMERA,
                "Could not open USB capture card (no MSMF device)",
                severity=LogSeverity.ERROR,
            )
            return None

        label = _GOGGLES_CAPTURE if want_capture or best_score >= 10_000_000 else name
        self.active_cam_idx = best_idx
        self.active_cam_name = label
        self.sys_config.camera.camera_index = best_idx
        self.camera_status = f"{label} — waiting for goggles HDMI"
        self.sys_log.log(
            LogCategory.CAMERA,
            f"Opened USB capture card {label} (MSMF index {best_idx})",
        )
        return best_cap

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
                # USB re-plug of telemetry can invalidate the MSMF handle.
                # Only re-open if the device itself died — black HDMI is normal.
                if cap is None or not cap.isOpened():
                    if cap is not None:
                        try:
                            cap.release()
                        except Exception:
                            pass
                    cap = self._open_camera(_GOGGLES_CAPTURE)
                    failed_reads = 0
                if self.active_cam_name:
                    self.camera_status = f"{self.active_cam_name} — waiting for goggles HDMI"
                frame = np.zeros((360, 640, 3), dtype=np.uint8)
                cv2.putText(
                    frame,
                    self.camera_status or "USB capture — waiting for goggles HDMI",
                    (24, 180),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 180, 255),
                    1,
                    cv2.LINE_AA,
                )
                time.sleep(0.2)

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

                    # Lock switch
                    lock_pwm = js.aux_pwm.get("Lock")
                    if lock_pwm is None:
                        for i, aux_cfg in enumerate(self.sys_config.joystick.aux_channels):
                            if "lock" in (aux_cfg.name or "").lower():
                                lock_pwm = js.aux_pwm.get(f"#{i}")
                                break
                    joy_wants_lock = self._joy_lock_state
                    if lock_pwm is not None:
                        pwm_val = int(lock_pwm)
                        if pwm_val > 1600:
                            joy_wants_lock = True
                        elif pwm_val < 1400:
                            joy_wants_lock = False
                        
                    if joy_wants_lock and not self._joy_lock_state:
                        self.trigger_auto_lock()
                    elif not joy_wants_lock and self._joy_lock_state:
                        self.reset_lock()
                    self._joy_lock_state = joy_wants_lock

                    # Follow AUX:
                    #   button  — press toggles Follow / Unfollow
                    #   switch  — high follows, low unfollows
                    # Either change updates the GUI Follow button.
                    follow_pwm = None
                    follow_is_button = True
                    for i, aux_cfg in enumerate(self.sys_config.joystick.aux_channels):
                        if "follow" in (aux_cfg.name or "").lower():
                            follow_pwm = js.aux_pwm.get(f"#{i}", js.aux_pwm.get(aux_cfg.name))
                            follow_is_button = bool(aux_cfg.is_button)
                            break
                    if follow_pwm is None:
                        follow_pwm = js.aux_pwm.get("Follow")
                    if follow_pwm is not None:
                        high = int(follow_pwm) > 1600
                        low = int(follow_pwm) < 1400
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

        # ── Crosshair ──
        cv2.drawMarker(frame, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 30, 2)

        dz_px_x = int(w * 0.5 * self.sys_config.offsets.deadzone_norm)
        dz_px_y = int(h * 0.5 * self.sys_config.offsets.deadzone_norm)
        cv2.rectangle(frame, (cx - dz_px_x, cy - dz_px_y), (cx + dz_px_x, cy + dz_px_y), (255, 255, 0), 1)

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
                cv2.line(frame, p1, p2, (0, 200, 255), 1, cv2.LINE_AA)
                cv2.putText(
                    frame,
                    f"AIM {cam.desired_elevation_deg:+.0f}deg  TILT {cam.mount_pitch_deg:+.0f}deg"
                    + ("  LVL" if cam.stabilize_with_attitude else ""),
                    (cx - half_span, max(14, min(h - 6, int(row) - 8))),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1, cv2.LINE_AA,
                )

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

            cv2.rectangle(frame, (gauge_x, gauge_top), (gauge_x + gauge_w, gauge_bot), (40, 40, 40), -1)
            cv2.rectangle(frame, (gauge_x, gauge_top), (gauge_x + gauge_w, gauge_bot), (120, 120, 120), 1)

            max_closing = 20.0
            fill_frac = max(0.0, min(1.0, closing_rate / max_closing))
            fill_h = int(gauge_h * fill_frac)
            if fill_h > 0:
                bar_color = (0, 220, 80) if closing_rate > 0 else (0, 0, 220)
                cv2.rectangle(
                    frame,
                    (gauge_x + 1, gauge_bot - fill_h),
                    (gauge_x + gauge_w - 1, gauge_bot),
                    bar_color, -1,
                )

            for tick_val in range(0, int(max_closing) + 1, 5):
                tick_y = gauge_bot - int(gauge_h * tick_val / max_closing)
                cv2.line(frame, (gauge_x - 4, tick_y), (gauge_x, tick_y), (180, 180, 180), 1)

            info_x = w - 220
            info_y = 28
            line_h = 22
            cv2.putText(frame, "INTERCEPTOR", (info_x, info_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2, cv2.LINE_AA)
            info_y += line_h + 4
            cv2.putText(frame, f"TTI: {tti_text}", (info_x, info_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50,
                        (0, 255, 255) if tti_text == "---" else (0, 0, 255), 1, cv2.LINE_AA)
            info_y += line_h
            cv2.putText(frame, f"TGT SPD: {tgt_speed:.1f} m/s", (info_x, info_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 165, 255), 1, cv2.LINE_AA)
            info_y += line_h
            cv2.putText(frame, f"CLR: {closing_rate:+.1f} m/s", (info_x, info_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (0, 220, 80) if closing_rate > 0 else (0, 0, 220), 1, cv2.LINE_AA)

        # ═══════════════════════════════════════════════════════
        # TARGET LOCK BOX + INTERCEPTION VISUALS
        # ═══════════════════════════════════════════════════════
        if locked and bbox is not None:
            bx, by, bw, bh = [int(v) for v in bbox]
            obj_cx, obj_cy = bx + bw // 2, by + bh // 2
            axis = getattr(self.sys_config.distance, "size_axis", "max") or "max"
            size_px = bbox_size_px((float(bx), float(by), float(bw), float(bh)), axis)

            # Tight lock box + corner ticks
            cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (0, 255, 200), 2)
            tick = max(6, min(18, min(bw, bh) // 6))
            for cx0, cy0, dx, dy in (
                (bx, by, 1, 1), (bx + bw, by, -1, 1),
                (bx, by + bh, 1, -1), (bx + bw, by + bh, -1, -1),
            ):
                cv2.line(frame, (cx0, cy0), (cx0 + dx * tick, cy0), (0, 255, 120), 2)
                cv2.line(frame, (cx0, cy0), (cx0, cy0 + dy * tick), (0, 255, 120), 2)

            # Measured axis highlight
            if axis == "height":
                cv2.line(frame, (obj_cx, by), (obj_cx, by + bh), (0, 220, 255), 2)
            elif axis == "diag":
                cv2.line(frame, (bx, by), (bx + bw, by + bh), (0, 220, 255), 2)
            else:
                if axis == "width" or bw >= bh:
                    cv2.line(frame, (bx, obj_cy), (bx + bw, obj_cy), (0, 220, 255), 2)
                else:
                    cv2.line(frame, (obj_cx, by), (obj_cx, by + bh), (0, 220, 255), 2)

            # Target centre dot
            cv2.circle(frame, (obj_cx, obj_cy), 4, (0, 255, 255), -1)

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
                cv2.polylines(frame, [pts], True, (0, 0, 255), 2, cv2.LINE_AA)
                cv2.circle(frame, (int_x, int_y), 3, (0, 0, 255), -1)

                # Steering line (crosshair → intercept)
                cv2.line(frame, (cx, cy), (int_x, int_y), (0, 0, 255), 2, cv2.LINE_AA)
                # Predicted path (target → intercept)
                cv2.line(frame, (obj_cx, obj_cy), (int_x, int_y), (0, 165, 255), 1, cv2.LINE_AA)

                # TTI label near intercept point
                if hasattr(ctrl, "last_t_intercept") and ctrl.last_t_intercept is not None:
                    cv2.putText(frame, f"{ctrl.last_t_intercept:.1f}s",
                                (int_x + 12, int_y - 4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 0, 255), 1, cv2.LINE_AA)
            else:
                cv2.line(frame, (cx, cy), (obj_cx, obj_cy), (255, 0, 255), 1)

            # ── Text telemetry under the box ──
            tid = self.active_target.target_id if self.active_target else "---"
            label_y = max(16, by - 8)
            cv2.putText(frame,
                        f"LOCK [{tid}] {source.upper()} {conf * 100:.0f}%",
                        (bx, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                        (0, 255, 255), 1, cv2.LINE_AA)

            cv2.putText(frame,
                        f"W:{bw}px  H:{bh}px  SIZE:{size_px:.0f}px ({axis})",
                        (bx, min(h - 8, by + bh + 18)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (80, 255, 160), 1, cv2.LINE_AA)

            cv2.putText(frame,
                        f"DIST {dist_m:.2f} m",
                        (bx, min(h - 8, by + bh + 38)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (100, 255, 100), 2, cv2.LINE_AA)

            brg = self.controller.last_bearing
            if brg is not None and mount_active:
                cv2.putText(frame,
                            f"AZ {math.degrees(brg.az_rad):+.1f}  EL {math.degrees(brg.el_rad):+.1f}  "
                            f"LOS {brg.slant_m:.2f}m  dALT {brg.vertical_m:+.2f}m",
                            (bx, min(h - 8, by + bh + 56)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1, cv2.LINE_AA)

        # ═══════════════════════════════════════════════════════
        # BOTTOM STATUS BAR
        # ═══════════════════════════════════════════════════════
        header_color = (0, 255, 0) if safety.is_safe else (0, 0, 255)
        cv2.putText(frame, f"STATUS: {safety.reason}",
                    (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, header_color, 2)

        cv2.putText(frame,
                    f"CAM:{self.active_cam_name or self.active_cam_idx}  FPS:{self.current_fps:.0f}  "
                    f"{'FOLLOW' if self.assist_enabled else 'LOCKED ONLY'}  "
                    f"SERIAL:{'OK' if self.is_connected else 'OFF'}",
                    (20, h - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        follow_color = (80, 220, 120) if self.follow_status.startswith("FOLLOWING") else (0, 180, 255)
        cv2.putText(frame,
                    f"CTRL: {self.follow_status}  ASSIST:{'ON' if self.assist_enabled else 'OFF'}",
                    (20, h - 52), cv2.FONT_HERSHEY_SIMPLEX, 0.50, follow_color, 2, cv2.LINE_AA)

    def stop(self) -> None:
        self.running = False
        self.wait()
