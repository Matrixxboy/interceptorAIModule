"""Real-time tracking worker thread with target database integration."""

from __future__ import annotations

import time

import cv2
import numpy as np
import serial
from PyQt6.QtCore import QObject, QThread, pyqtSignal

from config import SystemConfig
from control.fpv_follow import FPVFollowController
from control.msp_link import build_msp_set_raw_rc, make_rc_channels
from database.target_profile import TargetProfile, TargetStatus
from database.target_store import TargetStore
from detection.hybrid_tracker import HybridYoloLockTracker
from sys_logging.system_logger import LogCategory, LogSeverity, SystemLogger
from safety.failsafe_manager import FailsafeManager
from telemetry.telemetry_logger import TelemetryLogger, TelemetryRecord


def list_camera_devices(max_test: int = 6) -> list[tuple[int, str]]:
    """Probe available camera and video capture devices."""
    devices = []
    for idx in range(max_test):
        cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
        if cap.isOpened():
            ok, _ = cap.read()
            cap.release()
            label = f"Camera {idx} (Capture Card / Video Input)" if ok else f"Camera {idx}"
            devices.append((idx, label))
    if not devices:
        devices.append((0, "Default Camera 0 (Synthetic Mode)"))
    return devices


class TrackingWorkerThread(QThread):
    frame_processed = pyqtSignal(np.ndarray, object)
    target_changed = pyqtSignal(object)
    fps_updated = pyqtSignal(float)

    def __init__(
        self,
        sys_config: SystemConfig,
        target_store: TargetStore | None = None,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self.sys_config = sys_config
        self.target_store = target_store or TargetStore()
        self.sys_log = SystemLogger()
        self.running = False
        self.assist_enabled = False
        self.arm_requested = False
        self.mode_requested = False

        self.serial_link: serial.Serial | None = None
        self.is_connected = False
        self.port_name = ""
        self.baud_rate = 115200

        self.requested_cam_idx: int | None = self.sys_config.camera.camera_index
        self.active_cam_idx: int = self.sys_config.camera.camera_index

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

    @property
    def current_fps(self) -> float:
        if not self._fps_window:
            return 0.0
        return sum(self._fps_window) / len(self._fps_window)

    def switch_camera(self, cam_index: int) -> None:
        self.requested_cam_idx = cam_index

    def connect_serial(self, port_name: str, baud_rate: int = 115200) -> tuple[bool, str]:
        self.disconnect_serial()
        try:
            self.serial_link = serial.Serial(port_name, baud_rate, timeout=0.02)
            time.sleep(0.5)
            self.is_connected = True
            self.port_name = port_name
            self.baud_rate = baud_rate
            self.sys_log.log(
                LogCategory.DRONE,
                f"Connected to {port_name} @ {baud_rate}",
                module="MSP Link",
            )
            return True, f"Connected to {port_name} @ {baud_rate}"
        except Exception as e:
            self.is_connected = False
            self.serial_link = None
            self.sys_log.log(
                LogCategory.DRONE,
                f"Connection failed: {e}",
                severity=LogSeverity.ERROR,
                module="MSP Link",
            )
            return False, f"Could not open {port_name}: {e}"

    def disconnect_serial(self) -> None:
        if self.serial_link is not None and self.serial_link.is_open:
            try:
                neutral_ch = make_rc_channels(1500, 1500, 1500, 1000, arm=False, flight_mode=False)
                packet = build_msp_set_raw_rc(neutral_ch)
                self.serial_link.write(packet)
                self.serial_link.close()
            except Exception:
                pass
        self.serial_link = None
        self.is_connected = False

    def update_config(self, cfg: SystemConfig) -> None:
        self.sys_config = cfg
        self.controller.update_sys_config(cfg)
        self.failsafe.update_config(cfg.safety)

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

    def _open_camera(self, cam_idx: int) -> cv2.VideoCapture | None:
        for backend in [cv2.CAP_DSHOW, cv2.CAP_MSMF]:
            cap = cv2.VideoCapture(cam_idx, backend)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.sys_config.camera.frame_width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.sys_config.camera.frame_height)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                ok, frame = cap.read()
                if ok and frame is not None:
                    self.active_cam_idx = cam_idx
                    return cap
                cap.release()
                cap = cv2.VideoCapture(cam_idx, backend)
                if cap.isOpened():
                    ok, frame = cap.read()
                    if ok and frame is not None:
                        self.active_cam_idx = cam_idx
                        return cap
                    cap.release()
        return None

    def run(self) -> None:
        self.running = True
        if hasattr(cv2, "setLogLevel"):
            cv2.setLogLevel(cv2.LOG_LEVEL_SILENT)

        cap = self._open_camera(self.requested_cam_idx if self.requested_cam_idx is not None else 0)
        if cap is None:
            cap = self._open_camera(1 if self.requested_cam_idx == 0 else 0)
        self.requested_cam_idx = None

        self.sys_log.log(LogCategory.SYSTEM, "Tracking pipeline started", module="Worker")
        last_time = time.time()
        last_msp_send = 0.0
        msp_interval = 1.0 / 50.0
        failed_reads = 0

        while self.running:
            frame_start = time.time()

            if self.requested_cam_idx is not None:
                new_idx = self.requested_cam_idx
                self.requested_cam_idx = None
                if cap is not None and cap.isOpened():
                    cap.release()
                cap = self._open_camera(new_idx)
                failed_reads = 0
                self.sys_log.log(LogCategory.CAMERA, f"Switched to camera {new_idx}")

            ok = False
            frame = None
            if cap is not None and cap.isOpened():
                ok, frame = cap.read()
                if not ok or frame is None:
                    failed_reads += 1
                    if failed_reads >= 20:
                        cap.release()
                        cap = None
                else:
                    failed_reads = 0

            if cap is None and failed_reads >= 20 and (self.frame_count % 30 == 0):
                cap = self._open_camera(self.active_cam_idx)
                if cap is not None:
                    failed_reads = 0

            if not ok or frame is None:
                frame = np.zeros((720, 1280, 3), dtype=np.uint8)
                cv2.putText(
                    frame,
                    f"SIMULATED STREAM (Cam {self.active_cam_idx} No Feed / Unplugged)",
                    (280, 360),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (0, 180, 255),
                    2,
                )

            self.frame_count += 1
            now = time.time()
            dt = max(0.001, now - last_time)
            last_time = now

            h, w = frame.shape[:2]
            cx, cy = w // 2, h // 2

            if self.pending_roi is not None:
                rx, ry, rw, rh = self.pending_roi
                self.hybrid.lock_xywh(frame, (rx, ry, rw, rh), label="manual")
                self.controller.reset()
                self.assist_enabled = True
                self._start_target_lock(frame, (rx, ry, rw, rh), "manual", "manual")
                self.pending_roi = None

            if self.pending_auto_lock:
                best = self.hybrid.lock_best(frame)
                if best and self.hybrid._bbox:
                    self._start_target_lock(frame, self.hybrid._bbox, "yolo", self.hybrid._label or "yolo")
                self.controller.reset()
                self.assist_enabled = True
                self.pending_auto_lock = False

            dets = []
            locked = False
            bbox = None
            conf = 0.0
            source = "none"

            if self.hybrid.locked:
                res = self.hybrid.update(frame)
                dets = res.detections
                if res.ok and res.bbox_xywh is not None:
                    locked = True
                    bbox = res.bbox_xywh
                    conf = res.conf
                    source = res.source

            if locked and self.active_target:
                vx, vy = self.controller.motion_predictor.kalman.get_velocity()
                color_hist = None
                if self.hybrid._target_hist is not None:
                    color_hist = self.hybrid._target_hist.flatten().tolist()[:64]
                dist_m = 0.0
                if self.controller.last_distance:
                    dist_m = self.controller.last_distance.distance_m
                self.active_target.update_frame(
                    bbox_xywh=bbox,
                    confidence=conf,
                    source=source,
                    vx=vx,
                    vy=vy,
                    distance_m=dist_m,
                    color_hist=color_hist,
                )
                self.target_store.update_active(self.active_target, frame, bbox_xywh=bbox)
                if self.assist_enabled and not self._was_locked:
                    self.active_target.add_event(
                        "Drone Following",
                        confidence=conf,
                        drone_state="assist_enabled",
                        system_response="FPV follow controller active",
                    )
            elif self._was_locked and not locked and self.active_target:
                self.target_store.mark_lost(self.active_target)
                self.sys_log.log(
                    LogCategory.TRACKING,
                    f"Target {self.active_target.target_id} temporarily lost",
                    severity=LogSeverity.WARNING,
                    target_id=self.active_target.target_id,
                )
            elif not self._was_locked and locked and self.active_target:
                if self.active_target.status == TargetStatus.LOST:
                    self.target_store.mark_reacquired(self.active_target)

            self._was_locked = locked

            dist_m = 0.0
            if self.controller.last_distance:
                dist_m = self.controller.last_distance.distance_m

            safety_state = self.failsafe.evaluate(locked, conf, dist_m if locked else None)

            roll, pitch, yaw = 1500, 1500, 1500
            if locked and self.assist_enabled and safety_state.is_safe:
                roll, pitch, yaw = self.controller.update(bbox, w, h)
            else:
                roll, pitch, yaw = self.controller.fade_to_mid()

            if now - last_msp_send >= msp_interval:
                last_msp_send = now
                rc_channels = make_rc_channels(
                    roll=roll,
                    pitch=pitch,
                    yaw=yaw,
                    throttle=1000,
                    arm=self.arm_requested,
                    flight_mode=self.mode_requested or self.arm_requested,
                )
                if self.is_connected and self.serial_link is not None and self.serial_link.is_open:
                    try:
                        packet = build_msp_set_raw_rc(rc_channels)
                        self.serial_link.write(packet)
                    except Exception:
                        self.is_connected = False

            self._render_hud(frame, locked, bbox, conf, source, safety_state, roll, pitch, yaw, dist_m, w, h)

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
                throttle=1000,
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

            self.frame_processed.emit(frame, rec)
            time.sleep(0.01)

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
        dist_m: float,
        w: int,
        h: int,
    ) -> None:
        cx, cy = w // 2, h // 2
        cv2.drawMarker(frame, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 30, 2)

        dz_px_x = int(w * 0.5 * self.sys_config.offsets.deadzone_norm)
        dz_px_y = int(h * 0.5 * self.sys_config.offsets.deadzone_norm)
        cv2.rectangle(frame, (cx - dz_px_x, cy - dz_px_y), (cx + dz_px_x, cy + dz_px_y), (255, 255, 0), 1)

        if locked and bbox is not None:
            bx, by, bw, bh = bbox
            obj_cx, obj_cy = bx + bw // 2, by + bh // 2
            cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (0, 255, 255), 2)
            cv2.circle(frame, (obj_cx, obj_cy), 5, (0, 255, 255), -1)
            cv2.line(frame, (cx, cy), (obj_cx, obj_cy), (255, 0, 255), 2)

            if self.controller.last_trajectory:
                aim_x = int(self.controller.last_trajectory.aim_cx)
                aim_y = int(self.controller.last_trajectory.aim_cy)
                cv2.circle(frame, (aim_x, aim_y), 4, (0, 255, 0), -1)

            tid = self.active_target.target_id if self.active_target else "---"
            cv2.putText(
                frame,
                f"ARJUNA LOCK [{tid}] {source.upper()} ({conf * 100:.0f}%)",
                (bx, max(15, by - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                2,
            )
            cv2.putText(
                frame,
                f"DIST: {dist_m:.1f}m",
                (bx, by + bh + 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (100, 255, 100),
                2,
            )

        header_color = (0, 255, 0) if safety.is_safe else (0, 0, 255)
        cv2.putText(
            frame,
            f"STATUS: {safety.reason}",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            header_color,
            2,
        )
        cv2.putText(
            frame,
            f"CAM:{self.active_cam_idx}  FPS:{self.current_fps:.0f}  AETR R:{roll} P:{pitch} Y:{yaw}  SERIAL:{'OK' if self.is_connected else 'OFF'}",
            (20, h - 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
        )

    def stop(self) -> None:
        self.running = False
        self.wait()
