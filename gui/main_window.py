"""Main GUI Dashboard Window for Autonomous FPV Drone Tracking System."""

from __future__ import annotations

import os
import sys

os.environ["OPENCV_LOG_LEVEL"] = "OFF"
os.environ["OPENCV_VIDEOINPUT_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

import time
import cv2
import numpy as np
import serial
from PyQt6.QtCore import QObject, QThread, Qt, pyqtSignal, pyqtSlot
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDockWidget,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QSplitter,
    QStatusBar,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from config import SystemConfig
from control.fpv_follow import FPVFollowController
from control.msp_link import build_msp_set_raw_rc, list_serial_ports, make_rc_channels
from detection.hybrid_tracker import HybridYoloLockTracker
from gui.image_processing_widget import ImageProcessingWidget
from gui.parameters_panel import ParametersPanel
from gui.pid_panel import PIDTuningPanel
from gui.style import DARK_THEME_QSS
from gui.telemetry_widget import RealTimeTelemetryPlots
from gui.video_widget import VideoDisplayWidget
from safety.failsafe_manager import FailsafeManager
from telemetry.telemetry_logger import TelemetryLogger, TelemetryRecord


def list_camera_devices(max_test: int = 6) -> list[tuple[int, str]]:
    """Probe available camera & video capture card devices."""
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
    frame_processed = pyqtSignal(np.ndarray, object)  # frame_bgr, TelemetryRecord

    def __init__(self, sys_config: SystemConfig, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.sys_config = sys_config
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

        self.hybrid = HybridYoloLockTracker(det_cfg=self.sys_config.detection, tracker_cfg=self.sys_config.tracker)
        self.controller = FPVFollowController(self.sys_config)
        self.failsafe = FailsafeManager(self.sys_config.safety)
        self.logger = TelemetryLogger()

        self.pending_roi: tuple[int, int, int, int] | None = None
        self.pending_auto_lock = False
        self.frame_count = 0

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
            return True, f"Connected to {port_name} @ {baud_rate}"
        except Exception as e:
            self.is_connected = False
            self.serial_link = None
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
        self.hybrid.reset()
        self.controller.reset()
        self.failsafe.reset()

    def _open_camera(self, cam_idx: int) -> cv2.VideoCapture | None:
        for backend in [cv2.CAP_DSHOW, cv2.CAP_MSMF]:
            cap = cv2.VideoCapture(cam_idx, backend)
            if cap.isOpened():
                # Set MJPG FOURCC for USB capture cards (Elgato, MS2109, CamLink, etc.)
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
            # Fallback to index 1 or 0
            cap = self._open_camera(1 if self.requested_cam_idx == 0 else 0)
        self.requested_cam_idx = None

        last_time = time.time()
        last_msp_send = 0.0
        msp_interval = 1.0 / 50.0  # 50 Hz
        failed_reads = 0

        while self.running:
            # Handle camera device switch request
            if self.requested_cam_idx is not None:
                new_idx = self.requested_cam_idx
                self.requested_cam_idx = None
                if cap is not None and cap.isOpened():
                    cap.release()
                cap = self._open_camera(new_idx)
                failed_reads = 0

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

            # Auto-reconnect camera if stream dropped temporarily
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

            # Handle pending ROI lock
            if self.pending_roi is not None:
                rx, ry, rw, rh = self.pending_roi
                self.hybrid.lock_xywh(frame, (rx, ry, rw, rh), label="manual")
                self.controller.reset()
                self.assist_enabled = True
                self.pending_roi = None

            # Handle pending auto-lock
            if self.pending_auto_lock:
                self.hybrid.lock_best(frame)
                self.controller.reset()
                self.assist_enabled = True
                self.pending_auto_lock = False

            # Update hybrid tracking
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

            # Drone control & failsafe evaluation
            dist_m = 0.0
            if self.controller.last_distance:
                dist_m = self.controller.last_distance.distance_m

            safety_state = self.failsafe.evaluate(locked, conf, dist_m if locked else None)

            roll, pitch, yaw = 1500, 1500, 1500
            if locked and self.assist_enabled and safety_state.is_safe:
                roll, pitch, yaw = self.controller.update(bbox, w, h)
            else:
                roll, pitch, yaw = self.controller.fade_to_mid()

            # Continuous MSP Transmission over Serial Port (50 Hz)
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

            # Render overlay graphics on frame
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

        # Draw optical center crosshair
        cv2.drawMarker(frame, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 30, 2)

        # Draw deadzone box
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

            cv2.putText(frame, f"LOCK: {source.upper()} ({conf*100:.0f}%)", (bx, max(15, by - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            cv2.putText(frame, f"DIST: {dist_m:.1f}m", (bx, by + bh + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 100), 2)

        # Safety / HUD status header
        header_color = (0, 255, 0) if safety.is_safe else (0, 0, 255)
        cv2.putText(frame, f"STATUS: {safety.reason}", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, header_color, 2)
        cv2.putText(
            frame,
            f"CAM:{self.active_cam_idx}  AETR R:{roll} P:{pitch} Y:{yaw}  SERIAL:{'CONNECTED' if self.is_connected else 'DISCONNECTED'}",
            (20, h - 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
        )

    def stop(self) -> None:
        self.running = False
        self.wait()


class MainWindow(QMainWindow):
    def __init__(self, sys_config: SystemConfig | None = None) -> None:
        super().__init__()
        self.sys_config = sys_config or SystemConfig()
        self.setWindowTitle("Autonomous FPV Drone Tracking System & Real-Time Dashboard")
        self.resize(1500, 940)
        self.setStyleSheet(DARK_THEME_QSS)

        self._init_ui()

        # Start worker thread
        self.worker = TrackingWorkerThread(self.sys_config)
        self.worker.frame_processed.connect(self._on_frame_processed)
        self.worker.start()

    def _init_ui(self) -> None:
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        splitter = QSplitter(Qt.Orientation.Horizontal)

        # Left Column: Video & Controls
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)

        # Camera & Video Source Selection Group
        grp_cam = QGroupBox("Video Input & Capture Card Source")
        cam_layout = QHBoxLayout(grp_cam)

        self.combo_cameras = QComboBox()
        btn_refresh_cams = QPushButton("Refresh Devices")
        btn_refresh_cams.clicked.connect(self._refresh_camera_devices)

        self.combo_cameras.currentIndexChanged.connect(self._on_camera_changed)

        cam_layout.addWidget(QLabel("Select Capture Card / Camera:"))
        cam_layout.addWidget(self.combo_cameras, stretch=1)
        cam_layout.addWidget(btn_refresh_cams)

        self._refresh_camera_devices()

        self.video_widget = VideoDisplayWidget()
        self.video_widget.roi_selected.connect(self._on_roi_selected)

        # Serial Port Detection Group
        grp_serial = QGroupBox("Flight Controller Serial Port (MSP Link)")
        ser_layout = QHBoxLayout(grp_serial)

        self.combo_ports = QComboBox()
        self.combo_baud = QComboBox()
        for b in [115200, 57600, 230400, 460800, 921600]:
            self.combo_baud.addItem(str(b), b)

        btn_refresh_ports = QPushButton("Refresh Ports")
        btn_refresh_ports.clicked.connect(self._refresh_serial_ports)

        self.btn_connect = QPushButton("Connect Serial")
        self.btn_connect.clicked.connect(self._toggle_serial_connection)

        self.lbl_serial_status = QLabel("DISCONNECTED")
        self.lbl_serial_status.setStyleSheet("color: #ef4444; font-weight: bold; padding: 4px 8px;")

        ser_layout.addWidget(QLabel("Port:"))
        ser_layout.addWidget(self.combo_ports, stretch=1)
        ser_layout.addWidget(QLabel("Baud:"))
        ser_layout.addWidget(self.combo_baud)
        ser_layout.addWidget(btn_refresh_ports)
        ser_layout.addWidget(self.btn_connect)
        ser_layout.addWidget(self.lbl_serial_status)

        self._refresh_serial_ports()

        # Controls Group
        grp_ctrl = QGroupBox("Tracking & Autonomous Flight Controls")
        ctrl_layout = QHBoxLayout(grp_ctrl)

        btn_lock = QPushButton("YOLO Auto-Lock")
        btn_lock.clicked.connect(self._on_auto_lock)

        btn_reset = QPushButton("Unlock / Reset")
        btn_reset.clicked.connect(self._on_reset_lock)

        self.btn_assist = QPushButton("Enable Assist (Follow)")
        self.btn_assist.setCheckable(True)
        self.btn_assist.toggled.connect(self._on_toggle_assist)

        self.btn_arm = QPushButton("ARM DRONE")
        self.btn_arm.setObjectName("btn_arm")
        self.btn_arm.setCheckable(True)
        self.btn_arm.toggled.connect(self._on_toggle_arm)

        btn_override = QPushButton("MANUAL OVERRIDE")
        btn_override.clicked.connect(self._on_manual_override)

        ctrl_layout.addWidget(btn_lock)
        ctrl_layout.addWidget(btn_reset)
        ctrl_layout.addWidget(self.btn_assist)
        ctrl_layout.addWidget(self.btn_arm)
        ctrl_layout.addWidget(btn_override)

        # Status Bar / Metrics
        grp_metrics = QGroupBox("Live Status Metrics")
        m_layout = QHBoxLayout(grp_metrics)

        self.lbl_lock = QLabel("LOCK: NONE")
        self.lbl_dist = QLabel("DIST: -- m")
        self.lbl_rc = QLabel("RC: R=1500 P=1500 Y=1500")

        self.bar_conf = QProgressBar()
        self.bar_conf.setRange(0, 100)
        self.bar_conf.setValue(0)
        self.bar_conf.setFormat("Confidence: %p%")

        m_layout.addWidget(self.lbl_lock)
        m_layout.addWidget(self.lbl_dist)
        m_layout.addWidget(self.bar_conf)
        m_layout.addWidget(self.lbl_rc)

        left_layout.addWidget(grp_cam)
        left_layout.addWidget(grp_serial)
        left_layout.addWidget(self.video_widget, stretch=1)
        left_layout.addWidget(grp_ctrl)
        left_layout.addWidget(grp_metrics)

        # Right Column: Tuning & Telemetry Tabs
        right_tabs = QTabWidget()

        self.pid_panel = PIDTuningPanel(self.sys_config)
        self.pid_panel.pid_updated.connect(self._on_params_changed)

        self.params_panel = ParametersPanel(self.sys_config)
        self.params_panel.params_updated.connect(self._on_params_changed)
        self.params_panel.preset_loaded.connect(self._on_preset_loaded)

        self.telemetry_plots = RealTimeTelemetryPlots()
        self.img_proc_widget = ImageProcessingWidget()

        right_tabs.addTab(self.pid_panel, "PID Tuning")
        right_tabs.addTab(self.params_panel, "Adjustable Parameters")
        right_tabs.addTab(self.telemetry_plots, "Live Telemetry Charts")
        right_tabs.addTab(self.img_proc_widget, "Image Processing Inspector")

        splitter.addWidget(left_widget)
        splitter.addWidget(right_tabs)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

        main_layout.addWidget(splitter)

        self.statusBar().showMessage("System initialized. Select video capture device or COM port.")

    def _refresh_camera_devices(self) -> None:
        self.combo_cameras.blockSignals(True)
        self.combo_cameras.clear()
        devices = list_camera_devices()
        for idx, label in devices:
            self.combo_cameras.addItem(label, idx)
        self.combo_cameras.blockSignals(False)

    def _on_camera_changed(self) -> None:
        cam_idx = self.combo_cameras.currentData()
        if cam_idx is not None:
            self.worker.switch_camera(cam_idx)
            self.statusBar().showMessage(f"Switching video source to Camera {cam_idx}...")

    def _refresh_serial_ports(self) -> None:
        self.combo_ports.clear()
        ports = list_serial_ports()
        if not ports:
            self.combo_ports.addItem("No serial ports detected", "")
            self.combo_ports.setEnabled(False)
            self.btn_connect.setEnabled(False)
        else:
            self.combo_ports.setEnabled(True)
            self.btn_connect.setEnabled(True)
            for dev, label in ports:
                self.combo_ports.addItem(label, dev)

    def _toggle_serial_connection(self) -> None:
        if self.worker.is_connected:
            self.worker.disconnect_serial()
            self.btn_connect.setText("Connect Serial")
            self.lbl_serial_status.setText("DISCONNECTED")
            self.lbl_serial_status.setStyleSheet("color: #ef4444; font-weight: bold; padding: 4px 8px;")
            self.statusBar().showMessage("Serial port disconnected.")
        else:
            port = self.combo_ports.currentData()
            baud = self.combo_baud.currentData()
            if not port:
                QMessageBox.warning(self, "Serial Connection", "No valid COM port selected.")
                return

            ok, msg = self.worker.connect_serial(port, baud)
            if ok:
                self.btn_connect.setText("Disconnect")
                self.lbl_serial_status.setText(f"CONNECTED ({port})")
                self.lbl_serial_status.setStyleSheet("color: #22c55e; font-weight: bold; padding: 4px 8px;")
                self.statusBar().showMessage(msg)
            else:
                QMessageBox.critical(self, "Serial Connection Error", msg)
                self.lbl_serial_status.setText("ERROR")
                self.lbl_serial_status.setStyleSheet("color: #ef4444; font-weight: bold; padding: 4px 8px;")

    @pyqtSlot(int, int, int, int)
    def _on_roi_selected(self, x: int, y: int, w: int, h: int) -> None:
        self.worker.set_roi_lock(x, y, w, h)
        self.statusBar().showMessage(f"Target ROI Locked: ({x}, {y}, {w}, {h})")

    def _on_auto_lock(self) -> None:
        self.worker.trigger_auto_lock()

    def _on_reset_lock(self) -> None:
        self.worker.reset_lock()
        self.btn_assist.setChecked(False)
        self.statusBar().showMessage("Tracking Lock Reset.")

    def _on_toggle_assist(self, checked: bool) -> None:
        self.worker.assist_enabled = checked
        self.btn_assist.setText("Disable Assist" if checked else "Enable Assist (Follow)")

    def _on_toggle_arm(self, checked: bool) -> None:
        self.worker.arm_requested = checked
        if checked:
            self.btn_arm.setText("DISARM DRONE")
            self.btn_arm.setObjectName("btn_disarm")
            self.btn_arm.setStyle(self.btn_arm.style())
        else:
            self.btn_arm.setText("ARM DRONE")
            self.btn_arm.setObjectName("btn_arm")
            self.btn_arm.setStyle(self.btn_arm.style())

    def _on_manual_override(self) -> None:
        self.worker.failsafe.trigger_manual_override(True)
        self.btn_assist.setChecked(False)
        QMessageBox.warning(self, "Manual Override", "Manual Override Engaged! Drone commands reverted to neutral disarmed.")

    def _on_params_changed(self) -> None:
        self.worker.update_config(self.sys_config)

    def _on_preset_loaded(self, loaded_cfg: SystemConfig) -> None:
        self.sys_config = loaded_cfg
        self.worker.update_config(loaded_cfg)
        self.pid_panel.load_config(loaded_cfg)

    @pyqtSlot(np.ndarray, object)
    def _on_frame_processed(self, frame_bgr: np.ndarray, rec: TelemetryRecord) -> None:
        self.video_widget.update_frame(frame_bgr)
        self.telemetry_plots.update_telemetry(rec)
        self.img_proc_widget.update_processing_views(
            frame_bgr,
            pixel_engine=self.worker.hybrid.pixel_engine,
            target_hist=self.worker.hybrid._target_hist,
            bbox=(int(rec.bbox_x), int(rec.bbox_y), int(rec.bbox_w), int(rec.bbox_h)) if rec.locked else None,
        )

        self.lbl_lock.setText(f"LOCK: {'ACTIVE (' + rec.source.upper() + ')' if rec.locked else 'NONE'}")
        self.lbl_dist.setText(f"DIST: {rec.estimated_distance_m:.1f} m")
        self.bar_conf.setValue(int(rec.confidence * 100))
        self.lbl_rc.setText(f"RC: R={rec.rc_roll} P={rec.rc_pitch} Y={rec.rc_yaw}")

    def closeEvent(self, event) -> None:
        self.worker.stop()
        event.accept()
