"""Main GUI Dashboard Window — legacy layout (use gui.arjuna_shell for Arjuna GCS)."""

from __future__ import annotations

import os

os.environ["OPENCV_LOG_LEVEL"] = "OFF"
os.environ["OPENCV_VIDEOINPUT_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

from typing import Any
import numpy as np
from PyQt6.QtCore import Qt, QThread, pyqtSignal, pyqtSlot
from PyQt6.QtGui import QKeyEvent
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QSpinBox,
    QSplitter,
    QStatusBar,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from config import SystemConfig


class MainWindowConnectionWorker(QThread):
    finished_signal = pyqtSignal(bool, str, str)

    def __init__(self, worker: Any, port: str, baud: int) -> None:
        super().__init__()
        self.worker = worker
        self.port = port
        self.baud = baud

    def run(self) -> None:
        ok, msg = self.worker.connect_serial(self.port, self.baud)
        self.finished_signal.emit(ok, msg, str(self.port))
from control.msp_link import list_serial_ports
from core.tracking_worker import TrackingWorkerThread, list_camera_devices
from gui.image_processing_widget import ImageProcessingWidget
from gui.parameters_panel import ParametersPanel
from gui.pid_panel import PIDTuningPanel
from gui.style import ARJUNA_THEME_QSS
from gui.telemetry_widget import RealTimeTelemetryPlots
from gui.video_widget import VideoDisplayWidget
from telemetry.telemetry_logger import TelemetryRecord


class MainWindow(QMainWindow):
    def __init__(self, sys_config: SystemConfig | None = None) -> None:
        super().__init__()
        self.sys_config = sys_config or SystemConfig()
        self.setWindowTitle("Arjuna — Legacy Dashboard")
        self.resize(1500, 940)
        self.setStyleSheet(ARJUNA_THEME_QSS)

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

        self._refresh_camera_devices(probe=False)

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

        right_tabs.addTab(self.pid_panel, "Control")
        right_tabs.addTab(self.params_panel, "Adjustable Parameters")
        right_tabs.addTab(self.telemetry_plots, "Live Telemetry Charts")
        right_tabs.addTab(self.img_proc_widget, "Image Processing Inspector")

        splitter.addWidget(left_widget)
        splitter.addWidget(right_tabs)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

        main_layout.addWidget(splitter)

        self.statusBar().showMessage("System initialized. Select video capture device or COM port.")

    def _refresh_camera_devices(self, _checked: bool = False, probe: bool = True) -> None:
        self.combo_cameras.blockSignals(True)
        self.combo_cameras.clear()
        if probe:
            devices = list_camera_devices()
        else:
            idx = int(self.sys_config.camera.camera_index)
            devices = [(idx, f"Configured Camera {idx}")]
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
            self.btn_connect.setEnabled(True)
            self.lbl_serial_status.setText("DISCONNECTED")
            self.lbl_serial_status.setStyleSheet("color: #ef4444; font-weight: bold; padding: 4px 8px;")
            self.statusBar().showMessage("Serial port disconnected.")
        else:
            port = self.combo_ports.currentData()
            baud = self.combo_baud.currentData()
            if not port:
                QMessageBox.warning(self, "Serial Connection", "No valid COM port selected.")
                return

            self.btn_connect.setText("Connecting...")
            self.btn_connect.setEnabled(False)
            self.lbl_serial_status.setText("CONNECTING...")
            self.lbl_serial_status.setStyleSheet("color: #eab308; font-weight: bold; padding: 4px 8px;")

            self._conn_thread = MainWindowConnectionWorker(self.worker, port, baud)
            self._conn_thread.finished_signal.connect(self._on_connection_finished)
            self._conn_thread.start()

    def _on_connection_finished(self, ok: bool, msg: str, port: str) -> None:
        self.btn_connect.setEnabled(True)
        if ok:
            self.btn_connect.setText("Disconnect")
            self.lbl_serial_status.setText(f"CONNECTED ({port})")
            self.lbl_serial_status.setStyleSheet("color: #22c55e; font-weight: bold; padding: 4px 8px;")
            self.statusBar().showMessage(msg)
        else:
            self.btn_connect.setText("Connect Serial")
            self.lbl_serial_status.setText("ERROR")
            self.lbl_serial_status.setStyleSheet("color: #ef4444; font-weight: bold; padding: 4px 8px;")
            QMessageBox.critical(self, "Serial Connection Error", msg)

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
        if checked:
            self.worker.arm_drone()
            self.btn_arm.setText("DISARM DRONE")
            self.btn_arm.setObjectName("btn_disarm")
            self.btn_arm.setStyle(self.btn_arm.style())
        else:
            self.worker.disarm_drone()
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

    def keyPressEvent(self, event: QKeyEvent) -> None:
        focus_widget = QApplication.focusWidget()
        if focus_widget and isinstance(focus_widget, (QLineEdit, QTextEdit, QPlainTextEdit, QSpinBox, QDoubleSpinBox)):
            super().keyPressEvent(event)
            return

        key = event.key()
        text = event.text().upper()

        if key == Qt.Key.Key_L or text == "L":
            # L = start manual lock selection mode
            self.video_widget.setFocus()
            self.statusBar().showMessage("MANUAL LOCK MODE (Hotkey L): Drag box on video feed to lock target", 4000)
        elif key == Qt.Key.Key_A or text == "A":
            # A = arm
            self.worker.arm_drone()
            if hasattr(self, "btn_arm"):
                self.btn_arm.setChecked(True)
            self.statusBar().showMessage("DRONE ARMED (Hotkey A)", 4000)
        elif key == Qt.Key.Key_X or text == "X":
            # X = disarm
            self.worker.disarm_drone()
            if hasattr(self, "btn_arm"):
                self.btn_arm.setChecked(False)
            self.statusBar().showMessage("DRONE DISARMED - Throttle reset to 1000 µs (Hotkey X)", 4000)
        elif key == Qt.Key.Key_M or text == "M":
            # M = toggle flight mode only
            mode = self.worker.toggle_flight_mode()
            self.statusBar().showMessage(f"FLIGHT MODE TOGGLED: {mode} (Hotkey M)", 4000)
        elif key == Qt.Key.Key_U or text == "U":
            # U = throttle +25
            thr = self.worker.adjust_throttle(25)
            self.statusBar().showMessage(f"THROTTLE: {thr} µs (+25, Hotkey U)", 2000)
        elif key == Qt.Key.Key_J or text == "J":
            # J = throttle -25
            thr = self.worker.adjust_throttle(-25)
            self.statusBar().showMessage(f"THROTTLE: {thr} µs (-25, Hotkey J)", 2000)
        elif key == Qt.Key.Key_0 or text == "0":
            # 0 = set flight mode to toggle angle mode and acro mode
            mode = self.worker.toggle_flight_mode()
            self.statusBar().showMessage(f"FLIGHT MODE: {mode} (Hotkey 0)", 4000)
        elif key == Qt.Key.Key_R or text == "R":
            # R = reset tracker
            self.worker.reset_lock()
            self.statusBar().showMessage("TRACKER RESET / UNLOCKED (Hotkey R)", 4000)
        elif key == Qt.Key.Key_S or text == "S":
            # S = check arm status
            armed = getattr(self.worker, 'arm_requested', False)
            thr = getattr(self.worker, 'throttle_value', 1000)
            mode = getattr(self.worker, 'flight_mode', 'ANGLE')
            fc_status = "CONNECTED" if self.worker.is_connected else "DISCONNECTED"
            msg = f"STATUS CHECK: FC={fc_status} | ARM={'ARMED' if armed else 'DISARMED'} | Mode={mode} | Throttle={thr} µs"
            self.statusBar().showMessage(msg, 6000)
        else:
            super().keyPressEvent(event)

    def closeEvent(self, event) -> None:
        self.worker.stop()
        event.accept()
