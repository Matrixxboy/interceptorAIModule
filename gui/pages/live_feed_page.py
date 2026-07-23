"""Live camera feed and tracking control — responsive, no overflow."""

from __future__ import annotations

from typing import Any

from PyQt6.QtCore import Qt, QThread, pyqtSignal, pyqtSlot
from PyQt6.QtWidgets import (
    QComboBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QSizePolicy,
    QSpinBox,
    QSplitter,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from config import SystemConfig
from control.msp_link import list_serial_ports
from core.tracking_worker import TrackingWorkerThread, list_camera_devices
from gui.image_processing_widget import ImageProcessingWidget
from gui.parameters_panel import ParametersPanel
from gui.pid_panel import PIDTuningPanel
from gui.telemetry_widget import RealTimeTelemetryPlots
from gui.video_widget import VideoDisplayWidget
from gui.widgets.page_header import PageHeader, StatusPill


class ConnectionWorker(QThread):
    finished_signal = pyqtSignal(bool, str, str)

    def __init__(self, worker: Any, port: str, baud: int) -> None:
        super().__init__()
        self.worker = worker
        self.port = port
        self.baud = baud

    def run(self) -> None:
        ok, msg = self.worker.connect_serial(self.port, self.baud)
        self.finished_signal.emit(ok, msg, str(self.port))


class LiveFeedPage(QWidget):
    def __init__(self, worker: TrackingWorkerThread, sys_config: SystemConfig, parent=None) -> None:
        super().__init__(parent)
        self.worker = worker
        self.sys_config = sys_config
        self._build_ui()

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(8)

        header = PageHeader(
            "Live Camera Feed",
            "Drag ROI · Auto-Lock · MSP follow · Hotkeys U/J throttle",
        )
        self.pill_feed = StatusPill("FEED IDLE", "neutral")
        header.right_layout.addWidget(self.pill_feed)
        root.addWidget(header)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.setChildrenCollapsible(False)
        splitter.setHandleWidth(3)

        # ===================== LEFT =====================
        left = QWidget()
        left.setMinimumWidth(380)
        left.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        left_l = QVBoxLayout(left)
        left_l.setContentsMargins(0, 0, 4, 0)
        left_l.setSpacing(6)

        # Camera
        cam_bar = self._panel()
        cam_row = QHBoxLayout(cam_bar)
        cam_row.setContentsMargins(10, 8, 10, 8)
        cam_row.setSpacing(8)
        cam_row.addWidget(self._muted("CAMERA"))
        self.combo_cameras = QComboBox()
        self.combo_cameras.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.combo_cameras.setMinimumWidth(100)
        self.combo_cameras.currentIndexChanged.connect(self._on_camera_changed)
        cam_row.addWidget(self.combo_cameras, stretch=1)
        btn_refresh_cams = QPushButton("Refresh")
        btn_refresh_cams.setObjectName("btnGhost")
        btn_refresh_cams.setFixedWidth(72)
        btn_refresh_cams.clicked.connect(self._refresh_camera_devices)
        cam_row.addWidget(btn_refresh_cams)
        left_l.addWidget(cam_bar)

        # MSP — two rows so nothing clips
        msp_bar = self._panel()
        msp_grid = QGridLayout(msp_bar)
        msp_grid.setContentsMargins(10, 8, 10, 8)
        msp_grid.setHorizontalSpacing(8)
        msp_grid.setVerticalSpacing(6)

        msp_grid.addWidget(self._muted("PORT"), 0, 0)
        self.combo_ports = QComboBox()
        self.combo_ports.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.combo_ports.setMinimumWidth(80)
        msp_grid.addWidget(self.combo_ports, 0, 1, 1, 2)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setObjectName("btnPrimary")
        self.btn_connect.setFixedWidth(88)
        self.btn_connect.clicked.connect(self._toggle_serial_connection)
        msp_grid.addWidget(self.btn_connect, 0, 3)

        self.lbl_serial_status = StatusPill("OFFLINE", "error")
        msp_grid.addWidget(self.lbl_serial_status, 0, 4)

        msp_grid.addWidget(self._muted("BAUD"), 1, 0)
        self.combo_baud = QComboBox()
        for b in [115200, 57600, 230400, 460800, 921600]:
            self.combo_baud.addItem(str(b), b)
        self.combo_baud.setFixedWidth(100)
        msp_grid.addWidget(self.combo_baud, 1, 1)

        btn_refresh_ports = QPushButton("Refresh")
        btn_refresh_ports.setObjectName("btnGhost")
        btn_refresh_ports.setFixedWidth(72)
        btn_refresh_ports.clicked.connect(self._refresh_serial_ports)
        msp_grid.addWidget(btn_refresh_ports, 1, 2)

        msp_grid.setColumnStretch(1, 1)
        msp_grid.setColumnStretch(2, 1)
        left_l.addWidget(msp_bar)

        self._refresh_camera_devices()
        self._refresh_serial_ports()

        # Video
        self.video_widget = VideoDisplayWidget()
        self.video_widget.setMinimumHeight(200)
        self.video_widget.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.video_widget.roi_selected.connect(self._on_roi_selected)
        left_l.addWidget(self.video_widget, stretch=1)

        # Tracking actions — wrap to 2 rows
        ctrl_bar = self._panel()
        ctrl = QGridLayout(ctrl_bar)
        ctrl.setContentsMargins(10, 8, 10, 8)
        ctrl.setHorizontalSpacing(6)
        ctrl.setVerticalSpacing(6)

        btn_lock = QPushButton("Auto-Lock")
        btn_lock.setObjectName("btnPrimary")
        btn_lock.setToolTip("YOLO Auto-Lock (L)")
        btn_lock.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        btn_lock.clicked.connect(lambda: self.worker.trigger_auto_lock())

        btn_reset = QPushButton("Unlock")
        btn_reset.setObjectName("btnGhost")
        btn_reset.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        btn_reset.clicked.connect(self._on_reset_lock)

        self.btn_assist = QPushButton("Follow")
        self.btn_assist.setObjectName("btnSuccess")
        self.btn_assist.setCheckable(True)
        self.btn_assist.setToolTip("Enable / disable follow assist")
        self.btn_assist.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.btn_assist.toggled.connect(self._on_toggle_assist)

        self.btn_arm = QPushButton("ARM")
        self.btn_arm.setObjectName("btn_arm")
        self.btn_arm.setCheckable(True)
        self.btn_arm.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.btn_arm.toggled.connect(self._on_toggle_arm)

        btn_override = QPushButton("Override")
        btn_override.setObjectName("btnDanger")
        btn_override.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        btn_override.clicked.connect(self._on_manual_override)

        ctrl.addWidget(btn_lock, 0, 0)
        ctrl.addWidget(btn_reset, 0, 1)
        ctrl.addWidget(self.btn_assist, 0, 2)
        ctrl.addWidget(self.btn_arm, 0, 3)
        ctrl.addWidget(btn_override, 0, 4)
        for c in range(5):
            ctrl.setColumnStretch(c, 1)

        # Throttle row
        thr_wrap = QWidget()
        thr = QHBoxLayout(thr_wrap)
        thr.setContentsMargins(0, 2, 0, 0)
        thr.setSpacing(6)

        thr.addWidget(self._muted("THR"))
        self.sp_throttle = QSpinBox()
        self.sp_throttle.setRange(1000, 2000)
        self.sp_throttle.setSingleStep(25)
        self.sp_throttle.setSuffix(" µs")
        self.sp_throttle.setValue(1000)
        self.sp_throttle.setMinimumWidth(118)
        self.sp_throttle.setMaximumWidth(140)
        self.sp_throttle.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.sp_throttle.valueChanged.connect(self._on_spin_throttle_changed)
        thr.addWidget(self.sp_throttle)

        btn_thr_up = QPushButton("+25")
        btn_thr_up.setObjectName("btnCompact")
        btn_thr_up.setFixedWidth(44)
        btn_thr_up.clicked.connect(lambda: self._adjust_throttle(25))
        thr.addWidget(btn_thr_up)

        btn_thr_down = QPushButton("−25")
        btn_thr_down.setObjectName("btnCompact")
        btn_thr_down.setFixedWidth(44)
        btn_thr_down.clicked.connect(lambda: self._adjust_throttle(-25))
        thr.addWidget(btn_thr_down)

        btn_thr_min = QPushButton("MIN")
        btn_thr_min.setObjectName("btnDanger")
        btn_thr_min.setFixedWidth(52)
        btn_thr_min.clicked.connect(lambda: self._set_throttle(1000))
        thr.addWidget(btn_thr_min)

        thr_hint = QLabel("U/J  A/X")
        thr_hint.setStyleSheet("color: #6b7380; font-size: 7.5pt; background: transparent;")
        thr_hint.setToolTip("U +25 · J −25 · A arm · X disarm")
        thr.addWidget(thr_hint)
        thr.addStretch(1)
        ctrl.addWidget(thr_wrap, 1, 0, 1, 5)
        left_l.addWidget(ctrl_bar)

        # Metrics
        metrics_bar = self._panel()
        m_layout = QHBoxLayout(metrics_bar)
        m_layout.setContentsMargins(10, 6, 10, 6)
        m_layout.setSpacing(8)

        self.lbl_lock = self._metric_label("LOCK  NONE")
        self.lbl_dist = self._metric_label("DIST  -- m")
        self.bar_conf = QProgressBar()
        self.bar_conf.setRange(0, 100)
        self.bar_conf.setFormat("CONF %p%")
        self.bar_conf.setMinimumWidth(80)
        self.bar_conf.setMaximumWidth(160)
        self.bar_conf.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.lbl_rc = self._metric_label("RC  R1500 P1500 Y1500")
        self.lbl_rc.setMinimumWidth(140)

        m_layout.addWidget(self.lbl_lock)
        m_layout.addWidget(self.lbl_dist)
        m_layout.addWidget(self.bar_conf, stretch=1)
        m_layout.addWidget(self.lbl_rc)
        left_l.addWidget(metrics_bar)

        # ===================== RIGHT =====================
        right = QWidget()
        right.setMinimumWidth(280)
        right_l = QVBoxLayout(right)
        right_l.setContentsMargins(4, 0, 0, 0)
        right_l.setSpacing(0)

        right_tabs = QTabWidget()
        right_tabs.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        self.pid_panel = PIDTuningPanel(self.sys_config)
        self.pid_panel.pid_updated.connect(self._on_params_changed)
        self.params_panel = ParametersPanel(self.sys_config)
        self.params_panel.params_updated.connect(self._on_params_changed)
        self.params_panel.preset_loaded.connect(self._on_preset_loaded)
        self.telemetry_plots = RealTimeTelemetryPlots()
        self.telemetry_plots.setMinimumHeight(180)
        self.img_proc_widget = ImageProcessingWidget()

        right_tabs.addTab(self.pid_panel, "PID")
        right_tabs.addTab(self.params_panel, "Params")
        right_tabs.addTab(self.telemetry_plots, "Charts")
        right_tabs.addTab(self.img_proc_widget, "Vision")
        right_l.addWidget(right_tabs)

        splitter.addWidget(left)
        splitter.addWidget(right)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        splitter.setSizes([780, 360])
        root.addWidget(splitter, stretch=1)

        self.worker.frame_processed.connect(self._on_frame_processed)

    @staticmethod
    def _panel() -> QFrame:
        frame = QFrame()
        frame.setObjectName("panel")
        frame.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        return frame

    @staticmethod
    def _muted(text: str) -> QLabel:
        lbl = QLabel(text)
        lbl.setObjectName("formLabel")
        lbl.setAlignment(Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft)
        lbl.setFixedWidth(56)
        return lbl

    @staticmethod
    def _metric_label(text: str) -> QLabel:
        lbl = QLabel(text)
        lbl.setStyleSheet(
            "color: #9aa3b2; font-family: Consolas, 'Cascadia Mono', monospace; "
            "font-size: 8pt; background: transparent;"
        )
        return lbl

    def _refresh_camera_devices(self) -> None:
        self.combo_cameras.blockSignals(True)
        self.combo_cameras.clear()
        for idx, label in list_camera_devices():
            short = label.replace(" (Capture Card / Video Input)", "")
            if len(short) > 42:
                short = short[:39] + "…"
            self.combo_cameras.addItem(short, idx)
        self.combo_cameras.blockSignals(False)

    def _on_camera_changed(self) -> None:
        idx = self.combo_cameras.currentData()
        if idx is not None:
            self.worker.switch_camera(idx)

    def _refresh_serial_ports(self) -> None:
        self.combo_ports.clear()
        ports = list_serial_ports()
        if not ports:
            self.combo_ports.addItem("No ports", "")
            self.combo_ports.setEnabled(False)
            self.btn_connect.setEnabled(False)
        else:
            self.combo_ports.setEnabled(True)
            self.btn_connect.setEnabled(True)
            for dev, label in ports:
                short = label if len(label) <= 36 else label[:33] + "…"
                self.combo_ports.addItem(short, dev)

    def _toggle_serial_connection(self) -> None:
        if self.worker.is_connected:
            self.worker.disconnect_serial()
            self.btn_connect.setText("Connect")
            self.btn_connect.setEnabled(True)
            self.lbl_serial_status.set_status("OFFLINE", "error")
        else:
            port = self.combo_ports.currentData()
            baud = self.combo_baud.currentData()
            if not port:
                QMessageBox.warning(self, "Serial", "No port selected.")
                return

            self.btn_connect.setText("…")
            self.btn_connect.setEnabled(False)
            self.lbl_serial_status.set_status("WAIT", "warn")

            self._conn_thread = ConnectionWorker(self.worker, port, baud)
            self._conn_thread.finished_signal.connect(self._on_connection_finished)
            self._conn_thread.start()

    def _on_connection_finished(self, ok: bool, msg: str, port: str) -> None:
        self.btn_connect.setEnabled(True)
        if ok:
            self.btn_connect.setText("Disconnect")
            short = port if len(port) <= 8 else port[:8]
            self.lbl_serial_status.set_status(f"OK {short}", "ok")
        else:
            self.btn_connect.setText("Connect")
            self.lbl_serial_status.set_status("ERROR", "error")
            QMessageBox.critical(self, "Serial Error", msg)

    def _adjust_throttle(self, delta: int) -> None:
        thr = self.worker.adjust_throttle(delta)
        self.update_throttle_ui(thr)

    def _set_throttle(self, value: int) -> None:
        thr = self.worker.set_throttle(value)
        self.update_throttle_ui(thr)

    def _on_spin_throttle_changed(self, value: int) -> None:
        self.worker.set_throttle(value)

    def update_throttle_ui(self, value: int) -> None:
        self.sp_throttle.blockSignals(True)
        self.sp_throttle.setValue(int(value))
        self.sp_throttle.blockSignals(False)

    @pyqtSlot(int, int, int, int)
    def _on_roi_selected(self, x: int, y: int, w: int, h: int) -> None:
        self.worker.set_roi_lock(x, y, w, h)

    def _on_reset_lock(self) -> None:
        self.worker.reset_lock()
        self.btn_assist.setChecked(False)

    def _on_toggle_assist(self, checked: bool) -> None:
        self.worker.assist_enabled = checked
        self.btn_assist.setText("Following" if checked else "Follow")

    def _on_toggle_arm(self, checked: bool) -> None:
        self.worker.arm_requested = checked
        self.btn_arm.setText("DISARM" if checked else "ARM")

    def _on_manual_override(self) -> None:
        self.worker.failsafe.trigger_manual_override(True)
        self.btn_assist.setChecked(False)
        QMessageBox.warning(self, "Override", "Manual override engaged.")

    def _on_params_changed(self) -> None:
        self.worker.update_config(self.sys_config)

    def _on_preset_loaded(self, cfg) -> None:
        self.sys_config = cfg
        self.worker.update_config(cfg)
        self.pid_panel.load_config(cfg)

    @pyqtSlot(object, object)
    def _on_frame_processed(self, frame_bgr, rec) -> None:
        self.video_widget.update_frame(frame_bgr)
        self.telemetry_plots.update_telemetry(rec)
        self.img_proc_widget.update_processing_views(
            frame_bgr,
            pixel_engine=self.worker.hybrid.pixel_engine,
            target_hist=self.worker.hybrid._target_hist,
            bbox=(int(rec.bbox_x), int(rec.bbox_y), int(rec.bbox_w), int(rec.bbox_h)) if rec.locked else None,
        )
        if rec.locked:
            self.lbl_lock.setText(f"LOCK  ACTIVE · {rec.source.upper()}")
            self.lbl_lock.setStyleSheet(
                "color: #3d8f6a; font-family: Consolas, monospace; font-size: 8pt; background: transparent;"
            )
            self.pill_feed.set_status("TRACKING", "ok")
        else:
            self.lbl_lock.setText("LOCK  NONE")
            self.lbl_lock.setStyleSheet(
                "color: #9aa3b2; font-family: Consolas, monospace; font-size: 8pt; background: transparent;"
            )
            self.pill_feed.set_status("LIVE", "info")

        self.lbl_dist.setText(f"DIST  {rec.estimated_distance_m:.1f} m")
        self.bar_conf.setValue(int(rec.confidence * 100))
        self.lbl_rc.setText(f"RC  R{rec.rc_roll}  P{rec.rc_pitch}  Y{rec.rc_yaw}")
