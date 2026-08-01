"""Arjuna GCS main application shell with sidebar navigation."""

from __future__ import annotations

import os

os.environ["OPENCV_LOG_LEVEL"] = "OFF"
os.environ["OPENCV_VIDEOINPUT_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

from PyQt6.QtCore import Qt, pyqtSlot
from PyQt6.QtGui import QKeyEvent
from PyQt6.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QSizePolicy,
    QSpinBox,
    QStackedWidget,
    QStatusBar,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from config import SystemConfig
from control.joystick_manager import JoystickManager
from core.tracking_worker import TrackingWorkerThread
from database.target_store import TargetStore
from gui.calibration_wizard import CalibrationWizard
from gui.pages.dashboard_page import DashboardPage
from gui.pages.distance_calib_page import DistanceCalibPage
from gui.pages.joystick_page import JoystickPage
from gui.pages.live_feed_page import LiveFeedPage
from gui.pages.logs_page import LogsPage
from gui.pages.placeholder_page import PlaceholderPage
from gui.pages.target_database_page import TargetDatabasePage
from gui.pages.telemetry_page import TelemetryPage
from gui.pid_panel import PIDTuningPanel
from gui.style import ARJUNA_THEME_QSS
from gui.widgets.page_header import StatusPill
from sys_logging.system_logger import LogCategory, SystemLogger
from estimation.distance_calib import load_distance_calib


NAV_ITEMS: list[tuple[str, str]] = [
    ("dashboard", "01  Dashboard"),
    ("live_feed", "02  Live Camera Feed"),
    ("target_database", "03  Target Database"),
    ("telemetry", "04  Flight Telemetry"),
    ("joystick", "05  Remote Control"),
    ("logs", "06  Logs"),
    ("distance_calib", "07  Distance Calib"),
    ("calibration", "08  Calibration"),
    ("settings", "09  Settings"),
]


class ArjunaShell(QMainWindow):
    """Professional Ground Control Station shell for Arjuna."""

    def __init__(self, sys_config: SystemConfig | None = None) -> None:
        super().__init__()
        self.sys_config = sys_config or SystemConfig()
        load_distance_calib(self.sys_config)
        self.target_store = TargetStore()
        self.sys_log = SystemLogger()
        self.joystick_mgr = JoystickManager(self.sys_config)

        self.setWindowTitle("ARJUNA  ·  Ground Control Station")
        self.resize(1560, 920)
        self.setMinimumSize(1180, 720)
        self.setStyleSheet(ARJUNA_THEME_QSS)

        self.worker = TrackingWorkerThread(self.sys_config, self.target_store, self.joystick_mgr)
        self._init_ui()
        self._wire_signals()
        self.worker.start()

        self.sys_log.log(LogCategory.SYSTEM, "Arjuna GCS initialized", module="Shell")

    def _init_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ----- Sidebar -----
        sidebar = QFrame()
        sidebar.setObjectName("sidebar")
        sidebar.setFixedWidth(210)
        sb = QVBoxLayout(sidebar)
        sb.setContentsMargins(0, 0, 0, 0)
        sb.setSpacing(0)

        brand = QFrame()
        brand.setObjectName("brandBlock")
        brand_l = QVBoxLayout(brand)
        brand_l.setContentsMargins(14, 16, 14, 14)
        brand_l.setSpacing(2)

        mark = QLabel("◈  AUTONOMOUS GCS")
        mark.setObjectName("brandMark")
        brand_l.addWidget(mark)

        title = QLabel("ARJUNA")
        title.setObjectName("brandLabel")
        brand_l.addWidget(title)

        sub = QLabel("TARGET TRACKING PLATFORM")
        sub.setObjectName("brandSubtitle")
        brand_l.addWidget(sub)
        sb.addWidget(brand)

        self.nav_list = QListWidget()
        self.nav_list.setObjectName("navList")
        self.nav_list.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.nav_list.setVerticalScrollMode(QListWidget.ScrollMode.ScrollPerPixel)
        for key, label in NAV_ITEMS:
            item = QListWidgetItem(label)
            item.setData(Qt.ItemDataRole.UserRole, key)
            self.nav_list.addItem(item)
        self.nav_list.currentRowChanged.connect(self._on_nav_changed)
        sb.addWidget(self.nav_list, stretch=1)

        self.lbl_nav_status = QLabel("●  SYSTEM READY")
        self.lbl_nav_status.setObjectName("navStatus")
        self.lbl_nav_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        sb.addWidget(self.lbl_nav_status)

        root.addWidget(sidebar)

        # ----- Content -----
        content = QFrame()
        content.setObjectName("contentChrome")
        content_l = QVBoxLayout(content)
        content_l.setContentsMargins(0, 0, 0, 0)
        content_l.setSpacing(0)

        # Top status strip
        topbar = QFrame()
        topbar.setObjectName("panel")
        topbar.setStyleSheet(
            "QFrame#panel { border: none; border-bottom: 1px solid #2a3038; "
            "border-radius: 0; background: #13161b; }"
        )
        top_l = QHBoxLayout(topbar)
        top_l.setContentsMargins(16, 8, 16, 8)
        top_l.setSpacing(10)

        self.pill_mode = StatusPill("STANDBY", "neutral")
        self.pill_lock = StatusPill("NO LOCK", "neutral")
        self.pill_link = StatusPill("MSP OFF", "error")
        self.pill_fps = StatusPill("FPS --", "info")

        top_l.addWidget(self.pill_mode)
        top_l.addWidget(self.pill_lock)
        top_l.addWidget(self.pill_link)
        top_l.addWidget(self.pill_fps)
        top_l.addStretch()

        self.lbl_active_page = QLabel("DASHBOARD")
        self.lbl_active_page.setStyleSheet(
            "color: #6b7380; font-size: 8pt; letter-spacing: 2px; font-weight: 650; background: transparent;"
        )
        top_l.addWidget(self.lbl_active_page)
        content_l.addWidget(topbar)

        self.stack = QStackedWidget()
        self._pages: dict[str, QWidget] = {}

        self.page_dashboard = DashboardPage()
        self.page_live_feed = LiveFeedPage(self.worker, self.sys_config)
        self.page_target_db = TargetDatabasePage(self.target_store)
        self.page_joystick = JoystickPage(self.sys_config, self.joystick_mgr)
        self.page_logs = LogsPage(self.sys_log)
        self.page_telemetry = TelemetryPage()
        self.page_distance_calib = DistanceCalibPage(self.worker, self.sys_config)
        self.page_distance_calib.calib_saved.connect(self._on_config_updated)

        self._register_page("dashboard", self.page_dashboard)
        self._register_page("live_feed", self.page_live_feed)
        self._register_page("target_database", self.page_target_db)
        self._register_page("joystick", self.page_joystick)
        self._register_page("logs", self.page_logs)
        self._register_page("telemetry", self.page_telemetry)
        self._register_page("distance_calib", self.page_distance_calib)

        # Stick / follow calibration wizard (legacy)

        calib_widget = QWidget()
        calib_layout = QVBoxLayout(calib_widget)
        calib_layout.setContentsMargins(20, 20, 20, 20)
        from gui.widgets.page_header import PageHeader

        calib_layout.addWidget(PageHeader("Calibration", "Tune camera geometry and follow-controller response"))
        calib_note = QLabel("Use the calibration wizard to verify stick directions and follow gains before armed flight.")
        calib_note.setStyleSheet("color: #6b7380; background: transparent;")
        calib_note.setWordWrap(True)
        calib_layout.addWidget(calib_note)
        btn_calib = QPushButton("Open Calibration Wizard")
        btn_calib.setObjectName("btnPrimary")
        btn_calib.setFixedWidth(220)
        btn_calib.clicked.connect(self._open_calibration)
        calib_layout.addWidget(btn_calib)
        calib_layout.addStretch()
        self._register_page("calibration", calib_widget)

        from gui.pages.settings_page import SettingsPage
        self.settings_page = SettingsPage(self.sys_config)
        self.settings_page.config_updated.connect(self._on_config_updated)
        self._register_page("settings", self.settings_page)

        content_l.addWidget(self.stack, stretch=1)
        root.addWidget(content, stretch=1)

        self.nav_list.setCurrentRow(0)

        status = QStatusBar()
        self.lbl_status_fps = QLabel("FPS  --")
        self.lbl_status_target = QLabel("TARGET  NONE")
        self.lbl_status_serial = QLabel("SERIAL  OFF")
        self.lbl_status_frame = QLabel("FRAME  0")
        status.addPermanentWidget(self.lbl_status_frame)
        status.addPermanentWidget(self.lbl_status_fps)
        status.addPermanentWidget(self.lbl_status_target)
        status.addPermanentWidget(self.lbl_status_serial)
        self.setStatusBar(status)
        status.showMessage("ARJUNA GCS  ·  Operational  ·  Select a subsystem to begin")

    def _register_page(self, key: str, widget: QWidget) -> None:
        wrapper = QWidget()
        wrap_l = QVBoxLayout(wrapper)
        wrap_l.setContentsMargins(10, 8, 10, 8)
        wrap_l.setSpacing(0)
        widget.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        wrap_l.addWidget(widget)
        self._pages[key] = wrapper
        self.stack.addWidget(wrapper)

    def _wire_signals(self) -> None:
        self.worker.frame_processed.connect(self._on_frame_processed)
        self.worker.fps_updated.connect(self._on_fps_updated)
        self.worker.target_changed.connect(self._on_target_changed)

    def _on_nav_changed(self, index: int) -> None:
        item = self.nav_list.item(index)
        if not item:
            return
        key = item.data(Qt.ItemDataRole.UserRole)
        if key in self._pages:
            self.stack.setCurrentWidget(self._pages[key])
            label = item.text().split("  ", 1)[-1].upper()
            self.lbl_active_page.setText(label)
            if key == "target_database":
                self.page_target_db.refresh_list()

    @pyqtSlot(object, object)
    def _on_frame_processed(self, frame, rec) -> None:
        self.page_dashboard.update_telemetry(rec)
        self.page_telemetry.update_telemetry(rec)
        self.lbl_status_frame.setText(f"FRAME  {rec.frame_idx}")

        if rec.locked:
            self.pill_lock.set_status(f"LOCK {rec.source.upper()}", "ok")
            self.pill_mode.set_status("TRACKING", "info")
        else:
            self.pill_lock.set_status("NO LOCK", "neutral")
            self.pill_mode.set_status("STANDBY", "neutral")

        if self.worker.is_connected:
            self.pill_link.set_status("MSP LINK", "ok")
            self.lbl_status_serial.setText(f"SERIAL  {self.worker.port_name or 'ON'}")
            self.page_dashboard.set_serial_connected(True, self.worker.port_name)
            self.page_telemetry.set_serial_connected(True)
        else:
            self.pill_link.set_status("MSP OFF", "error")
            self.lbl_status_serial.setText("SERIAL  OFF")
            self.page_dashboard.set_serial_connected(False)
            self.page_telemetry.set_serial_connected(False)

    @pyqtSlot(float)
    def _on_fps_updated(self, fps: float) -> None:
        self.page_dashboard.update_fps(fps)
        self.lbl_status_fps.setText(f"FPS  {fps:.0f}")
        tone = "ok" if fps >= 20 else ("warn" if fps >= 10 else "error")
        self.pill_fps.set_status(f"FPS {fps:.0f}", tone)

    @pyqtSlot(object)
    def _on_target_changed(self, profile) -> None:
        if profile:
            tid = profile.target_id
            self.page_dashboard.set_active_target(tid)
            self.lbl_status_target.setText(f"TARGET  {tid}")
        else:
            self.page_dashboard.set_active_target(None)
            self.lbl_status_target.setText("TARGET  NONE")

    def _on_config_updated(self) -> None:
        self.worker.update_config(self.sys_config)
        # Keep Live Feed PID/Params panels in sync with Settings
        if hasattr(self.page_live_feed, "pid_panel"):
            self.page_live_feed.pid_panel.load_config(self.sys_config)
        if hasattr(self.page_live_feed, "params_panel"):
            self.page_live_feed.params_panel.load_config(self.sys_config)
        if hasattr(self, "page_distance_calib"):
            self.page_distance_calib._refresh_status_labels()
        if hasattr(self, "settings_page") and hasattr(self.settings_page, "panel_params"):
            self.settings_page.panel_params.load_config(self.sys_config)

    def _open_calibration(self) -> None:
        from core.config_manager import ConfigManager

        wizard = CalibrationWizard(ConfigManager(), parent=self)
        wizard.exec()

    def keyPressEvent(self, event: QKeyEvent) -> None:
        focus_widget = QApplication.focusWidget()
        if focus_widget and isinstance(focus_widget, (QLineEdit, QTextEdit, QPlainTextEdit, QSpinBox, QDoubleSpinBox)):
            super().keyPressEvent(event)
            return

        key = event.key()
        text = event.text().upper()

        if key == Qt.Key.Key_L or text == "L":
            # L = start manual lock selection mode
            if hasattr(self.page_live_feed, "video_widget"):
                self.page_live_feed.video_widget.setFocus()
            self.statusBar().showMessage("MANUAL LOCK MODE (Hotkey L): Drag box on video feed to lock target", 4000)
            self.sys_log.log(LogCategory.SYSTEM, "Manual Lock Selection Mode activated (Hotkey L)", module="Hotkeys")
        elif key == Qt.Key.Key_A or text == "A":
            # A = arm
            self.worker.arm_drone()
            if hasattr(self.page_live_feed, "btn_arm"):
                self.page_live_feed.btn_arm.setChecked(True)
            self.statusBar().showMessage("DRONE ARMED (Hotkey A)", 4000)
        elif key == Qt.Key.Key_X or text == "X":
            # X = disarm
            self.worker.disarm_drone()
            if hasattr(self.page_live_feed, "btn_arm"):
                self.page_live_feed.btn_arm.setChecked(False)
            if hasattr(self.page_live_feed, "update_throttle_ui"):
                self.page_live_feed.update_throttle_ui(1000)
            self.statusBar().showMessage("DRONE DISARMED - Throttle reset to 1000 µs (Hotkey X)", 4000)
        elif key == Qt.Key.Key_M or text == "M":
            # M = toggle flight mode only
            mode = self.worker.toggle_flight_mode()
            self.statusBar().showMessage(f"FLIGHT MODE TOGGLED: {mode} (Hotkey M)", 4000)
        elif key == Qt.Key.Key_U or text == "U":
            # U = throttle +25
            thr = self.worker.adjust_throttle(25)
            if hasattr(self.page_live_feed, "update_throttle_ui"):
                self.page_live_feed.update_throttle_ui(thr)
            self.statusBar().showMessage(f"THROTTLE: {thr} µs (+25, Hotkey U)", 2000)
        elif key == Qt.Key.Key_J or text == "J":
            # J = throttle -25
            thr = self.worker.adjust_throttle(-25)
            if hasattr(self.page_live_feed, "update_throttle_ui"):
                self.page_live_feed.update_throttle_ui(thr)
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
            # S = check arm status + FC arming disable flags
            armed = getattr(self.worker, 'arm_requested', False)
            thr = getattr(self.worker, 'throttle_value', 1000)
            mode = getattr(self.worker, 'flight_mode', 'ANGLE')
            fc_status = "CONNECTED" if self.worker.is_connected else "DISCONNECTED"
            msg = f"STATUS CHECK: FC={fc_status} | ARM={'ARMED' if armed else 'DISARMED'} | Mode={mode} | Throttle={thr} µs"
            # Query arming-disable flags from FC
            if self.worker.is_connected and hasattr(self.worker, "fc") and hasattr(self.worker.fc, "_query_arming_disable_flags"):
                reasons = self.worker.fc._query_arming_disable_flags()
                if reasons:
                    msg += f"  ⚠ FC BLOCK: {', '.join(reasons)}"
            self.statusBar().showMessage(msg, 8000)
            self.sys_log.log(LogCategory.SYSTEM, msg, module="Status Check")
        else:
            super().keyPressEvent(event)

    def closeEvent(self, event) -> None:
        self.worker.stop()
        event.accept()
