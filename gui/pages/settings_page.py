"""Comprehensive Settings Page combining PID, Tracking Params, AUX, and Device Options."""

from __future__ import annotations

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from config import SystemConfig
from gui.parameters_panel import ParametersPanel
from gui.pid_panel import PIDTuningPanel
from gui.widgets.page_header import PageHeader


class AuxChannelsPanel(QWidget):
    aux_updated = pyqtSignal()

    def __init__(self, sys_config: SystemConfig, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.sys_config = sys_config
        self._init_ui()

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)

        grp_mapping = QGroupBox("Channel Mapping")
        map_layout = QFormLayout(grp_mapping)

        self.sp_arm_ch = QSpinBox()
        self.sp_arm_ch.setRange(0, 15)
        self.sp_arm_ch.setValue(self.sys_config.aux_channels.arm_channel)
        self.sp_arm_ch.valueChanged.connect(self._on_change)

        self.sp_mode_ch = QSpinBox()
        self.sp_mode_ch.setRange(0, 15)
        self.sp_mode_ch.setValue(self.sys_config.aux_channels.mode_channel)
        self.sp_mode_ch.valueChanged.connect(self._on_change)

        map_layout.addRow("ARM Channel Index (0-15):", self.sp_arm_ch)
        map_layout.addRow("Flight Mode Channel Index (0-15):", self.sp_mode_ch)
        layout.addWidget(grp_mapping)

        grp_values = QGroupBox("PWM Values (µs)")
        val_layout = QFormLayout(grp_values)

        self.sp_arm_high = QSpinBox()
        self.sp_arm_high.setRange(1000, 2000)
        self.sp_arm_high.setValue(self.sys_config.aux_channels.arm_high)
        self.sp_arm_high.valueChanged.connect(self._on_change)

        self.sp_arm_low = QSpinBox()
        self.sp_arm_low.setRange(1000, 2000)
        self.sp_arm_low.setValue(self.sys_config.aux_channels.arm_low)
        self.sp_arm_low.valueChanged.connect(self._on_change)

        self.sp_mode_high = QSpinBox()
        self.sp_mode_high.setRange(1000, 2000)
        self.sp_mode_high.setValue(self.sys_config.aux_channels.mode_high)
        self.sp_mode_high.valueChanged.connect(self._on_change)

        self.sp_mode_low = QSpinBox()
        self.sp_mode_low.setRange(1000, 2000)
        self.sp_mode_low.setValue(self.sys_config.aux_channels.mode_low)
        self.sp_mode_low.valueChanged.connect(self._on_change)

        val_layout.addRow("ARM High (Armed):", self.sp_arm_high)
        val_layout.addRow("ARM Low (Disarmed):", self.sp_arm_low)
        val_layout.addRow("Mode High (Follow):", self.sp_mode_high)
        val_layout.addRow("Mode Low (Manual):", self.sp_mode_low)
        layout.addWidget(grp_values)

        layout.addStretch()

    def _on_change(self) -> None:
        self.sys_config.aux_channels.arm_channel = self.sp_arm_ch.value()
        self.sys_config.aux_channels.mode_channel = self.sp_mode_ch.value()
        self.sys_config.aux_channels.arm_high = self.sp_arm_high.value()
        self.sys_config.aux_channels.arm_low = self.sp_arm_low.value()
        self.sys_config.aux_channels.mode_high = self.sp_mode_high.value()
        self.sys_config.aux_channels.mode_low = self.sp_mode_low.value()
        self.aux_updated.emit()

    def load_config(self, cfg: SystemConfig) -> None:
        self.sys_config = cfg
        self.sp_arm_ch.blockSignals(True)
        self.sp_mode_ch.blockSignals(True)
        self.sp_arm_high.blockSignals(True)
        self.sp_arm_low.blockSignals(True)
        self.sp_mode_high.blockSignals(True)
        self.sp_mode_low.blockSignals(True)

        self.sp_arm_ch.setValue(cfg.aux_channels.arm_channel)
        self.sp_mode_ch.setValue(cfg.aux_channels.mode_channel)
        self.sp_arm_high.setValue(cfg.aux_channels.arm_high)
        self.sp_arm_low.setValue(cfg.aux_channels.arm_low)
        self.sp_mode_high.setValue(cfg.aux_channels.mode_high)
        self.sp_mode_low.setValue(cfg.aux_channels.mode_low)

        self.sp_arm_ch.blockSignals(False)
        self.sp_mode_ch.blockSignals(False)
        self.sp_arm_high.blockSignals(False)
        self.sp_arm_low.blockSignals(False)
        self.sp_mode_high.blockSignals(False)
        self.sp_mode_low.blockSignals(False)


class DeviceOptionsPanel(QWidget):
    device_updated = pyqtSignal()

    def __init__(self, sys_config: SystemConfig, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.sys_config = sys_config
        self._init_ui()

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)

        grp_camera = QGroupBox("Camera & Vision")
        cam_layout = QFormLayout(grp_camera)

        self.sp_cam_idx = QSpinBox()
        self.sp_cam_idx.setRange(0, 10)
        self.sp_cam_idx.setValue(self.sys_config.camera.camera_index)
        self.sp_cam_idx.valueChanged.connect(self._on_change)

        self.sp_fov_h = QDoubleSpinBox()
        self.sp_fov_h.setRange(10.0, 180.0)
        self.sp_fov_h.setValue(self.sys_config.camera.fov_h_deg)
        self.sp_fov_h.valueChanged.connect(self._on_change)

        self.sp_fov_v = QDoubleSpinBox()
        self.sp_fov_v.setRange(10.0, 180.0)
        self.sp_fov_v.setValue(self.sys_config.camera.fov_v_deg)
        self.sp_fov_v.valueChanged.connect(self._on_change)

        cam_layout.addRow("Camera Index:", self.sp_cam_idx)
        cam_layout.addRow("Horizontal FOV (deg):", self.sp_fov_h)
        cam_layout.addRow("Vertical FOV (deg):", self.sp_fov_v)
        layout.addWidget(grp_camera)

        grp_app = QGroupBox("Application Preferences")
        app_layout = QFormLayout(grp_app)
        
        self.sp_ui_scale = QDoubleSpinBox()
        self.sp_ui_scale.setRange(0.5, 2.5)
        self.sp_ui_scale.setSingleStep(0.1)
        self.sp_ui_scale.setValue(self.sys_config.device.ui_scale)
        self.sp_ui_scale.valueChanged.connect(self._on_change)

        app_layout.addRow("UI Scale Factor:", self.sp_ui_scale)
        layout.addWidget(grp_app)

        layout.addStretch()

    def _on_change(self) -> None:
        self.sys_config.camera.camera_index = self.sp_cam_idx.value()
        self.sys_config.camera.fov_h_deg = self.sp_fov_h.value()
        self.sys_config.camera.fov_v_deg = self.sp_fov_v.value()
        self.sys_config.device.ui_scale = self.sp_ui_scale.value()
        self.device_updated.emit()

    def load_config(self, cfg: SystemConfig) -> None:
        self.sys_config = cfg
        self.sp_cam_idx.blockSignals(True)
        self.sp_fov_h.blockSignals(True)
        self.sp_fov_v.blockSignals(True)
        self.sp_ui_scale.blockSignals(True)

        self.sp_cam_idx.setValue(cfg.camera.camera_index)
        self.sp_fov_h.setValue(cfg.camera.fov_h_deg)
        self.sp_fov_v.setValue(cfg.camera.fov_v_deg)
        self.sp_ui_scale.setValue(cfg.device.ui_scale)

        self.sp_cam_idx.blockSignals(False)
        self.sp_fov_h.blockSignals(False)
        self.sp_fov_v.blockSignals(False)
        self.sp_ui_scale.blockSignals(False)


class SettingsPage(QWidget):
    config_updated = pyqtSignal()

    def __init__(self, sys_config: SystemConfig, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.sys_config = sys_config
        self._init_ui()

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        
        layout.addWidget(PageHeader("System Settings", "Configure application, tracking, PID, and AUX parameters"))

        self.tabs = QTabWidget()
        
        self.panel_params = ParametersPanel(self.sys_config)
        self.panel_pid = PIDTuningPanel(self.sys_config)
        self.panel_aux = AuxChannelsPanel(self.sys_config)
        self.panel_device = DeviceOptionsPanel(self.sys_config)

        # Wire up signals
        self.panel_params.params_updated.connect(self.config_updated.emit)
        self.panel_params.preset_loaded.connect(self._on_preset_loaded)
        self.panel_pid.pid_updated.connect(self.config_updated.emit)
        self.panel_aux.aux_updated.connect(self.config_updated.emit)
        self.panel_device.device_updated.connect(self.config_updated.emit)

        # We'll put the Parameters panel in a scroll area since it has many items
        scroll_params = QScrollArea()
        scroll_params.setWidgetResizable(True)
        scroll_params.setWidget(self.panel_params)
        scroll_params.setFrameShape(QScrollArea.Shape.NoFrame)

        self.tabs.addTab(scroll_params, "Tracking Parameters")
        self.tabs.addTab(self.panel_pid, "PID Tuning")
        self.tabs.addTab(self.panel_aux, "AUX Channels")
        self.tabs.addTab(self.panel_device, "Device Options")

        layout.addWidget(self.tabs, stretch=1)

    def _on_preset_loaded(self, cfg: SystemConfig) -> None:
        self.sys_config = cfg
        self.panel_pid.load_config(cfg)
        self.panel_aux.load_config(cfg)
        self.panel_device.load_config(cfg)
        self.config_updated.emit()
