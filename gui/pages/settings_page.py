"""System Settings — compact single-page layout that fits the screen."""

from __future__ import annotations

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QScrollArea,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from config import SystemConfig
from gui.pages.aux_channels_panel import AuxChannelsPanel
from gui.parameters_panel import ParametersPanel
from gui.pid_panel import PIDTuningPanel
from gui.widgets.page_header import PageHeader


class DeviceOptionsPanel(QWidget):
    device_updated = pyqtSignal()

    def __init__(self, sys_config: SystemConfig, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.sys_config = sys_config
        self._init_ui()

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        box = QGroupBox("Camera & App")
        form = QFormLayout(box)
        form.setContentsMargins(10, 12, 10, 10)
        form.setSpacing(6)

        self.sp_cam_idx = QSpinBox()
        self.sp_cam_idx.setRange(0, 10)
        self.sp_cam_idx.setFixedWidth(88)
        self.sp_cam_idx.setValue(self.sys_config.camera.camera_index)
        self.sp_cam_idx.valueChanged.connect(self._on_change)

        self.sp_fov_h = QDoubleSpinBox()
        self.sp_fov_h.setRange(10.0, 180.0)
        self.sp_fov_h.setFixedWidth(88)
        self.sp_fov_h.setValue(self.sys_config.camera.fov_h_deg)
        self.sp_fov_h.valueChanged.connect(self._on_change)

        self.sp_fov_v = QDoubleSpinBox()
        self.sp_fov_v.setRange(10.0, 180.0)
        self.sp_fov_v.setFixedWidth(88)
        self.sp_fov_v.setValue(self.sys_config.camera.fov_v_deg)
        self.sp_fov_v.valueChanged.connect(self._on_change)

        self.sp_ui_scale = QDoubleSpinBox()
        self.sp_ui_scale.setRange(0.5, 2.5)
        self.sp_ui_scale.setSingleStep(0.1)
        self.sp_ui_scale.setFixedWidth(88)
        self.sp_ui_scale.setValue(self.sys_config.device.ui_scale)
        self.sp_ui_scale.valueChanged.connect(self._on_change)

        form.addRow("Camera index", self.sp_cam_idx)
        form.addRow("FOV H (°)", self.sp_fov_h)
        form.addRow("FOV V (°)", self.sp_fov_v)
        form.addRow("UI scale", self.sp_ui_scale)
        layout.addWidget(box)
        layout.addStretch(1)

    def _on_change(self) -> None:
        self.sys_config.camera.camera_index = self.sp_cam_idx.value()
        self.sys_config.camera.fov_h_deg = self.sp_fov_h.value()
        self.sys_config.camera.fov_v_deg = self.sp_fov_v.value()
        self.sys_config.device.ui_scale = self.sp_ui_scale.value()
        self.device_updated.emit()

    def load_config(self, cfg: SystemConfig) -> None:
        self.sys_config = cfg
        for w in (self.sp_cam_idx, self.sp_fov_h, self.sp_fov_v, self.sp_ui_scale):
            w.blockSignals(True)
        self.sp_cam_idx.setValue(cfg.camera.camera_index)
        self.sp_fov_h.setValue(cfg.camera.fov_h_deg)
        self.sp_fov_v.setValue(cfg.camera.fov_v_deg)
        self.sp_ui_scale.setValue(cfg.device.ui_scale)
        for w in (self.sp_cam_idx, self.sp_fov_h, self.sp_fov_v, self.sp_ui_scale):
            w.blockSignals(False)


class SettingsPage(QWidget):
    """All settings visible in one scrollable page — no nested tabs of tabs."""

    config_updated = pyqtSignal()

    def __init__(self, sys_config: SystemConfig, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.sys_config = sys_config
        self._init_ui()

    def _init_ui(self) -> None:
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(8)

        outer.addWidget(
            PageHeader("System Settings", "PID · tracking · AUX · camera — compact single view")
        )

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        content = QWidget()
        lay = QVBoxLayout(content)
        lay.setContentsMargins(0, 0, 8, 0)
        lay.setSpacing(10)

        self.panel_pid = PIDTuningPanel(self.sys_config)
        self.panel_params = ParametersPanel(self.sys_config)
        self.panel_aux = AuxChannelsPanel(self.sys_config)
        self.panel_device = DeviceOptionsPanel(self.sys_config)

        self.panel_params.params_updated.connect(self.config_updated.emit)
        self.panel_params.preset_loaded.connect(self._on_preset_loaded)
        self.panel_pid.pid_updated.connect(self.config_updated.emit)
        self.panel_aux.aux_updated.connect(self.config_updated.emit)
        self.panel_device.device_updated.connect(self.config_updated.emit)

        # Top: PID + Device side by side
        row1 = QHBoxLayout()
        row1.setSpacing(10)
        row1.addWidget(self.panel_pid, stretch=3)
        row1.addWidget(self.panel_device, stretch=1)
        lay.addLayout(row1)

        # Middle: Tracking params
        lay.addWidget(self.panel_params)

        # Bottom: AUX
        lay.addWidget(self.panel_aux)
        lay.addStretch(1)

        scroll.setWidget(content)
        outer.addWidget(scroll, stretch=1)

    def _on_preset_loaded(self, cfg: SystemConfig) -> None:
        self.sys_config = cfg
        self.panel_pid.load_config(cfg)
        self.panel_aux.load_config(cfg)
        self.panel_device.load_config(cfg)
        self.config_updated.emit()
