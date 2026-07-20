"""PID Tuning Panel Widget for Real-Time Parameter Adjustment."""

from __future__ import annotations

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QSlider,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from config import PIDAxisConfig, SystemConfig


class SinglePIDGroup(QGroupBox):
    changed = pyqtSignal()

    def __init__(self, title: str, config: PIDAxisConfig, max_kp: float = 1000.0, parent: QWidget | None = None) -> None:
        super().__init__(title, parent)
        self.cfg = config
        self._init_ui(max_kp)

    def _init_ui(self, max_kp: float) -> None:
        layout = QFormLayout(self)

        # Kp
        self.sp_kp = QDoubleSpinBox()
        self.sp_kp.setRange(0.0, max_kp)
        self.sp_kp.setSingleStep(5.0)
        self.sp_kp.setValue(self.cfg.kp)
        self.sp_kp.valueChanged.connect(self._on_change)

        # Ki
        self.sp_ki = QDoubleSpinBox()
        self.sp_ki.setRange(0.0, 500.0)
        self.sp_ki.setSingleStep(1.0)
        self.sp_ki.setValue(self.cfg.ki)
        self.sp_ki.valueChanged.connect(self._on_change)

        # Kd
        self.sp_kd = QDoubleSpinBox()
        self.sp_kd.setRange(0.0, 500.0)
        self.sp_kd.setSingleStep(1.0)
        self.sp_kd.setValue(self.cfg.kd)
        self.sp_kd.valueChanged.connect(self._on_change)

        # Max Output
        self.sp_max = QDoubleSpinBox()
        self.sp_max.setRange(10.0, 500.0)
        self.sp_max.setSingleStep(10.0)
        self.sp_max.setValue(self.cfg.max_output)
        self.sp_max.valueChanged.connect(self._on_change)

        layout.addRow("Kp (Proportional):", self.sp_kp)
        layout.addRow("Ki (Integral):", self.sp_ki)
        layout.addRow("Kd (Derivative):", self.sp_kd)
        layout.addRow("Max Stick Output (µs):", self.sp_max)

    def _on_change(self) -> None:
        self.cfg.kp = self.sp_kp.value()
        self.cfg.ki = self.sp_ki.value()
        self.cfg.kd = self.sp_kd.value()
        self.cfg.max_output = self.sp_max.value()
        self.changed.emit()

    def update_from_config(self, cfg: PIDAxisConfig) -> None:
        self.cfg = cfg
        self.sp_kp.blockSignals(True)
        self.sp_ki.blockSignals(True)
        self.sp_kd.blockSignals(True)
        self.sp_max.blockSignals(True)
        self.sp_kp.setValue(cfg.kp)
        self.sp_ki.setValue(cfg.ki)
        self.sp_kd.setValue(cfg.kd)
        self.sp_max.setValue(cfg.max_output)
        self.sp_kp.blockSignals(False)
        self.sp_ki.blockSignals(False)
        self.sp_kd.blockSignals(False)
        self.sp_max.blockSignals(False)


class PIDTuningPanel(QWidget):
    pid_updated = pyqtSignal()

    def __init__(self, sys_config: SystemConfig, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.sys_config = sys_config
        self._init_ui()

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)

        tabs = QTabWidget()

        self.grp_yaw = SinglePIDGroup("Yaw PID (Heading)", self.sys_config.yaw_pid, max_kp=1000.0)
        self.grp_yaw.changed.connect(self.pid_updated.emit)

        self.grp_alt = SinglePIDGroup("Altitude / Pitch PID", self.sys_config.altitude_pid, max_kp=1000.0)
        self.grp_alt.changed.connect(self.pid_updated.emit)

        self.grp_pos = SinglePIDGroup("Position PID", self.sys_config.position_pid, max_kp=500.0)
        self.grp_pos.changed.connect(self.pid_updated.emit)

        tabs.addTab(self.grp_yaw, "Yaw PID")
        tabs.addTab(self.grp_alt, "Altitude PID")
        tabs.addTab(self.grp_pos, "Position PID")

        layout.addWidget(tabs)

    def load_config(self, sys_config: SystemConfig) -> None:
        self.sys_config = sys_config
        self.grp_yaw.update_from_config(sys_config.yaw_pid)
        self.grp_alt.update_from_config(sys_config.altitude_pid)
        self.grp_pos.update_from_config(sys_config.position_pid)
