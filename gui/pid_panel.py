"""PID Tuning Panel Widget for Real-Time Parameter Adjustment."""

from __future__ import annotations

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QSizePolicy,
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
        layout.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.ExpandingFieldsGrow)
        layout.setLabelAlignment(Qt.AlignmentFlag.AlignLeft)
        layout.setContentsMargins(8, 12, 8, 8)
        layout.setSpacing(8)

        def _spin(value: float, lo: float, hi: float, step: float) -> QDoubleSpinBox:
            sp = QDoubleSpinBox()
            sp.setRange(lo, hi)
            sp.setSingleStep(step)
            sp.setValue(value)
            sp.setMinimumWidth(100)
            sp.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            sp.valueChanged.connect(self._on_change)
            return sp

        self.sp_kp = _spin(self.cfg.kp, 0.0, max_kp, 5.0)
        self.sp_ki = _spin(self.cfg.ki, 0.0, 500.0, 1.0)
        self.sp_kd = _spin(self.cfg.kd, 0.0, 500.0, 1.0)
        self.sp_max = _spin(self.cfg.max_output, 10.0, 500.0, 10.0)

        layout.addRow("Kp", self.sp_kp)
        layout.addRow("Ki", self.sp_ki)
        layout.addRow("Kd", self.sp_kd)
        layout.addRow("Max (µs)", self.sp_max)

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
