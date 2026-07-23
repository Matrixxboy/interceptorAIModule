"""Compact PID tuning — all axes in one frame (spinboxes only)."""

from __future__ import annotations

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QLabel,
    QVBoxLayout,
    QWidget,
)

from config import SystemConfig


def _spin(value: float, lo: float, hi: float, step: float) -> QDoubleSpinBox:
    sp = QDoubleSpinBox()
    sp.setRange(lo, hi)
    sp.setSingleStep(step)
    sp.setDecimals(1)
    sp.setValue(value)
    sp.setMinimumWidth(100)
    sp.setMaximumWidth(110)
    sp.setAlignment(Qt.AlignmentFlag.AlignRight)
    sp.setButtonSymbols(QDoubleSpinBox.ButtonSymbols.UpDownArrows)
    return sp


class PIDTuningPanel(QWidget):
    """Yaw / Altitude / Position PID values in a single compact table."""

    pid_updated = pyqtSignal()

    def __init__(self, sys_config: SystemConfig, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.sys_config = sys_config
        self._init_ui()

    def _init_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(4, 4, 4, 4)
        root.setSpacing(6)

        box = QGroupBox("PID Gains")
        grid = QGridLayout(box)
        grid.setContentsMargins(10, 12, 10, 10)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(6)

        headers = ["", "Yaw", "Altitude / Pitch", "Position"]
        for col, text in enumerate(headers):
            lbl = QLabel(text)
            lbl.setStyleSheet("color: #9aa3b2; font-weight: 650; font-size: 8pt; background: transparent;")
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter if col else Qt.AlignmentFlag.AlignLeft)
            grid.addWidget(lbl, 0, col)

        self.sp_yaw: dict[str, QDoubleSpinBox] = {}
        self.sp_alt: dict[str, QDoubleSpinBox] = {}
        self.sp_pos: dict[str, QDoubleSpinBox] = {}

        rows = [
            ("Kp", "kp", 1000.0, 5.0),
            ("Ki", "ki", 500.0, 1.0),
            ("Kd", "kd", 500.0, 1.0),
            ("Max µs", "max_output", 500.0, 10.0),
        ]

        for r, (label, field, hi, step) in enumerate(rows, start=1):
            name = QLabel(label)
            name.setStyleSheet("color: #6b7380; font-size: 8.5pt; background: transparent;")
            grid.addWidget(name, r, 0)

            for col, (store, cfg, max_kp) in enumerate(
                [
                    (self.sp_yaw, self.sys_config.yaw_pid, 1000.0),
                    (self.sp_alt, self.sys_config.altitude_pid, 1000.0),
                    (self.sp_pos, self.sys_config.position_pid, 500.0),
                ],
                start=1,
            ):
                use_hi = max_kp if field == "kp" else hi
                sp = _spin(getattr(cfg, field), 0.0 if field != "max_output" else 10.0, use_hi, step)
                sp.valueChanged.connect(self._on_change)
                store[field] = sp
                grid.addWidget(sp, r, col, alignment=Qt.AlignmentFlag.AlignCenter)

        tip = QLabel("Higher Kp = stronger correction · raise Max if the stick saturates early")
        tip.setStyleSheet("color: #6b7380; font-size: 7.5pt; background: transparent;")
        tip.setWordWrap(True)
        grid.addWidget(tip, len(rows) + 1, 0, 1, 4)

        root.addWidget(box)
        root.addStretch(1)

    def _on_change(self) -> None:
        for store, cfg in (
            (self.sp_yaw, self.sys_config.yaw_pid),
            (self.sp_alt, self.sys_config.altitude_pid),
            (self.sp_pos, self.sys_config.position_pid),
        ):
            cfg.kp = store["kp"].value()
            cfg.ki = store["ki"].value()
            cfg.kd = store["kd"].value()
            cfg.max_output = store["max_output"].value()
        self.pid_updated.emit()

    def load_config(self, sys_config: SystemConfig) -> None:
        self.sys_config = sys_config
        mapping = (
            (self.sp_yaw, sys_config.yaw_pid),
            (self.sp_alt, sys_config.altitude_pid),
            (self.sp_pos, sys_config.position_pid),
        )
        for store, cfg in mapping:
            for field in ("kp", "ki", "kd", "max_output"):
                store[field].blockSignals(True)
                store[field].setValue(getattr(cfg, field))
                store[field].blockSignals(False)
