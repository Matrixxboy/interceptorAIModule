"""Follow control panel — Speed Settings + Flight Controls (PID)."""

from __future__ import annotations

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QLabel,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from config import SystemConfig
from gui.widgets.axis_speed_row import AxisSpeedRow


def _spin(value: float, lo: float, hi: float, step: float, tooltip: str = "") -> QDoubleSpinBox:
    sp = QDoubleSpinBox()
    sp.setRange(lo, hi)
    sp.setSingleStep(step)
    sp.setDecimals(1)
    sp.setValue(value)
    sp.setMinimumWidth(88)
    sp.setMaximumWidth(100)
    sp.setAlignment(Qt.AlignmentFlag.AlignRight)
    sp.setButtonSymbols(QDoubleSpinBox.ButtonSymbols.UpDownArrows)
    if tooltip:
        sp.setToolTip(tooltip)
    return sp


class PIDTuningPanel(QWidget):
    """Real-time speed scales + yaw / altitude / position PID gains."""

    pid_updated = pyqtSignal()

    def __init__(self, sys_config: SystemConfig, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.sys_config = sys_config
        self._init_ui()

    def _init_ui(self) -> None:
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        body = QWidget()
        root = QVBoxLayout(body)
        root.setContentsMargins(4, 4, 4, 4)
        root.setSpacing(8)

        # ---- Speed Settings ----
        speed_box = QGroupBox("Speed Settings")
        speed_box.setToolTip("Live multipliers on stick authority for each axis. Applied immediately.")
        speed_l = QVBoxLayout(speed_box)
        speed_l.setContentsMargins(10, 14, 10, 10)
        speed_l.setSpacing(4)

        s = self.sys_config.safety
        self.row_yaw = AxisSpeedRow(
            "Yaw",
            "Horizontal turn rate toward the lock box. Higher = snaps left/right faster.",
            s.yaw_speed_scale,
            lo=0.0,
            hi=1.0,
            step=0.05,
        )
        self.row_pitch = AxisSpeedRow(
            "Pitch",
            "Forward / back chase speed from distance error. Higher = closes range faster.",
            s.pitch_speed_scale,
            lo=0.0,
            hi=1.5,
            step=0.05,
        )
        self.row_throttle = AxisSpeedRow(
            "Throttle",
            "Climb / descend speed to keep the target vertically centered.",
            s.throttle_speed_scale,
            lo=0.0,
            hi=1.0,
            step=0.05,
        )
        self.row_roll = AxisSpeedRow(
            "Roll",
            "Bank-assist scale (reserved). Kept at 0 unless roll assist is enabled.",
            s.roll_speed_scale,
            lo=0.0,
            hi=1.0,
            step=0.05,
        )

        for row in (self.row_yaw, self.row_pitch, self.row_throttle, self.row_roll):
            row.valueChanged.connect(self._on_speed_change)
            speed_l.addWidget(row)

        tip_speed = QLabel("0 = axis idle · 1 = full authority · changes apply on the next frame")
        tip_speed.setStyleSheet("color: #6b7380; font-size: 7.5pt; background: transparent;")
        tip_speed.setWordWrap(True)
        speed_l.addWidget(tip_speed)

        root.addWidget(speed_box)

        # ---- Flight Controls (PID) ----
        box = QGroupBox("Flight Controls")
        box.setToolTip("PID gains that shape the linear stick response for each axis.")
        grid = QGridLayout(box)
        grid.setContentsMargins(10, 14, 10, 10)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(6)

        headers = [
            ("", ""),
            ("Yaw", "Left/right aiming (horizontal error)"),
            ("Altitude", "Climb/descend (vertical error → throttle)"),
            ("Position", "Reserved position hold gains"),
        ]
        for col, (text, tip) in enumerate(headers):
            lbl = QLabel(text)
            lbl.setStyleSheet(
                "color: #9aa3b2; font-weight: 650; font-size: 8pt; background: transparent;"
            )
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter if col else Qt.AlignmentFlag.AlignLeft)
            if tip:
                lbl.setToolTip(tip)
            grid.addWidget(lbl, 0, col)

        self.sp_yaw: dict[str, QDoubleSpinBox] = {}
        self.sp_alt: dict[str, QDoubleSpinBox] = {}
        self.sp_pos: dict[str, QDoubleSpinBox] = {}

        rows = [
            ("Kp", "kp", 1000.0, 5.0, "Proportional gain — stick µs per unit error"),
            ("Ki", "ki", 500.0, 1.0, "Integral gain — clears steady-state offset"),
            ("Kd", "kd", 500.0, 1.0, "Derivative gain — damps overshoot"),
            ("Max µs", "max_output", 500.0, 10.0, "Hard clamp on axis stick offset"),
        ]

        for r, (label, field, hi, step, tip) in enumerate(rows, start=1):
            name = QLabel(label)
            name.setStyleSheet("color: #6b7380; font-size: 8.5pt; background: transparent;")
            name.setToolTip(tip)
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
                sp = _spin(
                    getattr(cfg, field),
                    0.0 if field != "max_output" else 10.0,
                    use_hi,
                    step,
                    tip,
                )
                sp.valueChanged.connect(self._on_pid_change)
                store[field] = sp
                grid.addWidget(sp, r, col, alignment=Qt.AlignmentFlag.AlignCenter)

        tip = QLabel("Higher Kp = stronger correction · raise Max µs if the stick saturates early")
        tip.setStyleSheet("color: #6b7380; font-size: 7.5pt; background: transparent;")
        tip.setWordWrap(True)
        grid.addWidget(tip, len(rows) + 1, 0, 1, 4)

        root.addWidget(box)
        root.addStretch(1)

        scroll.setWidget(body)
        outer.addWidget(scroll)

    def _on_speed_change(self) -> None:
        s = self.sys_config.safety
        s.yaw_speed_scale = self.row_yaw.value()
        s.pitch_speed_scale = self.row_pitch.value()
        s.throttle_speed_scale = self.row_throttle.value()
        s.roll_speed_scale = self.row_roll.value()
        # Keep legacy mirrors current for presets / older readers.
        s.follow_speed_scale = s.yaw_speed_scale
        s.follow_pitch_scale = s.pitch_speed_scale
        self.pid_updated.emit()

    def _on_pid_change(self) -> None:
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

    def _on_change(self) -> None:
        """Backward-compatible alias used by older call sites / tests."""
        self._on_pid_change()

    def load_config(self, sys_config: SystemConfig) -> None:
        self.sys_config = sys_config
        s = sys_config.safety
        self.row_yaw.set_value(s.yaw_speed_scale, emit=False)
        self.row_pitch.set_value(s.pitch_speed_scale, emit=False)
        self.row_throttle.set_value(s.throttle_speed_scale, emit=False)
        self.row_roll.set_value(s.roll_speed_scale, emit=False)

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
