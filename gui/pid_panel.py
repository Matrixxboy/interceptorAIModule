"""Follow control panel — PID gains + PPN guidance tunables."""

from __future__ import annotations

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QCheckBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QLabel,
    QVBoxLayout,
    QWidget,
)

from config import SystemConfig


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
    """Yaw / altitude / position PID + Pure Proportional Navigation gains."""

    pid_updated = pyqtSignal()

    def __init__(self, sys_config: SystemConfig, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.sys_config = sys_config
        self._init_ui()

    def _init_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(4, 4, 4, 4)
        root.setSpacing(8)

        # ---- PPN Guidance ----
        ppn_box = QGroupBox("PPN Guidance")
        ppn_box.setToolTip("Thesis-style Pure Proportional Navigation — additive to visual PID")
        ppn_grid = QGridLayout(ppn_box)
        ppn_grid.setContentsMargins(10, 14, 10, 10)
        ppn_grid.setHorizontalSpacing(8)
        ppn_grid.setVerticalSpacing(6)

        s = self.sys_config.safety
        self.chk_ppn = QCheckBox("Enable PPN")
        self.chk_ppn.setChecked(bool(getattr(s, "ppn_enabled", False)))
        self.chk_ppn.setToolTip("Chase FPV leaves this off. Enable for aerial intercept lead.")
        self.sp_ppn_n = _spin(
            float(getattr(s, "ppn_n", 3.0)), 1.0, 8.0, 0.1,
            "Navigation constant N_p (typically 3–5)",
        )
        self.sp_ppn_gain = _spin(
            float(getattr(s, "ppn_gain", 30.0)), 0.0, 120.0, 1.0,
            "µs stick bias per m/s² of PPN acceleration",
        )
        self.sp_ppn_yaw = _spin(
            float(getattr(s, "ppn_yaw_lead_gain", 80.0)), 0.0, 300.0, 5.0,
            "µs yaw lead per rad/s of LOS rate",
        )
        self.sp_ppn_fade = _spin(
            float(getattr(s, "ppn_fade_near_m", 2.0)), 0.2, 20.0, 0.2,
            "Fade PN authority within this distance of desired range",
        )

        ppn_grid.addWidget(self.chk_ppn, 0, 0, 1, 4)
        for col, (lab, sp) in enumerate(
            (
                ("N_p", self.sp_ppn_n),
                ("Gain µs", self.sp_ppn_gain),
                ("Yaw lead", self.sp_ppn_yaw),
                ("Fade m", self.sp_ppn_fade),
            )
        ):
            name = QLabel(lab)
            name.setStyleSheet("color: #6b7380; font-size: 8pt; background: transparent;")
            name.setToolTip(sp.toolTip())
            ppn_grid.addWidget(name, 1, col)
            ppn_grid.addWidget(sp, 2, col)

        tip = QLabel("PPN engages when 3D velocity is trusted · fades near desired distance")
        tip.setStyleSheet("color: #6b7380; font-size: 7.5pt; background: transparent;")
        tip.setWordWrap(True)
        ppn_grid.addWidget(tip, 3, 0, 1, 4)

        for w in (self.chk_ppn, self.sp_ppn_n, self.sp_ppn_gain, self.sp_ppn_yaw, self.sp_ppn_fade):
            if isinstance(w, QCheckBox):
                w.toggled.connect(self._on_change)
            else:
                w.valueChanged.connect(self._on_change)

        root.addWidget(ppn_box)

        # ---- Flight Controls (PID) ----
        box = QGroupBox("Flight Controls")
        grid = QGridLayout(box)
        grid.setContentsMargins(10, 12, 10, 10)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(6)

        headers = ["", "Yaw", "Altitude / Pitch", "Position"]
        for col, text in enumerate(headers):
            lbl = QLabel(text)
            lbl.setStyleSheet(
                "color: #9aa3b2; font-weight: 650; font-size: 8pt; background: transparent;"
            )
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

        tip2 = QLabel("Higher Kp = stronger correction · raise Max if the stick saturates early")
        tip2.setStyleSheet("color: #6b7380; font-size: 7.5pt; background: transparent;")
        tip2.setWordWrap(True)
        grid.addWidget(tip2, len(rows) + 1, 0, 1, 4)

        root.addWidget(box)
        root.addStretch(1)

    def _on_change(self) -> None:
        s = self.sys_config.safety
        s.ppn_enabled = self.chk_ppn.isChecked()
        s.ppn_n = self.sp_ppn_n.value()
        s.ppn_gain = self.sp_ppn_gain.value()
        s.ppn_yaw_lead_gain = self.sp_ppn_yaw.value()
        s.ppn_fade_near_m = self.sp_ppn_fade.value()

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
        s = sys_config.safety
        widgets = [
            self.chk_ppn, self.sp_ppn_n, self.sp_ppn_gain, self.sp_ppn_yaw, self.sp_ppn_fade,
        ]
        for w in widgets:
            w.blockSignals(True)
        self.chk_ppn.setChecked(bool(getattr(s, "ppn_enabled", False)))
        self.sp_ppn_n.setValue(float(getattr(s, "ppn_n", 3.0)))
        self.sp_ppn_gain.setValue(float(getattr(s, "ppn_gain", 30.0)))
        self.sp_ppn_yaw.setValue(float(getattr(s, "ppn_yaw_lead_gain", 80.0)))
        self.sp_ppn_fade.setValue(float(getattr(s, "ppn_fade_near_m", 2.0)))
        for w in widgets:
            w.blockSignals(False)

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
