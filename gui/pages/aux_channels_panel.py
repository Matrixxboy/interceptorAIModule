"""AUX channel settings — compact single frame."""

from __future__ import annotations

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QComboBox,
    QGridLayout,
    QGroupBox,
    QLabel,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from config import SystemConfig


def _ch_label(index_0based: int) -> str:
    ch = index_0based + 1
    if ch >= 5:
        return f"CH{ch} (AUX{ch - 4})"
    return f"CH{ch}"


class AuxChannelsPanel(QWidget):
    aux_updated = pyqtSignal()

    def __init__(self, sys_config: SystemConfig, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.sys_config = sys_config
        self._init_ui()

    def _init_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(4, 4, 4, 4)
        root.setSpacing(6)

        hint = QLabel("Match INAV Modes tab · ARM on AUX1 → CH5 · ANGLE on AUX2 → CH6")
        hint.setStyleSheet("color: #6b7380; font-size: 8pt;")
        root.addWidget(hint)

        box = QGroupBox("AUX / Mode Channels")
        grid = QGridLayout(box)
        grid.setContentsMargins(10, 12, 10, 10)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(6)

        self.combo_arm_ch = QComboBox()
        self.combo_mode_ch = QComboBox()
        for i in range(16):
            self.combo_arm_ch.addItem(_ch_label(i), i)
            self.combo_mode_ch.addItem(_ch_label(i), i)
        self.combo_arm_ch.setCurrentIndex(self.sys_config.aux_channels.arm_channel)
        self.combo_mode_ch.setCurrentIndex(self.sys_config.aux_channels.mode_channel)
        self.combo_arm_ch.setMinimumWidth(130)
        self.combo_mode_ch.setMinimumWidth(130)

        self.sp_arm_high = self._pwm(self.sys_config.aux_channels.arm_high)
        self.sp_arm_low = self._pwm(self.sys_config.aux_channels.arm_low)
        self.sp_mode_high = self._pwm(self.sys_config.aux_channels.mode_high)
        self.sp_mode_low = self._pwm(self.sys_config.aux_channels.mode_low)

        def lab(t: str) -> QLabel:
            x = QLabel(t)
            x.setObjectName("formLabel")
            return x

        grid.addWidget(lab("ARM channel"), 0, 0)
        grid.addWidget(self.combo_arm_ch, 0, 1)
        grid.addWidget(lab("Armed µs"), 0, 2)
        grid.addWidget(self.sp_arm_high, 0, 3)
        grid.addWidget(lab("Disarmed µs"), 0, 4)
        grid.addWidget(self.sp_arm_low, 0, 5)

        grid.addWidget(lab("Mode channel"), 1, 0)
        grid.addWidget(self.combo_mode_ch, 1, 1)
        grid.addWidget(lab("Mode ON µs"), 1, 2)
        grid.addWidget(self.sp_mode_high, 1, 3)
        grid.addWidget(lab("Mode OFF µs"), 1, 4)
        grid.addWidget(self.sp_mode_low, 1, 5)

        self.lbl_preview = QLabel()
        self.lbl_preview.setStyleSheet(
            "color: #9aa3b2; font-family: Consolas, monospace; font-size: 8pt; padding-top: 4px;"
        )
        grid.addWidget(self.lbl_preview, 2, 0, 1, 6)

        for w in (
            self.combo_arm_ch, self.combo_mode_ch,
            self.sp_arm_high, self.sp_arm_low, self.sp_mode_high, self.sp_mode_low,
        ):
            if hasattr(w, "currentIndexChanged"):
                w.currentIndexChanged.connect(self._on_change)
            else:
                w.valueChanged.connect(self._on_change)

        root.addWidget(box)
        root.addStretch(1)
        self._refresh_preview()

    def _pwm(self, value: int) -> QSpinBox:
        sp = QSpinBox()
        sp.setRange(1000, 2000)
        sp.setSingleStep(50)
        sp.setValue(value)
        sp.setMinimumWidth(96)
        sp.setMaximumWidth(110)
        sp.setAlignment(Qt.AlignmentFlag.AlignRight)
        return sp

    def _refresh_preview(self) -> None:
        a = self.sys_config.aux_channels
        self.lbl_preview.setText(
            f"ARM → {_ch_label(a.arm_channel)}  {a.arm_high}/{a.arm_low}   ·   "
            f"MODE → {_ch_label(a.mode_channel)}  {a.mode_high}/{a.mode_low}"
        )

    def _on_change(self) -> None:
        self.sys_config.aux_channels.arm_channel = int(self.combo_arm_ch.currentData())
        self.sys_config.aux_channels.mode_channel = int(self.combo_mode_ch.currentData())
        self.sys_config.aux_channels.arm_high = self.sp_arm_high.value()
        self.sys_config.aux_channels.arm_low = self.sp_arm_low.value()
        self.sys_config.aux_channels.mode_high = self.sp_mode_high.value()
        self.sys_config.aux_channels.mode_low = self.sp_mode_low.value()
        self._refresh_preview()
        self.aux_updated.emit()

    def load_config(self, cfg: SystemConfig) -> None:
        self.sys_config = cfg
        for w in (
            self.combo_arm_ch, self.combo_mode_ch,
            self.sp_arm_high, self.sp_arm_low, self.sp_mode_high, self.sp_mode_low,
        ):
            w.blockSignals(True)
        self.combo_arm_ch.setCurrentIndex(cfg.aux_channels.arm_channel)
        self.combo_mode_ch.setCurrentIndex(cfg.aux_channels.mode_channel)
        self.sp_arm_high.setValue(cfg.aux_channels.arm_high)
        self.sp_arm_low.setValue(cfg.aux_channels.arm_low)
        self.sp_mode_high.setValue(cfg.aux_channels.mode_high)
        self.sp_mode_low.setValue(cfg.aux_channels.mode_low)
        for w in (
            self.combo_arm_ch, self.combo_mode_ch,
            self.sp_arm_high, self.sp_arm_low, self.sp_mode_high, self.sp_mode_low,
        ):
            w.blockSignals(False)
        self._refresh_preview()
