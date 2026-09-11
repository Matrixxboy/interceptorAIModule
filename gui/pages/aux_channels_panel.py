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

        hint = QLabel(
            "Match INAV Modes tab · LOCK CH5 · FOLLOW CH6 · ARM CH7 (AUX3) · ANGLE/ACRO CH8 (AUX4). "
            "Lock and Follow stay on the GCS (not sent as flight modes)."
        )
        hint.setStyleSheet("color: #6E7682; font-size: 8pt;")
        hint.setWordWrap(True)
        root.addWidget(hint)

        box = QGroupBox("AUX / Mode Channels")
        grid = QGridLayout(box)
        grid.setContentsMargins(10, 12, 10, 10)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(6)

        a = self.sys_config.aux_channels
        self.combo_lock_ch = self._channel_combo(a.lock_channel)
        self.combo_follow_ch = self._channel_combo(a.follow_channel)
        self.combo_arm_ch = self._channel_combo(a.arm_channel)
        self.combo_mode_ch = self._channel_combo(a.mode_channel)

        self.sp_lock_high = self._pwm(a.lock_high)
        self.sp_lock_low = self._pwm(a.lock_low)
        self.sp_follow_high = self._pwm(a.follow_high)
        self.sp_follow_low = self._pwm(a.follow_low)
        self.sp_arm_high = self._pwm(a.arm_high)
        self.sp_arm_low = self._pwm(a.arm_low)
        self.sp_mode_high = self._pwm(a.mode_high)
        self.sp_mode_low = self._pwm(a.mode_low)

        def lab(t: str) -> QLabel:
            x = QLabel(t)
            x.setObjectName("formLabel")
            return x

        grid.addWidget(lab("LOCK channel"), 0, 0)
        grid.addWidget(self.combo_lock_ch, 0, 1)
        grid.addWidget(lab("Lock µs"), 0, 2)
        grid.addWidget(self.sp_lock_high, 0, 3)
        grid.addWidget(lab("Unlock µs"), 0, 4)
        grid.addWidget(self.sp_lock_low, 0, 5)

        grid.addWidget(lab("FOLLOW channel"), 1, 0)
        grid.addWidget(self.combo_follow_ch, 1, 1)
        grid.addWidget(lab("Follow µs"), 1, 2)
        grid.addWidget(self.sp_follow_high, 1, 3)
        grid.addWidget(lab("Unfollow µs"), 1, 4)
        grid.addWidget(self.sp_follow_low, 1, 5)

        grid.addWidget(lab("ARM channel"), 2, 0)
        grid.addWidget(self.combo_arm_ch, 2, 1)
        grid.addWidget(lab("Armed µs"), 2, 2)
        grid.addWidget(self.sp_arm_high, 2, 3)
        grid.addWidget(lab("Disarmed µs"), 2, 4)
        grid.addWidget(self.sp_arm_low, 2, 5)

        grid.addWidget(lab("Mode channel"), 3, 0)
        grid.addWidget(self.combo_mode_ch, 3, 1)
        grid.addWidget(lab("Mode ON µs"), 3, 2)
        grid.addWidget(self.sp_mode_high, 3, 3)
        grid.addWidget(lab("Mode OFF µs"), 3, 4)
        grid.addWidget(self.sp_mode_low, 3, 5)

        self.lbl_preview = QLabel()
        self.lbl_preview.setStyleSheet(
            "color: #A8B0BA; font-family: Consolas, monospace; font-size: 8pt; padding-top: 4px;"
        )
        grid.addWidget(self.lbl_preview, 4, 0, 1, 6)

        for w in self._all_controls():
            if hasattr(w, "currentIndexChanged"):
                w.currentIndexChanged.connect(self._on_change)
            else:
                w.valueChanged.connect(self._on_change)

        root.addWidget(box)
        root.addStretch(1)
        self._refresh_preview()

    def _channel_combo(self, index: int) -> QComboBox:
        combo = QComboBox()
        for i in range(16):
            combo.addItem(_ch_label(i), i)
        combo.setCurrentIndex(int(index))
        combo.setMinimumWidth(130)
        return combo

    def _pwm(self, value: int) -> QSpinBox:
        sp = QSpinBox()
        sp.setRange(1000, 2000)
        sp.setSingleStep(50)
        sp.setValue(int(value))
        sp.setMinimumWidth(96)
        sp.setMaximumWidth(110)
        sp.setAlignment(Qt.AlignmentFlag.AlignRight)
        return sp

    def _all_controls(self) -> list:
        return [
            self.combo_lock_ch, self.combo_follow_ch, self.combo_arm_ch, self.combo_mode_ch,
            self.sp_lock_high, self.sp_lock_low, self.sp_follow_high, self.sp_follow_low,
            self.sp_arm_high, self.sp_arm_low, self.sp_mode_high, self.sp_mode_low,
        ]

    def _refresh_preview(self) -> None:
        a = self.sys_config.aux_channels
        self.lbl_preview.setText(
            f"LOCK → {_ch_label(a.lock_channel)}  {a.lock_high}/{a.lock_low}   ·   "
            f"FOLLOW → {_ch_label(a.follow_channel)}  {a.follow_high}/{a.follow_low}   ·   "
            f"ARM → {_ch_label(a.arm_channel)}  {a.arm_high}/{a.arm_low}   ·   "
            f"MODE → {_ch_label(a.mode_channel)}  {a.mode_high}/{a.mode_low}"
        )

    def _on_change(self) -> None:
        a = self.sys_config.aux_channels
        a.lock_channel = int(self.combo_lock_ch.currentData())
        a.follow_channel = int(self.combo_follow_ch.currentData())
        a.arm_channel = int(self.combo_arm_ch.currentData())
        a.mode_channel = int(self.combo_mode_ch.currentData())
        a.lock_high = self.sp_lock_high.value()
        a.lock_low = self.sp_lock_low.value()
        a.follow_high = self.sp_follow_high.value()
        a.follow_low = self.sp_follow_low.value()
        a.arm_high = self.sp_arm_high.value()
        a.arm_low = self.sp_arm_low.value()
        a.mode_high = self.sp_mode_high.value()
        a.mode_low = self.sp_mode_low.value()
        self._refresh_preview()
        self.aux_updated.emit()

    def load_config(self, cfg: SystemConfig) -> None:
        self.sys_config = cfg
        a = cfg.aux_channels
        for w in self._all_controls():
            w.blockSignals(True)
        self.combo_lock_ch.setCurrentIndex(int(getattr(a, "lock_channel", 4)))
        self.combo_follow_ch.setCurrentIndex(int(getattr(a, "follow_channel", 5)))
        self.combo_arm_ch.setCurrentIndex(a.arm_channel)
        self.combo_mode_ch.setCurrentIndex(a.mode_channel)
        self.sp_lock_high.setValue(int(getattr(a, "lock_high", 1900)))
        self.sp_lock_low.setValue(int(getattr(a, "lock_low", 1000)))
        self.sp_follow_high.setValue(int(getattr(a, "follow_high", 1900)))
        self.sp_follow_low.setValue(int(getattr(a, "follow_low", 1000)))
        self.sp_arm_high.setValue(a.arm_high)
        self.sp_arm_low.setValue(a.arm_low)
        self.sp_mode_high.setValue(a.mode_high)
        self.sp_mode_low.setValue(a.mode_low)
        for w in self._all_controls():
            w.blockSignals(False)
        self._refresh_preview()
