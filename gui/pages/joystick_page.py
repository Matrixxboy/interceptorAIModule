"""Joystick and Remote Control Interface Page."""

from __future__ import annotations

from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QColor, QPainter, QPen
from PyQt6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QFormLayout,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from config import JoystickChannelConfig, SystemConfig
from control.joystick_manager import JoystickManager
from gui.widgets.page_header import PageHeader, StatusPill


def _rc_ch_label(index_0based: int) -> str:
    ch = index_0based + 1
    if ch >= 5:
        return f"CH{ch} (AUX{ch - 4})"
    return f"CH{ch}"


def _next_free_rc(aux_list: list[JoystickChannelConfig], start: int = 4) -> int:
    used = {a.rc_channel for a in aux_list if a.rc_channel >= 0}
    for cand in range(start, 16):
        if cand not in used:
            return cand
    return 15


def _next_free_joy_index(aux_list: list[JoystickChannelConfig], as_button: bool = True) -> int:
    """Next free button (or axis) index so new AUX does not share the same input."""
    used = {max(0, a.axis) for a in aux_list if a.is_button == as_button}
    for cand in range(0, 32):
        if cand not in used:
            return cand
    return 0


def _dedupe_aux_mappings(aux_list: list[JoystickChannelConfig]) -> list[str]:
    """Ensure unique FC channels and unique joy indices per type. Returns warning strings."""
    warnings: list[str] = []
    seen_rc: dict[int, int] = {}
    for i, aux in enumerate(aux_list):
        rc = aux.rc_channel
        if rc < 0:
            aux.rc_channel = _next_free_rc(aux_list[:i] + aux_list[i + 1 :])
            rc = aux.rc_channel
        if rc in seen_rc:
            old = aux.rc_channel
            aux.rc_channel = _next_free_rc([a for j, a in enumerate(aux_list) if j != i])
            warnings.append(
                f"'{aux.name}' FC {_rc_ch_label(old)} was shared — moved to {_rc_ch_label(aux.rc_channel)}"
            )
            seen_rc[aux.rc_channel] = i
        else:
            seen_rc[rc] = i

    # Unique button indices among buttons; unique axis indices among axes
    for as_btn in (True, False):
        seen_joy: dict[int, int] = {}
        for i, aux in enumerate(aux_list):
            if aux.is_button != as_btn:
                continue
            joy = max(0, aux.axis)
            if joy in seen_joy:
                old = joy
                others = [a for j, a in enumerate(aux_list) if j != i and a.is_button == as_btn]
                aux.axis = _next_free_joy_index(others, as_button=as_btn)
                kind = "button" if as_btn else "axis"
                warnings.append(
                    f"'{aux.name}' Joy {kind} {old + 1} was shared — moved to {aux.axis + 1}"
                )
                seen_joy[aux.axis] = i
            else:
                seen_joy[joy] = i
    return warnings


class GimbalWidget(QWidget):
    def __init__(self, label: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setFixedSize(112, 112)
        self.x_pwm = 1500
        self.y_pwm = 1500
        self.label = label

    def set_values(self, x: int, y: int) -> None:
        self.x_pwm = x
        self.y_pwm = y
        self.update()

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        rect = self.rect()
        w, h = rect.width(), rect.height()

        painter.setBrush(QColor(30, 35, 41))
        painter.setPen(QPen(QColor(42, 48, 56), 2))
        painter.drawRoundedRect(0, 0, w, h, 6, 6)

        painter.setPen(QPen(QColor(52, 59, 69), 1, Qt.PenStyle.DashLine))
        painter.drawLine(w // 2, 0, w // 2, h)
        painter.drawLine(0, h // 2, w, h // 2)

        x_norm = max(0.0, min(1.0, (self.x_pwm - 1000) / 1000.0))
        y_norm = max(0.0, min(1.0, (self.y_pwm - 1000) / 1000.0))
        x_pos = int(x_norm * w)
        y_pos = int((1.0 - y_norm) * h)

        painter.setBrush(QColor(79, 124, 172))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(x_pos - 7, y_pos - 7, 14, 14)
        painter.setPen(QColor(154, 163, 178))
        painter.drawText(8, 18, self.label)


class ChannelBar(QWidget):
    def __init__(self, name: str, min_val: int = 1000, max_val: int = 2000, parent=None) -> None:
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 2, 0, 2)

        self.lbl_name = QLabel(name)
        self.lbl_name.setMinimumWidth(100)
        self.lbl_name.setMaximumWidth(160)
        layout.addWidget(self.lbl_name)

        self.bar = QProgressBar()
        self.bar.setRange(min_val, max_val)
        self.bar.setTextVisible(False)
        self.bar.setFixedHeight(12)
        layout.addWidget(self.bar, stretch=1)

        self.lbl_val = QLabel("1500")
        self.lbl_val.setFixedWidth(48)
        self.lbl_val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        layout.addWidget(self.lbl_val)

    def set_value(self, val: int) -> None:
        self.bar.setValue(max(self.bar.minimum(), min(self.bar.maximum(), int(val))))
        self.lbl_val.setText(str(int(val)))

    def set_name(self, name: str) -> None:
        self.lbl_name.setText(name)


def _compact_spin_style() -> str:
    return (
        "QSpinBox, QDoubleSpinBox {"
        "  font-size: 8pt; font-weight: 600; min-height: 22px; padding: 1px 4px;"
        "  color: #e6e9ef; background: #13161b; border: 1px solid #2a3038; border-radius: 3px;"
        "}"
        "QSpinBox::up-button, QSpinBox::down-button,"
        "QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {"
        "  width: 14px; border-left: 1px solid #2a3038; background: #1e2329;"
        "}"
    )


def _pwm_spin(value: int) -> QSpinBox:
    sp = QSpinBox()
    sp.setRange(800, 2200)
    sp.setSingleStep(25)
    sp.setValue(value)
    sp.setFixedWidth(78)
    sp.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
    sp.setStyleSheet(_compact_spin_style())
    return sp


def _axis_spin(value: int) -> QSpinBox:
    sp = QSpinBox()
    sp.setRange(1, 32)
    sp.setValue(value)
    sp.setFixedWidth(56)
    sp.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
    sp.setStyleSheet(_compact_spin_style())
    return sp


def _form_lbl(text: str) -> QLabel:
    lab = QLabel(text)
    lab.setStyleSheet("color: #9aa3b2; font-size: 7.5pt; background: transparent;")
    return lab


class StickConfigRow(QFrame):
    """Primary stick mapping — single compact row."""

    updated = pyqtSignal()

    def __init__(self, title: str, cfg: JoystickChannelConfig, parent=None) -> None:
        super().__init__(parent)
        self.cfg = cfg
        self.setObjectName("panel")

        row = QHBoxLayout(self)
        row.setContentsMargins(8, 4, 8, 4)
        row.setSpacing(6)

        name = QLabel(title)
        name.setFixedWidth(58)
        name.setStyleSheet(
            "color: #e6e9ef; font-size: 8pt; font-weight: 650; background: transparent;"
        )
        row.addWidget(name)

        row.addWidget(_form_lbl("Axis"))
        self.sp_axis = _axis_spin(cfg.axis + 1)
        self.sp_axis.setToolTip(f"Joystick axis for {title} (1-based)")
        self.sp_axis.valueChanged.connect(self._on_change)
        row.addWidget(self.sp_axis)

        self.chk_inv = QCheckBox("Inv")
        self.chk_inv.setChecked(cfg.inverted)
        self.chk_inv.setStyleSheet("font-size: 7.5pt;")
        self.chk_inv.setToolTip("Invert axis")
        self.chk_inv.toggled.connect(self._on_change)
        row.addWidget(self.chk_inv)

        self.sp_min = _pwm_spin(cfg.min_val)
        self.sp_min.setToolTip("PWM at stick low")
        self.sp_min.valueChanged.connect(self._on_change)
        row.addWidget(_form_lbl("Low"))
        row.addWidget(self.sp_min)

        self.sp_center = _pwm_spin(cfg.center_val)
        self.sp_center.setToolTip("PWM at stick center")
        self.sp_center.valueChanged.connect(self._on_change)
        row.addWidget(_form_lbl("Ctr"))
        row.addWidget(self.sp_center)

        self.sp_max = _pwm_spin(cfg.max_val)
        self.sp_max.setToolTip("PWM at stick high")
        self.sp_max.valueChanged.connect(self._on_change)
        row.addWidget(_form_lbl("High"))
        row.addWidget(self.sp_max)
        row.addStretch(1)

    def _on_change(self) -> None:
        self.cfg.axis = self.sp_axis.value() - 1
        self.cfg.inverted = self.chk_inv.isChecked()
        self.cfg.min_val = self.sp_min.value()
        self.cfg.center_val = self.sp_center.value()
        self.cfg.max_val = self.sp_max.value()
        self.updated.emit()


class AuxConfigRow(QFrame):
    """AUX mapping — single compact row with unique FC / Joy enforcement."""

    updated = pyqtSignal()
    delete_requested = pyqtSignal(object)

    def __init__(
        self,
        cfg: JoystickChannelConfig,
        all_aux: list[JoystickChannelConfig],
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.cfg = cfg
        self._all_aux = all_aux
        self.setObjectName("panel")
        self._init_ui()

    def _init_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 4, 8, 4)
        root.setSpacing(2)

        row = QHBoxLayout()
        row.setSpacing(5)

        self.edit_name = QLineEdit(self.cfg.name)
        self.edit_name.setPlaceholderText("Name")
        self.edit_name.setFixedWidth(72)
        self.edit_name.setStyleSheet(
            "QLineEdit { font-size: 8pt; min-height: 22px; padding: 1px 4px; }"
        )
        self.edit_name.textChanged.connect(self._on_change)
        row.addWidget(_form_lbl("Name"))
        row.addWidget(self.edit_name)

        self.combo_rc = QComboBox()
        for i in range(4, 16):
            self.combo_rc.addItem(_rc_ch_label(i), i)
        rc = self.cfg.rc_channel if self.cfg.rc_channel >= 4 else 4
        idx = self.combo_rc.findData(rc)
        self.combo_rc.setCurrentIndex(idx if idx >= 0 else 0)
        self.combo_rc.setFixedWidth(118)
        self.combo_rc.setStyleSheet(
            "QComboBox { font-size: 8pt; min-height: 22px; padding: 1px 4px; }"
        )
        self.combo_rc.setToolTip("FC channel (must be unique — one AUX per channel)")
        self.combo_rc.currentIndexChanged.connect(self._on_change)
        row.addWidget(_form_lbl("FC"))
        row.addWidget(self.combo_rc)

        self.sp_axis = _axis_spin(max(1, self.cfg.axis + 1))
        self.sp_axis.setToolTip("Joystick button/axis number (1-based). Must be unique among AUX.")
        self.sp_axis.valueChanged.connect(self._on_change)
        row.addWidget(_form_lbl("Joy"))
        row.addWidget(self.sp_axis)

        self.chk_button = QCheckBox("Btn")
        self.chk_button.setChecked(self.cfg.is_button)
        self.chk_button.setStyleSheet("font-size: 7.5pt;")
        self.chk_button.setToolTip("On = digital button, Off = analog axis")
        self.chk_button.toggled.connect(self._on_change)
        row.addWidget(self.chk_button)

        self.chk_inv = QCheckBox("Inv")
        self.chk_inv.setChecked(self.cfg.inverted)
        self.chk_inv.setStyleSheet("font-size: 7.5pt;")
        self.chk_inv.toggled.connect(self._on_change)
        row.addWidget(self.chk_inv)

        self.sp_min = _pwm_spin(self.cfg.min_val)
        self.sp_min.valueChanged.connect(self._on_change)
        row.addWidget(_form_lbl("Low"))
        row.addWidget(self.sp_min)

        self.sp_center = _pwm_spin(self.cfg.center_val)
        self.sp_center.valueChanged.connect(self._on_change)
        row.addWidget(_form_lbl("Ctr"))
        row.addWidget(self.sp_center)

        self.sp_max = _pwm_spin(self.cfg.max_val)
        self.sp_max.valueChanged.connect(self._on_change)
        row.addWidget(_form_lbl("High"))
        row.addWidget(self.sp_max)

        btn_del = QPushButton("×")
        btn_del.setObjectName("btnGhost")
        btn_del.setFixedSize(26, 22)
        btn_del.setStyleSheet("QPushButton { font-size: 9pt; min-width: 26px; min-height: 22px; padding: 0; }")
        btn_del.setToolTip("Remove AUX channel")
        btn_del.clicked.connect(lambda: self.delete_requested.emit(self.cfg))
        row.addWidget(btn_del)
        row.addStretch(1)
        root.addLayout(row)

        self.lbl_warn = QLabel("")
        self.lbl_warn.setStyleSheet("color: #b08a3c; font-size: 7.5pt; background: transparent;")
        self.lbl_warn.setVisible(False)
        root.addWidget(self.lbl_warn)
        self._refresh_conflict()

    def _refresh_conflict(self) -> None:
        msgs = []
        rc = self.cfg.rc_channel
        for other in self._all_aux:
            if other is self.cfg:
                continue
            if other.rc_channel == rc and rc >= 0:
                msgs.append(f"FC {_rc_ch_label(rc)} also used by '{other.name}'")
            if (
                other.is_button == self.cfg.is_button
                and max(0, other.axis) == max(0, self.cfg.axis)
            ):
                kind = "button" if self.cfg.is_button else "axis"
                msgs.append(f"Joy {kind} {self.cfg.axis + 1} also used by '{other.name}'")
        if msgs:
            self.lbl_warn.setText("Conflict: " + " · ".join(msgs) + " (auto-fixed on change)")
            self.lbl_warn.setVisible(True)
        else:
            self.lbl_warn.clear()
            self.lbl_warn.setVisible(False)

    def _on_change(self) -> None:
        self.cfg.name = self.edit_name.text().strip() or "AUX"
        self.cfg.axis = self.sp_axis.value() - 1
        self.cfg.is_button = self.chk_button.isChecked()
        self.cfg.inverted = self.chk_inv.isChecked()
        self.cfg.min_val = self.sp_min.value()
        self.cfg.center_val = self.sp_center.value()
        self.cfg.max_val = self.sp_max.value()
        self.cfg.rc_channel = int(self.combo_rc.currentData())

        # If FC channel collides, move this row to next free CH
        for other in self._all_aux:
            if other is self.cfg:
                continue
            if other.rc_channel == self.cfg.rc_channel:
                self.cfg.rc_channel = _next_free_rc(
                    [a for a in self._all_aux if a is not self.cfg]
                )
                idx = self.combo_rc.findData(self.cfg.rc_channel)
                if idx >= 0:
                    self.combo_rc.blockSignals(True)
                    self.combo_rc.setCurrentIndex(idx)
                    self.combo_rc.blockSignals(False)
                break

        # If Joy index collides among same type, move to next free
        for other in self._all_aux:
            if other is self.cfg:
                continue
            if other.is_button == self.cfg.is_button and max(0, other.axis) == max(0, self.cfg.axis):
                others = [a for a in self._all_aux if a is not self.cfg and a.is_button == self.cfg.is_button]
                self.cfg.axis = _next_free_joy_index(others, as_button=self.cfg.is_button)
                self.sp_axis.blockSignals(True)
                self.sp_axis.setValue(self.cfg.axis + 1)
                self.sp_axis.blockSignals(False)
                break

        self._refresh_conflict()
        self.updated.emit()


class JoystickPage(QWidget):
    config_updated = pyqtSignal()

    def __init__(
        self,
        sys_config: SystemConfig,
        joystick_mgr: JoystickManager,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.sys_config = sys_config
        self.joystick_mgr = joystick_mgr
        self.aux_bars: dict[int, ChannelBar] = {}  # stable index keys
        self._aux_container_layout: QVBoxLayout | None = None

        self._init_ui()
        self._build_aux_config()
        self._rebuild_aux_bars()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._update_display)
        self.timer.start(50)

    def _init_ui(self) -> None:
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(8)

        header = PageHeader(
            "Remote Control",
            "Map joystick axes/buttons to FC channels · monitor live PWM",
        )
        main_layout.addWidget(header)

        top_split = QHBoxLayout()
        top_split.setSpacing(8)

        grp_device = QGroupBox("Connection")
        dev_layout = QVBoxLayout(grp_device)
        top_row = QHBoxLayout()
        self.chk_enable = QCheckBox("Enable Joystick RC Override")
        self.chk_enable.setChecked(self.sys_config.joystick.enabled)
        self.chk_enable.toggled.connect(self._on_enable_toggled)
        top_row.addWidget(self.chk_enable)
        self.pill_status = StatusPill("DISCONNECTED", "error")
        top_row.addStretch()
        top_row.addWidget(self.pill_status)
        dev_layout.addLayout(top_row)

        form = QFormLayout()
        self.combo_devices = QComboBox()
        self.btn_refresh = QPushButton("Refresh")
        self.btn_refresh.setObjectName("btnGhost")
        self.btn_refresh.clicked.connect(self._refresh_devices)
        dev_box = QHBoxLayout()
        dev_box.addWidget(self.combo_devices, stretch=1)
        dev_box.addWidget(self.btn_refresh)
        form.addRow("Device:", dev_box)
        dev_layout.addLayout(form)
        top_split.addWidget(grp_device, stretch=1)

        grp_visual = QGroupBox("Stick Preview")
        vis_layout = QHBoxLayout(grp_visual)
        self.gimbal_left = GimbalWidget("Yaw / Throttle")
        self.gimbal_right = GimbalWidget("Roll / Pitch")
        vis_layout.addStretch()
        vis_layout.addWidget(self.gimbal_left)
        vis_layout.addSpacing(16)
        vis_layout.addWidget(self.gimbal_right)
        vis_layout.addStretch()
        top_split.addWidget(grp_visual, stretch=1)
        main_layout.addLayout(top_split)

        bot_split = QHBoxLayout()
        bot_split.setSpacing(8)

        grp_monitors = QGroupBox("Live Channels")
        mon_outer = QVBoxLayout(grp_monitors)
        mon_outer.setSpacing(4)
        self.bar_roll = ChannelBar("Roll")
        self.bar_pitch = ChannelBar("Pitch")
        self.bar_throttle = ChannelBar("Throttle")
        self.bar_yaw = ChannelBar("Yaw")
        mon_outer.addWidget(self.bar_roll)
        mon_outer.addWidget(self.bar_pitch)
        mon_outer.addWidget(self.bar_throttle)
        mon_outer.addWidget(self.bar_yaw)

        aux_wrap = QWidget()
        self._aux_container_layout = QVBoxLayout(aux_wrap)
        self._aux_container_layout.setContentsMargins(0, 6, 0, 0)
        self._aux_container_layout.setSpacing(2)
        mon_outer.addWidget(aux_wrap)
        mon_outer.addStretch()
        bot_split.addWidget(grp_monitors, stretch=1)

        grp_config = QGroupBox("Channel Mapping")
        cfg_layout = QVBoxLayout(grp_config)
        cfg_layout.setSpacing(6)
        hint = QLabel(
            "Each AUX needs a UNIQUE FC channel and UNIQUE Joy button/axis — "
            "one input → one channel (duplicates are auto-fixed)."
        )
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #9aa3b2; font-size: 8pt; background: transparent;")
        cfg_layout.addWidget(hint)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.cfg_content = QWidget()
        self.cfg_form = QVBoxLayout(self.cfg_content)
        self.cfg_form.setSpacing(4)
        self.cfg_form.setContentsMargins(0, 0, 4, 0)
        scroll.setWidget(self.cfg_content)
        cfg_layout.addWidget(scroll, stretch=1)

        btn_add_aux = QPushButton("+ Add AUX")
        btn_add_aux.setObjectName("btnPrimary")
        btn_add_aux.setFixedWidth(120)
        btn_add_aux.clicked.connect(self._add_aux_channel)
        cfg_layout.addWidget(btn_add_aux)

        bot_split.addWidget(grp_config, stretch=2)
        main_layout.addLayout(bot_split, stretch=1)
        self._refresh_devices()

    def _build_aux_config(self) -> None:
        while self.cfg_form.count():
            item = self.cfg_form.takeAt(0)
            w = item.widget()
            if w:
                w.deleteLater()

        for msg in _dedupe_aux_mappings(self.sys_config.joystick.aux_channels):
            print(f"[Joystick] {msg}")

        cfg = self.sys_config.joystick
        for title, ch_cfg in [
            ("Roll", cfg.roll),
            ("Pitch", cfg.pitch),
            ("Throttle", cfg.throttle),
            ("Yaw", cfg.yaw),
        ]:
            row = StickConfigRow(title, ch_cfg)
            row.updated.connect(self._on_cfg_changed)
            self.cfg_form.addWidget(row)

        aux_hdr = QLabel("AUX → Flight Controller  (unique CH + unique Joy per row)")
        aux_hdr.setStyleSheet(
            "color: #9aa3b2; font-size: 8.5pt; font-weight: 650; background: transparent; padding-top: 4px;"
        )
        self.cfg_form.addWidget(aux_hdr)
        for aux in self.sys_config.joystick.aux_channels:
            if aux.rc_channel < 0:
                aux.rc_channel = _next_free_rc(self.sys_config.joystick.aux_channels)
            row_widget = AuxConfigRow(aux, self.sys_config.joystick.aux_channels)
            row_widget.updated.connect(self._on_cfg_changed)
            row_widget.delete_requested.connect(self._delete_aux_channel)
            self.cfg_form.addWidget(row_widget)

        self.cfg_form.addStretch(1)

    def _update_primary_ch(self, cfg_obj: JoystickChannelConfig, field: str, value) -> None:
        setattr(cfg_obj, field, value)
        self._on_cfg_changed()

    def _rebuild_aux_bars(self) -> None:
        layout = self._aux_container_layout
        if layout is None:
            return

        while layout.count():
            item = layout.takeAt(0)
            w = item.widget()
            if w:
                w.deleteLater()
        self.aux_bars.clear()

        for i, aux in enumerate(self.sys_config.joystick.aux_channels):
            rc = aux.rc_channel if aux.rc_channel >= 0 else (4 + i)
            label = f"{_rc_ch_label(rc)}  {aux.name or 'AUX'}"
            bar = ChannelBar(label)
            layout.addWidget(bar)
            self.aux_bars[i] = bar

    def _add_aux_channel(self) -> None:
        aux_list = self.sys_config.joystick.aux_channels
        next_rc = _next_free_rc(aux_list)
        next_joy = _next_free_joy_index(aux_list, as_button=True)
        n = len(aux_list) + 1
        new_ch = JoystickChannelConfig(
            name=f"AUX{n}",
            axis=next_joy,
            is_button=True,
            rc_channel=next_rc,
            min_val=1000,
            center_val=1000,
            max_val=1800,
        )
        aux_list.append(new_ch)
        self._build_aux_config()
        self._rebuild_aux_bars()
        self._on_cfg_changed()

    def _delete_aux_channel(self, cfg: JoystickChannelConfig) -> None:
        if cfg in self.sys_config.joystick.aux_channels:
            self.sys_config.joystick.aux_channels.remove(cfg)
            self._build_aux_config()
            self._rebuild_aux_bars()
            self._on_cfg_changed()

    def _on_cfg_changed(self) -> None:
        _dedupe_aux_mappings(self.sys_config.joystick.aux_channels)
        for i, aux in enumerate(self.sys_config.joystick.aux_channels):
            if i not in self.aux_bars:
                self._rebuild_aux_bars()
                break
            rc = aux.rc_channel if aux.rc_channel >= 0 else (4 + i)
            self.aux_bars[i].set_name(f"{_rc_ch_label(rc)}  {aux.name or 'AUX'}")
        else:
            if len(self.aux_bars) != len(self.sys_config.joystick.aux_channels):
                self._rebuild_aux_bars()

        self.joystick_mgr.update_config(self.sys_config)
        self.config_updated.emit()

    def _refresh_devices(self) -> None:
        devices = self.joystick_mgr.scan_devices()
        self.combo_devices.blockSignals(True)
        self.combo_devices.clear()
        self.combo_devices.addItems(devices)
        target = self.sys_config.joystick.device_name
        idx = self.combo_devices.findText(target)
        if idx >= 0:
            self.combo_devices.setCurrentIndex(idx)
        self.combo_devices.blockSignals(False)
        try:
            self.combo_devices.currentIndexChanged.disconnect(self._on_device_changed)
        except TypeError:
            pass
        self.combo_devices.currentIndexChanged.connect(self._on_device_changed)

    def _on_device_changed(self) -> None:
        self.sys_config.joystick.device_name = self.combo_devices.currentText()
        self.joystick_mgr.update_config(self.sys_config)
        self.config_updated.emit()

    def _on_enable_toggled(self, checked: bool) -> None:
        self.sys_config.joystick.enabled = checked
        self.joystick_mgr.update_config(self.sys_config)
        self.config_updated.emit()

    def _update_display(self) -> None:
        state = self.joystick_mgr.poll()
        if state.connected:
            self.pill_status.set_status("CONNECTED", "ok")
        else:
            self.pill_status.set_status("DISCONNECTED", "error")

        self.bar_roll.set_value(state.roll_pwm)
        self.bar_pitch.set_value(state.pitch_pwm)
        self.bar_yaw.set_value(state.yaw_pwm)
        self.bar_throttle.set_value(state.throttle_pwm)
        self.gimbal_left.set_values(state.yaw_pwm, state.throttle_pwm)
        self.gimbal_right.set_values(state.roll_pwm, state.pitch_pwm)

        for i, aux in enumerate(self.sys_config.joystick.aux_channels):
            bar = self.aux_bars.get(i)
            if not bar:
                continue
            val = state.aux_pwm.get(f"#{i}", state.aux_pwm.get(aux.name, aux.center_val))
            bar.set_value(val)
