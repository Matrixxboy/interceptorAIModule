"""Joystick and Remote Control Interface Page."""

from __future__ import annotations

from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QRect
from PyQt6.QtGui import QPainter, QColor, QPen
from PyQt6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QVBoxLayout,
    QWidget,
    QLineEdit,
)

from config import SystemConfig, JoystickChannelConfig
from control.joystick_manager import JoystickManager, JoystickState
from gui.widgets.page_header import PageHeader, StatusPill


class GimbalWidget(QWidget):
    def __init__(self, label: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setMinimumSize(120, 120)
        self.setMaximumSize(200, 200)
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
        w = rect.width()
        h = rect.height()
        
        # Background
        painter.setBrush(QColor(35, 40, 48))
        painter.setPen(QPen(QColor(60, 65, 75), 2))
        painter.drawRoundedRect(0, 0, w, h, 8, 8)
        
        # Crosshairs
        painter.setPen(QPen(QColor(80, 85, 95), 1, Qt.PenStyle.DashLine))
        painter.drawLine(w // 2, 0, w // 2, h)
        painter.drawLine(0, h // 2, w, h // 2)
        
        # Map 1000-2000 to 0-w and h-0
        x_norm = max(0.0, min(1.0, (self.x_pwm - 1000) / 1000.0))
        y_norm = max(0.0, min(1.0, (self.y_pwm - 1000) / 1000.0))
        
        x_pos = int(x_norm * w)
        y_pos = int((1.0 - y_norm) * h)
        
        # Stick dot
        painter.setBrush(QColor(0, 180, 255))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(x_pos - 8, y_pos - 8, 16, 16)
        
        # Label
        painter.setPen(QColor(150, 150, 160))
        painter.drawText(8, 20, self.label)


class ChannelBar(QWidget):
    def __init__(self, name: str, min_val: int = 1000, max_val: int = 2000, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.lbl_name = QLabel(name)
        self.lbl_name.setFixedWidth(100)
        layout.addWidget(self.lbl_name)
        
        self.bar = QProgressBar()
        self.bar.setRange(min_val, max_val)
        self.bar.setTextVisible(False)
        self.bar.setFixedHeight(14)
        layout.addWidget(self.bar, stretch=1)
        
        self.lbl_val = QLabel("1500")
        self.lbl_val.setFixedWidth(50)
        self.lbl_val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        layout.addWidget(self.lbl_val)

    def set_value(self, val: int) -> None:
        self.bar.setValue(val)
        self.lbl_val.setText(str(val))


class AuxConfigRow(QWidget):
    updated = pyqtSignal()
    delete_requested = pyqtSignal(object)

    def __init__(self, cfg: JoystickChannelConfig, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.cfg = cfg
        self._init_ui()

    def _init_ui(self) -> None:
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 4, 0, 4)

        row1 = QHBoxLayout()
        self.edit_name = QLineEdit(self.cfg.name)
        self.edit_name.setPlaceholderText("Function (e.g. Arm)")
        self.edit_name.setFixedWidth(120)
        self.edit_name.textChanged.connect(self._on_change)
        row1.addWidget(self.edit_name)

        self.sp_axis = QSpinBox()
        self.sp_axis.setRange(1, 32)
        self.sp_axis.setValue(self.cfg.axis + 1)
        self.sp_axis.setToolTip("Hardware Axis / Button Index (1-based)")
        self.sp_axis.valueChanged.connect(self._on_change)
        row1.addWidget(QLabel("Index:"))
        row1.addWidget(self.sp_axis)

        self.chk_button = QCheckBox("Is Button")
        self.chk_button.setChecked(self.cfg.is_button)
        self.chk_button.toggled.connect(self._on_change)
        row1.addWidget(self.chk_button)

        self.chk_inv = QCheckBox("Inverted")
        self.chk_inv.setChecked(self.cfg.inverted)
        self.chk_inv.toggled.connect(self._on_change)
        row1.addWidget(self.chk_inv)

        btn_del = QPushButton("Delete")
        btn_del.setFixedWidth(60)
        btn_del.clicked.connect(lambda: self.delete_requested.emit(self.cfg))
        row1.addWidget(btn_del)
        row1.addStretch()
        main_layout.addLayout(row1)

        row2 = QHBoxLayout()
        row2.addWidget(QLabel("Min:"))
        self.sp_min = QSpinBox()
        self.sp_min.setRange(800, 2200)
        self.sp_min.setValue(self.cfg.min_val)
        self.sp_min.valueChanged.connect(self._on_change)
        row2.addWidget(self.sp_min)

        row2.addWidget(QLabel("Center:"))
        self.sp_center = QSpinBox()
        self.sp_center.setRange(800, 2200)
        self.sp_center.setValue(self.cfg.center_val)
        self.sp_center.valueChanged.connect(self._on_change)
        row2.addWidget(self.sp_center)

        row2.addWidget(QLabel("Max:"))
        self.sp_max = QSpinBox()
        self.sp_max.setRange(800, 2200)
        self.sp_max.setValue(self.cfg.max_val)
        self.sp_max.valueChanged.connect(self._on_change)
        row2.addWidget(self.sp_max)
        row2.addStretch()
        
        main_layout.addLayout(row2)

    def _on_change(self) -> None:
        self.cfg.name = self.edit_name.text()
        self.cfg.axis = self.sp_axis.value() - 1
        self.cfg.is_button = self.chk_button.isChecked()
        self.cfg.inverted = self.chk_inv.isChecked()
        self.cfg.min_val = self.sp_min.value()
        self.cfg.center_val = self.sp_center.value()
        self.cfg.max_val = self.sp_max.value()
        self.updated.emit()


class JoystickPage(QWidget):
    config_updated = pyqtSignal()

    def __init__(self, sys_config: SystemConfig, joystick_mgr: JoystickManager, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.sys_config = sys_config
        self.joystick_mgr = joystick_mgr
        self.aux_bars: dict[str, ChannelBar] = {}
        
        self._init_ui()
        self._build_aux_config()
        self._build_aux_bars()
        
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._update_display)
        self.timer.start(50)  # 20 Hz update

    def _init_ui(self) -> None:
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)
        
        header = PageHeader("Remote Control", "Configure, map, and monitor physical joysticks and RC transmitters")
        main_layout.addWidget(header)
        
        top_split = QHBoxLayout()
        
        # --- Device Selection & Status ---
        grp_device = QGroupBox("Connection Status")
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
        self.btn_refresh.clicked.connect(self._refresh_devices)
        
        dev_box = QHBoxLayout()
        dev_box.addWidget(self.combo_devices, stretch=1)
        dev_box.addWidget(self.btn_refresh)
        
        form.addRow("Active Device:", dev_box)
        dev_layout.addLayout(form)
        top_split.addWidget(grp_device, stretch=1)
        
        # --- Live Visualizer ---
        grp_visual = QGroupBox("Gimbal Visualizer")
        vis_layout = QHBoxLayout(grp_visual)
        self.gimbal_left = GimbalWidget("Left Stick (Yaw/Thr)")
        self.gimbal_right = GimbalWidget("Right Stick (Roll/Pitch)")
        vis_layout.addStretch()
        vis_layout.addWidget(self.gimbal_left)
        vis_layout.addSpacing(20)
        vis_layout.addWidget(self.gimbal_right)
        vis_layout.addStretch()
        top_split.addWidget(grp_visual, stretch=1)
        
        main_layout.addLayout(top_split)
        
        # --- Channels & Configuration Split ---
        bot_split = QHBoxLayout()
        
        # Live Monitors
        grp_monitors = QGroupBox("Live Channels")
        self.mon_layout = QVBoxLayout(grp_monitors)
        self.bar_roll = ChannelBar("Roll")
        self.bar_pitch = ChannelBar("Pitch")
        self.bar_throttle = ChannelBar("Throttle")
        self.bar_yaw = ChannelBar("Yaw")
        self.mon_layout.addWidget(self.bar_roll)
        self.mon_layout.addWidget(self.bar_pitch)
        self.mon_layout.addWidget(self.bar_throttle)
        self.mon_layout.addWidget(self.bar_yaw)
        self.mon_layout.addSpacing(10)
        # AUX bars will be added dynamically here
        self.mon_layout.addStretch()
        bot_split.addWidget(grp_monitors, stretch=1)
        
        # General / Channel Configuration
        grp_config = QGroupBox("General / Channel Configuration")
        cfg_layout = QVBoxLayout(grp_config)
        
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        self.cfg_content = QWidget()
        self.cfg_form = QFormLayout(self.cfg_content)
        scroll.setWidget(self.cfg_content)
        cfg_layout.addWidget(scroll, stretch=1)
        
        btn_add_aux = QPushButton("+ Add AUX Channel")
        btn_add_aux.clicked.connect(self._add_aux_channel)
        cfg_layout.addWidget(btn_add_aux)
        
        bot_split.addWidget(grp_config, stretch=2)
        main_layout.addLayout(bot_split, stretch=1)
        
        self._refresh_devices()

    def _build_aux_config(self) -> None:
        # Clear existing
        while self.cfg_form.count():
            item = self.cfg_form.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
                
        # Primary Axes
        cfg = self.sys_config.joystick
        for title, ch_cfg in [
            ("Roll Axis:", cfg.roll),
            ("Pitch Axis:", cfg.pitch),
            ("Throttle Axis:", cfg.throttle),
            ("Yaw Axis:", cfg.yaw),
        ]:
            row = QWidget()
            rl = QVBoxLayout(row)
            rl.setContentsMargins(0, 0, 0, 0)
            
            top_row = QHBoxLayout()
            sp_idx = QSpinBox()
            sp_idx.setRange(1, 32)
            sp_idx.setValue(ch_cfg.axis + 1)
            sp_idx.valueChanged.connect(lambda v, c=ch_cfg: self._update_primary_ch(c, 'axis', v - 1))
            
            chk_inv = QCheckBox("Inverted")
            chk_inv.setChecked(ch_cfg.inverted)
            chk_inv.toggled.connect(lambda v, c=ch_cfg: self._update_primary_ch(c, 'inverted', v))
            
            top_row.addWidget(QLabel("Index:"))
            top_row.addWidget(sp_idx)
            top_row.addWidget(chk_inv)
            top_row.addStretch()
            rl.addLayout(top_row)
            
            bot_row = QHBoxLayout()
            bot_row.addWidget(QLabel("Min:"))
            sp_min = QSpinBox()
            sp_min.setRange(800, 2200)
            sp_min.setValue(ch_cfg.min_val)
            sp_min.valueChanged.connect(lambda v, c=ch_cfg: self._update_primary_ch(c, 'min_val', v))
            bot_row.addWidget(sp_min)
            
            bot_row.addWidget(QLabel("Center:"))
            sp_center = QSpinBox()
            sp_center.setRange(800, 2200)
            sp_center.setValue(ch_cfg.center_val)
            sp_center.valueChanged.connect(lambda v, c=ch_cfg: self._update_primary_ch(c, 'center_val', v))
            bot_row.addWidget(sp_center)
            
            bot_row.addWidget(QLabel("Max:"))
            sp_max = QSpinBox()
            sp_max.setRange(800, 2200)
            sp_max.setValue(ch_cfg.max_val)
            sp_max.valueChanged.connect(lambda v, c=ch_cfg: self._update_primary_ch(c, 'max_val', v))
            bot_row.addWidget(sp_max)
            bot_row.addStretch()
            
            rl.addLayout(bot_row)
            self.cfg_form.addRow(title, row)
            
        # AUX Channels
        self.cfg_form.addRow(QLabel("<b>AUX Channels:</b>"))
        for aux in self.sys_config.joystick.aux_channels:
            row_widget = AuxConfigRow(aux)
            row_widget.updated.connect(self._on_cfg_changed)
            row_widget.delete_requested.connect(self._delete_aux_channel)
            self.cfg_form.addRow("", row_widget)

    def _update_primary_ch(self, cfg_obj: JoystickChannelConfig, field: str, value) -> None:
        setattr(cfg_obj, field, value)
        self._on_cfg_changed()

    def _build_aux_bars(self) -> None:
        # Remove existing aux bars
        for bar in self.aux_bars.values():
            self.mon_layout.removeWidget(bar)
            bar.deleteLater()
        self.aux_bars.clear()
        
        # Remove the stretch so we can add it back at the end
        item = self.mon_layout.takeAt(self.mon_layout.count() - 1)
        
        for aux in self.sys_config.joystick.aux_channels:
            bar = ChannelBar(f"AUX: {aux.name or 'Unassigned'}")
            self.mon_layout.addWidget(bar)
            self.aux_bars[aux.name] = bar
            
        self.mon_layout.addItem(item)

    def _add_aux_channel(self) -> None:
        new_ch = JoystickChannelConfig(name=f"AUX {len(self.sys_config.joystick.aux_channels) + 1}")
        self.sys_config.joystick.aux_channels.append(new_ch)
        self._build_aux_config()
        self._build_aux_bars()
        self._on_cfg_changed()

    def _delete_aux_channel(self, cfg: JoystickChannelConfig) -> None:
        if cfg in self.sys_config.joystick.aux_channels:
            self.sys_config.joystick.aux_channels.remove(cfg)
            self._build_aux_config()
            self._build_aux_bars()
            self._on_cfg_changed()

    def _on_cfg_changed(self) -> None:
        # Re-sync names to bars
        for aux in self.sys_config.joystick.aux_channels:
            if aux.name in self.aux_bars:
                self.aux_bars[aux.name].lbl_name.setText(f"AUX: {aux.name or 'Unassigned'}")
        
        self.joystick_mgr.update_config(self.sys_config)
        self.config_updated.emit()

    def _refresh_devices(self) -> None:
        devices = self.joystick_mgr.scan_devices()
        self.combo_devices.clear()
        self.combo_devices.addItems(devices)
        
        target = self.sys_config.joystick.device_name
        idx = self.combo_devices.findText(target)
        if idx >= 0:
            self.combo_devices.setCurrentIndex(idx)
            
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
            self.pill_status.set_status(f"CONNECTED: {state.device_name}", "ok")
        else:
            self.pill_status.set_status("DISCONNECTED", "error")
            
        self.bar_roll.set_value(state.roll_pwm)
        self.bar_pitch.set_value(state.pitch_pwm)
        self.bar_yaw.set_value(state.yaw_pwm)
        self.bar_throttle.set_value(state.throttle_pwm)
        
        # Update Gimbals
        self.gimbal_left.set_values(state.yaw_pwm, state.throttle_pwm)
        self.gimbal_right.set_values(state.roll_pwm, state.pitch_pwm)
        
        # Update AUX bars dynamically
        for aux in self.sys_config.joystick.aux_channels:
            if aux.name in self.aux_bars:
                val = state.aux_pwm.get(aux.name, 1500)
                self.aux_bars[aux.name].set_value(val)
