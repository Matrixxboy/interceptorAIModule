"""System Settings — compact single-page layout that fits the screen."""

from __future__ import annotations

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QCheckBox,
    QComboBox,
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

        self.sp_target_fps = QDoubleSpinBox()
        self.sp_target_fps.setRange(15.0, 120.0)
        self.sp_target_fps.setSingleStep(5.0)
        self.sp_target_fps.setDecimals(0)
        self.sp_target_fps.setFixedWidth(88)
        self.sp_target_fps.setValue(self.sys_config.camera.target_fps)
        self.sp_target_fps.setToolTip(
            "Vision-loop cap. Uncapped loops hit 150–300 FPS on fast GPUs and freeze the UI."
        )
        self.sp_target_fps.valueChanged.connect(self._on_change)

        self.sp_ui_scale = QDoubleSpinBox()
        self.sp_ui_scale.setRange(0.5, 2.5)
        self.sp_ui_scale.setSingleStep(0.1)
        self.sp_ui_scale.setFixedWidth(88)
        self.sp_ui_scale.setValue(self.sys_config.device.ui_scale)
        self.sp_ui_scale.valueChanged.connect(self._on_change)

        form.addRow("Camera index", self.sp_cam_idx)
        form.addRow("FOV H (°)", self.sp_fov_h)
        form.addRow("FOV V (°)", self.sp_fov_v)
        form.addRow("Target FPS", self.sp_target_fps)
        form.addRow("UI scale", self.sp_ui_scale)
        layout.addWidget(box)

        layout.addWidget(self._build_mount_box())
        layout.addStretch(1)

    def _build_mount_box(self) -> QGroupBox:
        """Mount geometry — lets the camera be bolted on at any angle."""
        cam = self.sys_config.camera
        box = QGroupBox("Camera Mount")
        form = QFormLayout(box)
        form.setContentsMargins(10, 12, 10, 10)
        form.setSpacing(6)

        def _ang(value: float, lo: float, hi: float, tip: str) -> QDoubleSpinBox:
            sp = QDoubleSpinBox()
            sp.setRange(lo, hi)
            sp.setSingleStep(1.0)
            sp.setDecimals(1)
            sp.setFixedWidth(88)
            sp.setValue(float(value))
            sp.setToolTip(tip)
            sp.valueChanged.connect(self._on_change)
            return sp

        self.sp_mount_pitch = _ang(
            cam.mount_pitch_deg, -90.0, 90.0,
            "Camera tilt. Positive = tilted UP (measure with a phone level on the lens).",
        )
        self.sp_mount_roll = _ang(
            cam.mount_roll_deg, -180.0, 180.0,
            "Camera rotation in the image. Positive = rotated clockwise.",
        )
        self.sp_mount_yaw = _ang(
            cam.mount_yaw_deg, -90.0, 90.0,
            "Sideways aim. Positive = camera points right of the nose.",
        )
        self.sp_aim_el = _ang(
            cam.desired_elevation_deg, -60.0, 60.0,
            "Elevation to hold the target at. 0 = keep it at the drone's own height.",
        )

        self.cmb_vert_ref = QComboBox()
        self.cmb_vert_ref.addItem("Level (mount corrected)", "level")
        self.cmb_vert_ref.addItem("Image centre (legacy)", "image")
        self.cmb_vert_ref.setCurrentIndex(0 if str(cam.vertical_ref) != "image" else 1)
        self.cmb_vert_ref.setToolTip(
            "Level: fly the target to a true elevation, so a tilted camera holds it "
            "off-centre in the picture (correct).\n"
            "Image centre: old behaviour — parks the target mid-frame."
        )
        self.cmb_vert_ref.currentIndexChanged.connect(self._on_change)

        self.chk_stab = QCheckBox("Level with FC attitude")
        self.chk_stab.setChecked(bool(cam.stabilize_with_attitude))
        self.chk_stab.setToolTip(
            "Subtract live roll/pitch from the flight controller so the aim stays "
            "gravity-referenced while the airframe leans. Needs a serial link."
        )
        self.chk_stab.toggled.connect(self._on_change)

        self.chk_calib_focal = QCheckBox("Use calibrated focal")
        self.chk_calib_focal.setChecked(bool(cam.use_calibrated_focal))
        self.chk_calib_focal.setToolTip(
            "Derive angles from the measured focal length (Distance Calibration) "
            "instead of the nominal FOV numbers."
        )
        self.chk_calib_focal.toggled.connect(self._on_change)

        form.addRow("Tilt up (°)", self.sp_mount_pitch)
        form.addRow("Roll (°)", self.sp_mount_roll)
        form.addRow("Yaw (°)", self.sp_mount_yaw)
        form.addRow("Aim elev (°)", self.sp_aim_el)
        form.addRow("Vertical ref", self.cmb_vert_ref)
        form.addRow(self.chk_stab)
        form.addRow(self.chk_calib_focal)

        hint = QLabel("Set tilt to match how the camera is mounted — the aim line in the feed moves with it.")
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #8a94a6; font-size: 10px;")
        form.addRow(hint)
        return box

    def _on_change(self) -> None:
        cam = self.sys_config.camera
        cam.camera_index = self.sp_cam_idx.value()
        cam.fov_h_deg = self.sp_fov_h.value()
        cam.fov_v_deg = self.sp_fov_v.value()
        cam.target_fps = float(self.sp_target_fps.value())
        cam.mount_pitch_deg = self.sp_mount_pitch.value()
        cam.mount_roll_deg = self.sp_mount_roll.value()
        cam.mount_yaw_deg = self.sp_mount_yaw.value()
        cam.desired_elevation_deg = self.sp_aim_el.value()
        cam.vertical_ref = self.cmb_vert_ref.currentData() or "level"
        cam.stabilize_with_attitude = self.chk_stab.isChecked()
        cam.use_calibrated_focal = self.chk_calib_focal.isChecked()
        self.sys_config.device.ui_scale = self.sp_ui_scale.value()
        self.device_updated.emit()

    def _widgets(self) -> tuple[QWidget, ...]:
        return (
            self.sp_cam_idx, self.sp_fov_h, self.sp_fov_v, self.sp_target_fps, self.sp_ui_scale,
            self.sp_mount_pitch, self.sp_mount_roll, self.sp_mount_yaw, self.sp_aim_el,
            self.cmb_vert_ref, self.chk_stab, self.chk_calib_focal,
        )

    def load_config(self, cfg: SystemConfig) -> None:
        self.sys_config = cfg
        for w in self._widgets():
            w.blockSignals(True)
        self.sp_cam_idx.setValue(cfg.camera.camera_index)
        self.sp_fov_h.setValue(cfg.camera.fov_h_deg)
        self.sp_fov_v.setValue(cfg.camera.fov_v_deg)
        self.sp_target_fps.setValue(cfg.camera.target_fps)
        self.sp_ui_scale.setValue(cfg.device.ui_scale)
        self.sp_mount_pitch.setValue(cfg.camera.mount_pitch_deg)
        self.sp_mount_roll.setValue(cfg.camera.mount_roll_deg)
        self.sp_mount_yaw.setValue(cfg.camera.mount_yaw_deg)
        self.sp_aim_el.setValue(cfg.camera.desired_elevation_deg)
        self.cmb_vert_ref.setCurrentIndex(0 if str(cfg.camera.vertical_ref) != "image" else 1)
        self.chk_stab.setChecked(bool(cfg.camera.stabilize_with_attitude))
        self.chk_calib_focal.setChecked(bool(cfg.camera.use_calibrated_focal))
        for w in self._widgets():
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
            PageHeader("System Settings", "Control speeds · PID · tracking · AUX · camera")
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
