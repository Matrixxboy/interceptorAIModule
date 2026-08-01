"""Tracking parameters — distance, aim offsets, and lock confidence."""

from __future__ import annotations

from pathlib import Path

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from config import PRESETS_DIR, SystemConfig


def _dspin(
    value: float,
    lo: float,
    hi: float,
    step: float,
    decimals: int = 2,
    tooltip: str = "",
) -> QDoubleSpinBox:
    sp = QDoubleSpinBox()
    sp.setRange(lo, hi)
    sp.setSingleStep(step)
    sp.setDecimals(decimals)
    sp.setValue(value)
    sp.setMinimumWidth(100)
    sp.setMaximumWidth(112)
    sp.setAlignment(Qt.AlignmentFlag.AlignRight)
    if tooltip:
        sp.setToolTip(tooltip)
    return sp


def _ispin(value: int, lo: int, hi: int, tooltip: str = "") -> QSpinBox:
    sp = QSpinBox()
    sp.setRange(lo, hi)
    sp.setValue(value)
    sp.setMinimumWidth(100)
    sp.setMaximumWidth(112)
    sp.setAlignment(Qt.AlignmentFlag.AlignRight)
    if tooltip:
        sp.setToolTip(tooltip)
    return sp


def _lbl(text: str, tooltip: str = "") -> QLabel:
    lab = QLabel(text)
    lab.setObjectName("formLabel")
    if tooltip:
        lab.setToolTip(tooltip)
    return lab


class ParametersPanel(QWidget):
    params_updated = pyqtSignal()
    preset_loaded = pyqtSignal(object)

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

        # Presets row
        preset_row = QHBoxLayout()
        preset_row.setSpacing(6)
        self.combo_presets = QComboBox()
        self._refresh_presets()
        btn_load = QPushButton("Load")
        btn_load.setObjectName("btnGhost")
        btn_load.setFixedWidth(64)
        btn_load.clicked.connect(self._load_selected_preset)
        btn_save = QPushButton("Save")
        btn_save.setObjectName("btnGhost")
        btn_save.setFixedWidth(64)
        btn_save.clicked.connect(self._save_preset_dialog)
        preset_row.addWidget(_lbl("Preset"))
        preset_row.addWidget(self.combo_presets, stretch=1)
        preset_row.addWidget(btn_load)
        preset_row.addWidget(btn_save)
        root.addLayout(preset_row)

        c = self.sys_config

        # ---- Distance / follow geometry ----
        dist_box = QGroupBox("Distance Follow")
        dist_grid = QGridLayout(dist_box)
        dist_grid.setContentsMargins(10, 14, 10, 10)
        dist_grid.setHorizontalSpacing(12)
        dist_grid.setVerticalSpacing(6)

        self.sp_desired_dist = _dspin(
            c.distance.desired_distance_m, 0.5, 50.0, 0.5, 1,
            "Target stand-off distance the pitch loop tries to hold.",
        )
        self.sp_min_dist = _dspin(
            c.distance.min_safe_distance_m, 0.2, 20.0, 0.2, 1,
            "Minimum safe range — pitch backs off harder below this.",
        )
        self.sp_max_dist = _dspin(
            c.distance.max_follow_distance_m, 2.0, 100.0, 1.0, 1,
            "Beyond this range, chase authority is limited.",
        )
        self.sp_known_w = _dspin(
            c.distance.known_object_width_m, 0.01, 10.0, 0.05, 2,
            "Real-world size of the locked object along the calibrated axis.",
        )
        self.sp_focal = _dspin(
            c.distance.focal_length_px, 50.0, 20000.0, 10.0, 1,
            "Calibrated focal length in pixels (from Distance Calibration).",
        )

        dist_fields = [
            (0, 0, "Desired dist (m)", self.sp_desired_dist, self.sp_desired_dist.toolTip()),
            (0, 2, "Min safe (m)", self.sp_min_dist, self.sp_min_dist.toolTip()),
            (1, 0, "Max range (m)", self.sp_max_dist, self.sp_max_dist.toolTip()),
            (1, 2, "Object width (m)", self.sp_known_w, self.sp_known_w.toolTip()),
            (2, 0, "Focal length (px)", self.sp_focal, self.sp_focal.toolTip()),
        ]
        for row, col, text, widget, tip in dist_fields:
            dist_grid.addWidget(_lbl(text, tip), row, col)
            dist_grid.addWidget(widget, row, col + 1)
            widget.valueChanged.connect(self._on_param_change)
        root.addWidget(dist_box)

        # ---- Aim / lock ----
        aim_box = QGroupBox("Aim & Lock")
        aim_grid = QGridLayout(aim_box)
        aim_grid.setContentsMargins(10, 14, 10, 10)
        aim_grid.setHorizontalSpacing(12)
        aim_grid.setVerticalSpacing(6)

        self.sp_deadzone = _dspin(
            c.offsets.deadzone_norm, 0.0, 0.2, 0.005, 3,
            "Normalized deadband around center — suppresses jitter near aim.",
        )
        self.sp_horiz_off = _dspin(
            c.offsets.horizontal_offset_norm, -0.5, 0.5, 0.05, 2,
            "Horizontal aim bias (−1 left … +1 right of frame center).",
        )
        self.sp_vert_off = _dspin(
            c.offsets.vertical_offset_norm, -0.5, 0.5, 0.05, 2,
            "Vertical aim bias (−1 up … +1 down of frame center).",
        )
        self.sp_lead = _dspin(
            c.prediction.lead_time_s, 0.0, 1.0, 0.02, 2,
            "Seconds of Kalman lead (prediction). 0 = aim at current box center.",
        )
        self.sp_conf_thresh = _dspin(
            c.safety.min_conf_threshold, 0.1, 0.95, 0.05, 2,
            "Minimum detection confidence to keep a candidate lock.",
        )
        self.sp_follow_conf = _dspin(
            c.safety.follow_min_confidence, 0.3, 0.95, 0.05, 2,
            "Confidence required before AI may drive sticks.",
        )
        self.sp_max_lost = _ispin(
            c.safety.max_lost_frames, 5, 200,
            "Frames without a lock before follow fades to mid-stick.",
        )
        self.chk_kalman = QCheckBox("Kalman prediction")
        self.chk_kalman.setChecked(c.prediction.enable_kalman)
        self.chk_kalman.setToolTip("Smooth and predict target motion between detections.")

        aim_fields = [
            (0, 0, "Deadzone", self.sp_deadzone, self.sp_deadzone.toolTip()),
            (0, 2, "H offset", self.sp_horiz_off, self.sp_horiz_off.toolTip()),
            (1, 0, "V offset", self.sp_vert_off, self.sp_vert_off.toolTip()),
            (1, 2, "Lead time (s)", self.sp_lead, self.sp_lead.toolTip()),
            (2, 0, "Min confidence", self.sp_conf_thresh, self.sp_conf_thresh.toolTip()),
            (2, 2, "Follow conf ≥", self.sp_follow_conf, self.sp_follow_conf.toolTip()),
            (3, 0, "Max lost frames", self.sp_max_lost, self.sp_max_lost.toolTip()),
        ]
        for row, col, text, widget, tip in aim_fields:
            aim_grid.addWidget(_lbl(text, tip), row, col)
            aim_grid.addWidget(widget, row, col + 1)
            widget.valueChanged.connect(self._on_param_change)

        self.chk_kalman.toggled.connect(self._on_param_change)
        aim_grid.addWidget(self.chk_kalman, 4, 0, 1, 4)
        root.addWidget(aim_box)

        tip = QLabel("Axis speeds live on the Control tab · changes here apply immediately")
        tip.setStyleSheet("color: #6b7380; font-size: 7.5pt; background: transparent;")
        tip.setWordWrap(True)
        root.addWidget(tip)
        root.addStretch(1)

        scroll.setWidget(body)
        outer.addWidget(scroll)

    def _refresh_presets(self) -> None:
        self.combo_presets.clear()
        presets = list(PRESETS_DIR.glob("*.json"))
        if not presets:
            self.sys_config.save_json(PRESETS_DIR / "default.json")
            presets = [PRESETS_DIR / "default.json"]
        for p in presets:
            self.combo_presets.addItem(p.stem, str(p))

    def _on_param_change(self) -> None:
        self.sys_config.distance.desired_distance_m = self.sp_desired_dist.value()
        self.sys_config.distance.min_safe_distance_m = self.sp_min_dist.value()
        self.sys_config.distance.max_follow_distance_m = self.sp_max_dist.value()
        self.sys_config.distance.known_object_width_m = self.sp_known_w.value()
        self.sys_config.distance.focal_length_px = self.sp_focal.value()
        self.sys_config.offsets.deadzone_norm = self.sp_deadzone.value()
        self.sys_config.offsets.horizontal_offset_norm = self.sp_horiz_off.value()
        self.sys_config.offsets.vertical_offset_norm = self.sp_vert_off.value()
        self.sys_config.prediction.enable_kalman = self.chk_kalman.isChecked()
        self.sys_config.prediction.lead_time_s = self.sp_lead.value()
        self.sys_config.safety.min_conf_threshold = self.sp_conf_thresh.value()
        self.sys_config.safety.follow_min_confidence = self.sp_follow_conf.value()
        self.sys_config.safety.max_lost_frames = self.sp_max_lost.value()
        self.params_updated.emit()

    def _load_selected_preset(self) -> None:
        filepath = self.combo_presets.currentData()
        if filepath and Path(filepath).exists():
            loaded_cfg = SystemConfig.load_json(filepath)
            self.sys_config = loaded_cfg
            self.load_config(loaded_cfg)
            self.preset_loaded.emit(loaded_cfg)

    def _save_preset_dialog(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Preset", str(PRESETS_DIR), "JSON Files (*.json)"
        )
        if path:
            self.sys_config.save_json(path)
            self._refresh_presets()

    def load_config(self, cfg: SystemConfig) -> None:
        self.sys_config = cfg
        widgets = [
            self.sp_desired_dist, self.sp_min_dist, self.sp_max_dist, self.sp_known_w,
            self.sp_focal, self.sp_deadzone, self.sp_horiz_off, self.sp_vert_off, self.sp_lead,
            self.sp_conf_thresh, self.sp_follow_conf, self.sp_max_lost, self.chk_kalman,
        ]
        for w in widgets:
            w.blockSignals(True)
        self.sp_desired_dist.setValue(cfg.distance.desired_distance_m)
        self.sp_min_dist.setValue(cfg.distance.min_safe_distance_m)
        self.sp_max_dist.setValue(cfg.distance.max_follow_distance_m)
        self.sp_known_w.setValue(cfg.distance.known_object_width_m)
        self.sp_focal.setValue(cfg.distance.focal_length_px)
        self.sp_deadzone.setValue(cfg.offsets.deadzone_norm)
        self.sp_horiz_off.setValue(cfg.offsets.horizontal_offset_norm)
        self.sp_vert_off.setValue(cfg.offsets.vertical_offset_norm)
        self.chk_kalman.setChecked(cfg.prediction.enable_kalman)
        self.sp_lead.setValue(cfg.prediction.lead_time_s)
        self.sp_conf_thresh.setValue(cfg.safety.min_conf_threshold)
        self.sp_follow_conf.setValue(cfg.safety.follow_min_confidence)
        self.sp_max_lost.setValue(cfg.safety.max_lost_frames)
        for w in widgets:
            w.blockSignals(False)
