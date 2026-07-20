"""System Parameters Adjustment & Profile Presets Panel."""

from __future__ import annotations

from pathlib import Path
from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from config import PRESETS_DIR, SystemConfig


class ParametersPanel(QWidget):
    params_updated = pyqtSignal()
    preset_loaded = pyqtSignal(object)

    def __init__(self, sys_config: SystemConfig, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.sys_config = sys_config
        self._init_ui()

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)

        # 1. Preset Management
        grp_preset = QGroupBox("Configuration Profiles")
        preset_layout = QHBoxLayout(grp_preset)
        self.combo_presets = QComboBox()
        self._refresh_presets()

        btn_load = QPushButton("Load")
        btn_load.clicked.connect(self._load_selected_preset)

        btn_save = QPushButton("Save As…")
        btn_save.clicked.connect(self._save_preset_dialog)

        preset_layout.addWidget(self.combo_presets, stretch=1)
        preset_layout.addWidget(btn_load)
        preset_layout.addWidget(btn_save)
        layout.addWidget(grp_preset)

        # 2. Distance Parameters
        grp_dist = QGroupBox("Distance Following Parameters")
        dist_layout = QFormLayout(grp_dist)

        self.sp_desired_dist = QDoubleSpinBox()
        self.sp_desired_dist.setRange(0.5, 50.0)
        self.sp_desired_dist.setSingleStep(0.5)
        self.sp_desired_dist.setValue(self.sys_config.distance.desired_distance_m)
        self.sp_desired_dist.valueChanged.connect(self._on_param_change)

        self.sp_min_dist = QDoubleSpinBox()
        self.sp_min_dist.setRange(0.2, 20.0)
        self.sp_min_dist.setSingleStep(0.2)
        self.sp_min_dist.setValue(self.sys_config.distance.min_safe_distance_m)
        self.sp_min_dist.valueChanged.connect(self._on_param_change)

        self.sp_max_dist = QDoubleSpinBox()
        self.sp_max_dist.setRange(2.0, 100.0)
        self.sp_max_dist.setSingleStep(1.0)
        self.sp_max_dist.setValue(self.sys_config.distance.max_follow_distance_m)
        self.sp_max_dist.valueChanged.connect(self._on_param_change)

        self.sp_known_w = QDoubleSpinBox()
        self.sp_known_w.setRange(0.01, 10.0)
        self.sp_known_w.setSingleStep(0.05)
        self.sp_known_w.setValue(self.sys_config.distance.known_object_width_m)
        self.sp_known_w.valueChanged.connect(self._on_param_change)

        dist_layout.addRow("Desired Distance (m):", self.sp_desired_dist)
        dist_layout.addRow("Min Safe Distance (m):", self.sp_min_dist)
        dist_layout.addRow("Max Follow Distance (m):", self.sp_max_dist)
        dist_layout.addRow("Object Width Calibration (m):", self.sp_known_w)
        layout.addWidget(grp_dist)

        # 3. Offsets & Deadzone
        grp_off = QGroupBox("Tracking Deadzone & Offsets")
        off_layout = QFormLayout(grp_off)

        self.sp_deadzone = QDoubleSpinBox()
        self.sp_deadzone.setRange(0.0, 0.2)
        self.sp_deadzone.setSingleStep(0.005)
        self.sp_deadzone.setValue(self.sys_config.offsets.deadzone_norm)
        self.sp_deadzone.valueChanged.connect(self._on_param_change)

        self.sp_horiz_off = QDoubleSpinBox()
        self.sp_horiz_off.setRange(-0.5, 0.5)
        self.sp_horiz_off.setSingleStep(0.05)
        self.sp_horiz_off.setValue(self.sys_config.offsets.horizontal_offset_norm)
        self.sp_horiz_off.valueChanged.connect(self._on_param_change)

        self.sp_vert_off = QDoubleSpinBox()
        self.sp_vert_off.setRange(-0.5, 0.5)
        self.sp_vert_off.setSingleStep(0.05)
        self.sp_vert_off.setValue(self.sys_config.offsets.vertical_offset_norm)
        self.sp_vert_off.valueChanged.connect(self._on_param_change)

        off_layout.addRow("Deadzone Size (norm):", self.sp_deadzone)
        off_layout.addRow("Horizontal Offset:", self.sp_horiz_off)
        off_layout.addRow("Vertical Offset:", self.sp_vert_off)
        layout.addWidget(grp_off)

        # 4. Motion Prediction & Safety
        grp_pred = QGroupBox("Motion Prediction & Safety Thresholds")
        pred_layout = QFormLayout(grp_pred)

        self.chk_kalman = QCheckBox("Enable Kalman Filter Prediction")
        self.chk_kalman.setChecked(self.sys_config.prediction.enable_kalman)
        self.chk_kalman.toggled.connect(self._on_param_change)

        self.sp_lead = QDoubleSpinBox()
        self.sp_lead.setRange(0.0, 1.0)
        self.sp_lead.setSingleStep(0.02)
        self.sp_lead.setValue(self.sys_config.prediction.lead_time_s)
        self.sp_lead.valueChanged.connect(self._on_param_change)

        self.sp_conf_thresh = QDoubleSpinBox()
        self.sp_conf_thresh.setRange(0.1, 0.95)
        self.sp_conf_thresh.setSingleStep(0.05)
        self.sp_conf_thresh.setValue(self.sys_config.safety.min_conf_threshold)
        self.sp_conf_thresh.valueChanged.connect(self._on_param_change)

        self.sp_max_lost = QSpinBox()
        self.sp_max_lost.setRange(5, 200)
        self.sp_max_lost.setValue(self.sys_config.safety.max_lost_frames)
        self.sp_max_lost.valueChanged.connect(self._on_param_change)

        pred_layout.addRow(self.chk_kalman)
        pred_layout.addRow("Lead Time (seconds):", self.sp_lead)
        pred_layout.addRow("Min Confidence Threshold:", self.sp_conf_thresh)
        pred_layout.addRow("Max Lost Frames Timeout:", self.sp_max_lost)
        layout.addWidget(grp_pred)

    def _refresh_presets(self) -> None:
        self.combo_presets.clear()
        presets = list(PRESETS_DIR.glob("*.json"))
        if not presets:
            # Create default preset if none exist
            self.sys_config.save_json(PRESETS_DIR / "default.json")
            presets = [PRESETS_DIR / "default.json"]

        for p in presets:
            self.combo_presets.addItem(p.stem, str(p))

    def _on_param_change(self) -> None:
        self.sys_config.distance.desired_distance_m = self.sp_desired_dist.value()
        self.sys_config.distance.min_safe_distance_m = self.sp_min_dist.value()
        self.sys_config.distance.max_follow_distance_m = self.sp_max_dist.value()
        self.sys_config.distance.known_object_width_m = self.sp_known_w.value()

        self.sys_config.offsets.deadzone_norm = self.sp_deadzone.value()
        self.sys_config.offsets.horizontal_offset_norm = self.sp_horiz_off.value()
        self.sys_config.offsets.vertical_offset_norm = self.sp_vert_off.value()

        self.sys_config.prediction.enable_kalman = self.chk_kalman.isChecked()
        self.sys_config.prediction.lead_time_s = self.sp_lead.value()

        self.sys_config.safety.min_conf_threshold = self.sp_conf_thresh.value()
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
        path, _ = QFileDialog.getSaveFileName(self, "Save Preset Profile", str(PRESETS_DIR), "JSON Files (*.json)")
        if path:
            self.sys_config.save_json(path)
            self._refresh_presets()

    def load_config(self, cfg: SystemConfig) -> None:
        self.sys_config = cfg
        self.sp_desired_dist.blockSignals(True)
        self.sp_min_dist.blockSignals(True)
        self.sp_max_dist.blockSignals(True)
        self.sp_known_w.blockSignals(True)
        self.sp_deadzone.blockSignals(True)
        self.sp_horiz_off.blockSignals(True)
        self.sp_vert_off.blockSignals(True)
        self.chk_kalman.blockSignals(True)
        self.sp_lead.blockSignals(True)
        self.sp_conf_thresh.blockSignals(True)
        self.sp_max_lost.blockSignals(True)

        self.sp_desired_dist.setValue(cfg.distance.desired_distance_m)
        self.sp_min_dist.setValue(cfg.distance.min_safe_distance_m)
        self.sp_max_dist.setValue(cfg.distance.max_follow_distance_m)
        self.sp_known_w.setValue(cfg.distance.known_object_width_m)
        self.sp_deadzone.setValue(cfg.offsets.deadzone_norm)
        self.sp_horiz_off.setValue(cfg.offsets.horizontal_offset_norm)
        self.sp_vert_off.setValue(cfg.offsets.vertical_offset_norm)
        self.chk_kalman.setChecked(cfg.prediction.enable_kalman)
        self.sp_lead.setValue(cfg.prediction.lead_time_s)
        self.sp_conf_thresh.setValue(cfg.safety.min_conf_threshold)
        self.sp_max_lost.setValue(cfg.safety.max_lost_frames)

        self.sp_desired_dist.blockSignals(False)
        self.sp_min_dist.blockSignals(False)
        self.sp_max_dist.blockSignals(False)
        self.sp_known_w.blockSignals(False)
        self.sp_deadzone.blockSignals(False)
        self.sp_horiz_off.blockSignals(False)
        self.sp_vert_off.blockSignals(False)
        self.chk_kalman.blockSignals(False)
        self.sp_lead.blockSignals(False)
        self.sp_conf_thresh.blockSignals(False)
        self.sp_max_lost.blockSignals(False)
