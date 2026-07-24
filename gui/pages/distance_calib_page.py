"""Distance calibration parameters — uses Live Feed lock data (no separate camera)."""

from __future__ import annotations

from collections import deque

from PyQt6.QtCore import Qt, pyqtSignal, pyqtSlot
from PyQt6.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from config import SystemConfig
from core.tracking_worker import TrackingWorkerThread
from estimation.distance_calib import (
    MIN_CALIB_DISTANCE_M,
    bbox_size_px,
    estimate_distance_m,
    focal_from_sample,
    is_calibrated,
    load_distance_calib,
    save_distance_calib,
    validate_calib_inputs,
)
from gui.widgets.page_header import PageHeader, StatusPill


def _dspin(value: float, lo: float, hi: float, step: float, decimals: int = 2) -> QDoubleSpinBox:
    sp = QDoubleSpinBox()
    sp.setRange(lo, hi)
    sp.setSingleStep(step)
    sp.setDecimals(decimals)
    sp.setValue(value)
    sp.setMinimumWidth(110)
    sp.setAlignment(Qt.AlignmentFlag.AlignRight)
    return sp


class DistanceCalibPage(QWidget):
    """Set every distance parameter; calibrate focal from Live Feed lock bbox."""

    calib_saved = pyqtSignal()

    def __init__(
        self,
        worker: TrackingWorkerThread,
        sys_config: SystemConfig,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.worker = worker
        self.sys_config = sys_config
        self._locked = False
        self._size_history: deque[float] = deque(maxlen=60)
        self._last_size_px: float | None = None
        self._last_dist_m: float | None = None

        self._build_ui()
        self._load_from_config()
        self.worker.frame_processed.connect(self._on_live_telemetry)

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(8)

        header = PageHeader(
            "Distance Calibration",
            "Lock a tight box on Live Feed, then set tape distance + object size here. "
            "Formula: Dist = (object_m × focal_px) / box_px",
        )
        self.pill_status = StatusPill("NO LIVE LOCK", "warn")
        header.right_layout.addWidget(self.pill_status)
        root.addWidget(header)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        body = QWidget()
        body_l = QVBoxLayout(body)
        body_l.setContentsMargins(0, 0, 8, 0)
        body_l.setSpacing(12)

        # ---- Live readout from Live Feed ----
        live = QGroupBox("Live Feed Readout")
        live_l = QGridLayout(live)
        live_l.setContentsMargins(12, 14, 12, 12)
        live_l.setHorizontalSpacing(16)
        live_l.setVerticalSpacing(8)
        self.lbl_lock = QLabel("Lock: —")
        self.lbl_box = QLabel("Box size: —")
        self.lbl_dist = QLabel("Estimated distance: —")
        self.lbl_bbox = QLabel("BBox: —")
        for i, lbl in enumerate((self.lbl_lock, self.lbl_box, self.lbl_dist, self.lbl_bbox)):
            lbl.setStyleSheet("background: transparent; font-size: 10pt;")
            live_l.addWidget(lbl, i // 2, i % 2)
        tip = QLabel(
            "Go to Live Camera Feed → drag a tight ROI on the object → return here to calibrate."
        )
        tip.setWordWrap(True)
        tip.setStyleSheet("color: #6b7380; background: transparent;")
        live_l.addWidget(tip, 2, 0, 1, 2)
        body_l.addWidget(live)

        # ---- Tape-measure calibration ----
        tape = QGroupBox("Tape-Measure Calibration")
        tape_form = QFormLayout(tape)
        tape_form.setContentsMargins(12, 14, 12, 12)
        tape_form.setHorizontalSpacing(12)
        tape_form.setVerticalSpacing(8)

        self.sp_tape_m = _dspin(1.0, MIN_CALIB_DISTANCE_M, 30.0, 0.05, 3)
        self.sp_tape_m.setSuffix(" m")
        tape_form.addRow("Known distance (tape)", self.sp_tape_m)

        self.sp_object_cm = _dspin(
            max(1.0, self.sys_config.distance.known_object_width_m * 100.0),
            1.0,
            250.0,
            0.5,
            1,
        )
        self.sp_object_cm.setSuffix(" cm")
        self.sp_object_cm.valueChanged.connect(self._apply_params_live)
        tape_form.addRow("Object size in box", self.sp_object_cm)

        self.combo_axis = QComboBox()
        self.combo_axis.addItems(["max", "width", "height", "diag"])
        tape_form.addRow("Size axis", self.combo_axis)

        help_lbl = QLabel(
            "Q1 = camera → object in meters.  Q2 = real size of what is INSIDE the lock box "
            "(phone ~7 cm, bottle ~6–8, book ~15). Loose box = wrong distance."
        )
        help_lbl.setWordWrap(True)
        help_lbl.setStyleSheet("color: #6b7380; background: transparent; font-size: 8.5pt;")
        tape_form.addRow(help_lbl)

        btn_row = QHBoxLayout()
        self.btn_calibrate = QPushButton("Compute Focal from Live Lock")
        self.btn_calibrate.setObjectName("btnPrimary")
        self.btn_calibrate.clicked.connect(self._calibrate_from_live)
        btn_row.addWidget(self.btn_calibrate)
        self.btn_save = QPushButton("Save Calib File")
        self.btn_save.setObjectName("btnGhost")
        self.btn_save.clicked.connect(self._save_only)
        btn_row.addWidget(self.btn_save)
        self.btn_reload = QPushButton("Reload File")
        self.btn_reload.setObjectName("btnGhost")
        self.btn_reload.clicked.connect(self._reload_calib)
        btn_row.addWidget(self.btn_reload)
        btn_row.addStretch()
        tape_form.addRow(btn_row)

        self.lbl_result = QLabel("")
        self.lbl_result.setWordWrap(True)
        self.lbl_result.setStyleSheet("background: transparent; color: #9aa3ad;")
        tape_form.addRow(self.lbl_result)
        body_l.addWidget(tape)

        # ---- All distance parameters ----
        params = QGroupBox("Distance Parameters")
        grid = QGridLayout(params)
        grid.setContentsMargins(12, 14, 12, 12)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(8)

        d = self.sys_config.distance
        self.sp_focal = _dspin(d.focal_length_px, 50.0, 20000.0, 10.0, 1)
        self.sp_focal.setSuffix(" px")
        self.sp_known_m = _dspin(d.known_object_width_m, 0.01, 2.5, 0.01, 3)
        self.sp_known_m.setSuffix(" m")
        self.sp_desired = _dspin(d.desired_distance_m, 0.2, 50.0, 0.5, 1)
        self.sp_desired.setSuffix(" m")
        self.sp_min_safe = _dspin(d.min_safe_distance_m, 0.1, 20.0, 0.2, 1)
        self.sp_min_safe.setSuffix(" m")
        self.sp_max_range = _dspin(d.max_follow_distance_m, 1.0, 100.0, 1.0, 1)
        self.sp_max_range.setSuffix(" m")
        self.sp_kp = _dspin(d.kp, 0.0, 1000.0, 5.0, 1)
        self.sp_ki = _dspin(d.ki, 0.0, 500.0, 1.0, 1)
        self.sp_kd = _dspin(d.kd, 0.0, 500.0, 1.0, 1)
        self.sp_max_pitch = _dspin(d.max_pitch_offset, 10.0, 500.0, 5.0, 0)
        self.sp_max_pitch.setSuffix(" µs")

        fields = [
            (0, 0, "Focal length", self.sp_focal),
            (0, 2, "Known object width", self.sp_known_m),
            (1, 0, "Desired follow distance", self.sp_desired),
            (1, 2, "Min safe distance", self.sp_min_safe),
            (2, 0, "Max follow range", self.sp_max_range),
            (2, 2, "Max pitch offset", self.sp_max_pitch),
            (3, 0, "Distance Kp", self.sp_kp),
            (3, 2, "Distance Ki", self.sp_ki),
            (4, 0, "Distance Kd", self.sp_kd),
        ]
        for row, col, text, widget in fields:
            grid.addWidget(QLabel(text), row, col)
            grid.addWidget(widget, row, col + 1)
            widget.valueChanged.connect(self._apply_params_live)

        apply_row = QHBoxLayout()
        self.btn_apply = QPushButton("Apply Parameters to Live Feed")
        self.btn_apply.setObjectName("btnPrimary")
        self.btn_apply.clicked.connect(self._apply_and_notify)
        apply_row.addWidget(self.btn_apply)
        apply_row.addStretch()
        grid.addLayout(apply_row, 5, 0, 1, 4)
        body_l.addWidget(params)

        body_l.addStretch(1)
        scroll.setWidget(body)
        root.addWidget(scroll, stretch=1)

        self.combo_axis.currentTextChanged.connect(self._on_axis_changed)

    def _load_from_config(self) -> None:
        d = self.sys_config.distance
        widgets = [
            self.sp_focal, self.sp_known_m, self.sp_desired, self.sp_min_safe,
            self.sp_max_range, self.sp_kp, self.sp_ki, self.sp_kd, self.sp_max_pitch,
            self.sp_object_cm,
        ]
        for w in widgets:
            w.blockSignals(True)
        self.sp_focal.setValue(d.focal_length_px)
        self.sp_known_m.setValue(d.known_object_width_m)
        self.sp_object_cm.setValue(d.known_object_width_m * 100.0)
        self.sp_desired.setValue(d.desired_distance_m)
        self.sp_min_safe.setValue(d.min_safe_distance_m)
        self.sp_max_range.setValue(d.max_follow_distance_m)
        self.sp_kp.setValue(d.kp)
        self.sp_ki.setValue(d.ki)
        self.sp_kd.setValue(d.kd)
        self.sp_max_pitch.setValue(d.max_pitch_offset)
        idx = self.combo_axis.findText(getattr(d, "size_axis", "max") or "max")
        if idx >= 0:
            self.combo_axis.setCurrentIndex(idx)
        for w in widgets:
            w.blockSignals(False)
        self._refresh_status_labels()

    def _refresh_status_labels(self) -> None:
        if self._locked:
            self.pill_status.set_status("LIVE LOCK", "ok")
        elif is_calibrated(self.sys_config):
            self.pill_status.set_status("CALIBRATED", "ok")
        else:
            self.pill_status.set_status("NO LIVE LOCK", "warn")

    def _on_axis_changed(self, _axis: str) -> None:
        self._size_history.clear()
        self._apply_params_live()

    def _write_config_from_widgets(self) -> None:
        d = self.sys_config.distance
        d.focal_length_px = float(self.sp_focal.value())
        d.known_object_width_m = float(self.sp_known_m.value())
        d.size_axis = self.combo_axis.currentText()
        d.desired_distance_m = float(self.sp_desired.value())
        d.min_safe_distance_m = float(self.sp_min_safe.value())
        d.max_follow_distance_m = float(self.sp_max_range.value())
        d.kp = float(self.sp_kp.value())
        d.ki = float(self.sp_ki.value())
        d.kd = float(self.sp_kd.value())
        d.max_pitch_offset = float(self.sp_max_pitch.value())

    def _apply_params_live(self) -> None:
        # Keep object cm spin in sync when known_m edited
        if self.sender() is self.sp_known_m:
            self.sp_object_cm.blockSignals(True)
            self.sp_object_cm.setValue(self.sp_known_m.value() * 100.0)
            self.sp_object_cm.blockSignals(False)
        elif self.sender() is self.sp_object_cm:
            self.sp_known_m.blockSignals(True)
            self.sp_known_m.setValue(self.sp_object_cm.value() / 100.0)
            self.sp_known_m.blockSignals(False)

        self._write_config_from_widgets()
        self.worker.update_config(self.sys_config)

    def _apply_and_notify(self) -> None:
        self._apply_params_live()
        self.calib_saved.emit()
        self.lbl_result.setText("Parameters applied to Live Feed / follow controller.")

    @pyqtSlot(object, object)
    def _on_live_telemetry(self, _frame, rec) -> None:
        try:
            locked = bool(getattr(rec, "locked", False))
            self._locked = locked
            if not locked:
                self.lbl_lock.setText("Lock: none — lock on Live Feed first")
                self.lbl_box.setText("Box size: —")
                self.lbl_dist.setText("Estimated distance: —")
                self.lbl_bbox.setText("BBox: —")
                self._size_history.clear()
                self._last_size_px = None
                self._last_dist_m = None
                self._refresh_status_labels()
                return

            bw = float(getattr(rec, "bbox_w", 0.0))
            bh = float(getattr(rec, "bbox_h", 0.0))
            bx = float(getattr(rec, "bbox_x", 0.0))
            by = float(getattr(rec, "bbox_y", 0.0))
            axis = self.combo_axis.currentText()
            size_px = bbox_size_px((bx, by, bw, bh), axis)
            dist_m = estimate_distance_m(
                size_px,
                self.sys_config.distance.focal_length_px,
                self.sys_config.distance.known_object_width_m,
            )
            self._size_history.append(size_px)
            self._last_size_px = size_px
            self._last_dist_m = dist_m
            avg = sum(self._size_history) / len(self._size_history)

            src = str(getattr(rec, "source", "lock")).upper()
            self.lbl_lock.setText(f"Lock: YES ({src})")
            self.lbl_box.setText(f"Box size: {size_px:.1f} px  (avg {avg:.1f}, axis={axis})")
            self.lbl_dist.setText(f"Estimated distance: {dist_m:.3f} m  ({dist_m * 100:.0f} cm)")
            self.lbl_bbox.setText(f"BBox: {bw:.0f}×{bh:.0f} px at ({bx:.0f},{by:.0f})")
            self._refresh_status_labels()
        except Exception:
            pass

    def _calibrate_from_live(self) -> None:
        if not self._locked or len(self._size_history) < 5:
            QMessageBox.warning(
                self,
                "Need Live Feed lock",
                "Open Live Camera Feed, drag a tight box on the object, wait a second, "
                "then come back and calibrate.",
            )
            return

        dist_m = float(self.sp_tape_m.value())
        object_cm = float(self.sp_object_cm.value())
        err = validate_calib_inputs(dist_m, object_cm)
        if err:
            QMessageBox.warning(self, "Invalid input", err)
            return

        size_px = sum(self._size_history) / len(self._size_history)
        width_m = object_cm / 100.0
        axis = self.combo_axis.currentText()
        new_f = focal_from_sample(size_px, dist_m, width_m)

        self.sys_config.distance.focal_length_px = new_f
        self.sys_config.distance.known_object_width_m = width_m
        self.sys_config.distance.size_axis = axis

        self.sp_focal.blockSignals(True)
        self.sp_known_m.blockSignals(True)
        self.sp_focal.setValue(new_f)
        self.sp_known_m.setValue(width_m)
        self.sp_focal.blockSignals(False)
        self.sp_known_m.blockSignals(False)

        self._write_config_from_widgets()
        verify = estimate_distance_m(size_px, new_f, width_m)

        try:
            path = save_distance_calib(self.sys_config)
        except OSError as exc:
            QMessageBox.critical(self, "Save failed", str(exc))
            return

        self.worker.update_config(self.sys_config)
        self.calib_saved.emit()
        self._refresh_status_labels()

        ok = abs(verify - dist_m) < 0.02
        self.lbl_result.setText(
            f"Avg box {size_px:.1f} px · focal {new_f:.1f} px · verify {verify:.3f} m "
            f"(tape {dist_m:.3f} m)\nSaved → {path}"
        )
        if ok:
            QMessageBox.information(
                self,
                "Calibration OK",
                f"Focal = {new_f:.1f} px\nObject = {object_cm:.1f} cm\n"
                f"Verify = {verify:.3f} m\n\n"
                "On Live Feed, move closer/farther — distance should change.",
            )
        else:
            QMessageBox.warning(
                self,
                "Verify mismatch",
                f"Verify {verify:.3f} m vs tape {dist_m:.3f} m. Use a tighter lock box.",
            )

    def _save_only(self) -> None:
        self._write_config_from_widgets()
        try:
            path = save_distance_calib(self.sys_config)
        except OSError as exc:
            QMessageBox.critical(self, "Save failed", str(exc))
            return
        self.worker.update_config(self.sys_config)
        self.calib_saved.emit()
        self.lbl_result.setText(f"Saved current parameters → {path}")

    def _reload_calib(self) -> None:
        data = load_distance_calib(self.sys_config)
        if not data:
            QMessageBox.information(self, "No file", "No distance_calib.json found yet.")
            return
        self._load_from_config()
        self.worker.update_config(self.sys_config)
        self.calib_saved.emit()
        self.lbl_result.setText("Reloaded saved calibration into Live Feed config.")
