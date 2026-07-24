"""Distance calibration — Live Feed lock KPIs + tape-measure focal setup."""

from __future__ import annotations

from collections import deque
from statistics import median

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
    QSizePolicy,
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
from gui.style import PALETTE
from gui.widgets.metric_card import MetricCard
from gui.widgets.page_header import PageHeader, StatusPill


def _dspin(value: float, lo: float, hi: float, step: float, decimals: int = 2) -> QDoubleSpinBox:
    sp = QDoubleSpinBox()
    sp.setRange(lo, hi)
    sp.setSingleStep(step)
    sp.setDecimals(decimals)
    sp.setValue(value)
    sp.setMinimumWidth(118)
    sp.setAlignment(Qt.AlignmentFlag.AlignRight)
    return sp


class DistanceCalibPage(QWidget):
    """Parameters + KPIs driven by the Live Feed lock box (no separate camera)."""

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
        self._size_history: deque[float] = deque(maxlen=90)
        self._w_history: deque[float] = deque(maxlen=90)
        self._h_history: deque[float] = deque(maxlen=90)
        self._last_size_px: float | None = None
        self._last_dist_m: float | None = None
        self._last_bw = 0.0
        self._last_bh = 0.0

        self._build_ui()
        self._load_from_config()
        self.worker.frame_processed.connect(self._on_live_telemetry)

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(10)

        header = PageHeader(
            "Distance Calibration",
            "Lock a tight box on Live Feed · Dist = (object_m × focal_px) / box_px",
        )
        self.pill_status = StatusPill("NO LIVE LOCK", "warn")
        header.right_layout.addWidget(self.pill_status)
        root.addWidget(header)

        # ---- KPI row ----
        kpi_row = QHBoxLayout()
        kpi_row.setSpacing(8)
        self.kpi_dist = MetricCard("Distance", "-- m", "from lock box", PALETTE["ok"])
        self.kpi_size = MetricCard("Box size", "-- px", "measured axis", PALETTE["accent"])
        self.kpi_w = MetricCard("Width", "-- px", "bbox W", PALETTE["info"])
        self.kpi_h = MetricCard("Height", "-- px", "bbox H", "#c9a227")
        self.kpi_focal = MetricCard("Focal", "-- px", "calibrated", PALETTE["accent"])
        self.kpi_stable = MetricCard("Stability", "--", "size jitter", PALETTE["warn"])
        for card in (
            self.kpi_dist, self.kpi_size, self.kpi_w,
            self.kpi_h, self.kpi_focal, self.kpi_stable,
        ):
            card.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            card.setMinimumHeight(88)
            kpi_row.addWidget(card)
        root.addLayout(kpi_row)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        body = QWidget()
        body_l = QHBoxLayout(body)
        body_l.setContentsMargins(0, 4, 4, 0)
        body_l.setSpacing(12)

        # ---- Left: tape calib ----
        left = QVBoxLayout()
        left.setSpacing(10)

        tape = QGroupBox("Tape-Measure Calibration")
        tape.setObjectName("panel")
        tape_form = QFormLayout(tape)
        tape_form.setContentsMargins(14, 18, 14, 14)
        tape_form.setHorizontalSpacing(14)
        tape_form.setVerticalSpacing(10)
        tape_form.setLabelAlignment(Qt.AlignmentFlag.AlignLeft)

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
        self.combo_axis.setToolTip(
            "Which lock-box dimension drives distance.\n"
            "Use width for phones/books edge-on; height for upright bottles; max is safest default."
        )
        tape_form.addRow("Size axis (pixel edge)", self.combo_axis)

        help_lbl = QLabel(
            "1) Live Feed → drag a TIGHT box on object edges\n"
            "2) Enter tape distance (m) + real size inside the box (cm)\n"
            "3) Compute Focal — then walk closer/farther to verify"
        )
        help_lbl.setWordWrap(True)
        help_lbl.setStyleSheet(
            f"color: {PALETTE['text_mute']}; background: transparent; font-size: 8.5pt; "
            "padding: 6px 0;"
        )
        tape_form.addRow(help_lbl)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(8)
        self.btn_calibrate = QPushButton("Compute Focal from Lock")
        self.btn_calibrate.setObjectName("btnPrimary")
        self.btn_calibrate.setMinimumHeight(34)
        self.btn_calibrate.clicked.connect(self._calibrate_from_live)
        btn_row.addWidget(self.btn_calibrate)
        self.btn_save = QPushButton("Save")
        self.btn_save.setObjectName("btnGhost")
        self.btn_save.clicked.connect(self._save_only)
        btn_row.addWidget(self.btn_save)
        self.btn_reload = QPushButton("Reload")
        self.btn_reload.setObjectName("btnGhost")
        self.btn_reload.clicked.connect(self._reload_calib)
        btn_row.addWidget(self.btn_reload)
        tape_form.addRow(btn_row)

        self.lbl_result = QLabel("")
        self.lbl_result.setWordWrap(True)
        self.lbl_result.setStyleSheet(
            f"color: {PALETTE['text_dim']}; background: transparent; font-size: 9pt;"
        )
        tape_form.addRow(self.lbl_result)
        left.addWidget(tape)

        readout = QGroupBox("Lock Box Pixels")
        ro = QVBoxLayout(readout)
        ro.setContentsMargins(14, 16, 14, 14)
        ro.setSpacing(6)
        self.lbl_lock = QLabel("Waiting for Live Feed lock…")
        self.lbl_bbox = QLabel("BBox: —")
        self.lbl_axis = QLabel("Measured edge: —")
        for lbl in (self.lbl_lock, self.lbl_bbox, self.lbl_axis):
            lbl.setStyleSheet(
                f"color: {PALETTE['text']}; background: transparent; font-size: 10pt;"
            )
            ro.addWidget(lbl)
        note = QLabel(
            "Yellow line on Live Feed = measured axis. "
            "Keep the box snug — empty margin inflates distance error."
        )
        note.setWordWrap(True)
        note.setStyleSheet(
            f"color: {PALETTE['text_mute']}; background: transparent; font-size: 8.5pt;"
        )
        ro.addWidget(note)
        left.addWidget(readout)
        left.addStretch(1)
        body_l.addLayout(left, stretch=2)

        # ---- Right: all distance params ----
        params = QGroupBox("Distance Parameters")
        grid = QGridLayout(params)
        grid.setContentsMargins(14, 18, 14, 14)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(10)

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
            (1, 0, "Desired follow", self.sp_desired),
            (1, 2, "Min safe", self.sp_min_safe),
            (2, 0, "Max range", self.sp_max_range),
            (2, 2, "Max pitch", self.sp_max_pitch),
            (3, 0, "Distance Kp", self.sp_kp),
            (3, 2, "Distance Ki", self.sp_ki),
            (4, 0, "Distance Kd", self.sp_kd),
        ]
        for row, col, text, widget in fields:
            lbl = QLabel(text)
            lbl.setStyleSheet(f"color: {PALETTE['text_mute']}; background: transparent;")
            grid.addWidget(lbl, row, col)
            grid.addWidget(widget, row, col + 1)
            widget.valueChanged.connect(self._apply_params_live)

        self.btn_apply = QPushButton("Apply to Live Feed")
        self.btn_apply.setObjectName("btnPrimary")
        self.btn_apply.setMinimumHeight(34)
        self.btn_apply.clicked.connect(self._apply_and_notify)
        grid.addWidget(self.btn_apply, 5, 0, 1, 4)

        body_l.addWidget(params, stretch=3)

        scroll.setWidget(body)
        root.addWidget(scroll, stretch=1)

        self.combo_axis.currentTextChanged.connect(self._on_axis_changed)
        self._update_focal_kpi()

    def _stable_size_px(self) -> float | None:
        """Median of recent samples with outlier rejection — stable pixel count for calib."""
        if len(self._size_history) < 5:
            return None
        samples = list(self._size_history)
        med = float(median(samples))
        if med < 8.0:
            return None
        filtered = [s for s in samples if abs(s - med) / med <= 0.18]
        if len(filtered) < 3:
            filtered = samples
        return float(median(filtered))

    def _jitter_pct(self) -> float | None:
        if len(self._size_history) < 8:
            return None
        med = float(median(self._size_history))
        if med < 1.0:
            return None
        # mean absolute deviation / median
        mad = sum(abs(s - med) for s in self._size_history) / len(self._size_history)
        return 100.0 * mad / med

    def _update_focal_kpi(self) -> None:
        f = self.sys_config.distance.focal_length_px
        cal = "calibrated" if is_calibrated(self.sys_config) else "default / FOV"
        self.kpi_focal.set_value(f"{f:.0f} px", cal)

    def _clear_kpis(self) -> None:
        self.kpi_dist.set_value("-- m", "no lock")
        self.kpi_size.set_value("-- px", "no lock")
        self.kpi_w.set_value("-- px", "bbox W")
        self.kpi_h.set_value("-- px", "bbox H")
        self.kpi_stable.set_value("--", "size jitter")
        self.kpi_stable.set_accent(PALETTE["warn"])

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
        self._update_focal_kpi()
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
        self._update_focal_kpi()

    def _apply_and_notify(self) -> None:
        self._apply_params_live()
        self.calib_saved.emit()
        self.lbl_result.setText("Parameters applied to Live Feed.")

    @pyqtSlot(object, object)
    def _on_live_telemetry(self, _frame, rec) -> None:
        try:
            locked = bool(getattr(rec, "locked", False))
            self._locked = locked
            if not locked:
                self.lbl_lock.setText("Waiting for Live Feed lock…")
                self.lbl_bbox.setText("BBox: —")
                self.lbl_axis.setText("Measured edge: —")
                self._size_history.clear()
                self._w_history.clear()
                self._h_history.clear()
                self._last_size_px = None
                self._last_dist_m = None
                self._clear_kpis()
                self._refresh_status_labels()
                return

            bw = max(1.0, float(getattr(rec, "bbox_w", 0.0)))
            bh = max(1.0, float(getattr(rec, "bbox_h", 0.0)))
            bx = float(getattr(rec, "bbox_x", 0.0))
            by = float(getattr(rec, "bbox_y", 0.0))
            axis = self.combo_axis.currentText()
            size_px = bbox_size_px((bx, by, bw, bh), axis)

            # Reject tiny / exploding box noise
            if self._last_size_px and self._last_size_px > 12:
                if size_px > self._last_size_px * 2.8 or size_px < self._last_size_px * 0.35:
                    size_px = self._last_size_px

            self._w_history.append(bw)
            self._h_history.append(bh)
            self._size_history.append(size_px)
            self._last_bw, self._last_bh = bw, bh
            self._last_size_px = size_px

            stable = self._stable_size_px() or size_px
            dist_m = estimate_distance_m(
                stable,
                self.sys_config.distance.focal_length_px,
                self.sys_config.distance.known_object_width_m,
            )
            self._last_dist_m = dist_m
            jitter = self._jitter_pct()

            src = str(getattr(rec, "source", "lock")).upper()
            self.lbl_lock.setText(f"Lock active · {src} · conf {float(getattr(rec, 'confidence', 0)) * 100:.0f}%")
            self.lbl_bbox.setText(f"BBox: {bw:.0f} × {bh:.0f} px   at ({bx:.0f}, {by:.0f})")
            self.lbl_axis.setText(
                f"Measured edge ({axis}): {stable:.1f} px   "
                f"(raw {size_px:.1f} · n={len(self._size_history)})"
            )

            self.kpi_dist.set_value(f"{dist_m:.2f} m", f"{dist_m * 100:.0f} cm")
            self.kpi_size.set_value(f"{stable:.0f} px", f"axis={axis}")
            self.kpi_w.set_value(f"{bw:.0f} px", "bbox width")
            self.kpi_h.set_value(f"{bh:.0f} px", "bbox height")
            self._update_focal_kpi()
            if jitter is None:
                self.kpi_stable.set_value("…", "warming up")
                self.kpi_stable.set_accent(PALETTE["warn"])
            else:
                tone = PALETTE["ok"] if jitter < 4.0 else (PALETTE["warn"] if jitter < 10.0 else PALETTE["error"])
                self.kpi_stable.set_value(f"{jitter:.1f}%", "MAD / median")
                self.kpi_stable.set_accent(tone)

            self._refresh_status_labels()
        except Exception:
            pass

    def _calibrate_from_live(self) -> None:
        size_px = self._stable_size_px()
        if not self._locked or size_px is None:
            QMessageBox.warning(
                self,
                "Need stable Live Feed lock",
                "Open Live Camera Feed, drag a tight box on the object edges, "
                "wait until Stability KPI drops under ~5%, then calibrate.",
            )
            return

        dist_m = float(self.sp_tape_m.value())
        object_cm = float(self.sp_object_cm.value())
        err = validate_calib_inputs(dist_m, object_cm)
        if err:
            QMessageBox.warning(self, "Invalid input", err)
            return

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
        self._update_focal_kpi()
        self._refresh_status_labels()

        ok = abs(verify - dist_m) < 0.02
        jitter = self._jitter_pct()
        jit_txt = f"{jitter:.1f}%" if jitter is not None else "n/a"
        self.lbl_result.setText(
            f"Median box {size_px:.1f} px · jitter {jit_txt} · focal {new_f:.1f} px · "
            f"verify {verify:.3f} m (tape {dist_m:.3f} m)\nSaved → {path}"
        )
        if ok:
            QMessageBox.information(
                self,
                "Calibration OK",
                f"Focal = {new_f:.1f} px\nBox = {size_px:.1f} px ({axis})\n"
                f"Verify = {verify:.3f} m\n\n"
                "On Live Feed, move closer/farther — W/H px and DIST should track.",
            )
        else:
            QMessageBox.warning(
                self,
                "Verify mismatch",
                f"Verify {verify:.3f} m vs tape {dist_m:.3f} m.\n"
                "Tighten the lock box on Live Feed and retry.",
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
        self._update_focal_kpi()
        self.lbl_result.setText(f"Saved → {path}")

    def _reload_calib(self) -> None:
        data = load_distance_calib(self.sys_config)
        if not data:
            QMessageBox.information(self, "No file", "No distance_calib.json found yet.")
            return
        self._load_from_config()
        self.worker.update_config(self.sys_config)
        self.calib_saved.emit()
        self.lbl_result.setText("Reloaded saved calibration.")
