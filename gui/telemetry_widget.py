"""Real-time telemetry charts for Arjuna GCS."""

from __future__ import annotations

import collections
import time

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import QGridLayout, QHBoxLayout, QLabel, QVBoxLayout, QWidget

from telemetry.telemetry_logger import TelemetryRecord

pg.setConfigOptions(antialias=True, background="#0f1115", foreground="#9aa3b2")

# Clean slate chart palette
C_PRIMARY = "#4f7cac"
C_SECONDARY = "#b05656"
C_OK = "#3d8f6a"
C_WARN = "#b08a3c"
C_MUTE = "#6b7380"
C_GRID = "#2a3038"


def _make_pen(color: str, width: float = 2.0) -> pg.mkPen:
    return pg.mkPen(color=color, width=width)


class ChartPanel(QWidget):
    """Single chart with header label and live value readout."""

    def __init__(self, title: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(4)

        header = QHBoxLayout()
        header.setContentsMargins(4, 0, 4, 0)
        self.lbl_title = QLabel(title)
        self.lbl_title.setStyleSheet(
            "color: #9aa3b2; font-size: 8.5pt; font-weight: 600; letter-spacing: 1px;"
        )
        self.lbl_value = QLabel("--")
        self.lbl_value.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.lbl_value.setStyleSheet(
            "color: #e6e9ef; font-size: 9.5pt; font-family: Consolas, 'Courier New', monospace;"
        )
        header.addWidget(self.lbl_title)
        header.addWidget(self.lbl_value, stretch=1)
        root.addLayout(header)

        self.plot = pg.PlotWidget()
        self.plot.setBackground("#171a1f")
        self.plot.showGrid(x=True, y=True, alpha=0.2)
        self.plot.setMenuEnabled(False)
        self.plot.hideButtons()
        self.plot.setMouseEnabled(x=False, y=False)
        self.plot.setClipToView(True)
        self.plot.setDownsampling(auto=True, mode="peak")

        axis_pen = pg.mkPen(C_MUTE, width=1)
        for axis_name in ("bottom", "left"):
            axis = self.plot.getAxis(axis_name)
            axis.setPen(axis_pen)
            axis.setTextPen(pg.mkPen(C_MUTE))
            axis.setStyle(tickFont=QFont("Segoe UI", 8), tickTextOffset=4)

        self.plot.getAxis("bottom").setLabel("time (s)", color=C_MUTE)
        self.plot.getPlotItem().layout.setContentsMargins(6, 4, 10, 4)
        self.plot.getPlotItem().setContentsMargins(0, 0, 0, 0)

        root.addWidget(self.plot, stretch=1)

    def set_value_text(self, text: str, color: str = "#e6e9ef") -> None:
        self.lbl_value.setText(text)
        self.lbl_value.setStyleSheet(
            f"color: {color}; font-size: 10pt; font-family: Consolas, 'Courier New', monospace;"
        )


class RealTimeTelemetryPlots(QWidget):
    def __init__(self, history_len: int = 200, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.history_len = history_len
        self._t0: float | None = None
        self._last_draw = 0.0
        self._draw_interval = 1.0 / 20.0  # Cap chart redraws at ~20 Hz

        self.t_buf: collections.deque[float] = collections.deque(maxlen=history_len)
        self.err_x_buf: collections.deque[float] = collections.deque(maxlen=history_len)
        self.err_y_buf: collections.deque[float] = collections.deque(maxlen=history_len)
        self.dist_buf: collections.deque[float] = collections.deque(maxlen=history_len)
        self.conf_buf: collections.deque[float] = collections.deque(maxlen=history_len)
        self.yaw_cmd_buf: collections.deque[float] = collections.deque(maxlen=history_len)
        self.pitch_cmd_buf: collections.deque[float] = collections.deque(maxlen=history_len)
        self.speed_buf: collections.deque[float] = collections.deque(maxlen=history_len)

        self._init_ui()

    def _init_ui(self) -> None:
        layout = QGridLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(10)

        # --- Tracking error ---
        self.panel_err = ChartPanel("TRACKING ERROR (px)")
        self.panel_err.plot.getAxis("left").setLabel("error", color=C_MUTE)
        self.panel_err.plot.addLegend(offset=(8, 8), labelTextSize="8pt")
        self.panel_err.plot.addItem(
            pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen(C_MUTE, width=1, style=Qt.PenStyle.DashLine))
        )
        self.curve_err_x = self.panel_err.plot.plot(
            pen=_make_pen(C_PRIMARY, 2.0),
            name="Err X",
            fillLevel=0,
            brush=pg.mkBrush(79, 124, 172, 28),
        )
        self.curve_err_y = self.panel_err.plot.plot(pen=_make_pen(C_SECONDARY, 2.0), name="Err Y")

        # --- Distance ---
        self.panel_dist = ChartPanel("TARGET DISTANCE (m)")
        self.panel_dist.plot.getAxis("left").setLabel("meters", color=C_MUTE)
        self.panel_dist.plot.addLegend(offset=(8, 8), labelTextSize="8pt")
        self.curve_dist = self.panel_dist.plot.plot(
            pen=_make_pen(C_OK, 2.0),
            name="Distance",
            fillLevel=0,
            brush=pg.mkBrush(61, 143, 106, 30),
        )

        # --- Confidence ---
        self.panel_conf = ChartPanel("LOCK CONFIDENCE")
        self.panel_conf.plot.getAxis("left").setLabel("%", color=C_MUTE)
        self.panel_conf.plot.setYRange(0, 100, padding=0.02)
        self.panel_conf.plot.addLegend(offset=(8, 8), labelTextSize="8pt")
        self.panel_conf.plot.addItem(
            pg.InfiniteLine(pos=50, angle=0, pen=pg.mkPen(C_WARN, width=1, style=Qt.PenStyle.DotLine))
        )
        self.curve_conf = self.panel_conf.plot.plot(
            pen=_make_pen(C_WARN, 2.0),
            name="Confidence",
            fillLevel=0,
            brush=pg.mkBrush(160, 138, 92, 28),
        )

        # --- RC commands ---
        self.panel_cmd = ChartPanel("RC COMMANDS (µs)")
        self.panel_cmd.plot.getAxis("left").setLabel("PWM", color=C_MUTE)
        self.panel_cmd.plot.addLegend(offset=(8, 8), labelTextSize="8pt")
        self.panel_cmd.plot.addItem(
            pg.InfiniteLine(pos=1500, angle=0, pen=pg.mkPen(C_MUTE, width=1, style=Qt.PenStyle.DashLine))
        )
        self.curve_yaw = self.panel_cmd.plot.plot(pen=_make_pen("#9aa3b2", 2.0), name="Yaw")
        self.curve_pitch = self.panel_cmd.plot.plot(pen=_make_pen(C_PRIMARY, 2.0), name="Pitch")

        layout.addWidget(self.panel_err, 0, 0)
        layout.addWidget(self.panel_dist, 0, 1)
        layout.addWidget(self.panel_conf, 1, 0)
        layout.addWidget(self.panel_cmd, 1, 1)
        layout.setRowStretch(0, 1)
        layout.setRowStretch(1, 1)
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 1)

    def clear(self) -> None:
        self._t0 = None
        for buf in (
            self.t_buf,
            self.err_x_buf,
            self.err_y_buf,
            self.dist_buf,
            self.conf_buf,
            self.yaw_cmd_buf,
            self.pitch_cmd_buf,
            self.speed_buf,
        ):
            buf.clear()
        for curve in (
            self.curve_err_x,
            self.curve_err_y,
            self.curve_dist,
            self.curve_conf,
            self.curve_yaw,
            self.curve_pitch,
        ):
            curve.clear()

    def update_telemetry(self, rec: TelemetryRecord) -> None:
        now = time.time()
        if self._t0 is None:
            self._t0 = now

        # Prefer wall-clock relative seconds for a smooth X axis
        t_rel = now - self._t0
        speed = float((rec.velocity_x ** 2 + rec.velocity_y ** 2) ** 0.5)

        self.t_buf.append(t_rel)
        self.err_x_buf.append(float(rec.error_x))
        self.err_y_buf.append(float(rec.error_y))
        self.dist_buf.append(float(rec.estimated_distance_m) if rec.locked else 0.0)
        self.conf_buf.append(float(rec.confidence) * 100.0)
        self.yaw_cmd_buf.append(float(rec.rc_yaw))
        self.pitch_cmd_buf.append(float(rec.rc_pitch))
        self.speed_buf.append(speed)

        # Live value readouts (update every frame)
        self.panel_err.set_value_text(
            f"X {rec.error_x:+.0f}   Y {rec.error_y:+.0f}",
            C_PRIMARY if abs(rec.error_x) < 40 else C_SECONDARY,
        )
        self.panel_dist.set_value_text(
            f"{rec.estimated_distance_m:.1f} m" if rec.locked else "-- m",
            C_OK,
        )
        conf_pct = rec.confidence * 100.0
        self.panel_conf.set_value_text(
            f"{conf_pct:.0f}%" if rec.locked else "0%",
            C_OK if conf_pct >= 60 else (C_WARN if conf_pct >= 35 else C_SECONDARY),
        )
        self.panel_cmd.set_value_text(
            f"Y {rec.rc_yaw}  P {rec.rc_pitch}",
            C_PRIMARY,
        )

        # Throttle curve redraws for smoother UI
        if now - self._last_draw < self._draw_interval:
            return
        self._last_draw = now

        if len(self.t_buf) < 2:
            return

        t = np.asarray(self.t_buf, dtype=np.float64)

        self.curve_err_x.setData(t, np.asarray(self.err_x_buf, dtype=np.float64))
        self.curve_err_y.setData(t, np.asarray(self.err_y_buf, dtype=np.float64))
        self.curve_dist.setData(t, np.asarray(self.dist_buf, dtype=np.float64))
        self.curve_conf.setData(t, np.asarray(self.conf_buf, dtype=np.float64))
        self.curve_yaw.setData(t, np.asarray(self.yaw_cmd_buf, dtype=np.float64))
        self.curve_pitch.setData(t, np.asarray(self.pitch_cmd_buf, dtype=np.float64))

        # Keep a readable rolling window
        x_min = float(t[0])
        x_max = float(t[-1])
        if x_max - x_min < 1.0:
            x_max = x_min + 1.0
        for panel in (self.panel_err, self.panel_dist, self.panel_conf, self.panel_cmd):
            panel.plot.setXRange(x_min, x_max, padding=0.02)

        # Auto Y ranges with sensible floors
        err_vals = list(self.err_x_buf) + list(self.err_y_buf)
        err_span = max(40.0, max(abs(v) for v in err_vals) * 1.25)
        self.panel_err.plot.setYRange(-err_span, err_span, padding=0.05)

        dist_max = max(5.0, max(self.dist_buf) * 1.2)
        self.panel_dist.plot.setYRange(0, dist_max, padding=0.05)

        yaw_vals = list(self.yaw_cmd_buf) + list(self.pitch_cmd_buf)
        lo = min(1200.0, min(yaw_vals) - 40)
        hi = max(1800.0, max(yaw_vals) + 40)
        self.panel_cmd.plot.setYRange(lo, hi, padding=0.05)
