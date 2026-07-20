"""Real-Time Rolling Telemetry Graphs using pyqtgraph."""

from __future__ import annotations

import collections
import pyqtgraph as pg
from PyQt6.QtWidgets import QVBoxLayout, QWidget

from telemetry.telemetry_logger import TelemetryRecord

# Configure pyqtgraph dark theme
pg.setConfigOption("background", "#0f172a")
pg.setConfigOption("foreground", "#94a3b8")


class RealTimeTelemetryPlots(QWidget):
    def __init__(self, history_len: int = 150, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.history_len = history_len

        # Rolling buffers
        self.t_buf = collections.deque(maxlen=history_len)
        self.err_x_buf = collections.deque(maxlen=history_len)
        self.err_y_buf = collections.deque(maxlen=history_len)
        self.dist_buf = collections.deque(maxlen=history_len)
        self.conf_buf = collections.deque(maxlen=history_len)
        self.yaw_cmd_buf = collections.deque(maxlen=history_len)
        self.pitch_cmd_buf = collections.deque(maxlen=history_len)

        self._init_ui()

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # Plot 1: Tracking Error X & Y
        self.p_err = pg.PlotWidget(title="Image Tracking Error (px)")
        self.p_err.showGrid(x=True, y=True, alpha=0.3)
        self.curve_err_x = self.p_err.plot(pen=pg.mkPen("#38bdf8", width=2), name="Error X")
        self.curve_err_y = self.p_err.plot(pen=pg.mkPen("#f43f5e", width=2), name="Error Y")

        # Plot 2: Target Distance (m)
        self.p_dist = pg.PlotWidget(title="Estimated Target Distance (m)")
        self.p_dist.showGrid(x=True, y=True, alpha=0.3)
        self.curve_dist = self.p_dist.plot(pen=pg.mkPen("#10b981", width=2), name="Distance (m)")

        # Plot 3: Drone RC Commands (µs)
        self.p_cmd = pg.PlotWidget(title="MSP RC Output Commands (µs)")
        self.p_cmd.showGrid(x=True, y=True, alpha=0.3)
        self.curve_yaw = self.p_cmd.plot(pen=pg.mkPen("#a855f7", width=2), name="Yaw Cmd")
        self.curve_pitch = self.p_cmd.plot(pen=pg.mkPen("#eab308", width=2), name="Pitch Cmd")

        layout.addWidget(self.p_err)
        layout.addWidget(self.p_dist)
        layout.addWidget(self.p_cmd)

    def update_telemetry(self, rec: TelemetryRecord) -> None:
        self.t_buf.append(rec.timestamp)
        self.err_x_buf.append(rec.error_x)
        self.err_y_buf.append(rec.error_y)
        self.dist_buf.append(rec.estimated_distance_m)
        self.conf_buf.append(rec.confidence)
        self.yaw_cmd_buf.append(rec.rc_yaw)
        self.pitch_cmd_buf.append(rec.rc_pitch)

        t_list = list(self.t_buf)

        self.curve_err_x.setData(t_list, list(self.err_x_buf))
        self.curve_err_y.setData(t_list, list(self.err_y_buf))
        self.curve_dist.setData(t_list, list(self.dist_buf))
        self.curve_yaw.setData(t_list, list(self.yaw_cmd_buf))
        self.curve_pitch.setData(t_list, list(self.pitch_cmd_buf))
