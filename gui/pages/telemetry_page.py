"""Flight telemetry dashboard page."""

from __future__ import annotations

from PyQt6.QtCore import pyqtSlot
from PyQt6.QtWidgets import QGridLayout, QVBoxLayout, QWidget

from gui.style import PALETTE
from gui.telemetry_widget import RealTimeTelemetryPlots
from gui.widgets.metric_card import MetricCard
from gui.widgets.page_header import PageHeader, StatusPill
from telemetry.telemetry_logger import TelemetryRecord


class TelemetryPage(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)

        header = PageHeader(
            "Flight Telemetry",
            "Real-time tracking metrics, RC output, and link health",
        )
        self.pill_telem = StatusPill("STREAMING", "info")
        header.right_layout.addWidget(self.pill_telem)
        layout.addWidget(header)

        grid = QGridLayout()
        grid.setSpacing(10)
        a = PALETTE["accent"]
        self.card_gps = MetricCard("GPS", "N/A", "Awaiting MAVLink", a)
        self.card_alt = MetricCard("Altitude", "-- m", "Barometer/GPS", a)
        self.card_heading = MetricCard("Heading", "--°", "Compass", a)
        self.card_speed = MetricCard("Speed", "-- m/s", "Ground speed", a)
        self.card_battery = MetricCard("Battery", "-- V", "Cell voltage", PALETTE["warn"])
        self.card_signal = MetricCard("Signal", "-- dBm", "RC link", a)
        self.card_track_conf = MetricCard("Track Conf", "--", "AI confidence", a)
        self.card_obj_vel = MetricCard("Obj Velocity", "--", "Target motion", a)

        cards = [
            self.card_gps,
            self.card_alt,
            self.card_heading,
            self.card_speed,
            self.card_battery,
            self.card_signal,
            self.card_track_conf,
            self.card_obj_vel,
        ]
        for i, card in enumerate(cards):
            grid.addWidget(card, i // 4, i % 4)
        layout.addLayout(grid)

        self.telemetry_plots = RealTimeTelemetryPlots()
        layout.addWidget(self.telemetry_plots, stretch=1)

    @pyqtSlot(object)
    def update_telemetry(self, rec: TelemetryRecord) -> None:
        self.telemetry_plots.update_telemetry(rec)
        self.card_track_conf.set_value(f"{rec.confidence * 100:.0f}%")
        speed = (rec.velocity_x ** 2 + rec.velocity_y ** 2) ** 0.5
        self.card_obj_vel.set_value(f"{speed:.0f} px/s" if rec.locked else "--")

    def set_serial_connected(self, connected: bool) -> None:
        self.card_signal.set_value("LINK OK" if connected else "NO LINK")
