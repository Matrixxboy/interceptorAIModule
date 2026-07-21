"""Arjuna mission dashboard with live metric widgets."""

from __future__ import annotations

from PyQt6.QtCore import pyqtSlot
from PyQt6.QtWidgets import QGridLayout, QHBoxLayout, QLabel, QVBoxLayout, QWidget

from gui.style import PALETTE
from gui.widgets.metric_card import MetricCard
from gui.widgets.page_header import PageHeader, Panel, StatusPill
from telemetry.telemetry_logger import TelemetryRecord


class DashboardPage(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(14)

        header = PageHeader("Mission Dashboard", "Live operational overview of tracking, link, and flight command state")
        self.pill_ops = StatusPill("OPS IDLE", "neutral")
        header.right_layout.addWidget(self.pill_ops)
        layout.addWidget(header)

        grid = QGridLayout()
        grid.setSpacing(12)
        grid.setContentsMargins(0, 0, 0, 0)

        a = PALETTE["accent"]
        ok = PALETTE["ok"]
        warn = PALETTE["warn"]
        err = PALETTE["error"]
        mute = PALETTE["text_mute"]

        self.card_fps = MetricCard("FPS", "--", "Pipeline throughput", ok)
        self.card_conf = MetricCard("Confidence", "--", "Active target lock", a)
        self.card_dist = MetricCard("Distance", "-- m", "Estimated range", a)
        self.card_target = MetricCard("Active Target", "NONE", "Target ID", mute)
        self.card_serial = MetricCard("Serial Link", "OFFLINE", "MSP connection", err)
        self.card_rc = MetricCard("RC Output", "R1500 P1500 Y1500", "Flight commands", a)
        self.card_velocity = MetricCard("Object Velocity", "-- px/s", "Image-plane speed", a)
        self.card_failsafe = MetricCard("Failsafe", "OK", "Safety status", ok)

        cards = [
            self.card_fps,
            self.card_conf,
            self.card_dist,
            self.card_target,
            self.card_serial,
            self.card_rc,
            self.card_velocity,
            self.card_failsafe,
        ]
        for i, card in enumerate(cards):
            grid.addWidget(card, i // 4, i % 4)
        layout.addLayout(grid)

        # Bottom operational strip
        bottom = QHBoxLayout()
        bottom.setSpacing(12)

        panel_status = Panel("System Status")
        self.lbl_status = QLabel("Awaiting pipeline data…")
        self.lbl_status.setStyleSheet(
            "color: #94a3b8; font-family: Consolas, 'Cascadia Mono', monospace; "
            "font-size: 9.5pt; background: transparent; padding: 4px 0;"
        )
        self.lbl_status.setWordWrap(True)
        panel_status.add_widget(self.lbl_status)
        bottom.addWidget(panel_status, stretch=2)

        panel_hint = Panel("Quick Actions")
        hint = QLabel(
            "1. Open Live Camera Feed\n"
            "2. Connect MSP serial link\n"
            "3. Drag ROI or YOLO Auto-Lock\n"
            "4. Enable Follow · monitor Telemetry"
        )
        hint.setStyleSheet(
            "color: #64748b; font-size: 9pt; background: transparent; line-height: 1.4;"
        )
        panel_hint.add_widget(hint)
        bottom.addWidget(panel_hint, stretch=1)

        layout.addLayout(bottom)
        layout.addStretch()

    def set_serial_connected(self, connected: bool, port: str = "") -> None:
        if connected:
            self.card_serial.set_value("ONLINE", port or "MSP linked")
            self.card_serial.set_accent(PALETTE["ok"])
        else:
            self.card_serial.set_value("OFFLINE", "No MSP link")
            self.card_serial.set_accent(PALETTE["error"])

    def set_active_target(self, target_id: str | None) -> None:
        if target_id:
            self.card_target.set_value(target_id, "Locked & tracking")
            self.card_target.set_accent(PALETTE["ok"])
            self.pill_ops.set_status("OPS ACTIVE", "ok")
        else:
            self.card_target.set_value("NONE", "No active lock")
            self.card_target.set_accent(PALETTE["text_mute"])
            self.pill_ops.set_status("OPS IDLE", "neutral")

    @pyqtSlot(float)
    def update_fps(self, fps: float) -> None:
        self.card_fps.set_value(f"{fps:.1f}", "Real-time pipeline")
        self.card_fps.set_accent(PALETTE["ok"] if fps >= 20 else PALETTE["warn"])

    @pyqtSlot(object)
    def update_telemetry(self, rec: TelemetryRecord) -> None:
        self.card_conf.set_value(
            f"{rec.confidence * 100:.0f}%",
            rec.source.upper() if rec.locked else "No lock",
        )
        if not rec.locked:
            conf_color = PALETTE["text_mute"]
        elif rec.confidence >= 0.6:
            conf_color = PALETTE["ok"]
        elif rec.confidence >= 0.35:
            conf_color = PALETTE["warn"]
        else:
            conf_color = PALETTE["error"]
        self.card_conf.set_accent(conf_color)

        self.card_dist.set_value(
            f"{rec.estimated_distance_m:.1f} m" if rec.locked else "-- m",
            "Monocular estimate",
        )
        self.card_rc.set_value(f"R{rec.rc_roll}  P{rec.rc_pitch}  Y{rec.rc_yaw}")
        speed = (rec.velocity_x ** 2 + rec.velocity_y ** 2) ** 0.5
        self.card_velocity.set_value(f"{speed:.0f} px/s" if rec.locked else "-- px/s")

        safe = rec.failsafe_status in ("OK", "SAFE", "NOMINAL")
        self.card_failsafe.set_value(rec.failsafe_status, "Safety monitor")
        self.card_failsafe.set_accent(PALETTE["ok"] if safe else PALETTE["error"])

        lock_str = "LOCKED" if rec.locked else "SEARCHING"
        self.lbl_status.setText(
            f"[{lock_str}]  Frame {rec.frame_idx}   ·   "
            f"Err X:{rec.error_x:+.0f}  Y:{rec.error_y:+.0f}   ·   "
            f"Thr:{rec.rc_throttle}   ·   Src:{rec.source}   ·   FS:{rec.failsafe_status}"
        )
