"""Placeholder page for modules under development."""

from __future__ import annotations

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QLabel, QVBoxLayout, QWidget

from gui.widgets.page_header import PageHeader, Panel, StatusPill


class PlaceholderPage(QWidget):
    def __init__(self, title: str, description: str, parent=None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(14)

        header = PageHeader(title, description)
        pill = StatusPill("MODULE READY", "info")
        header.right_layout.addWidget(pill)
        layout.addWidget(header)

        panel = Panel("Integration Status")
        body = QLabel(
            f"<p style='color:#94a3b8; margin:0;'>{description}</p>"
            "<p style='color:#475569; margin-top:12px;'>"
            "This subsystem is registered in the Arjuna navigation shell and ready for expansion. "
            "Core tracking, telemetry, and target-database workflows are available from the "
            "primary operational pages."
            "</p>"
        )
        body.setWordWrap(True)
        body.setTextFormat(Qt.TextFormat.RichText)
        body.setStyleSheet("background: transparent;")
        panel.add_widget(body)
        layout.addWidget(panel)
        layout.addStretch()
