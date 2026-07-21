"""Shared UI building blocks for Arjuna GCS."""

from __future__ import annotations

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QFrame, QHBoxLayout, QLabel, QSizePolicy, QVBoxLayout, QWidget

from gui.style import PALETTE


class PageHeader(QWidget):
    """Consistent page title bar used across Arjuna screens."""

    def __init__(
        self,
        title: str,
        subtitle: str = "",
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 8)
        layout.setSpacing(2)

        row = QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)

        title_col = QVBoxLayout()
        title_col.setSpacing(2)

        lbl = QLabel(title.upper())
        lbl.setObjectName("pageTitle")
        title_col.addWidget(lbl)

        if subtitle:
            sub = QLabel(subtitle)
            sub.setObjectName("pageSubtitle")
            title_col.addWidget(sub)

        row.addLayout(title_col, stretch=1)
        self.right_layout = QHBoxLayout()
        self.right_layout.setSpacing(8)
        row.addLayout(self.right_layout)
        layout.addLayout(row)

        rule = QFrame()
        rule.setObjectName("headerRule")
        rule.setFixedHeight(1)
        layout.addWidget(rule)


class StatusPill(QLabel):
    """Muted status chip — industrial, not neon."""

    def __init__(self, text: str = "IDLE", tone: str = "neutral", parent=None) -> None:
        super().__init__(text, parent)
        self.setObjectName("statusPill")
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumWidth(72)
        self.setMinimumHeight(22)
        self.setSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)
        self.set_tone(tone)

    def set_tone(self, tone: str) -> None:
        # bg, border, fg — desaturated operational colors
        colors = {
            "ok": ("#1a2420", PALETTE["ok"], "#a8c0b0"),
            "warn": ("#242018", PALETTE["warn"], "#c8b890"),
            "error": ("#241a1a", PALETTE["error"], "#c4a0a0"),
            "info": ("#1a2024", PALETTE["info"], "#b0c0cc"),
            "neutral": ("#1a1e24", PALETTE["border"], PALETTE["text_dim"]),
        }
        bg, border, fg = colors.get(tone, colors["neutral"])
        self.setStyleSheet(
            f"""
            QLabel#statusPill {{
                background-color: {bg};
                color: {fg};
                border: 1px solid {border};
                border-radius: 2px;
                padding: 3px 10px;
                font-size: 8pt;
                font-weight: 650;
                letter-spacing: 0.8px;
            }}
            """
        )

    def set_status(self, text: str, tone: str = "neutral") -> None:
        self.setText(text.upper())
        self.set_tone(tone)
        self.adjustSize()


class Panel(QFrame):
    """Framed content panel."""

    def __init__(self, title: str = "", parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("panel")
        self._root = QVBoxLayout(self)
        self._root.setContentsMargins(12, 10, 12, 12)
        self._root.setSpacing(8)

        if title:
            hdr = QLabel(title.upper())
            hdr.setObjectName("panelTitle")
            self._root.addWidget(hdr)

        self.body = QVBoxLayout()
        self.body.setSpacing(8)
        self._root.addLayout(self.body)

    def add_widget(self, widget: QWidget) -> None:
        self.body.addWidget(widget)

    def add_layout(self, layout) -> None:
        self.body.addLayout(layout)
