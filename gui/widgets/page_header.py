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
        layout.setContentsMargins(0, 0, 0, 6)
        layout.setSpacing(2)

        row = QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(8)

        title_col = QVBoxLayout()
        title_col.setSpacing(2)

        lbl = QLabel(title.upper())
        lbl.setObjectName("pageTitle")
        title_col.addWidget(lbl)

        if subtitle:
            sub = QLabel(subtitle)
            sub.setObjectName("pageSubtitle")
            sub.setWordWrap(True)
            title_col.addWidget(sub)

        row.addLayout(title_col, stretch=1)
        self.right_layout = QHBoxLayout()
        self.right_layout.setSpacing(6)
        row.addLayout(self.right_layout)
        layout.addLayout(row)

        rule = QFrame()
        rule.setObjectName("headerRule")
        rule.setFixedHeight(1)
        layout.addWidget(rule)


class StatusPill(QLabel):
    """Compact status chip."""

    def __init__(self, text: str = "IDLE", tone: str = "neutral", parent=None) -> None:
        super().__init__(text, parent)
        self.setObjectName("statusPill")
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumWidth(64)
        self.setMinimumHeight(24)
        self.setSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)
        self.set_tone(tone)

    def set_tone(self, tone: str) -> None:
        colors = {
            "ok": ("#14241c", PALETTE["ok"], "#b8e0cc"),
            "warn": ("#242014", PALETTE["warn"], "#e8d4a0"),
            "error": ("#241616", PALETTE["error"], "#f0c0c0"),
            "info": ("#141c24", PALETTE["info"], "#c0d8f0"),
            "neutral": ("#171a1f", PALETTE["border"], PALETTE["text_dim"]),
        }
        bg, border, fg = colors.get(tone, colors["neutral"])
        self.setStyleSheet(
            f"""
            QLabel#statusPill {{
                background-color: {bg};
                color: {fg};
                border: 1px solid {border};
                border-radius: 4px;
                padding: 3px 8px;
                font-size: 7.5pt;
                font-weight: 650;
                letter-spacing: 0.6px;
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
