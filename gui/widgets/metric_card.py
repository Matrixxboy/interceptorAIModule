"""Reusable dashboard metric card — muted industrial style."""

from __future__ import annotations

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QFrame, QHBoxLayout, QLabel, QVBoxLayout

from gui.style import PALETTE


class MetricCard(QFrame):
    def __init__(
        self,
        title: str,
        value: str = "--",
        subtitle: str = "",
        accent: str | None = None,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self._accent = accent or PALETTE["accent"]
        self.setObjectName("metricCard")
        self.setMinimumHeight(96)
        self._apply_style(self._accent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(6)

        top = QHBoxLayout()
        top.setContentsMargins(0, 0, 0, 0)

        self.lbl_title = QLabel(title.upper())
        self.lbl_title.setStyleSheet(
            f"color: {PALETTE['text_mute']}; font-size: 8pt; font-weight: 650; "
            "letter-spacing: 1px; background: transparent;"
        )

        self.dot = QLabel("●")
        self.dot.setStyleSheet(
            f"color: {self._accent}; font-size: 7pt; background: transparent;"
        )
        self.dot.setFixedWidth(12)

        top.addWidget(self.lbl_title)
        top.addStretch()
        top.addWidget(self.dot)

        self.lbl_value = QLabel(value)
        self.lbl_value.setStyleSheet(
            f"color: {PALETTE['text']}; font-size: 18pt; font-weight: 650; background: transparent;"
        )
        self.lbl_value.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)

        self.lbl_sub = QLabel(subtitle)
        self.lbl_sub.setStyleSheet(
            f"color: {PALETTE['text_mute']}; font-size: 8pt; background: transparent;"
        )

        layout.addLayout(top)
        layout.addWidget(self.lbl_value)
        layout.addWidget(self.lbl_sub)
        layout.addStretch()

    def _apply_style(self, accent: str) -> None:
        self.setStyleSheet(
            f"""
            QFrame#metricCard {{
                background-color: #171a1f;
                border: 1px solid #2a3038;
                border-top: 2px solid {accent};
                border-radius: 6px;
            }}
            QFrame#metricCard:hover {{
                border: 1px solid #343b45;
                border-top: 2px solid {accent};
                background-color: #1e2329;
            }}
            """
        )

    def set_value(self, value: str, subtitle: str = "") -> None:
        self.lbl_value.setText(value)
        if subtitle:
            self.lbl_sub.setText(subtitle)

    def set_accent(self, accent: str) -> None:
        self._accent = accent
        self._apply_style(accent)
        self.dot.setStyleSheet(
            f"color: {accent}; font-size: 7pt; background: transparent;"
        )
