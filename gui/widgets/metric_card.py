"""Reusable dashboard metric card — instrumentation module."""

from __future__ import annotations

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor, QPainter, QPen
from PyQt6.QtWidgets import QFrame, QHBoxLayout, QLabel, QVBoxLayout

from gui.style import PALETTE

_GLYPHS = {
    "fps": "▦",
    "conf": "⊕",
    "dist": "◎",
    "target": "⌖",
    "serial": "⇄",
    "rc": "▷",
    "vel": "→",
    "fail": "⬡",
    "generic": "▣",
}


class MetricCard(QFrame):
    def __init__(
        self,
        title: str,
        value: str = "--",
        subtitle: str = "",
        accent: str | None = None,
        icon: str = "generic",
        parent=None,
    ) -> None:
        super().__init__(parent)
        self._accent = accent or PALETTE["accent"]
        self.setObjectName("metricCard")
        self.setMinimumHeight(108)
        self._apply_style(self._accent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(6)

        top = QHBoxLayout()
        top.setContentsMargins(0, 0, 0, 0)
        top.setSpacing(8)

        self.lbl_icon = QLabel(_GLYPHS.get(icon, _GLYPHS["generic"]))
        self.lbl_icon.setFixedWidth(16)
        self.lbl_icon.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_icon.setStyleSheet(
            f"color: {self._accent}; font-size: 10pt; background: transparent;"
        )

        self.lbl_title = QLabel(title.upper())
        self.lbl_title.setStyleSheet(
            f"color: {PALETTE['text_mute']}; font-size: 7.5pt; font-weight: 650; "
            "letter-spacing: 1.3px; background: transparent;"
        )

        self.dot = _StatusMark(self._accent)
        self.dot.setFixedSize(8, 8)

        top.addWidget(self.lbl_icon)
        top.addWidget(self.lbl_title)
        top.addStretch()
        top.addWidget(self.dot, alignment=Qt.AlignmentFlag.AlignVCenter)

        self.lbl_value = QLabel(value)
        self.lbl_value.setStyleSheet(
            f"color: {PALETTE['text']}; font-size: 18pt; font-weight: 650; "
            "font-family: Consolas, 'Cascadia Mono', monospace; background: transparent;"
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
                background-color: {PALETTE['bg_panel']};
                border: 1px solid {PALETTE['border']};
                border-left: 2px solid {accent};
                border-radius: 9px;
            }}
            QFrame#metricCard:hover {{
                background-color: {PALETTE['bg_elevated']};
                border: 1px solid {PALETTE['border_soft']};
                border-left: 2px solid {accent};
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
        self.lbl_icon.setStyleSheet(
            f"color: {accent}; font-size: 10pt; background: transparent;"
        )
        self.dot.set_color(accent)


class _StatusMark(QFrame):
    """Hard rectangular status lamp — no glow."""

    def __init__(self, color: str, parent=None) -> None:
        super().__init__(parent)
        self._color = QColor(color)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)

    def set_color(self, color: str) -> None:
        self._color = QColor(color)
        self.update()

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, False)
        painter.setPen(QPen(self._color, 1))
        painter.setBrush(self._color)
        painter.drawRect(1, 1, self.width() - 3, self.height() - 3)
