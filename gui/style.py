"""Arjuna GCS design system — matte military command console (no glow)."""

from pathlib import Path

_ASSETS = Path(__file__).resolve().parent / "assets"
_GRAIN = (_ASSETS / "grain.png").as_posix()

PALETTE = {
    "bg": "#050607",
    "bg_secondary": "#0A0C0F",
    "bg_panel": "#101318",
    "bg_elevated": "#151920",
    "bg_input": "#0B0E12",
    "bg_sidebar": "#0A0C0F",
    "bg_hover": "#12161C",
    "border": "#242932",
    "border_soft": "#2C313A",
    "text": "#E8EAEF",
    "text_dim": "#A8B0BA",
    "text_mute": "#6E7682",
    "accent": "#9D3FF5",
    "accent_soft": "#6B2BA8",
    "accent_surface": "#16101C",
    "ok": "#5FAF83",
    "warn": "#B89A5A",
    "error": "#B84A52",
    "info": "#6287C7",
    "grain": _GRAIN,
}


def _theme_qss() -> str:
    return """
/* ===== Base ===== */
QMainWindow, QDialog {
    background-color: %(bg)s;
    background-image: url("%(grain)s");
    color: %(text)s;
    font-family: 'Segoe UI', 'Segoe UI Variable Text', Bahnschrift, system-ui, sans-serif;
}

QWidget {
    background-color: transparent;
    color: %(text_dim)s;
    font-size: 9.5pt;
}

QToolTip {
    background-color: %(bg_elevated)s;
    color: %(text)s;
    border: 1px solid %(border)s;
    padding: 6px 8px;
    border-radius: 6px;
}

/* ===== Sidebar ===== */
QFrame#sidebar {
    background-color: %(bg_sidebar)s;
    border-right: 1px solid %(border)s;
}

QFrame#brandBlock {
    background-color: %(bg_sidebar)s;
    border-bottom: 1px solid %(border)s;
}

QLabel#brandMark {
    color: %(accent)s;
    font-size: 7pt;
    font-weight: 600;
    letter-spacing: 2.2px;
    background: transparent;
}

QLabel#brandLabel {
    font-size: 17pt;
    font-weight: 700;
    color: %(text)s;
    letter-spacing: 4px;
    padding: 2px 0 0 0;
    background: transparent;
}

QLabel#brandSubtitle {
    font-size: 7pt;
    color: %(text_mute)s;
    letter-spacing: 1.6px;
    padding-bottom: 2px;
    background: transparent;
}

QLabel#navStatus {
    font-size: 7.5pt;
    color: %(ok)s;
    letter-spacing: 1.1px;
    padding: 10px 12px;
    border-top: 1px solid %(border)s;
    background: transparent;
}

QListWidget#navList {
    background-color: transparent;
    border: none;
    outline: none;
    font-size: 8.5pt;
    padding: 8px 0;
}

QListWidget#navList::item {
    padding: 8px 10px 8px 12px;
    margin: 2px 8px;
    border-radius: 6px;
    border: 1px solid transparent;
    border-left: 2px solid transparent;
    color: %(text_dim)s;
}

QListWidget#navList::item:selected {
    background-color: %(accent_surface)s;
    border: 1px solid %(border)s;
    border-left: 2px solid %(accent)s;
    color: %(text)s;
    font-weight: 600;
}

QListWidget#navList::item:hover:!selected {
    background-color: %(bg_hover)s;
    color: %(text)s;
}

/* ===== Content chrome ===== */
QFrame#contentChrome {
    background-color: %(bg)s;
}

QFrame#topStatusBar {
    background-color: %(bg_secondary)s;
    border: none;
    border-bottom: 1px solid %(border)s;
    border-radius: 0;
}

QLabel#pageTitle {
    font-size: 13pt;
    font-weight: 650;
    color: %(text)s;
    letter-spacing: 1.4px;
    background: transparent;
}

QLabel#pageSubtitle {
    font-size: 8pt;
    color: %(text_mute)s;
    background: transparent;
}

QFrame#headerRule {
    background-color: %(border)s;
    border: none;
    max-height: 1px;
}

QLabel#panelTitle {
    color: %(text_mute)s;
    font-size: 7.5pt;
    font-weight: 650;
    letter-spacing: 1.4px;
    background: transparent;
}

QFrame#panel {
    background-color: %(bg_panel)s;
    border: 1px solid %(border)s;
    border-radius: 10px;
}

/* ===== Group boxes ===== */
QGroupBox {
    background-color: %(bg_panel)s;
    border: 1px solid %(border)s;
    border-radius: 10px;
    margin-top: 12px;
    font-weight: 600;
    color: %(text_mute)s;
    padding: 12px 10px 10px 10px;
}

QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 10px;
    padding: 0 6px;
    color: %(text_dim)s;
    font-size: 8pt;
    letter-spacing: 0.8px;
    background-color: %(bg)s;
}

/* ===== Buttons ===== */
QPushButton {
    background-color: %(bg_elevated)s;
    color: %(text)s;
    font-weight: 600;
    font-size: 8.5pt;
    border: 1px solid %(border_soft)s;
    border-radius: 7px;
    padding: 5px 12px;
    min-height: 26px;
    min-width: 64px;
}

QPushButton:hover {
    background-color: %(bg_hover)s;
    border-color: %(accent)s;
    color: #ffffff;
}

QPushButton:pressed {
    background-color: %(bg_panel)s;
}

QPushButton:disabled {
    background-color: %(bg_panel)s;
    color: #4a5260;
    border-color: %(border)s;
}

QPushButton:focus {
    border-color: %(accent)s;
}

QPushButton#btnCompact {
    min-width: 40px;
    min-height: 26px;
    padding: 4px 8px;
    font-size: 8pt;
    border-radius: 6px;
}

QPushButton#btn_arm {
    background-color: #1A1214;
    border-color: #6E353A;
    color: #D8A8AC;
}
QPushButton#btn_arm:hover {
    background-color: #241618;
    border-color: %(error)s;
    color: #F0D0D2;
}
QPushButton#btn_arm:checked {
    background-color: #3A1E22;
    border-color: %(error)s;
    color: #F2D6D8;
}

QPushButton#btnPrimary {
    background-color: %(accent_surface)s;
    border-color: %(accent)s;
    color: #EDE4F8;
}
QPushButton#btnPrimary:hover {
    background-color: #1C1426;
    border-color: %(accent)s;
    color: #ffffff;
}

QPushButton#btnDanger {
    background-color: #1A1214;
    border-color: #6E353A;
    color: #D8A8AC;
}
QPushButton#btnDanger:hover {
    background-color: #241618;
    border-color: %(error)s;
    color: #F0D0D2;
}

QPushButton#btnSuccess {
    background-color: #101814;
    border-color: #3E6E54;
    color: #B6D4C4;
}
QPushButton#btnSuccess:hover {
    background-color: #16201A;
    border-color: %(ok)s;
    color: #D8EEE4;
}
QPushButton#btnSuccess:checked {
    background-color: #1A2A22;
    border-color: %(ok)s;
    color: #E4F4EC;
}

QPushButton#btnGhost {
    background-color: transparent;
    border: 1px solid %(border)s;
    color: %(text_dim)s;
}
QPushButton#btnGhost:hover {
    border-color: %(accent)s;
    color: %(text)s;
    background-color: %(accent_surface)s;
}

/* ===== Inputs ===== */
QSlider::groove:horizontal {
    height: 4px;
    background: %(border)s;
    border-radius: 2px;
}

QSlider::sub-page:horizontal {
    background: %(accent)s;
    border-radius: 2px;
}

QSlider::handle:horizontal {
    background: %(text)s;
    border: 1px solid %(accent)s;
    width: 12px;
    margin-top: -5px;
    margin-bottom: -5px;
    border-radius: 6px;
}

QDoubleSpinBox, QSpinBox, QLineEdit, QComboBox {
    background-color: %(bg_input)s;
    border: 1px solid %(border)s;
    border-radius: 7px;
    padding: 4px 8px;
    color: %(text)s;
    selection-background-color: %(accent_surface)s;
    min-height: 28px;
    font-size: 10pt;
    font-weight: 600;
}

QSpinBox, QDoubleSpinBox {
    padding-right: 4px;
    min-width: 72px;
}

QSpinBox::up-button, QDoubleSpinBox::up-button,
QSpinBox::down-button, QDoubleSpinBox::down-button {
    subcontrol-origin: border;
    width: 20px;
    border-left: 1px solid %(border)s;
    background-color: %(bg_elevated)s;
}

QSpinBox::up-button, QDoubleSpinBox::up-button {
    subcontrol-position: top right;
}

QSpinBox::down-button, QDoubleSpinBox::down-button {
    subcontrol-position: bottom right;
}

QDoubleSpinBox:focus, QSpinBox:focus, QLineEdit:focus, QComboBox:focus {
    border: 1px solid %(accent)s;
}

QComboBox {
    padding-right: 8px;
}

QComboBox::drop-down {
    subcontrol-origin: padding;
    subcontrol-position: top right;
    width: 22px;
    border-left: 1px solid %(border)s;
    background: %(bg_elevated)s;
}

QComboBox QAbstractItemView {
    background-color: %(bg_panel)s;
    border: 1px solid %(border)s;
    selection-background-color: %(accent_surface)s;
    color: %(text)s;
    outline: none;
    padding: 4px;
}

/* ===== Tabs ===== */
QTabWidget::pane {
    border: 1px solid %(border)s;
    border-radius: 0 0 10px 10px;
    background-color: %(bg_panel)s;
    top: -1px;
}

QTabBar::tab {
    background-color: %(bg)s;
    color: %(text_mute)s;
    padding: 7px 14px;
    border: 1px solid %(border)s;
    border-bottom: none;
    margin-right: 2px;
    font-size: 8.5pt;
    font-weight: 600;
    letter-spacing: 0.4px;
    border-top-left-radius: 7px;
    border-top-right-radius: 7px;
    min-width: 56px;
}

QTabBar::tab:selected {
    background-color: %(accent_surface)s;
    color: %(text)s;
    border-top: 2px solid %(accent)s;
}

QTabBar::tab:hover:!selected {
    color: %(text_dim)s;
    background-color: %(bg_secondary)s;
}

/* ===== Tables ===== */
QTableWidget {
    background-color: %(bg_input)s;
    alternate-background-color: %(bg_panel)s;
    gridline-color: %(border)s;
    border: 1px solid %(border)s;
    border-radius: 8px;
    selection-background-color: %(accent_surface)s;
    selection-color: %(text)s;
}

QTableWidget::item {
    padding: 4px 8px;
}

QHeaderView::section {
    background-color: %(bg_panel)s;
    color: %(text_mute)s;
    padding: 7px 8px;
    border: none;
    border-bottom: 1px solid %(border)s;
    border-right: 1px solid %(border)s;
    font-weight: 650;
    font-size: 8pt;
    letter-spacing: 0.4px;
}

/* ===== Status / progress ===== */
QStatusBar {
    background-color: %(bg_secondary)s;
    color: %(text_mute)s;
    border-top: 1px solid %(border)s;
    font-size: 8pt;
    font-family: Consolas, 'Cascadia Mono', monospace;
}

QStatusBar QLabel {
    color: %(text_dim)s;
    padding: 0 12px;
    background: transparent;
    border-left: 1px solid %(border)s;
    letter-spacing: 0.6px;
}

QProgressBar {
    background-color: %(bg_input)s;
    border: 1px solid %(border)s;
    border-radius: 6px;
    text-align: center;
    color: %(text_dim)s;
    max-height: 18px;
    font-size: 8pt;
}

QProgressBar::chunk {
    background-color: %(info)s;
    border-radius: 4px;
}

QTextEdit, QPlainTextEdit {
    background-color: %(bg_input)s;
    border: 1px solid %(border)s;
    border-radius: 8px;
    color: %(text_dim)s;
    selection-background-color: %(accent_surface)s;
}

QCheckBox {
    spacing: 6px;
    color: %(text_dim)s;
    font-size: 8.5pt;
}

QCheckBox::indicator {
    width: 14px;
    height: 14px;
    border: 1px solid %(border_soft)s;
    border-radius: 3px;
    background: %(bg_input)s;
}

QCheckBox::indicator:checked {
    background: %(accent_surface)s;
    border-color: %(accent)s;
}

/* ===== Splitter / scroll ===== */
QSplitter::handle {
    background-color: %(border)s;
    width: 2px;
    height: 2px;
}

QScrollBar:vertical {
    background: %(bg)s;
    width: 8px;
    margin: 0;
}

QScrollBar::handle:vertical {
    background: %(border_soft)s;
    border-radius: 4px;
    min-height: 24px;
}

QScrollBar::handle:vertical:hover {
    background: %(text_mute)s;
}

QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical,
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {
    background: none;
    height: 0;
}

QScrollBar:horizontal {
    background: %(bg)s;
    height: 8px;
}

QScrollBar::handle:horizontal {
    background: %(border_soft)s;
    border-radius: 4px;
    min-width: 24px;
}

QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {
    width: 0;
}

/* ===== Video ===== */
QLabel#videoSurface {
    background-color: #030405;
    border: 1px solid %(border)s;
    border-radius: 8px;
}

/* ===== Form labels ===== */
QLabel#formLabel {
    color: %(text_dim)s;
    font-size: 8pt;
    letter-spacing: 0.8px;
    background: transparent;
}
""" % PALETTE


ARJUNA_THEME_QSS = _theme_qss()
DARK_THEME_QSS = ARJUNA_THEME_QSS
