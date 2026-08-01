"""Arjuna GCS design system — clean slate industrial (no neon, no glass)."""

# Shared palette for Python-painted widgets
PALETTE = {
    "bg": "#0f1115",
    "bg_panel": "#171a1f",
    "bg_elevated": "#1e2329",
    "border": "#2a3038",
    "border_soft": "#22272e",
    "text": "#e6e9ef",
    "text_dim": "#9aa3b2",
    "text_mute": "#6b7380",
    "accent": "#4f7cac",
    "accent_soft": "#3d628a",
    "ok": "#3d8f6a",
    "warn": "#b08a3c",
    "error": "#b05656",
    "info": "#4f7cac",
}

ARJUNA_THEME_QSS = """
/* ===== Base ===== */
QMainWindow, QDialog {
    background-color: #0f1115;
    color: #e6e9ef;
    font-family: 'Segoe UI', 'Segoe UI Variable Text', system-ui, sans-serif;
}

QWidget {
    background-color: transparent;
    color: #c5cad3;
    font-size: 9.5pt;
}

QToolTip {
    background-color: #1e2329;
    color: #e6e9ef;
    border: 1px solid #2a3038;
    padding: 6px 8px;
    border-radius: 4px;
}

/* ===== Sidebar ===== */
QFrame#sidebar {
    background-color: #13161b;
    border-right: 1px solid #2a3038;
}

QFrame#brandBlock {
    background-color: #13161b;
    border-bottom: 1px solid #2a3038;
}

QLabel#brandMark {
    color: #4f7cac;
    font-size: 7.5pt;
    font-weight: 600;
    letter-spacing: 2px;
    background: transparent;
}

QLabel#brandLabel {
    font-size: 17pt;
    font-weight: 700;
    color: #e6e9ef;
    letter-spacing: 3px;
    padding: 2px 0 0 0;
    background: transparent;
}

QLabel#brandSubtitle {
    font-size: 7pt;
    color: #6b7380;
    letter-spacing: 1.5px;
    padding-bottom: 2px;
    background: transparent;
}

QLabel#navStatus {
    font-size: 7.5pt;
    color: #3d8f6a;
    letter-spacing: 1px;
    padding: 10px 12px;
    border-top: 1px solid #2a3038;
    background: transparent;
}

QListWidget#navList {
    background-color: transparent;
    border: none;
    outline: none;
    font-size: 9pt;
    padding: 6px 0;
}

QListWidget#navList::item {
    padding: 8px 12px 8px 14px;
    margin: 1px 6px;
    border-radius: 4px;
    border-left: 2px solid transparent;
    color: #9aa3b2;
}

QListWidget#navList::item:selected {
    background-color: #1e2329;
    border-left: 2px solid #4f7cac;
    color: #e6e9ef;
    font-weight: 600;
}

QListWidget#navList::item:hover:!selected {
    background-color: #171a1f;
    color: #d0d5de;
}

/* ===== Content chrome ===== */
QFrame#contentChrome {
    background-color: #0f1115;
}

QLabel#pageTitle {
    font-size: 13pt;
    font-weight: 650;
    color: #e6e9ef;
    letter-spacing: 1px;
    background: transparent;
}

QLabel#pageSubtitle {
    font-size: 8pt;
    color: #6b7380;
    background: transparent;
}

QFrame#headerRule {
    background-color: #2a3038;
    border: none;
    max-height: 1px;
}

QLabel#panelTitle {
    color: #6b7380;
    font-size: 7.5pt;
    font-weight: 650;
    letter-spacing: 1px;
    background: transparent;
}

QFrame#panel {
    background-color: #171a1f;
    border: 1px solid #2a3038;
    border-radius: 6px;
}

/* ===== Group boxes ===== */
QGroupBox {
    background-color: #171a1f;
    border: 1px solid #2a3038;
    border-radius: 6px;
    margin-top: 12px;
    font-weight: 600;
    color: #6b7380;
    padding: 12px 10px 10px 10px;
}

QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 10px;
    padding: 0 6px;
    color: #9aa3b2;
    font-size: 8pt;
    letter-spacing: 0.6px;
    background-color: #0f1115;
}

/* ===== Buttons ===== */
QPushButton {
    background-color: #1e2329;
    color: #e6e9ef;
    font-weight: 600;
    font-size: 8.5pt;
    border: 1px solid #343b45;
    border-radius: 4px;
    padding: 5px 12px;
    min-height: 26px;
    min-width: 64px;
}

QPushButton:hover {
    background-color: #262c34;
    border-color: #4f7cac;
    color: #ffffff;
}

QPushButton:pressed {
    background-color: #171a1f;
}

QPushButton:disabled {
    background-color: #171a1f;
    color: #4a5260;
    border-color: #2a3038;
}

QPushButton#btnCompact {
    min-width: 40px;
    min-height: 26px;
    padding: 4px 8px;
    font-size: 8pt;
}

QPushButton#btn_arm {
    background-color: #2a1f1f;
    border-color: #8a4a4a;
    color: #e0b0b0;
}
QPushButton#btn_arm:hover {
    background-color: #3a2828;
    border-color: #b05656;
    color: #ffe0e0;
}
QPushButton#btn_arm:checked {
    background-color: #b05656;
    border-color: #c07070;
    color: #ffffff;
}

QPushButton#btnPrimary {
    background-color: #2a3a4a;
    border-color: #4f7cac;
    color: #d8e6f4;
}
QPushButton#btnPrimary:hover {
    background-color: #34506a;
    border-color: #6a98c4;
    color: #ffffff;
}

QPushButton#btnDanger {
    background-color: #2a1f1f;
    border-color: #8a4a4a;
    color: #e0b0b0;
}
QPushButton#btnDanger:hover {
    background-color: #3a2828;
    border-color: #b05656;
    color: #ffe0e0;
}

QPushButton#btnSuccess {
    background-color: #1c2a22;
    border-color: #3d8f6a;
    color: #b0dcc8;
}
QPushButton#btnSuccess:hover {
    background-color: #243830;
    border-color: #50a87c;
    color: #e0ffe8;
}
QPushButton#btnSuccess:checked {
    background-color: #3d8f6a;
    border-color: #50a87c;
    color: #ffffff;
}

QPushButton#btnGhost {
    background-color: transparent;
    border: 1px solid #2a3038;
    color: #9aa3b2;
}
QPushButton#btnGhost:hover {
    border-color: #4f7cac;
    color: #e6e9ef;
    background-color: #1e2329;
}

QPushButton#btnStep {
    min-width: 28px;
    max-width: 28px;
    min-height: 28px;
    max-height: 28px;
    padding: 0;
    font-size: 12pt;
    font-weight: 700;
    background-color: #1e2329;
    border: 1px solid #2a3038;
    color: #c5cad3;
    border-radius: 4px;
}
QPushButton#btnStep:hover {
    border-color: #4f7cac;
    color: #ffffff;
    background-color: #262c34;
}
QPushButton#btnStep:pressed {
    background-color: #171a1f;
}

/* ===== Inputs — room for text + arrows ===== */
QSlider::groove:horizontal {
    height: 4px;
    background: #2a3038;
    border-radius: 2px;
}

QSlider::sub-page:horizontal {
    background: #4f7cac;
    border-radius: 2px;
}

QSlider::handle:horizontal {
    background: #e6e9ef;
    border: 1px solid #4f7cac;
    width: 12px;
    margin-top: -5px;
    margin-bottom: -5px;
    border-radius: 6px;
}

QDoubleSpinBox, QSpinBox, QLineEdit, QComboBox {
    background-color: #13161b;
    border: 1px solid #2a3038;
    border-radius: 4px;
    padding: 4px 8px;
    color: #e6e9ef;
    selection-background-color: #34506a;
    min-height: 28px;
    font-size: 10pt;
    font-weight: 600;
}

/* Leave room for buttons; do NOT restyle ::up-arrow / ::down-arrow
   (custom arrow images break digit visibility on Windows). */
QSpinBox, QDoubleSpinBox {
    padding-right: 4px;
    min-width: 72px;
}

QSpinBox::up-button, QDoubleSpinBox::up-button,
QSpinBox::down-button, QDoubleSpinBox::down-button {
    subcontrol-origin: border;
    width: 20px;
    border-left: 1px solid #2a3038;
    background-color: #1e2329;
}

QSpinBox::up-button, QDoubleSpinBox::up-button {
    subcontrol-position: top right;
}

QSpinBox::down-button, QDoubleSpinBox::down-button {
    subcontrol-position: bottom right;
}

QDoubleSpinBox:focus, QSpinBox:focus, QLineEdit:focus, QComboBox:focus {
    border: 1px solid #4f7cac;
}

QComboBox {
    padding-right: 8px;
}

QComboBox::drop-down {
    subcontrol-origin: padding;
    subcontrol-position: top right;
    width: 22px;
    border-left: 1px solid #2a3038;
    background: #1e2329;
}

QComboBox QAbstractItemView {
    background-color: #171a1f;
    border: 1px solid #2a3038;
    selection-background-color: #2a3a4a;
    color: #e6e9ef;
    outline: none;
    padding: 4px;
}

/* ===== Tabs ===== */
QTabWidget::pane {
    border: 1px solid #2a3038;
    border-radius: 0 0 6px 6px;
    background-color: #171a1f;
    top: -1px;
}

QTabBar::tab {
    background-color: #0f1115;
    color: #6b7380;
    padding: 7px 14px;
    border: 1px solid #2a3038;
    border-bottom: none;
    margin-right: 2px;
    font-size: 8.5pt;
    font-weight: 600;
    border-top-left-radius: 4px;
    border-top-right-radius: 4px;
    min-width: 56px;
}

QTabBar::tab:selected {
    background-color: #171a1f;
    color: #e6e9ef;
    border-top: 2px solid #4f7cac;
}

QTabBar::tab:hover:!selected {
    color: #9aa3b2;
    background-color: #13161b;
}

/* ===== Tables ===== */
QTableWidget {
    background-color: #13161b;
    alternate-background-color: #171a1f;
    gridline-color: #22272e;
    border: 1px solid #2a3038;
    border-radius: 6px;
    selection-background-color: #2a3a4a;
    selection-color: #e6e9ef;
}

QTableWidget::item {
    padding: 4px 8px;
}

QHeaderView::section {
    background-color: #171a1f;
    color: #6b7380;
    padding: 7px 8px;
    border: none;
    border-bottom: 1px solid #2a3038;
    border-right: 1px solid #22272e;
    font-weight: 650;
    font-size: 8pt;
    letter-spacing: 0.3px;
}

/* ===== Status / progress ===== */
QStatusBar {
    background-color: #0c0e12;
    color: #6b7380;
    border-top: 1px solid #2a3038;
    font-size: 8pt;
    font-family: Consolas, 'Cascadia Mono', monospace;
}

QStatusBar QLabel {
    color: #6b7380;
    padding: 0 10px;
    background: transparent;
    border-left: 1px solid #2a3038;
}

QProgressBar {
    background-color: #13161b;
    border: 1px solid #2a3038;
    border-radius: 4px;
    text-align: center;
    color: #c5cad3;
    max-height: 18px;
    font-size: 8pt;
}

QProgressBar::chunk {
    background-color: #4f7cac;
    border-radius: 3px;
}

QTextEdit, QPlainTextEdit {
    background-color: #0f1115;
    border: 1px solid #2a3038;
    border-radius: 4px;
    color: #c5cad3;
    selection-background-color: #34506a;
}

QCheckBox {
    spacing: 6px;
    color: #c5cad3;
    font-size: 8.5pt;
}

QCheckBox::indicator {
    width: 14px;
    height: 14px;
    border: 1px solid #343b45;
    border-radius: 3px;
    background: #13161b;
}

QCheckBox::indicator:checked {
    background: #4f7cac;
    border-color: #4f7cac;
}

/* ===== Splitter / scroll ===== */
QSplitter::handle {
    background-color: #2a3038;
    width: 2px;
    height: 2px;
}

QScrollBar:vertical {
    background: #0f1115;
    width: 8px;
    margin: 0;
}

QScrollBar::handle:vertical {
    background: #343b45;
    border-radius: 4px;
    min-height: 24px;
}

QScrollBar::handle:vertical:hover {
    background: #6b7380;
}

QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical,
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {
    background: none;
    height: 0;
}

QScrollBar:horizontal {
    background: #0f1115;
    height: 8px;
}

QScrollBar::handle:horizontal {
    background: #343b45;
    border-radius: 4px;
    min-width: 24px;
}

QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {
    width: 0;
}

/* ===== Video ===== */
QLabel#videoSurface {
    background-color: #080a0c;
    border: 1px solid #2a3038;
    border-radius: 6px;
}

/* ===== Form labels ===== */
QLabel#formLabel {
    color: #9aa3b2;
    font-size: 8pt;
    background: transparent;
}
"""

DARK_THEME_QSS = ARJUNA_THEME_QSS
