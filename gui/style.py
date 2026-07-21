"""Professional industrial theme for Arjuna GCS — muted aerospace palette (no neon)."""

# Shared palette constants for Python widgets
PALETTE = {
    "bg": "#12151a",
    "bg_panel": "#1a1e24",
    "bg_elevated": "#22262e",
    "border": "#2c323c",
    "border_soft": "#252a32",
    "text": "#d8dde6",
    "text_dim": "#8b929e",
    "text_mute": "#5c6470",
    "accent": "#7a8fa3",
    "accent_soft": "#5f7386",
    "ok": "#6a8a74",
    "warn": "#a08a5c",
    "error": "#9a6363",
    "info": "#7a8fa3",
}

ARJUNA_THEME_QSS = """
/* ===== Base — charcoal industrial ===== */
QMainWindow, QDialog {
    background-color: #12151a;
    color: #d8dde6;
    font-family: 'Segoe UI', 'Segoe UI Variable', system-ui, sans-serif;
}

QWidget {
    background-color: transparent;
    color: #b4bac4;
    font-size: 10pt;
}

QToolTip {
    background-color: #22262e;
    color: #d8dde6;
    border: 1px solid #2c323c;
    padding: 6px 8px;
}

/* ===== Sidebar ===== */
QFrame#sidebar {
    background-color: #161a20;
    border-right: 1px solid #2c323c;
}

QFrame#brandBlock {
    background-color: #161a20;
    border-bottom: 1px solid #2c323c;
}

QLabel#brandMark {
    color: #7a8fa3;
    font-size: 8pt;
    font-weight: 600;
    letter-spacing: 2.5px;
    background: transparent;
}

QLabel#brandLabel {
    font-size: 18pt;
    font-weight: 700;
    color: #e8ecf0;
    letter-spacing: 4px;
    padding: 2px 0 0 0;
    background: transparent;
}

QLabel#brandSubtitle {
    font-size: 7.5pt;
    color: #5c6470;
    letter-spacing: 2px;
    padding-bottom: 4px;
    background: transparent;
}

QLabel#navStatus {
    font-size: 8pt;
    color: #6a8a74;
    letter-spacing: 1.2px;
    padding: 10px 12px;
    border-top: 1px solid #2c323c;
    background: transparent;
}

QListWidget#navList {
    background-color: transparent;
    border: none;
    outline: none;
    font-size: 9.5pt;
    padding: 8px 0;
}

QListWidget#navList::item {
    padding: 9px 14px 9px 16px;
    margin: 1px 8px;
    border-radius: 2px;
    border-left: 2px solid transparent;
    color: #8b929e;
}

QListWidget#navList::item:selected {
    background-color: #22262e;
    border-left: 2px solid #7a8fa3;
    color: #e8ecf0;
    font-weight: 600;
}

QListWidget#navList::item:hover:!selected {
    background-color: #1a1e24;
    color: #c4cad4;
}

/* ===== Content chrome ===== */
QFrame#contentChrome {
    background-color: #12151a;
}

QLabel#pageTitle {
    font-size: 14pt;
    font-weight: 650;
    color: #e8ecf0;
    letter-spacing: 1.5px;
    background: transparent;
}

QLabel#pageSubtitle {
    font-size: 8.5pt;
    color: #5c6470;
    background: transparent;
}

QFrame#headerRule {
    background-color: #2c323c;
    border: none;
    max-height: 1px;
}

QLabel#panelTitle {
    color: #5c6470;
    font-size: 8pt;
    font-weight: 650;
    letter-spacing: 1.2px;
    background: transparent;
}

QFrame#panel {
    background-color: #1a1e24;
    border: 1px solid #2c323c;
    border-radius: 3px;
}

/* ===== Group boxes ===== */
QGroupBox {
    background-color: #1a1e24;
    border: 1px solid #2c323c;
    border-radius: 3px;
    margin-top: 14px;
    font-weight: 600;
    color: #5c6470;
    padding: 12px 10px 10px 10px;
}

QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 10px;
    padding: 0 6px;
    color: #5c6470;
    font-size: 8pt;
    letter-spacing: 1px;
    background-color: #12151a;
}

/* ===== Buttons — flat steel ===== */
QPushButton {
    background-color: #22262e;
    color: #d8dde6;
    font-weight: 600;
    font-size: 9pt;
    border: 1px solid #3a424e;
    border-radius: 2px;
    padding: 7px 14px;
    min-height: 26px;
    min-width: 72px;
}

QPushButton:hover {
    background-color: #2a303a;
    border-color: #7a8fa3;
    color: #f0f2f5;
}

QPushButton:pressed {
    background-color: #1a1e24;
}

QPushButton:disabled {
    background-color: #1a1e24;
    color: #4a5260;
    border-color: #2c323c;
}

QPushButton#btn_arm {
    background-color: #2a1e1e;
    border-color: #6a4545;
    color: #c4a0a0;
}
QPushButton#btn_arm:hover {
    background-color: #3a2828;
    border-color: #9a6363;
    color: #e8d0d0;
}
QPushButton#btn_arm:checked {
    background-color: #4a3030;
    border-color: #9a6363;
    color: #f0e0e0;
}

QPushButton#btnPrimary {
    background-color: #24303a;
    border-color: #5f7386;
    color: #d0dae4;
}
QPushButton#btnPrimary:hover {
    background-color: #2e3c48;
    border-color: #7a8fa3;
    color: #eef2f6;
}

QPushButton#btnDanger {
    background-color: #2a1e1e;
    border-color: #6a4545;
    color: #c4a0a0;
}
QPushButton#btnDanger:hover {
    background-color: #3a2828;
    border-color: #9a6363;
    color: #e8d0d0;
}

QPushButton#btnSuccess {
    background-color: #1e2a22;
    border-color: #4a6554;
    color: #a8c0b0;
}
QPushButton#btnSuccess:hover {
    background-color: #263830;
    border-color: #6a8a74;
    color: #d0e4d8;
}

QPushButton#btnGhost {
    background-color: transparent;
    border: 1px solid #2c323c;
    color: #8b929e;
}
QPushButton#btnGhost:hover {
    border-color: #3a424e;
    color: #d8dde6;
    background-color: #1a1e24;
}

/* ===== Inputs ===== */
QSlider::groove:horizontal {
    height: 3px;
    background: #2c323c;
    border-radius: 1px;
}

QSlider::sub-page:horizontal {
    background: #5f7386;
    border-radius: 1px;
}

QSlider::handle:horizontal {
    background: #c4cad4;
    border: 1px solid #5f7386;
    width: 12px;
    margin-top: -5px;
    margin-bottom: -5px;
    border-radius: 6px;
}

QDoubleSpinBox, QSpinBox, QLineEdit, QComboBox {
    background-color: #161a20;
    border: 1px solid #2c323c;
    border-radius: 2px;
    padding: 5px 8px;
    color: #e8ecf0;
    selection-background-color: #2e3c48;
}

QDoubleSpinBox:focus, QSpinBox:focus, QLineEdit:focus, QComboBox:focus {
    border: 1px solid #5f7386;
}

QComboBox::drop-down {
    border: none;
    width: 22px;
}

QComboBox QAbstractItemView {
    background-color: #1a1e24;
    border: 1px solid #2c323c;
    selection-background-color: #2a303a;
    color: #d8dde6;
    outline: none;
}

/* ===== Tabs ===== */
QTabWidget::pane {
    border: 1px solid #2c323c;
    border-radius: 0 0 3px 3px;
    background-color: #1a1e24;
    top: -1px;
}

QTabBar::tab {
    background-color: #12151a;
    color: #5c6470;
    padding: 8px 16px;
    border: 1px solid #2c323c;
    border-bottom: none;
    margin-right: 2px;
    font-size: 9pt;
    font-weight: 600;
}

QTabBar::tab:selected {
    background-color: #1a1e24;
    color: #c4cad4;
    border-top: 2px solid #7a8fa3;
}

QTabBar::tab:hover:!selected {
    color: #8b929e;
    background-color: #161a20;
}

/* ===== Tables ===== */
QTableWidget {
    background-color: #161a20;
    alternate-background-color: #1a1e24;
    gridline-color: #252a32;
    border: 1px solid #2c323c;
    border-radius: 3px;
    selection-background-color: #2a303a;
    selection-color: #e8ecf0;
}

QTableWidget::item {
    padding: 4px 8px;
}

QHeaderView::section {
    background-color: #1a1e24;
    color: #5c6470;
    padding: 8px;
    border: none;
    border-bottom: 1px solid #2c323c;
    border-right: 1px solid #252a32;
    font-weight: 650;
    font-size: 8pt;
    letter-spacing: 0.4px;
}

/* ===== Status / progress ===== */
QStatusBar {
    background-color: #0e1115;
    color: #5c6470;
    border-top: 1px solid #2c323c;
    font-size: 8.5pt;
    font-family: Consolas, 'Cascadia Mono', monospace;
}

QStatusBar QLabel {
    color: #5c6470;
    padding: 0 10px;
    background: transparent;
    border-left: 1px solid #2c323c;
}

QProgressBar {
    background-color: #161a20;
    border: 1px solid #2c323c;
    border-radius: 2px;
    text-align: center;
    color: #c4cad4;
    max-height: 16px;
    font-size: 8pt;
}

QProgressBar::chunk {
    background-color: #5f7386;
    border-radius: 1px;
}

QTextEdit {
    background-color: #12151a;
    border: 1px solid #2c323c;
    border-radius: 2px;
    color: #b4bac4;
    selection-background-color: #2e3c48;
}

/* ===== Splitter / scroll ===== */
QSplitter::handle {
    background-color: #2c323c;
    width: 1px;
    height: 1px;
}

QScrollBar:vertical {
    background: #12151a;
    width: 8px;
    margin: 0;
}

QScrollBar::handle:vertical {
    background: #3a424e;
    border-radius: 4px;
    min-height: 24px;
}

QScrollBar::handle:vertical:hover {
    background: #5c6470;
}

QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical,
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {
    background: none;
    height: 0;
}

QScrollBar:horizontal {
    background: #12151a;
    height: 8px;
}

QScrollBar::handle:horizontal {
    background: #3a424e;
    border-radius: 4px;
    min-width: 24px;
}

QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {
    width: 0;
}

/* ===== Video ===== */
QLabel#videoSurface {
    background-color: #0a0c0f;
    border: 1px solid #2c323c;
    border-radius: 2px;
}
"""

DARK_THEME_QSS = ARJUNA_THEME_QSS
