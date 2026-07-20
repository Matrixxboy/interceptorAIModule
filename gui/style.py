"""Dark theme QSS styling for FPV Interceptor GUI."""

DARK_THEME_QSS = """
QMainWindow {
    background-color: #0f172a;
    color: #f8fafc;
    font-family: 'Segoe UI', Inter, Helvetica, Arial, sans-serif;
}

QWidget {
    background-color: #0f172a;
    color: #cbd5e1;
    font-size: 10pt;
}

QGroupBox {
    background-color: #1e293b;
    border: 1px solid #334155;
    border-radius: 8px;
    margin-top: 12px;
    font-weight: bold;
    color: #38bdf8;
    padding: 10px;
}

QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 2px 8px;
    background-color: #0f172a;
    border: 1px solid #38bdf8;
    border-radius: 4px;
    color: #38bdf8;
}

QPushButton {
    background-color: #0284c7;
    color: #ffffff;
    font-weight: bold;
    border: none;
    border-radius: 6px;
    padding: 8px 16px;
    min-height: 24px;
}

QPushButton:hover {
    background-color: #0369a1;
}

QPushButton:pressed {
    background-color: #075985;
}

QPushButton:disabled {
    background-color: #334155;
    color: #64748b;
}

QPushButton#btn_arm {
    background-color: #dc2626;
}
QPushButton#btn_arm:hover {
    background-color: #b91c1c;
}

QPushButton#btn_disarm {
    background-color: #16a34a;
}
QPushButton#btn_disarm:hover {
    background-color: #15803d;
}

QSlider::groove:horizontal {
    height: 6px;
    background: #334155;
    border-radius: 3px;
}

QSlider::sub-page:horizontal {
    background: #38bdf8;
    border-radius: 3px;
}

QSlider::handle:horizontal {
    background: #f8fafc;
    border: 2px solid #0284c7;
    width: 16px;
    margin-top: -5px;
    margin-bottom: -5px;
    border-radius: 8px;
}

QDoubleSpinBox, QSpinBox, QLineEdit {
    background-color: #1e293b;
    border: 1px solid #475569;
    border-radius: 4px;
    padding: 4px 8px;
    color: #f8fafc;
}

QDoubleSpinBox:focus, QSpinBox:focus, QLineEdit:focus {
    border: 1px solid #38bdf8;
}

QTabWidget::pane {
    border: 1px solid #334155;
    border-radius: 6px;
    background-color: #1e293b;
}

QTabBar::tab {
    background-color: #0f172a;
    color: #94a3b8;
    padding: 8px 16px;
    border-top-left-radius: 6px;
    border-top-right-radius: 6px;
    border: 1px solid #334155;
    margin-right: 2px;
}

QTabBar::tab:selected {
    background-color: #1e293b;
    color: #38bdf8;
    border-bottom: 2px solid #38bdf8;
}

QStatusBar {
    background-color: #020617;
    color: #94a3b8;
    border-top: 1px solid #1e293b;
}
"""
