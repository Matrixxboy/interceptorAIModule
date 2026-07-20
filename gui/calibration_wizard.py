from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, 
    QStackedWidget, QWidget, QSlider, QFormLayout, QLineEdit, QComboBox
)
from PyQt6.QtCore import Qt
from core.config_manager import ConfigManager

class CalibrationWizard(QDialog):
    def __init__(self, config_manager: ConfigManager, parent=None):
        super().__init__(parent)
        self.config_manager = config_manager
        self.setWindowTitle("System Calibration Wizard")
        self.resize(600, 400)
        
        self.layout = QVBoxLayout(self)
        self.stacked_widget = QStackedWidget()
        self.layout.addWidget(self.stacked_widget)
        
        # Pages
        self.page_intro = self._create_intro_page()
        self.page_camera = self._create_camera_page()
        self.page_pid = self._create_pid_page()
        self.page_summary = self._create_summary_page()
        
        self.stacked_widget.addWidget(self.page_intro)
        self.stacked_widget.addWidget(self.page_camera)
        self.stacked_widget.addWidget(self.page_pid)
        self.stacked_widget.addWidget(self.page_summary)
        
        # Navigation
        self.nav_layout = QHBoxLayout()
        self.btn_back = QPushButton("Back")
        self.btn_next = QPushButton("Next")
        self.btn_back.clicked.connect(self._go_back)
        self.btn_next.clicked.connect(self._go_next)
        
        self.nav_layout.addWidget(self.btn_back)
        self.nav_layout.addWidget(self.btn_next)
        self.layout.addLayout(self.nav_layout)
        
        self._update_buttons()

    def _create_intro_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.addWidget(QLabel("<h2>Welcome to the Calibration Wizard</h2>"))
        layout.addWidget(QLabel("This wizard will guide you through calibrating the drone's systems."))
        layout.addStretch()
        return page
        
    def _create_camera_page(self):
        page = QWidget()
        layout = QFormLayout(page)
        layout.addRow(QLabel("<h2>Camera Calibration</h2>"))
        
        self.combo_cam_index = QComboBox()
        self.combo_cam_index.addItems(["0", "1", "2", "3"])
        layout.addRow("Camera Index:", self.combo_cam_index)
        
        self.input_fov = QLineEdit()
        self.input_fov.setText("90")
        layout.addRow("Camera FOV (degrees):", self.input_fov)
        return page
        
    def _create_pid_page(self):
        page = QWidget()
        layout = QFormLayout(page)
        layout.addRow(QLabel("<h2>PID Tuning Defaults</h2>"))
        
        self.input_p = QLineEdit("0.1")
        self.input_i = QLineEdit("0.01")
        self.input_d = QLineEdit("0.05")
        
        layout.addRow("Proportional (P):", self.input_p)
        layout.addRow("Integral (I):", self.input_i)
        layout.addRow("Derivative (D):", self.input_d)
        return page
        
    def _create_summary_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.addWidget(QLabel("<h2>Calibration Complete</h2>"))
        layout.addWidget(QLabel("Click Finish to save parameters to the active profile."))
        layout.addStretch()
        return page
        
    def _go_next(self):
        idx = self.stacked_widget.currentIndex()
        if idx < self.stacked_widget.count() - 1:
            self.stacked_widget.setCurrentIndex(idx + 1)
        elif idx == self.stacked_widget.count() - 1:
            self._save_calibration()
            self.accept()
        self._update_buttons()
            
    def _go_back(self):
        idx = self.stacked_widget.currentIndex()
        if idx > 0:
            self.stacked_widget.setCurrentIndex(idx - 1)
        self._update_buttons()
            
    def _update_buttons(self):
        idx = self.stacked_widget.currentIndex()
        self.btn_back.setEnabled(idx > 0)
        if idx == self.stacked_widget.count() - 1:
            self.btn_next.setText("Finish")
        else:
            self.btn_next.setText("Next")

    def _save_calibration(self):
        # Save camera
        self.config_manager.set("camera", "index", int(self.combo_cam_index.currentText()))
        self.config_manager.set("camera", "fov", float(self.input_fov.text()))
        
        # Save PID
        self.config_manager.set("pid", "p", float(self.input_p.text()))
        self.config_manager.set("pid", "i", float(self.input_i.text()))
        self.config_manager.set("pid", "d", float(self.input_d.text()))
        
        self.config_manager.save_profile()
