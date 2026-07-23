"""Live system logs viewer with filtering and export."""

from __future__ import annotations

from PyQt6.QtCore import Qt, pyqtSlot
from PyQt6.QtWidgets import (
    QComboBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from gui.widgets.page_header import PageHeader, StatusPill
from sys_logging.system_logger import LogCategory, LogEntry, LogSeverity, SystemLogger


class LogsPage(QWidget):
    def __init__(self, sys_logger: SystemLogger, parent=None) -> None:
        super().__init__(parent)
        self.logger = sys_logger
        self._build_ui()
        self.logger.subscribe(self._on_new_log)
        self.refresh_logs()

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)

        header = PageHeader("System Logs", "Categorized event stream · filter · export")
        self.pill_logs = StatusPill("LIVE", "ok")
        header.right_layout.addWidget(self.pill_logs)
        layout.addWidget(header)

        filters = QHBoxLayout()
        self.search_input = QLineEdit()
        self.search_input.setPlaceholderText("Search logs...")
        self.search_input.textChanged.connect(self.refresh_logs)

        self.filter_category = QComboBox()
        self.filter_category.addItem("All Categories", "")
        for c in LogCategory:
            self.filter_category.addItem(c.value, c.value)
        self.filter_category.currentIndexChanged.connect(self.refresh_logs)

        self.filter_severity = QComboBox()
        self.filter_severity.addItem("All Severities", "")
        for s in LogSeverity:
            self.filter_severity.addItem(s.value, s.value)
        self.filter_severity.currentIndexChanged.connect(self.refresh_logs)

        btn_refresh = QPushButton("Refresh")
        btn_refresh.setObjectName("btnGhost")
        btn_refresh.clicked.connect(self.refresh_logs)
        btn_export_json = QPushButton("Export JSON")
        btn_export_json.setObjectName("btnGhost")
        btn_export_json.clicked.connect(lambda: self.logger.export_json())
        btn_export_csv = QPushButton("Export CSV")
        btn_export_csv.setObjectName("btnGhost")
        btn_export_csv.clicked.connect(lambda: self.logger.export_csv())
        btn_clear = QPushButton("Clear")
        btn_clear.setObjectName("btnDanger")
        btn_clear.clicked.connect(self._clear_logs)

        filters.addWidget(self.search_input, stretch=2)
        filters.addWidget(self.filter_category)
        filters.addWidget(self.filter_severity)
        filters.addWidget(btn_refresh)
        filters.addWidget(btn_export_json)
        filters.addWidget(btn_export_csv)
        filters.addWidget(btn_clear)
        layout.addLayout(filters)

        self.table = QTableWidget(0, 8)
        self.table.setHorizontalHeaderLabels(
            ["Time", "Severity", "Category", "Module", "Message", "Target", "FPS", "Latency"]
        )
        self.table.horizontalHeader().setSectionResizeMode(4, QHeaderView.ResizeMode.Stretch)
        self.table.setAlternatingRowColors(True)
        self.table.setStyleSheet("font-family: Consolas, monospace; font-size: 9pt;")
        layout.addWidget(self.table)

    def _severity_color(self, severity: LogSeverity) -> str:
        return {
            LogSeverity.DEBUG: "#6b7380",
            LogSeverity.INFO: "#4f7cac",
            LogSeverity.WARNING: "#b08a3c",
            LogSeverity.ERROR: "#b05656",
            LogSeverity.CRITICAL: "#b05656",
        }.get(severity, "#9aa3b2")

    def refresh_logs(self) -> None:
        cat_val = self.filter_category.currentData()
        sev_val = self.filter_severity.currentData()
        category = LogCategory(cat_val) if cat_val else None
        severity = LogSeverity(sev_val) if sev_val else None
        entries = self.logger.get_entries(
            category=category,
            severity=severity,
            search=self.search_input.text(),
            limit=500,
        )
        self._populate(entries)

    def _populate(self, entries: list[LogEntry]) -> None:
        self.table.setRowCount(len(entries))
        for row, e in enumerate(entries):
            items = [
                e.time_str,
                e.severity.value,
                e.category.value,
                e.module,
                e.message,
                e.target_id,
                f"{e.fps:.1f}" if e.fps else "",
                f"{e.latency_ms:.1f}ms" if e.latency_ms else "",
            ]
            for col, text in enumerate(items):
                item = QTableWidgetItem(text)
                if col == 1:
                    item.setForeground(Qt.GlobalColor.white)
                self.table.setItem(row, col, item)

    @pyqtSlot(object)
    def _on_new_log(self, entry: LogEntry) -> None:
        if self.table.rowCount() >= 500:
            self.table.removeRow(self.table.rowCount() - 1)
        self.table.insertRow(0)
        items = [
            entry.time_str,
            entry.severity.value,
            entry.category.value,
            entry.module,
            entry.message,
            entry.target_id,
            f"{entry.fps:.1f}" if entry.fps else "",
            f"{entry.latency_ms:.1f}ms" if entry.latency_ms else "",
        ]
        for col, text in enumerate(items):
            self.table.setItem(0, col, QTableWidgetItem(text))

    def _clear_logs(self) -> None:
        self.logger.clear()
        self.table.setRowCount(0)
