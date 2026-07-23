"""Target database management interface."""

from __future__ import annotations

from pathlib import Path

import cv2
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import (
    QComboBox,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from database.target_profile import TargetStatus
from database.target_store import TargetStore
from gui.widgets.page_header import PageHeader, StatusPill


class TargetDatabasePage(QWidget):
    refresh_requested = pyqtSignal()

    def __init__(self, target_store: TargetStore, parent=None) -> None:
        super().__init__(parent)
        self.store = target_store
        self._selected_id: str | None = None
        self._build_ui()
        self.refresh_list()

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)

        page_header = PageHeader(
            "Target Database",
            "Evidence · box crops · timeline · pattern data",
        )
        self.pill_db = StatusPill("DB READY", "info")
        page_header.right_layout.addWidget(self.pill_db)
        layout.addWidget(page_header)

        header = QHBoxLayout()
        header.setSpacing(8)

        self.search_input = QLineEdit()
        self.search_input.setPlaceholderText("Search by Target ID...")
        self.search_input.textChanged.connect(self.refresh_list)
        self.filter_status = QComboBox()
        self.filter_status.addItem("All Statuses", "")
        for s in TargetStatus:
            self.filter_status.addItem(s.value.title(), s.value)
        self.filter_status.currentIndexChanged.connect(self.refresh_list)

        btn_refresh = QPushButton("Refresh")
        btn_refresh.setObjectName("btnGhost")
        btn_refresh.clicked.connect(self.refresh_list)
        btn_export = QPushButton("Export")
        btn_export.setObjectName("btnPrimary")
        btn_export.clicked.connect(self._export_selected)
        btn_delete = QPushButton("Delete")
        btn_delete.setObjectName("btnDanger")
        btn_delete.clicked.connect(self._delete_selected)

        header.addWidget(self.search_input, stretch=1)
        header.addWidget(self.filter_status)
        header.addWidget(btn_refresh)
        header.addWidget(btn_export)
        header.addWidget(btn_delete)
        layout.addLayout(header)

        splitter = QSplitter(Qt.Orientation.Horizontal)

        self.table = QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(["Target ID", "Status", "Confidence", "Duration", "Frames", "Label"])
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        self.table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.table.setAlternatingRowColors(True)
        self.table.itemSelectionChanged.connect(self._on_selection_changed)
        splitter.addWidget(self.table)

        detail_tabs = QTabWidget()

        self.lbl_image = QLabel("No image")
        self.lbl_image.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_image.setMinimumSize(320, 240)
        self.lbl_image.setStyleSheet("background: #0f172a; border: 1px solid #334155;")

        meta_widget = QWidget()
        meta_layout = QVBoxLayout(meta_widget)
        self.txt_metadata = QTextEdit()
        self.txt_metadata.setReadOnly(True)
        self.txt_metadata.setStyleSheet("font-family: Consolas, monospace; font-size: 9pt;")
        meta_layout.addWidget(self.lbl_image)
        meta_layout.addWidget(self.txt_metadata)
        detail_tabs.addTab(meta_widget, "Metadata & Images")

        self.txt_timeline = QTextEdit()
        self.txt_timeline.setReadOnly(True)
        self.txt_timeline.setStyleSheet("font-family: Consolas, monospace; font-size: 9pt;")
        detail_tabs.addTab(self.txt_timeline, "Tracking Timeline")

        self.txt_pattern = QTextEdit()
        self.txt_pattern.setReadOnly(True)
        self.txt_pattern.setStyleSheet("font-family: Consolas, monospace; font-size: 9pt;")
        detail_tabs.addTab(self.txt_pattern, "Pattern Explorer")

        splitter.addWidget(detail_tabs)
        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 3)
        layout.addWidget(splitter)

        stats = self.store.stats()
        self.lbl_stats = QLabel(f"Total targets: {stats['total']}")
        self.lbl_stats.setStyleSheet("color: #6b7380;")
        layout.addWidget(self.lbl_stats)

    def refresh_list(self) -> None:
        status_val = self.filter_status.currentData()
        status = TargetStatus(status_val) if status_val else None
        targets = self.store.list_targets(status=status, search_id=self.search_input.text())

        self.table.setRowCount(len(targets))
        for row, t in enumerate(targets):
            self.table.setItem(row, 0, QTableWidgetItem(t.target_id))
            self.table.setItem(row, 1, QTableWidgetItem(t.status.value))
            self.table.setItem(row, 2, QTableWidgetItem(f"{t.confidence * 100:.0f}%"))
            self.table.setItem(row, 3, QTableWidgetItem(f"{t.tracking_duration_s:.1f}s"))
            self.table.setItem(row, 4, QTableWidgetItem(str(t.frames_processed)))
            self.table.setItem(row, 5, QTableWidgetItem(t.label))

        stats = self.store.stats()
        self.lbl_stats.setText(f"Total targets: {stats['total']} | {stats.get('by_status', {})}")

    def _on_selection_changed(self) -> None:
        rows = self.table.selectionModel().selectedRows()
        if not rows:
            return
        tid = self.table.item(rows[0].row(), 0).text()
        self._selected_id = tid
        self._show_detail(tid)

    def _show_detail(self, target_id: str) -> None:
        profile = self.store.get(target_id)
        if not profile:
            return

        meta = (
            f"Target ID:     {profile.target_id}\n"
            f"Status:        {profile.status.value}\n"
            f"Label:         {profile.label}\n"
            f"Detection:     {profile.detection_time:.3f}\n"
            f"Lock Time:     {profile.lock_time or 'N/A'}\n"
            f"Duration:      {profile.tracking_duration_s:.1f}s\n"
            f"Confidence:    {profile.confidence * 100:.1f}%\n"
            f"Distance:      {profile.distance_m:.1f}m\n"
            f"Velocity:      ({profile.velocity[0]:.1f}, {profile.velocity[1]:.1f}) px/s\n"
            f"Direction:     {profile.direction_deg:.0f}°\n"
            f"Source:        {profile.tracking_source}\n"
            f"Frames:        {profile.frames_processed}\n"
            f"Images:        {list(profile.image_paths.keys())}\n"
        )
        self.txt_metadata.setPlainText(meta)

        # Prefer exact target-box crops for precise tracking inspection
        img_path = None
        for key in ("target_box", "reference_crop", "best_quality", "initial_lock", "latest"):
            img_path = self.store.get_image_path(target_id, key)
            if img_path is not None:
                break
        if img_path and img_path.exists():
            frame = cv2.imread(str(img_path))
            if frame is not None:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb.shape
                qimg = QImage(rgb.data, w, h, ch * w, QImage.Format.Format_RGB888)
                pix = QPixmap.fromImage(qimg).scaled(
                    480, 360, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation
                )
                self.lbl_image.setPixmap(pix)
                self.lbl_image.setToolTip(f"Target-box crop ({w}×{h}px) — {img_path.name}")
            else:
                self.lbl_image.setText("No image available")
        else:
            self.lbl_image.setText("No target-box crop available")

        timeline_text = ""
        for ev in profile.timeline:
            timeline_text += (
                f"[{ev.time_str}] {ev.event}\n"
                f"  Confidence: {ev.confidence * 100:.0f}% | Drone: {ev.drone_state or 'N/A'}\n"
                f"  Response: {ev.system_response}\n\n"
            )
        self.txt_timeline.setPlainText(timeline_text or "No timeline events.")

        pattern = (
            f"Color Signature ({len(profile.color_signature)} bins):\n"
            f"  {profile.color_signature[:16]}{'...' if len(profile.color_signature) > 16 else ''}\n\n"
            f"Feature Embedding ({len(profile.feature_embedding)} dims):\n"
            f"  {profile.feature_embedding[:8] or 'Not computed'}\n\n"
            f"Bounding Box History: {len(profile.bbox_history)} entries\n"
            f"Confidence History: {len(profile.confidence_history)} entries\n"
            f"Motion Vectors: {len(profile.motion_vectors)} entries\n"
        )
        if profile.bbox_history:
            last = profile.bbox_history[-1]
            pattern += f"\nLast BBox: x={last[0]:.0f} y={last[1]:.0f} w={last[2]:.0f} h={last[3]:.0f}"
        self.txt_pattern.setPlainText(pattern)

    def _export_selected(self) -> None:
        if not self._selected_id:
            QMessageBox.information(self, "Export", "Select a target first.")
            return
        json_path = self.store.export_target_json(self._selected_id)
        csv_path = self.store.export_target_csv(self._selected_id)
        QMessageBox.information(self, "Export", f"Exported to:\n{json_path}\n{csv_path}")

    def _delete_selected(self) -> None:
        if not self._selected_id:
            return
        reply = QMessageBox.question(self, "Delete", f"Delete target {self._selected_id}?")
        if reply == QMessageBox.StandardButton.Yes:
            self.store.delete_target(self._selected_id)
            self._selected_id = None
            self.refresh_list()
