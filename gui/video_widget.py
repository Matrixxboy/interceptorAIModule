"""Real-time OpenCV / PyQt Video Widget with Interactive ROI Drag & Telemetry HUD."""

from __future__ import annotations

import cv2
import numpy as np
from PyQt6.QtCore import QPoint, QRect, Qt, pyqtSignal
from PyQt6.QtGui import QImage, QMouseEvent, QPainter, QPen, QPixmap
from PyQt6.QtWidgets import QLabel, QSizePolicy, QWidget


class VideoDisplayWidget(QLabel):
    roi_selected = pyqtSignal(int, int, int, int)
    point_clicked = pyqtSignal(int, int)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("videoSurface")
        self.setMinimumSize(400, 240)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setStyleSheet(
            "QLabel#videoSurface {"
            "background-color: #02060c; border: 1px solid #152033; border-radius: 2px;"
            "}"
        )

        self.dragging = False
        self.start_point = QPoint()
        self.end_point = QPoint()
        self.current_frame: np.ndarray | None = None

    def mousePressEvent(self, event: QMouseEvent) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            self.dragging = True
            self.start_point = event.pos()
            self.end_point = event.pos()
            self.update()

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        if self.dragging:
            self.end_point = event.pos()
            self.update()

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        if event.button() == Qt.MouseButton.LeftButton and self.dragging:
            self.dragging = False
            self.end_point = event.pos()

            # Map widget coordinates to image coordinates
            rect = self._normalize_rect(self.start_point, self.end_point)
            if rect.width() > 10 and rect.height() > 10 and self.current_frame is not None:
                img_h, img_w = self.current_frame.shape[:2]
                disp_w = self.width()
                disp_h = self.height()

                scale_x = img_w / max(1, disp_w)
                scale_y = img_h / max(1, disp_h)

                ix = int(rect.x() * scale_x)
                iy = int(rect.y() * scale_y)
                iw = int(rect.width() * scale_x)
                ih = int(rect.height() * scale_y)

                self.roi_selected.emit(ix, iy, iw, ih)
            elif rect.width() <= 5 and rect.height() <= 5 and self.current_frame is not None:
                img_h, img_w = self.current_frame.shape[:2]
                disp_w = self.width()
                disp_h = self.height()
                ix = int(event.pos().x() * (img_w / max(1, disp_w)))
                iy = int(event.pos().y() * (img_h / max(1, disp_h)))
                self.point_clicked.emit(ix, iy)

            self.update()

    def _normalize_rect(self, p1: QPoint, p2: QPoint) -> QRect:
        x = min(p1.x(), p2.x())
        y = min(p1.y(), p2.y())
        w = abs(p1.x() - p2.x())
        h = abs(p1.y() - p2.y())
        return QRect(x, y, w, h)

    def update_frame(self, frame_bgr: np.ndarray) -> None:
        self.current_frame = frame_bgr
        h, w, ch = frame_bgr.shape
        bytes_per_line = ch * w
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        qimg = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
        pix = QPixmap.fromImage(qimg)
        self.setPixmap(pix.scaled(self.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation))

    def paintEvent(self, event) -> None:
        super().paintEvent(event)
        if self.dragging:
            painter = QPainter(self)
            pen = QPen(Qt.GlobalColor.magenta, 2, Qt.PenStyle.DashLine)
            painter.setPen(pen)
            rect = self._normalize_rect(self.start_point, self.end_point)
            painter.drawRect(rect)
