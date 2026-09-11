"""Real-time OpenCV / PyQt Video Widget with Interactive ROI Drag & Telemetry HUD."""

from __future__ import annotations

import cv2
import numpy as np
from PyQt6.QtCore import QPoint, QRect, Qt, pyqtSignal
from PyQt6.QtGui import QColor, QImage, QMouseEvent, QPainter, QPen, QPixmap
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
            "background-color: #030405; border: 1px solid #242932; border-radius: 8px;"
            "}"
        )

        self.dragging = False
        self.start_point = QPoint()
        self.end_point = QPoint()
        self.current_frame: np.ndarray | None = None
        self._rgb_keep: np.ndarray | None = None  # keep buffer alive for QImage

    def _content_rect(self) -> QRect:
        """Letterboxed image rect inside the widget (KeepAspectRatio)."""
        if self.current_frame is None:
            return self.rect()
        img_h, img_w = self.current_frame.shape[:2]
        widget_w = max(1, self.width())
        widget_h = max(1, self.height())
        scale = min(widget_w / max(1, img_w), widget_h / max(1, img_h))
        disp_w = max(1, int(round(img_w * scale)))
        disp_h = max(1, int(round(img_h * scale)))
        ox = (widget_w - disp_w) // 2
        oy = (widget_h - disp_h) // 2
        return QRect(ox, oy, disp_w, disp_h)

    def _widget_to_image(self, pos: QPoint) -> tuple[int, int] | None:
        if self.current_frame is None:
            return None
        content = self._content_rect()
        if not content.contains(pos):
            return None
        img_h, img_w = self.current_frame.shape[:2]
        ix = int(round((pos.x() - content.x()) * img_w / max(1, content.width())))
        iy = int(round((pos.y() - content.y()) * img_h / max(1, content.height())))
        ix = max(0, min(img_w - 1, ix))
        iy = max(0, min(img_h - 1, iy))
        return ix, iy

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

            p0 = self._widget_to_image(self.start_point)
            p1 = self._widget_to_image(self.end_point)
            if p0 is not None and p1 is not None:
                ix0, iy0 = p0
                ix1, iy1 = p1
                ix = min(ix0, ix1)
                iy = min(iy0, iy1)
                iw = abs(ix1 - ix0)
                ih = abs(iy1 - iy0)
                if iw > 10 and ih > 10:
                    self.roi_selected.emit(ix, iy, iw, ih)
                elif iw <= 5 and ih <= 5:
                    self.point_clicked.emit(ix0, iy0)

            self.update()

    def _normalize_rect(self, p1: QPoint, p2: QPoint) -> QRect:
        x = min(p1.x(), p2.x())
        y = min(p1.y(), p2.y())
        w = abs(p1.x() - p2.x())
        h = abs(p1.y() - p2.y())
        return QRect(x, y, w, h)

    def update_frame(self, frame_bgr: np.ndarray) -> None:
        if frame_bgr is None or getattr(frame_bgr, "size", 0) == 0:
            return
        try:
            frame_bgr = np.array(frame_bgr, copy=True)
        except Exception:
            return
        self.current_frame = frame_bgr
        h, w = frame_bgr.shape[:2]
        if h < 2 or w < 2:
            return

        # Downscale large frames for UI (keeps tracking at full res on worker)
        max_disp_w = max(640, self.width() * 2)
        if w > max_disp_w:
            scale = max_disp_w / float(w)
            frame_bgr = cv2.resize(
                frame_bgr,
                (int(w * scale), int(h * scale)),
                interpolation=cv2.INTER_AREA,
            )
            h, w = frame_bgr.shape[:2]

        try:
            self._rgb_keep = np.ascontiguousarray(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
            bytes_per_line = int(self._rgb_keep.strides[0])
            qimg = QImage(
                self._rgb_keep.data, w, h, bytes_per_line, QImage.Format.Format_RGB888
            )
            pix = QPixmap.fromImage(qimg)
            self.setPixmap(
                pix.scaled(
                    self.size(),
                    Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.FastTransformation,
                )
            )
        except Exception:
            return

    def paintEvent(self, event) -> None:
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, False)
        r = self.rect().adjusted(4, 4, -5, -5)
        frame = QColor("#2C313A")
        painter.setPen(QPen(frame, 1))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawRect(r)

        tick = 14
        accent = QColor("#9D3FF5")
        painter.setPen(QPen(accent, 1))
        # Corner brackets — ISR-style frame, not a glow
        x0, y0, x1, y1 = r.left(), r.top(), r.right(), r.bottom()
        painter.drawLine(x0, y0, x0 + tick, y0)
        painter.drawLine(x0, y0, x0, y0 + tick)
        painter.drawLine(x1, y0, x1 - tick, y0)
        painter.drawLine(x1, y0, x1, y0 + tick)
        painter.drawLine(x0, y1, x0 + tick, y1)
        painter.drawLine(x0, y1, x0, y1 - tick)
        painter.drawLine(x1, y1, x1 - tick, y1)
        painter.drawLine(x1, y1, x1, y1 - tick)

        if self.dragging:
            pen = QPen(QColor("#9D3FF5"), 1, Qt.PenStyle.DashLine)
            painter.setPen(pen)
            rect = self._normalize_rect(self.start_point, self.end_point)
            painter.drawRect(rect)
