"""Real-time Image Processing & Feature Lock Debug Inspector Widget."""

from __future__ import annotations

import cv2
import numpy as np
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import QGroupBox, QGridLayout, QHBoxLayout, QLabel, QVBoxLayout, QWidget


class ImageProcessingWidget(QWidget):
    """Visualizes internal image processing steps: Optical flow, template matching, edge gradients, and color histograms."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        main_layout = QVBoxLayout(self)
        grid_layout = QGridLayout()

        # 1. Optical Flow & Keypoints View
        grp_flow = QGroupBox("1. Lucas-Kanade Sub-Pixel Keypoints & Motion Vectors")
        l_flow = QVBoxLayout(grp_flow)
        self.lbl_flow = QLabel("Waiting for camera feed...")
        self.lbl_flow.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_flow.setMinimumSize(320, 240)
        self.lbl_flow.setStyleSheet("background-color: #020617; border: 1px solid #334155; border-radius: 4px;")
        l_flow.addWidget(self.lbl_flow)

        # 2. NCC Template Match Heatmap
        grp_tmpl = QGroupBox("2. NCC Template Correlation Heatmap")
        l_tmpl = QVBoxLayout(grp_tmpl)
        self.lbl_tmpl = QLabel("No Target Locked")
        self.lbl_tmpl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_tmpl.setMinimumSize(320, 240)
        self.lbl_tmpl.setStyleSheet("background-color: #020617; border: 1px solid #334155; border-radius: 4px;")
        l_tmpl.addWidget(self.lbl_tmpl)

        # 3. Canny Edge Gradient Filter
        grp_edge = QGroupBox("3. Canny High-Frequency Edge Structure")
        l_edge = QVBoxLayout(grp_edge)
        self.lbl_edge = QLabel("Waiting for camera feed...")
        self.lbl_edge.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_edge.setMinimumSize(320, 240)
        self.lbl_edge.setStyleSheet("background-color: #020617; border: 1px solid #334155; border-radius: 4px;")
        l_edge.addWidget(self.lbl_edge)

        # 4. HSV Color Histogram Signature
        grp_hist = QGroupBox("4. Target HSV Color Signature")
        l_hist = QVBoxLayout(grp_hist)
        self.lbl_hist = QLabel("No Target Locked")
        self.lbl_hist.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_hist.setMinimumSize(320, 240)
        self.lbl_hist.setStyleSheet("background-color: #020617; border: 1px solid #334155; border-radius: 4px;")
        l_hist.addWidget(self.lbl_hist)

        grid_layout.addWidget(grp_flow, 0, 0)
        grid_layout.addWidget(grp_tmpl, 0, 1)
        grid_layout.addWidget(grp_edge, 1, 0)
        grid_layout.addWidget(grp_hist, 1, 1)

        main_layout.addLayout(grid_layout)

    def update_processing_views(
        self,
        frame_bgr: np.ndarray,
        pixel_engine: object | None = None,
        target_hist: np.ndarray | None = None,
        bbox: tuple[int, int, int, int] | None = None,
    ) -> None:
        if frame_bgr is None or frame_bgr.size == 0:
            return

        h, w = frame_bgr.shape[:2]
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        # --- 1. Optical Flow & Motion Vectors ---
        flow_img = frame_bgr.copy()
        if pixel_engine is not None and hasattr(pixel_engine, "p0") and pixel_engine.p0 is not None:
            pts = pixel_engine.p0.reshape(-1, 2)
            for pt in pts:
                px, py = int(pt[0]), int(pt[1])
                cv2.circle(flow_img, (px, py), 3, (0, 255, 0), -1)

        if bbox is not None:
            bx, by, bw, bh = [int(v) for v in bbox]
            cv2.rectangle(flow_img, (bx, by), (bx + bw, by + bh), (0, 255, 255), 2)

        self._set_label_image(self.lbl_flow, flow_img)

        # --- 2. NCC Template Heatmap ---
        if bbox is not None and pixel_engine is not None and hasattr(pixel_engine, "template") and pixel_engine.template is not None:
            tmpl = pixel_engine.template
            th, tw = tmpl.shape[:2]
            bx, by, bw, bh = [int(v) for v in bbox]
            cx, cy = bx + bw // 2, by + bh // 2
            sw, sh = int(bw * 1.8), int(bh * 1.8)
            sx = max(0, cx - sw // 2)
            sy = max(0, cy - sh // 2)
            ew = min(w - sx, sw)
            eh = min(h - sy, sh)

            if ew > tw and eh > th:
                search_crop = gray[sy : sy + eh, sx : sx + ew]
                res = cv2.matchTemplate(search_crop, tmpl, cv2.TM_CCOEFF_NORMED)
                res_norm = cv2.normalize(res, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                heatmap = cv2.applyColorMap(res_norm, cv2.COLORMAP_JET)
                self._set_label_image(self.lbl_tmpl, heatmap)
            else:
                self._set_label_text(self.lbl_tmpl, "Template Search Window Out of Bounds")
        else:
            self._set_label_text(self.lbl_tmpl, "No Target Locked (Select ROI or Auto-Lock)")

        # --- 3. Canny Edge Structure ---
        edges = cv2.Canny(gray, 80, 180)
        edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        if bbox is not None:
            bx, by, bw, bh = [int(v) for v in bbox]
            cv2.rectangle(edges_bgr, (bx, by), (bx + bw, by + bh), (0, 255, 255), 2)
        self._set_label_image(self.lbl_edge, edges_bgr)

        # --- 4. HSV Color Histogram ---
        if target_hist is not None:
            hist_img = np.zeros((240, 320, 3), dtype=np.uint8)
            hist_norm = cv2.normalize(target_hist, None, 0, 200, cv2.NORM_MINMAX).astype(np.int32)
            bin_w = 320 // 16
            for h_idx in range(16):
                for s_idx in range(16):
                    val = int(hist_norm[h_idx, s_idx])
                    if val > 0:
                        x1 = h_idx * bin_w
                        y1 = 240 - val
                        x2 = x1 + bin_w
                        y2 = 240
                        # Convert HSV bin to RGB color for visualization
                        color_hsv = np.uint8([[[int(h_idx * 180 / 16), int(s_idx * 255 / 16), 200]]])
                        color_bgr = cv2.cvtColor(color_hsv, cv2.COLOR_HSV2BGR)[0, 0].tolist()
                        cv2.rectangle(hist_img, (x1, y1), (x2, y2), color_bgr, -1)
            cv2.putText(hist_img, "Target Color Signature (HSV)", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            self._set_label_image(self.lbl_hist, hist_img)
        else:
            self._set_label_text(self.lbl_hist, "No Color Signature (Lock Target to Capture)")

    def _set_label_image(self, label: QLabel, img_bgr: np.ndarray) -> None:
        rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        bytes_per_line = ch * w
        qimg = QImage(rgb.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
        pix = QPixmap.fromImage(qimg).scaled(
            label.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation
        )
        label.setPixmap(pix)

    def _set_label_text(self, label: QLabel, text: str) -> None:
        label.setPixmap(QPixmap())
        label.setText(text)
