"""
Lightweight OpenCV DNN YOLO detector for 2GB RAM Linux SBCs (Radxa).
Uses ONNX models to avoid PyTorch overhead.
"""

from __future__ import annotations

from pathlib import Path
from typing import Sequence
import cv2
import numpy as np

from config import AERIAL_THREAT_CLASSES, CONFIG, DetectionConfig, MODELS_DIR, ROOT
from utils.helpers import BBox
from utils.logger import setup_logger

log = setup_logger("cuas.yolo_dnn")

class YOLODetector:
    def __init__(self, cfg: DetectionConfig | None = None) -> None:
        self.cfg = cfg or CONFIG.detection
        self._net = None
        self._rknn = None
        self._is_rknn = False
        self._device = "cpu"
        self._half = False
        self._names: dict[int, str] = {i: c for i, c in enumerate(AERIAL_THREAT_CLASSES)}
        self._load()

    @property
    def device(self) -> str:
        return "rknpu" if self._is_rknn else self._device

    @property
    def half(self) -> bool:
        return self._half

    def _resolve_weights(self) -> tuple[str, bool]:
        MODELS_DIR.mkdir(parents=True, exist_ok=True)
        
        # 1. Check for single decided RKNN model (Rockchip NPU 0.8 TOPS Acceleration)
        rknn_path = MODELS_DIR / "yolo11_fast_precision.rknn"
        if rknn_path.is_file() and rknn_path.stat().st_size > 0:
            return str(rknn_path.resolve()), True

        # 2. Check for configured model path / name if specified
        if self.cfg.model_path:
            p = Path(self.cfg.model_path)
            if not p.is_absolute():
                p = MODELS_DIR / p.name
            if p.is_file() and p.stat().st_size > 0:
                return str(p.resolve()), p.suffix == ".rknn"

        # 3. Direct single decided ONNX model
        onnx_path = MODELS_DIR / "yolo11_fast_precision.onnx"
        return str(onnx_path.resolve()), False

    def _load(self) -> None:
        weights, is_rknn = self._resolve_weights()
        
        if is_rknn or weights.endswith(".rknn"):
            log.info("Attempting to load Rockchip RKNN NPU Model: %s", weights)
            try:
                try:
                    from rknnlite.api import RKNNLite
                    rknn_cls = RKNNLite
                except ImportError:
                    from rknn.api import RKNN
                    rknn_cls = RKNN

                rknn = rknn_cls()
                ret = rknn.load_rknn(weights)
                if ret == 0:
                    ret = rknn.init_runtime(target="rk3566")
                    if ret == 0:
                        self._rknn = rknn
                        self._is_rknn = True
                        self._device = "rknpu"
                        log.info("SUCCESS: RKNN Model loaded on Rockchip RK3566 NPU (/dev/rknpu).")
                        return
            except Exception as exc:
                log.warning("RKNN NPU runtime initialization failed (%s). Falling back to OpenCV ONNX CPU engine.", exc)

        # Fallback to OpenCV DNN ONNX Engine
        if weights.endswith(".rknn"):
            weights = str(Path(weights).with_suffix(".onnx"))
            if not Path(weights).is_file():
                weights = str((MODELS_DIR / "yolov8_fast_precision.onnx").resolve())

        log.info("Loading Lightweight ONNX YOLO: %s", weights)
        try:
            self._net = cv2.dnn.readNetFromONNX(weights)
            self._net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self._net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            self._device = "cpu"
            self._half = False
            self._is_rknn = False
            log.info("ONNX YOLO loaded successfully on CPU (NEON optimized).")
        except Exception as exc:
            log.error("Failed to load ONNX model via cv2.dnn from %s: %s", weights, exc)

    @property
    def names(self) -> dict[int, str]:
        return self._names

    def detect(self, frame: np.ndarray) -> list[BBox]:
        boxes: list[BBox] = []
        imgsz = self.cfg.imgsz
        h, w = frame.shape[:2]

        # 1. RKNN Rockchip NPU Hardware Acceleration (2.5ms - 3.5ms on /dev/rknpu)
        if self._is_rknn and self._rknn is not None:
            try:
                img_rgb = cv2.cvtColor(cv2.resize(frame, (imgsz, imgsz)), cv2.COLOR_BGR2RGB)
                img_input = np.expand_dims(img_rgb, axis=0)
                outputs = self._rknn.inference(inputs=[img_input])
                if outputs and len(outputs) > 0:
                    return self._parse_outputs(outputs[0], w, h)
            except Exception as e:
                log.error("RKNN NPU inference failed: %s. Falling back to OpenCV DNN ONNX CPU engine.", e)

        # 2. OpenCV DNN ONNX Fallback (CPU NEON)
        if self._net is None:
            return boxes
        
        blob = cv2.dnn.blobFromImage(frame, 1/255.0, (imgsz, imgsz), swapRB=True, crop=False)
        self._net.setInput(blob)
        
        try:
            outputs = self._net.forward()
            boxes = self._parse_outputs(outputs[0], w, h)
        except Exception as e:
            log.error("DNN forward pass failed: %s", e)
            
        return boxes

    def track(self, frame: np.ndarray, tracker_yaml: str = "") -> list[BBox]:
        # Without Ultralytics ByteTrack, fallback to standard detect + manual IDs if needed.
        # The hybrid_tracker uses optical flow/CSRT anyway, so raw detect is sufficient.
        return self.detect(frame)

    def _parse_outputs(self, output: np.ndarray, frame_w: int, frame_h: int) -> list[BBox]:
        # YOLOv8/11 ONNX output shape: (1, 84, N) -> transpose to (N, 84)
        if len(output.shape) == 2 and output.shape[0] < output.shape[1]:
            output = output.T

        x_factor = frame_w / self.cfg.imgsz
        y_factor = frame_h / self.cfg.imgsz
        conf_thres = self.cfg.conf_threshold
        class_filter = set(self.cfg.class_filter) if (self.cfg.mode == "coco" and self.cfg.class_filter) else set()

        # Vectorized: extract max class scores and class IDs in one shot
        scores_matrix = output[:, 4:]
        max_scores = np.max(scores_matrix, axis=1)
        cls_ids = np.argmax(scores_matrix, axis=1)

        # Filter by confidence threshold
        conf_mask = max_scores >= conf_thres
        if not np.any(conf_mask):
            return []

        # Apply confidence filter
        valid_output = output[conf_mask]
        valid_scores = max_scores[conf_mask]
        valid_cls_ids = cls_ids[conf_mask]

        # Vectorized coordinate calculation
        cx = valid_output[:, 0]
        cy = valid_output[:, 1]
        w = valid_output[:, 2]
        h = valid_output[:, 3]

        lefts = ((cx - 0.5 * w) * x_factor).astype(np.int32)
        tops = ((cy - 0.5 * h) * y_factor).astype(np.int32)
        widths = (w * x_factor).astype(np.int32)
        heights = (h * y_factor).astype(np.int32)

        # Build BBox list (only target classes)
        filtered_boxes: list[BBox] = []
        for i in range(len(valid_scores)):
            cid = int(valid_cls_ids[i])
            if class_filter and cid not in class_filter:
                continue
            left = max(0, int(lefts[i]))
            top = max(0, int(tops[i]))
            label = self._names.get(cid, f"obj_{cid}")
            filtered_boxes.append(BBox(
                x1=left,
                y1=top,
                x2=min(frame_w, left + int(widths[i])),
                y2=min(frame_h, top + int(heights[i])),
                conf=float(valid_scores[i]),
                cls_id=cid,
                track_id=-1,
                label=label,
            ))

        # NMS
        if filtered_boxes:
            cv_boxes = [[b.x1, b.y1, b.x2 - b.x1, b.y2 - b.y1] for b in filtered_boxes]
            cv_scores = [b.conf for b in filtered_boxes]
            indices = cv2.dnn.NMSBoxes(cv_boxes, cv_scores, conf_thres, self.cfg.iou_threshold)
            if len(indices) > 0:
                return sorted([filtered_boxes[i] for i in indices.flatten()], key=lambda b: b.conf, reverse=True)

        return filtered_boxes
