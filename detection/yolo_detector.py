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
        self._device = "cpu"
        self._half = False
        self._names: dict[int, str] = {i: c for i, c in enumerate(AERIAL_THREAT_CLASSES)}
        self._load()

    @property
    def device(self) -> str:
        return self._device

    @property
    def half(self) -> bool:
        return self._half

    def _resolve_weights(self) -> str:
        MODELS_DIR.mkdir(parents=True, exist_ok=True)
        
        # Primary candidate paths to check based on configuration
        candidates: list[Path] = []
        
        if self.cfg.mode == "custom" and self.cfg.custom_weights:
            p = Path(self.cfg.custom_weights)
            candidates.extend([p, ROOT / p, MODELS_DIR / p.name])
            if p.suffix == ".pt":
                candidates.extend([p.with_suffix(".onnx"), MODELS_DIR / p.with_suffix(".onnx").name])

        if self.cfg.model_path:
            p = Path(self.cfg.model_path)
            candidates.extend([p, ROOT / p, MODELS_DIR / p.name])
            if p.suffix == ".pt":
                candidates.extend([p.with_suffix(".onnx"), MODELS_DIR / p.with_suffix(".onnx").name])

        if self.cfg.model_name:
            name_p = Path(self.cfg.model_name)
            candidates.extend([ROOT / name_p, MODELS_DIR / name_p.name])
            if name_p.suffix == ".pt":
                candidates.extend([ROOT / name_p.with_suffix(".onnx"), MODELS_DIR / name_p.with_suffix(".onnx").name])

        # Standard fallback defaults
        candidates.extend([
            MODELS_DIR / "yolov8n.onnx",
            ROOT / "yolov8n.onnx",
            MODELS_DIR / "drone_missile_best.onnx",
            ROOT / "yolov8n.pt",
            MODELS_DIR / "yolov8n.pt"
        ])

        for cand in candidates:
            try:
                if cand.is_file() and cand.stat().st_size > 0:
                    return str(cand.resolve())
            except Exception:
                continue

        # Fall back to nominal target path even if missing (error caught in _load)
        return str((MODELS_DIR / "yolov8n.onnx").resolve())

    def _load(self) -> None:
        weights = self._resolve_weights()
        log.info("Loading Lightweight ONNX YOLO: %s", weights)
        try:
            self._net = cv2.dnn.readNetFromONNX(weights)
            # Use CPU backend for maximum stability on ARM Cortex-A55 without specific NPU drivers
            self._net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self._net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            self._device = "cpu"
            self._half = False
            log.info("ONNX YOLO loaded successfully on CPU (NEON optimized).")
        except Exception as exc:
            log.error("Failed to load ONNX model via cv2.dnn from %s: %s", weights, exc)

    @property
    def names(self) -> dict[int, str]:
        return self._names

    def detect(self, frame: np.ndarray) -> list[BBox]:
        boxes: list[BBox] = []
        if self._net is None:
            return boxes

        imgsz = self.cfg.imgsz
        h, w = frame.shape[:2]
        
        # Prepare DNN blob
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
        boxes = []
        # YOLOv8 ONNX output shape: (1, 84, 8400) -> transpose to (8400, 84)
        if len(output.shape) == 2 and output.shape[0] < output.shape[1]:
            output = output.T
            
        x_factor = frame_w / self.cfg.imgsz
        y_factor = frame_h / self.cfg.imgsz
        
        class_filter = self.cfg.class_filter if self.cfg.mode == "coco" else ()

        conf_thres = self.cfg.conf_threshold
        
        for row in output:
            scores = row[4:]
            _, max_score, _, max_idx = cv2.minMaxLoc(scores)
            
            if max_score >= conf_thres:
                cls_id = max_idx[1]
                if class_filter and cls_id not in class_filter:
                    continue
                    
                x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item()
                left = int((x - 0.5 * w) * x_factor)
                top = int((y - 0.5 * h) * y_factor)
                width = int(w * x_factor)
                height = int(h * y_factor)
                
                label = self._names.get(cls_id, str(cls_id))
                boxes.append(BBox(
                    x1=max(0, left), 
                    y1=max(0, top), 
                    x2=min(frame_w, left + width), 
                    y2=min(frame_h, top + height), 
                    conf=float(max_score),
                    cls_id=cls_id,
                    track_id=-1,
                    label=label
                ))
                
        # NMS
        if boxes:
            cv_boxes = [[b.x1, b.y1, b.x2-b.x1, b.y2-b.y1] for b in boxes]
            cv_scores = [b.conf for b in boxes]
            indices = cv2.dnn.NMSBoxes(cv_boxes, cv_scores, conf_thres, self.cfg.iou_threshold)
            final_boxes = []
            if len(indices) > 0:
                for i in indices.flatten():
                    final_boxes.append(boxes[i])
            return sorted(final_boxes, key=lambda b: b.conf, reverse=True)
            
        return boxes
