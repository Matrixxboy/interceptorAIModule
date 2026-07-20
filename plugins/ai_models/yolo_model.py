import numpy as np
from typing import Tuple
from interfaces.ai_model import AIModel
from detection.yolo_detector import YOLODetector
from config import DetectionConfig

class YOLOPlugin(AIModel):
    """
    YOLO Object Detection plugin wrapping the existing YOLODetector.
    """
    
    def __init__(self):
        self.detector = None
        self._is_loaded = False
        
    def load_model(self, model_path: str, **kwargs) -> bool:
        try:
            # We can create a custom config here or use default
            cfg = DetectionConfig()
            if model_path:
                cfg.model_path = model_path
                cfg.mode = "world" # Override to ensure it uses the path
            
            # Allow kwargs to override config
            if "confidence_threshold" in kwargs:
                cfg.conf_threshold = kwargs["confidence_threshold"]
                
            self.detector = YOLODetector(cfg)
            self._is_loaded = True
            return True
        except Exception as e:
            import logging
            logging.getLogger("YOLOPlugin").error(f"Failed to load YOLO model: {e}")
            self.detector = None
            self._is_loaded = False
            return False

    def unload_model(self) -> None:
        self.detector = None
        self._is_loaded = False
        import torch
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

    def predict(self, frame: np.ndarray) -> Tuple[bool, tuple, float]:
        if not self._is_loaded or self.detector is None:
            return False, (0, 0, 0, 0), 0.0
            
        bboxes = self.detector.detect(frame)
        if bboxes:
            # Get highest confidence bbox
            best_bbox = bboxes[0]
            # Convert to (x, y, w, h)
            w = best_bbox.x2 - best_bbox.x1
            h = best_bbox.y2 - best_bbox.y1
            return True, (int(best_bbox.x1), int(best_bbox.y1), int(w), int(h)), float(best_bbox.conf)
            
        return False, (0, 0, 0, 0), 0.0
