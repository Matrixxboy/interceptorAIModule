import cv2
import numpy as np
from typing import List, Dict, Any, Callable

class Filter:
    def __init__(self, name: str, enabled: bool = True, **kwargs):
        self.name = name
        self.enabled = enabled
        self.params = kwargs

    def apply(self, frame: np.ndarray) -> np.ndarray:
        raise NotImplementedError

class CLAHEFilter(Filter):
    def apply(self, frame: np.ndarray) -> np.ndarray:
        if not self.enabled:
            return frame
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l_channel, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=self.params.get('clip_limit', 2.0), 
                                tileGridSize=self.params.get('tile_grid_size', (8,8)))
        cl = clahe.apply(l_channel)
        limg = cv2.merge((cl,a,b))
        return cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)

class SharpenFilter(Filter):
    def apply(self, frame: np.ndarray) -> np.ndarray:
        if not self.enabled:
            return frame
        kernel = np.array([[0, -1, 0], 
                           [-1, 5,-1], 
                           [0, -1, 0]])
        return cv2.filter2D(frame, -1, kernel)

class BrightnessContrastFilter(Filter):
    def apply(self, frame: np.ndarray) -> np.ndarray:
        if not self.enabled:
            return frame
        alpha = self.params.get('alpha', 1.0) # Contrast [1.0-3.0]
        beta = self.params.get('beta', 0)     # Brightness [0-100]
        return cv2.convertScaleAbs(frame, alpha=alpha, beta=beta)

class VisionPipeline:
    """
    Manages a sequence of image processing filters.
    """
    def __init__(self):
        self.filters: List[Filter] = []
        self._available_filters = {
            "CLAHE": CLAHEFilter,
            "Sharpen": SharpenFilter,
            "BrightnessContrast": BrightnessContrastFilter
        }

    def add_filter(self, filter_name: str, enabled: bool = True, **kwargs):
        if filter_name in self._available_filters:
            self.filters.append(self._available_filters[filter_name](name=filter_name, enabled=enabled, **kwargs))
            
    def remove_filter(self, index: int):
        if 0 <= index < len(self.filters):
            self.filters.pop(index)
            
    def clear_filters(self):
        self.filters = []

    def set_filter_enabled(self, index: int, enabled: bool):
        if 0 <= index < len(self.filters):
            self.filters[index].enabled = enabled
            
    def process(self, frame: np.ndarray) -> np.ndarray:
        processed_frame = frame
        for f in self.filters:
            processed_frame = f.apply(processed_frame)
        return processed_frame
