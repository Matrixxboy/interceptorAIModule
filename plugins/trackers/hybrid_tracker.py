import numpy as np
from typing import Tuple
from interfaces.tracker import Tracker
from detection.hybrid_tracker import HybridYoloLockTracker
from config import TrackerConfig

class HybridTrackerPlugin(Tracker):
    """
    Hybrid Tracker plugin wrapping the existing HybridYoloLockTracker.
    """
    
    def __init__(self):
        self.tracker = HybridYoloLockTracker()
        
    def init(self, frame: np.ndarray, bbox: tuple) -> bool:
        """
        Initialize the tracker with a bounding box (x, y, w, h).
        """
        if len(bbox) != 4:
            return False
            
        x, y, w, h = bbox
        return self.tracker.lock_xywh(frame, (x, y, w, h), label="manual")

    def update(self, frame: np.ndarray) -> Tuple[bool, tuple]:
        """
        Update the tracker on a new frame.
        """
        result = self.tracker.update(frame)
        if result.ok and result.bbox_xywh is not None:
            return True, result.bbox_xywh
        return False, (0, 0, 0, 0)
        
    def reset(self) -> None:
        """
        Reset internal tracker state.
        """
        self.tracker.reset()
