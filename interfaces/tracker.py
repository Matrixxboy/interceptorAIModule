from abc import ABC, abstractmethod
import numpy as np
from typing import Tuple

class Tracker(ABC):
    """
    Abstract base class for Object Trackers.
    """

    @abstractmethod
    def init(self, frame: np.ndarray, bbox: tuple) -> bool:
        """Initialize the tracker with the first frame and bounding box."""
        pass

    @abstractmethod
    def update(self, frame: np.ndarray) -> Tuple[bool, tuple]:
        """
        Update the tracker on a new frame.
        Returns:
            Tuple containing:
            - success (bool): True if tracking successful.
            - bbox (tuple): (x, y, w, h) of target.
        """
        pass
        
    @abstractmethod
    def reset(self) -> None:
        """Reset internal tracker state."""
        pass
