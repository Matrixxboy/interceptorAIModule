from abc import ABC, abstractmethod
from typing import Tuple, Any
import numpy as np

class AIModel(ABC):
    """
    Abstract base class for AI Detection Models.
    """

    @abstractmethod
    def load_model(self, model_path: str, **kwargs) -> bool:
        """Load the model weights into memory."""
        pass

    @abstractmethod
    def unload_model(self) -> None:
        """Unload the model to free resources."""
        pass

    @abstractmethod
    def predict(self, frame: np.ndarray) -> Tuple[bool, tuple, float]:
        """
        Run inference on a single frame.
        Returns:
            Tuple containing:
            - success (bool): True if target found.
            - bbox (tuple): (x, y, w, h) of target.
            - confidence (float): Detection confidence (0-1).
        """
        pass
