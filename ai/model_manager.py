import time
import logging
import numpy as np
from typing import Tuple, Dict, Any
from core.plugin_manager import PluginManager
from interfaces.ai_model import AIModel

class ModelManager:
    """
    Manages loading, unloading, and switching AI models using the PluginManager.
    """
    def __init__(self, plugin_manager: PluginManager):
        self.plugin_manager = plugin_manager
        self.active_model: AIModel = None
        self.active_model_name: str = None
        self.logger = logging.getLogger("ModelManager")
        
        # Benchmarking
        self.latency_ms = 0.0
        self.fps = 0.0
        self._last_frame_time = time.time()

    def switch_model(self, plugin_name: str, model_path: str, **kwargs) -> bool:
        """Switch to a different AI model plugin."""
        self.logger.info(f"Switching to model plugin {plugin_name}")
        
        if self.active_model:
            self.active_model.unload_model()
            
        new_model = self.plugin_manager.get_plugin_instance("ai_models", plugin_name)
        if new_model is None:
            self.logger.error(f"Failed to instantiate model plugin {plugin_name}")
            return False
            
        if new_model.load_model(model_path, **kwargs):
            self.active_model = new_model
            self.active_model_name = plugin_name
            self.logger.info(f"Successfully loaded {plugin_name} with {model_path}")
            return True
        else:
            self.logger.error(f"Failed to load weights for {plugin_name}")
            self.active_model = None
            self.active_model_name = None
            return False

    def predict(self, frame: np.ndarray) -> Tuple[bool, tuple, float]:
        """Run prediction and update benchmarks."""
        if not self.active_model:
            return False, (0, 0, 0, 0), 0.0
            
        start_time = time.time()
        
        # Calculate FPS
        self.fps = 1.0 / (start_time - self._last_frame_time + 1e-6)
        self._last_frame_time = start_time
        
        success, bbox, conf = self.active_model.predict(frame)
        
        end_time = time.time()
        self.latency_ms = (end_time - start_time) * 1000
        
        return success, bbox, conf
        
    def get_benchmarks(self) -> Dict[str, float]:
        return {
            "fps": self.fps,
            "latency_ms": self.latency_ms
        }
