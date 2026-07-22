import os
import json
import logging
from typing import Dict, Any

class ConfigManager:
    """
    Centralized configuration management for the framework.
    Supports loading, saving, and managing configuration profiles.
    """
    
    def __init__(self, config_dir: str | None = None):
        from paths import CONFIGS_DIR
        self.config_dir = str(CONFIGS_DIR) if config_dir is None or config_dir == "config" else config_dir
        self.active_profile_name = "default"
        self.config: Dict[str, Any] = {}
        self.logger = logging.getLogger("ConfigManager")
        
        if not os.path.exists(self.config_dir):
            os.makedirs(self.config_dir)
            
    def load_profile(self, profile_name: str = "default") -> bool:
        """Load a configuration profile."""
        filepath = os.path.join(self.config_dir, f"{profile_name}.json")
        if os.path.exists(filepath):
            try:
                with open(filepath, 'r') as f:
                    self.config = json.load(f)
                self.active_profile_name = profile_name
                self.logger.info(f"Loaded profile: {profile_name}")
                return True
            except Exception as e:
                self.logger.error(f"Failed to load profile {profile_name}: {e}")
                return False
        else:
            self.logger.warning(f"Profile {profile_name} not found. Creating default.")
            self.config = self._get_default_config()
            self.active_profile_name = profile_name
            self.save_profile(profile_name)
            return True
            
    def save_profile(self, profile_name: str = None) -> bool:
        """Save the current configuration to a profile."""
        if profile_name is None:
            profile_name = self.active_profile_name
            
        filepath = os.path.join(self.config_dir, f"{profile_name}.json")
        try:
            with open(filepath, 'w') as f:
                json.dump(self.config, f, indent=4)
            self.logger.info(f"Saved profile: {profile_name}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to save profile {profile_name}: {e}")
            return False

    def get(self, section: str, key: str = None, default=None) -> Any:
        """Get a configuration value."""
        if section not in self.config:
            return default
        if key is None:
            return self.config[section]
        return self.config[section].get(key, default)
        
    def set(self, section: str, key: str, value: Any) -> None:
        """Set a configuration value."""
        if section not in self.config:
            self.config[section] = {}
        self.config[section][key] = value
        
    def _get_default_config(self) -> Dict[str, Any]:
        """Generate a default configuration."""
        return {
            "flight_controller": {
                "type": "msp",
                "port": "COM4",
                "baudrate": 115200
            },
            "ai_model": {
                "type": "yolo",
                "model_path": "yolov8n.pt",
                "confidence_threshold": 0.5
            },
            "tracker": {
                "type": "hybrid"
            },
            "pid": {
                "yaw_p": 0.1,
                "yaw_i": 0.0,
                "yaw_d": 0.0,
                "pitch_p": 0.1,
                "pitch_i": 0.0,
                "pitch_d": 0.0
            }
        }
