import importlib
import logging
import os
import sys
from typing import Dict, Any, Type
from interfaces.flight_controller import FlightController
from interfaces.ai_model import AIModel
from interfaces.tracker import Tracker

class PluginManager:
    """
    Manages loading and instantiation of plugins for flight controllers, models, and trackers.
    """
    def __init__(self):
        self.logger = logging.getLogger("PluginManager")
        self.plugins: Dict[str, Dict[str, Type]] = {
            "flight_controllers": {},
            "ai_models": {},
            "trackers": {}
        }
        
    def load_plugins(self, plugin_dir: str | Path | None = None) -> None:
        """Discover and load all plugins in the plugins directory."""
        from pathlib import Path
        from paths import PLUGINS_DIR, BUNDLE_DIR, ROOT

        if str(ROOT) not in sys.path:
            sys.path.insert(0, str(ROOT))

        dirs_to_check: list[Path] = []
        if plugin_dir is not None:
            dirs_to_check.append(Path(plugin_dir))
        else:
            dirs_to_check.append(PLUGINS_DIR)
            dirs_to_check.append(BUNDLE_DIR / "plugins")

        found_any = False
        for p_dir in dirs_to_check:
            if not p_dir.exists():
                continue
            found_any = True
            for category in self.plugins.keys():
                cat_dir = p_dir / category
                if not cat_dir.exists():
                    continue

                for filename in os.listdir(cat_dir):
                    if filename.endswith(".py") and not filename.startswith("__"):
                        module_name = filename[:-3]
                        full_module_name = f"plugins.{category}.{module_name}"
                        try:
                            module = importlib.import_module(full_module_name)
                            self._register_classes(module, category, module_name)
                        except Exception as e:
                            self.logger.error(f"Failed to load plugin {full_module_name}: {e}")

        if not found_any:
            self.logger.warning("No valid plugin directories found.")
                        
    def _register_classes(self, module, category: str, module_name: str) -> None:
        """Register valid classes from a loaded module."""
        for attr_name in dir(module):
            attr = getattr(module, attr_name)
            if isinstance(attr, type):
                if category == "flight_controllers" and issubclass(attr, FlightController) and attr is not FlightController:
                    self.plugins[category][module_name] = attr
                    self.logger.info(f"Registered FlightController: {module_name}")
                elif category == "ai_models" and issubclass(attr, AIModel) and attr is not AIModel:
                    self.plugins[category][module_name] = attr
                    self.logger.info(f"Registered AIModel: {module_name}")
                elif category == "trackers" and issubclass(attr, Tracker) and attr is not Tracker:
                    self.plugins[category][module_name] = attr
                    self.logger.info(f"Registered Tracker: {module_name}")

    def get_plugin_instance(self, category: str, plugin_name: str, **kwargs) -> Any:
        """Instantiate a requested plugin."""
        if category in self.plugins and plugin_name in self.plugins[category]:
            return self.plugins[category][plugin_name](**kwargs)
        else:
            self.logger.error(f"Plugin {plugin_name} not found in {category}")
            return None
