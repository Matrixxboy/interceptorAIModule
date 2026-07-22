"""Central paths definition for Arjuna GCS and PyInstaller bundle compatibility."""

from __future__ import annotations

import os
import sys
from pathlib import Path

# Detect PyInstaller frozen binary execution context
IS_FROZEN = getattr(sys, "frozen", False)

if IS_FROZEN:
    BUNDLE_DIR = Path(sys._MEIPASS)
else:
    BUNDLE_DIR = Path(__file__).resolve().parent

ROOT = BUNDLE_DIR

# Define persistent user data directory (AppData on Windows)
if sys.platform == "win32":
    appdata_env = os.environ.get("APPDATA")
    if appdata_env:
        APP_DATA_DIR = Path(appdata_env) / "ArjunaGCS"
    else:
        APP_DATA_DIR = Path.home() / "AppData" / "Roaming" / "ArjunaGCS"
else:
    APP_DATA_DIR = Path.home() / ".config" / "ArjunaGCS"

# Writable application paths for persistent state
PRESETS_DIR = APP_DATA_DIR / "presets"
LOGS_DIR = APP_DATA_DIR / "logs"
DATA_DIR = APP_DATA_DIR / "data"
CONFIGS_DIR = APP_DATA_DIR / "config"
PLUGINS_DIR = APP_DATA_DIR / "plugins"

# Read-only bundled asset paths
MODELS_DIR = BUNDLE_DIR / "models"
CALIBRATION_FILE = APP_DATA_DIR / "calibration.json"
DEFAULT_CALIBRATION_FILE = BUNDLE_DIR / "calibration.json"

# Ensure all writable directories exist
for _dir in (APP_DATA_DIR, PRESETS_DIR, LOGS_DIR, DATA_DIR, CONFIGS_DIR, PLUGINS_DIR):
    _dir.mkdir(parents=True, exist_ok=True)
