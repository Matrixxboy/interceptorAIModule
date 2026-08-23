"""Central paths definition for T.R.I.V.E.N.I and PyInstaller bundle compatibility."""

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
        APP_DATA_DIR = Path(appdata_env) / "T.R.I.V.E.N.I"
    else:
        APP_DATA_DIR = Path.home() / "AppData" / "Roaming" / "T.R.I.V.E.N.I"
else:
    APP_DATA_DIR = Path.home() / ".config" / "T.R.I.V.E.N.I"

# Writable application paths for persistent state
PRESETS_DIR = APP_DATA_DIR / "presets"
LOGS_DIR = APP_DATA_DIR / "logs"
DATA_DIR = APP_DATA_DIR / "data"
CONFIGS_DIR = APP_DATA_DIR / "config"
PLUGINS_DIR = APP_DATA_DIR / "plugins"

# Read-only bundled asset paths
MODELS_DIR = BUNDLE_DIR / "models"
CONFIG_FILE = APP_DATA_DIR / "config.json"
DEFAULT_CONFIG_FILE = BUNDLE_DIR / "config.json"

# Ensure all writable directories exist
for _dir in (APP_DATA_DIR, PRESETS_DIR, LOGS_DIR, DATA_DIR, CONFIGS_DIR, PLUGINS_DIR):
    _dir.mkdir(parents=True, exist_ok=True)
