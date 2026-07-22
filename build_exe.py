"""
Build script to package Arjuna GCS into a standalone graphical executable using PyInstaller.
"""

from __future__ import annotations

import sys
import subprocess
from pathlib import Path
from paths import ROOT, MODELS_DIR, CALIBRATION_FILE, DEFAULT_CALIBRATION_FILE


def ensure_pyinstaller() -> None:
    try:
        import PyInstaller  # type: ignore # noqa: F401
    except ImportError:
        print("PyInstaller not found in environment. Installing pyinstaller...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "pyinstaller"])


def build() -> None:
    ensure_pyinstaller()

    print("Building Arjuna GCS Graphical Executable (PyInstaller)...")

    # Data separator for PyInstaller (';' on Windows, ':' on Unix)
    sep = ";" if sys.platform == "win32" else ":"

    add_data_args: list[str] = []

    # Include models directory if present
    if MODELS_DIR.exists():
        add_data_args.extend(["--add-data", f"{MODELS_DIR}{sep}models"])

    # Include calibration.json if present
    if DEFAULT_CALIBRATION_FILE.exists():
        add_data_args.extend(["--add-data", f"{DEFAULT_CALIBRATION_FILE}{sep}."])
    elif CALIBRATION_FILE.exists():
        add_data_args.extend(["--add-data", f"{CALIBRATION_FILE}{sep}."])

    # Include plugins directory if present
    plugins_dir = ROOT / "plugins"
    if plugins_dir.exists():
        add_data_args.extend(["--add-data", f"{plugins_dir}{sep}plugins"])

    # Include interfaces directory if present
    interfaces_dir = ROOT / "interfaces"
    if interfaces_dir.exists():
        add_data_args.extend(["--add-data", f"{interfaces_dir}{sep}interfaces"])

    hidden_imports = [
        "--hidden-import=PyQt6",
        "--hidden-import=PyQt6.QtCore",
        "--hidden-import=PyQt6.QtGui",
        "--hidden-import=PyQt6.QtWidgets",
        "--hidden-import=cv2",
        "--hidden-import=ultralytics",
        "--hidden-import=serial",
        "--hidden-import=serial.tools",
        "--hidden-import=serial.tools.list_ports",
        "--hidden-import=pymavlink",
        "--hidden-import=pymavlink.mavutil",
        "--hidden-import=pymavlink.dialects",
        "--hidden-import=pymavlink.dialects.v20",
        "--hidden-import=pymavlink.dialects.v20.common",
        "--hidden-import=pymavlink.dialects.v20.ardupilotmega",
        "--hidden-import=pymavlink.dialects.v10",
        "--hidden-import=pymavlink.dialects.v10.common",
    ]

    cmd = [
        sys.executable,
        "-m",
        "PyInstaller",
        "--name=Arjuna",
        "--noconfirm",
        "--clean",
        "--windowed",  # Full graphical mode (hides console window)
        "--onedir",
        "--collect-all=pymavlink",
        *hidden_imports,
        *add_data_args,
        str(ROOT / "main.py"),
    ]

    print(f"Executing: {' '.join(cmd)}")
    res = subprocess.run(cmd, cwd=str(ROOT))
    if res.returncode == 0:
        print("\nBuild completed successfully!")
        print(f"Graphical Executable output directory: {ROOT / 'dist' / 'Arjuna'}")
    else:
        print(f"\nBuild failed with exit code {res.returncode}")


if __name__ == "__main__":
    build()
