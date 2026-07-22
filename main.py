"""
Arjuna — AI-Powered Autonomous Target Tracking & Drone Control Platform.

Runs the Arjuna GCS (PyQt6) by default.
Supports --cli flag for headless / OpenCV bench mode.
Supports --legacy flag for the previous main window layout.
"""

from __future__ import annotations

import argparse
import os
import sys

os.environ["OPENCV_LOG_LEVEL"] = "OFF"
os.environ["OPENCV_VIDEOINPUT_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

from paths import BUNDLE_DIR, ROOT

if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from config import SystemConfig


def run_gui(cfg: SystemConfig, legacy: bool = False) -> None:
    try:
        from PyQt6.QtWidgets import QApplication

        app = QApplication(sys.argv)
        app.setApplicationName("Arjuna")
        app.setOrganizationName("Arjuna GCS")

        if legacy:
            from gui.main_window import MainWindow
            window = MainWindow(cfg)
        else:
            from gui.arjuna_shell import ArjunaShell
            window = ArjunaShell(cfg)

        window.show()
        sys.exit(app.exec())
    except Exception as e:
        import traceback
        print(f"[WARN] PyQt6 GUI unavailable ({e}). Falling back to CLI mode.")
        traceback.print_exc()
        run_cli(cfg)


def run_cli(cfg: SystemConfig) -> None:
    import cv2
    import numpy as np
    import time
    from control.fpv_follow import FPVFollowController

    if hasattr(cv2, "setLogLevel"):
        cv2.setLogLevel(cv2.LOG_LEVEL_SILENT)

    print("=" * 80)
    print(" FPV MSP LOCK + FOLLOW (CLI Bench Mode)")
    print("=" * 80)

    cap = cv2.VideoCapture(cfg.camera.camera_index)
    if not cap.isOpened():
        cap = cv2.VideoCapture(0)

    if cap is not None and cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.camera.frame_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.camera.frame_height)

    controller = FPVFollowController(cfg)
    window_name = "FPV Autonomous Tracking (CLI Mode)"
    cv2.namedWindow(window_name)

    print("\nPress 'Q' to quit CLI mode.\n")

    failed_reads = 0
    while True:
        ok = False
        frame = None
        if cap is not None and cap.isOpened():
            ok, frame = cap.read()
            if not ok or frame is None:
                failed_reads += 1
                if failed_reads >= 5:
                    cap.release()
                    cap = None
            else:
                failed_reads = 0

        if not ok or frame is None:
            frame = np.zeros((720, 1280, 3), dtype=np.uint8)
            cv2.putText(frame, "SIMULATED STREAM (No Camera Detected)", (350, 360), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 180, 255), 2)

        cv2.imshow(window_name, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


def main() -> None:
    parser = argparse.ArgumentParser(description="Arjuna — AI Target Tracking & Drone Control GCS")
    parser.add_argument("--cli", action="store_true", help="Run in OpenCV CLI mode instead of PyQt6 GUI")
    parser.add_argument("--legacy", action="store_true", help="Use legacy main window layout")
    parser.add_argument("--config", type=str, default=None, help="Path to JSON configuration preset")
    args = parser.parse_args()

    cfg = SystemConfig()
    if args.config:
        cfg = SystemConfig.load_json(args.config)

    if args.cli:
        run_cli(cfg)
    else:
        run_gui(cfg, legacy=args.legacy)


if __name__ == "__main__":
    main()
