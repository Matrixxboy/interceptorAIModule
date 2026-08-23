"""Print environment diagnostics for startup, Radxa ZERO 3 hardware, ONNX models, and serial/camera nodes."""

from __future__ import annotations

import glob
import os
import platform
import sys
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def report(label: str, value) -> None:
    print(f"{label:<25} {value}", flush=True)


def section(title: str) -> None:
    print(f"\n--- {title} ---", flush=True)


section("System & Platform")
report("Python", sys.version.replace("\n", " "))
report("Executable", sys.executable)
report("OS / Platform", platform.platform())
report("Architecture", platform.machine())
report("Project Path", ROOT)

section("Radxa ZERO 3 Hardware Nodes")
# Serial UART Nodes
uart_nodes = ["/dev/ttyS2", "/dev/ttyS0", "/dev/ttyS4", "/dev/ttyFIQ0", "/dev/ttyUSB0", "/dev/ttyACM0"]
found_uarts = [p for p in uart_nodes if os.path.exists(p)]
report("FC UART Nodes", found_uarts if found_uarts else "None found (Check GPIO Pin 8/10 wiring)")

# Camera Nodes
video_nodes = glob.glob("/dev/video*")
report("Camera V4L2 Nodes", sorted(video_nodes) if video_nodes else "No /dev/video devices found")

# NPU Device Node
rknpu_node = os.path.exists("/dev/rknpu") or os.path.exists("/dev/mali0")
report("Rockchip NPU/GPU Node", "/dev/rknpu present" if os.path.exists("/dev/rknpu") else ("GPU node present" if os.path.exists("/dev/mali0") else "Standard CPU"))

section("Dependencies")
for module_name in ("numpy", "cv2", "serial", "ultralytics", "rknn"):
    started = time.perf_counter()
    try:
        module = __import__(module_name)
        version = getattr(module, "__version__", "installed")
        report(module_name, f"OK {version} ({time.perf_counter() - started:.2f}s)")
    except Exception as exc:
        report(module_name, f"NOT INSTALLED / OPTIONAL ({type(exc).__name__})")

section("Model Files")
model_paths = [
    ROOT / "models" / "yolov8n.onnx",
    ROOT / "models" / "drone_missile_best.onnx",
    ROOT / "yolov8n.onnx",
    ROOT / "yolov8n.pt",
    ROOT / "models" / "yolov8n.pt",
    ROOT / "models" / "drone_missile_best.pt",
]
for path in model_paths:
    rel = str(path.relative_to(ROOT))
    if path.is_file():
        report(rel, f"OK ({path.stat().st_size / 1_048_576:.1f} MB)")
    else:
        report(rel, "MISSING")

section("Detector Initialization")
try:
    from config import SystemConfig
    from detection.yolo_detector import YOLODetector

    cfg = SystemConfig()
    report("Configured Mode", cfg.detection.mode)
    report("Configured Model", cfg.detection.model_name)
    started = time.perf_counter()
    detector = YOLODetector(cfg.detection)
    report("Detector Status", f"OK loaded in {time.perf_counter() - started:.2f}s")
    report("Selected Backend", f"OpenCV DNN ({detector.device})")
except Exception:
    traceback.print_exc()

print("\nDIAGNOSTIC COMPLETE", flush=True)
