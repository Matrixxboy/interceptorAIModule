"""Print environment diagnostics for startup, CUDA and model-loading failures."""

from __future__ import annotations

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


section("System")
report("Python", sys.version.replace("\n", " "))
report("Executable", sys.executable)
report("OS", platform.platform())
report("Project", ROOT)

section("Imports")
for module_name in ("numpy", "cv2", "PyQt6", "ultralytics", "pygame", "serial"):
    started = time.perf_counter()
    try:
        module = __import__(module_name)
        version = getattr(module, "__version__", "installed")
        report(module_name, f"OK {version} ({time.perf_counter() - started:.2f}s)")
    except Exception as exc:
        report(module_name, f"FAILED: {type(exc).__name__}: {exc}")

section("PyTorch / CUDA")
try:
    import torch

    report("torch", torch.__version__)
    report("torch CUDA build", torch.version.cuda or "CPU-only wheel")
    report("CUDA available", torch.cuda.is_available())
    report("cuDNN", torch.backends.cudnn.version())
    if torch.cuda.is_available():
        report("GPU count", torch.cuda.device_count())
        report("GPU 0", torch.cuda.get_device_name(0))
        report("Capability", torch.cuda.get_device_capability(0))
        report("Driver API", torch._C._cuda_getDriverVersion() if hasattr(torch._C, "_cuda_getDriverVersion") else "n/a")
        print("Running CUDA tensor test...", flush=True)
        started = time.perf_counter()
        x = torch.randn((1024, 1024), device="cuda", dtype=torch.float16)
        y = x @ x
        torch.cuda.synchronize()
        report("CUDA tensor test", f"OK ({time.perf_counter() - started:.2f}s, mean={y.float().mean().item():.3f})")
    else:
        report("CUDA diagnosis", "NVIDIA GPU is NOT usable by this PyTorch installation")
except Exception:
    traceback.print_exc()

section("Model files")
for path in (
    ROOT / "yolov8n.pt",
    ROOT / "models" / "yolov8n.pt",
    ROOT / "models" / "yolov8s.pt",
    ROOT / "models" / "drone_missile_best.pt",
):
    report(str(path.relative_to(ROOT)), f"{path.stat().st_size / 1_048_576:.1f} MB" if path.is_file() else "MISSING")

section("T.R.I.V.E.N.I detector")
try:
    from config import SystemConfig
    from detection.yolo_detector import YOLODetector

    cfg = SystemConfig()
    report("Configured device", cfg.detection.device)
    report("Configured FP16", cfg.detection.half)
    print("Loading detector (if output stops here, model/CUDA initialization is the fault)...", flush=True)
    started = time.perf_counter()
    detector = YOLODetector(cfg.detection)
    report("Detector load", f"OK ({time.perf_counter() - started:.2f}s)")
    report("Selected device", detector.device)
    report("Selected FP16", detector.half)
except Exception:
    traceback.print_exc()

print("\nDIAGNOSTIC COMPLETE", flush=True)
