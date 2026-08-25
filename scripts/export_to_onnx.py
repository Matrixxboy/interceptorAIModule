"""Utility script to convert larger YOLO .pt weights to slimmed/quantized ONNX format for Radxa ZERO 3."""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODELS_DIR = ROOT / "models"


def export_model(model_spec: str, imgsz: int = 640, int8: bool = False) -> str | None:
    MODELS_DIR.mkdir(parents=True, exist_ok=True)
    path = Path(model_spec)
    
    # Check if local file or name
    if path.is_file():
        weights_src = str(path.resolve())
    elif (ROOT / path).is_file():
        weights_src = str((ROOT / path).resolve())
    elif (MODELS_DIR / path.name).is_file():
        weights_src = str((MODELS_DIR / path.name).resolve())
    else:
        # Standard model name (e.g. yolov8s.pt, yolov8m.pt)
        weights_src = model_spec

    print(f"[INFO] Exporting model '{weights_src}' to slimmed ONNX (imgsz={imgsz}, int8={int8})...")

    try:
        from ultralytics import YOLO

        model = YOLO(weights_src)
        exported_path_str = model.export(
            format="onnx",
            imgsz=imgsz,
            dynamic=False,
            simplify=True,
            int8=int8
        )
        
        exported_p = Path(exported_path_str)
        target_in_models = MODELS_DIR / exported_p.name
        
        if exported_p.resolve() != target_in_models.resolve():
            shutil.copy2(exported_p, target_in_models)
            print(f"[INFO] Copied ONNX model to: {target_in_models}")
            
        print(f"[SUCCESS] Exported ONNX model to: {target_in_models}")
        return str(target_in_models)
    except ImportError:
        print("[ERROR] 'ultralytics' package is required for export. Install with: pip install ultralytics")
    except Exception as e:
        print(f"[ERROR] Failed to export model: {e}")

    return None


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Export PyTorch YOLO (.pt) to slimmed ONNX (.onnx) for Radxa ZERO 3")
    parser.add_argument("--weights", default="yolov8s.pt", help="Path or name of .pt weights file (default: yolov8s.pt)")
    parser.add_argument("--imgsz", type=int, default=640, help="Inference image size (default: 640)")
    parser.add_argument("--int8", action="store_true", help="Enable INT8 quantization for ONNX export")
    args = parser.parse_args()

    export_model(args.weights, args.imgsz, int8=args.int8)
