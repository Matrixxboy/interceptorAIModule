"""Utility script to convert YOLO .pt weights to ONNX format for Radxa ZERO 3."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def export_model(weights_path: str, imgsz: int = 640) -> str | None:
    path = Path(weights_path)
    if not path.is_absolute():
        path = ROOT / path

    if not path.is_file():
        print(f"[ERROR] Input weights file '{path}' does not exist.")
        return None

    print(f"[INFO] Exporting '{path}' to ONNX (imgsz={imgsz})...")

    try:
        from ultralytics import YOLO

        model = YOLO(str(path))
        exported_path = model.export(format="onnx", imgsz=imgsz, dynamic=False, simplify=True)
        print(f"[SUCCESS] Exported ONNX model to: {exported_path}")
        return exported_path
    except ImportError:
        print("[ERROR] 'ultralytics' package is required for export. Install with: pip install ultralytics")
    except Exception as e:
        print(f"[ERROR] Failed to export model: {e}")

    return None


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Export PyTorch YOLO (.pt) to ONNX (.onnx) for Radxa ZERO 3")
    parser.add_argument("--weights", default="models/drone_missile_best.pt", help="Path to input .pt weights file")
    parser.add_argument("--imgsz", type=int, default=640, help="Inference image size (default: 640)")
    args = parser.parse_args()

    export_model(args.weights, args.imgsz)
