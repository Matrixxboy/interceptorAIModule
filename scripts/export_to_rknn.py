"""Utility script to convert ONNX models to Rockchip RKNN (.rknn) format for Radxa ZERO 3 (RK3566 NPU)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODELS_DIR = ROOT / "models"


def convert_onnx_to_rknn(
    onnx_path: str,
    target_platform: str = "rk3566",
    do_quantization: bool = False,
    dataset_path: str = ""
) -> str | None:
    path = Path(onnx_path)
    if not path.is_absolute():
        path = (MODELS_DIR / path.name) if (MODELS_DIR / path.name).is_file() else (ROOT / path)

    if not path.is_file():
        print(f"[ERROR] ONNX model '{path}' does not exist.")
        return None

    rknn_output = path.with_suffix(".rknn")
    print(f"[INFO] Converting '{path}' -> '{rknn_output}' for platform '{target_platform}'...")

    try:
        from rknn.api import RKNN

        rknn = RKNN(verbose=False)
        print("--> Config RKNN model parameters...")
        rknn.config(mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]], target_platform=target_platform)

        print("--> Loading ONNX model...")
        ret = rknn.load_onnx(model=str(path))
        if ret != 0:
            print("[ERROR] Failed to load ONNX model into RKNN Toolkit.")
            return None

        print("--> Building RKNN model...")
        ret = rknn.build(do_quantization=do_quantization, dataset=dataset_path if do_quantization else None)
        if ret != 0:
            print("[ERROR] Failed to build RKNN model.")
            return None

        print("--> Exporting RKNN model...")
        ret = rknn.export_rknn(str(rknn_output))
        if ret != 0:
            print("[ERROR] Failed to export RKNN model.")
            return None

        print(f"[SUCCESS] RKNN Model exported successfully to: {rknn_output}")
        rknn.release()
        return str(rknn_output)

    except ImportError:
        print("[WARN] 'rknn-toolkit2' Python package is not installed on host machine.")
        print("       Install 'rknn-toolkit2' or run conversion directly on Radxa ZERO 3 / Linux host.")
        print(f"       Target RKNN path will be: {rknn_output}")
    except Exception as e:
        print(f"[ERROR] Failed to convert model to RKNN: {e}")

    return None


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert ONNX to Rockchip RKNN (.rknn) for Radxa ZERO 3 (RK3566 NPU)")
    parser.add_argument("--onnx", default="models/yolov8_fast_precision.onnx", help="Path to input ONNX model")
    parser.add_argument("--target", default="rk3566", help="Target Rockchip platform (default: rk3566)")
    parser.add_argument("--quantize", action="store_true", help="Enable INT8 quantization for RKNN export")
    args = parser.parse_args()

    convert_onnx_to_rknn(args.onnx, target_platform=args.target, do_quantization=args.quantize)
