"""
Fine-tune YOLO on a drone/missile dataset (best long-term accuracy).

Usage:
  1. Put a Ultralytics-format dataset under datasets/drone_missile/
     (images/train, images/val, labels/train, labels/val + data.yaml)
  2. Edit data.yaml class names, e.g.:
       names: {0: drone, 1: missile, 2: aircraft}
  3. Run:
       python scripts/train_drone_missile.py

Output weights: models/drone_missile_best.pt
Then set in config.py:
  DetectionConfig.mode = "custom"
"""

from __future__ import annotations

import argparse
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    parser = argparse.ArgumentParser(description="Train drone/missile YOLO detector")
    parser.add_argument(
        "--data",
        type=str,
        default=str(ROOT / "datasets" / "drone_missile" / "data.yaml"),
        help="Path to Ultralytics data.yaml",
    )
    parser.add_argument(
        "--base",
        type=str,
        default="yolov8s.pt",
        help="Base pretrained weights",
    )
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=960)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--device", type=str, default="0")
    args = parser.parse_args()

    data = Path(args.data)
    if not data.is_file():
        raise SystemExit(
            f"Dataset yaml not found: {data}\n"
            "Create datasets/drone_missile/data.yaml first "
            "(see datasets/drone_missile/data.yaml.example)."
        )

    from ultralytics import YOLO

    model = YOLO(args.base)
    results = model.train(
        data=str(data),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        project=str(ROOT / "runs" / "detect"),
        name="drone_missile",
        exist_ok=True,
        patience=30,
        # Small-object friendly augmentations
        mosaic=1.0,
        close_mosaic=15,
        degrees=10.0,
        scale=0.6,
        fliplr=0.5,
        hsv_h=0.015,
        hsv_s=0.7,
        hsv_v=0.4,
    )

    best = Path(results.save_dir) / "weights" / "best.pt"
    out = ROOT / "models" / "drone_missile_best.pt"
    out.parent.mkdir(parents=True, exist_ok=True)
    if best.is_file():
        out.write_bytes(best.read_bytes())
        print(f"Copied best weights → {out}")
        print('Set DetectionConfig.mode = "custom" in config.py to use them.')
    else:
        print(f"Training finished but best.pt not found at {best}")


if __name__ == "__main__":
    main()
