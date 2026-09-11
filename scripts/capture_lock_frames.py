"""Dump camera or video frames into datasets/drone_missile/images/raw for labeling.

Examples:
  python scripts/capture_lock_frames.py --camera 0 --every 5 --max 200
  python scripts/capture_lock_frames.py --video flight.mp4 --every 10

Label the saved images (YOLO txt), then split into images/train|val + labels/train|val
and create data.yaml from data.yaml.example before training.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
OUT_DEFAULT = ROOT / "datasets" / "drone_missile" / "images" / "raw"


def main() -> None:
    parser = argparse.ArgumentParser(description="Capture frames for drone/missile dataset")
    parser.add_argument("--camera", type=int, default=None, help="Camera index")
    parser.add_argument("--video", type=str, default=None, help="Video file path")
    parser.add_argument("--out", type=str, default=str(OUT_DEFAULT))
    parser.add_argument("--every", type=int, default=5, help="Save every Nth frame")
    parser.add_argument("--max", type=int, default=300, help="Max frames to save")
    parser.add_argument("--prefix", type=str, default="cap")
    args = parser.parse_args()

    if args.camera is None and not args.video:
        raise SystemExit("Provide --camera INDEX or --video PATH")

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    if args.video:
        cap = cv2.VideoCapture(args.video)
    else:
        cap = cv2.VideoCapture(int(args.camera))
    if not cap.isOpened():
        raise SystemExit("Failed to open camera/video")

    saved = 0
    idx = 0
    print(f"Saving every {args.every} frames → {out} (max {args.max})")
    print("Press Q in the preview window to stop early.")

    while saved < args.max:
        ok, frame = cap.read()
        if not ok:
            break
        idx += 1
        if idx % max(1, args.every) != 0:
            continue
        stamp = int(time.time() * 1000)
        path = out / f"{args.prefix}_{stamp}_{saved:04d}.jpg"
        cv2.imwrite(str(path), frame)
        saved += 1
        preview = frame.copy()
        cv2.putText(
            preview,
            f"saved {saved}/{args.max}",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 220, 120),
            2,
        )
        cv2.imshow("capture_lock_frames", preview)
        if (cv2.waitKey(1) & 0xFF) in (ord("q"), ord("Q"), 27):
            break

    cap.release()
    cv2.destroyAllWindows()
    print(f"Done — wrote {saved} frames to {out}")
    print("Next: label with YOLO format, split train/val, copy data.yaml.example → data.yaml")


if __name__ == "__main__":
    main()
