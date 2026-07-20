"""Telemetry recorder and CSV/JSON log exporter."""

from __future__ import annotations

import csv
import json
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

LOGS_DIR = Path(__file__).resolve().parent.parent / "logs"
LOGS_DIR.mkdir(exist_ok=True, parents=True)


@dataclass
class TelemetryRecord:
    timestamp: float
    frame_idx: int
    locked: bool
    confidence: float
    source: str
    error_x: float
    error_y: float
    bbox_x: float
    bbox_y: float
    bbox_w: float
    bbox_h: float
    estimated_distance_m: float
    velocity_x: float
    velocity_y: float
    rc_roll: int
    rc_pitch: int
    rc_yaw: int
    rc_throttle: int
    failsafe_status: str


class TelemetryLogger:
    def __init__(self, max_buffer: int = 5000) -> None:
        self.max_buffer = max_buffer
        self.buffer: list[TelemetryRecord] = []
        self.recording = False
        self.start_time = 0.0

    def start_recording(self) -> None:
        self.buffer.clear()
        self.recording = True
        self.start_time = time.time()

    def stop_recording(self) -> None:
        self.recording = False

    def log(
        self,
        frame_idx: int,
        locked: bool,
        confidence: float,
        source: str,
        error_x: float,
        error_y: float,
        bbox_xywh: tuple[float, float, float, float] | None,
        distance_m: float,
        vx: float,
        vy: float,
        roll: int,
        pitch: int,
        yaw: int,
        throttle: int,
        failsafe: str,
    ) -> TelemetryRecord:
        bx, by, bw, bh = bbox_xywh if bbox_xywh else (0.0, 0.0, 0.0, 0.0)
        rec = TelemetryRecord(
            timestamp=time.time() - (self.start_time if self.start_time > 0 else time.time()),
            frame_idx=frame_idx,
            locked=locked,
            confidence=confidence,
            source=source,
            error_x=error_x,
            error_y=error_y,
            bbox_x=bx,
            bbox_y=by,
            bbox_w=bw,
            bbox_h=bh,
            estimated_distance_m=distance_m,
            velocity_x=vx,
            velocity_y=vy,
            rc_roll=roll,
            rc_pitch=pitch,
            rc_yaw=yaw,
            rc_throttle=throttle,
            failsafe_status=failsafe,
        )

        if self.recording:
            self.buffer.append(rec)
            if len(self.buffer) > self.max_buffer:
                self.buffer.pop(0)

        return rec

    def export_csv(self, filepath: str | Path | None = None) -> Path:
        if filepath is None:
            filename = f"telemetry_{int(time.time())}.csv"
            filepath = LOGS_DIR / filename
        else:
            filepath = Path(filepath)

        filepath.parent.mkdir(parents=True, exist_ok=True)
        if not self.buffer:
            with open(filepath, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(list(TelemetryRecord.__dataclass_fields__.keys()))
            return filepath

        with open(filepath, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(asdict(self.buffer[0]).keys()))
            writer.writeheader()
            for r in self.buffer:
                writer.writerow(asdict(r))

        return filepath

    def export_json(self, filepath: str | Path | None = None) -> Path:
        if filepath is None:
            filename = f"telemetry_{int(time.time())}.json"
            filepath = LOGS_DIR / filename
        else:
            filepath = Path(filepath)

        filepath.parent.mkdir(parents=True, exist_ok=True)
        data = [asdict(r) for r in self.buffer]
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

        return filepath
