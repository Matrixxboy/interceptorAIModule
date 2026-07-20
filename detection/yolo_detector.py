"""
GPU-accelerated YOLO detector optimized for drones & missiles.

Modes (config.detection.mode):
  world  — YOLO-World open vocabulary (prompt: drone, missile, UAV, …)
  coco   — standard COCO YOLO (weak proxy: airplane/bird/kite)
  custom — fine-tuned weights in models/drone_missile_best.pt

Uses CUDA + FP16 on GTX 1650 when available; falls back to CPU automatically.
"""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

import numpy as np

from config import AERIAL_THREAT_CLASSES, CONFIG, DetectionConfig, MODELS_DIR, ROOT
from utils.helpers import BBox, select_torch_device
from utils.logger import setup_logger

log = setup_logger("cuas.yolo")


class YOLODetector:
    def __init__(self, cfg: DetectionConfig | None = None) -> None:
        self.cfg = cfg or CONFIG.detection
        self.device = select_torch_device(self.cfg.device)
        self.half = bool(self.cfg.half and self.device == "cuda")
        self._model = None
        self._names: dict[int, str] = {}
        self._is_world = False
        self._load()

    def _resolve_weights(self) -> str:
        mode = self.cfg.mode
        MODELS_DIR.mkdir(parents=True, exist_ok=True)

        if mode == "custom":
            custom = Path(self.cfg.custom_weights)
            if custom.is_file():
                return str(custom)
            log.warning(
                "Custom weights missing at %s — falling back to YOLO-World",
                custom,
            )
            mode = "world"

        if mode == "coco":
            for candidate in (
                ROOT / "yolov8n.pt",
                MODELS_DIR / "yolov8n.pt",
                MODELS_DIR / "yolov8s.pt",
                ROOT / "yolov8s.pt",
            ):
                if candidate.is_file():
                    return str(candidate)
            return "yolov8n.pt"

        # world (default)
        path = Path(self.cfg.model_path)
        if path.is_file():
            return str(path)
        name = self.cfg.model_name
        if "world" not in name.lower():
            name = "yolov8s-world.pt"
        return name

    def _load(self) -> None:
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise ImportError(
                "ultralytics is required. Install with: pip install ultralytics"
            ) from exc

        weights = self._resolve_weights()
        self._is_world = "world" in Path(weights).name.lower() or self.cfg.mode == "world"
        log.info(
            "Loading YOLO mode=%s weights=%s device=%s half=%s imgsz=%d",
            self.cfg.mode,
            weights,
            self.device,
            self.half,
            self.cfg.imgsz,
        )
        self._model = YOLO(weights)

        if self._is_world:
            prompts = list(self.cfg.world_classes or AERIAL_THREAT_CLASSES)
            # Empty string as background can improve open-vocab precision
            if "" not in prompts:
                prompts = prompts + [""]
            try:
                self._model.set_classes(prompts)
                log.info("YOLO-World aerial prompts: %s", [p for p in prompts if p])
            except Exception as exc:  # noqa: BLE001
                log.warning("set_classes failed (%s) — using model defaults", exc)

        try:
            self._model.to(self.device)
            if self.half:
                self._model.model.half()
        except Exception as exc:  # noqa: BLE001
            log.warning("GPU/half setup failed (%s) — using defaults", exc)
            self.device = "cpu"
            self.half = False

        names = getattr(self._model, "names", {}) or {}
        if isinstance(names, dict):
            self._names = {int(k): str(v) for k, v in names.items() if str(v).strip()}
        else:
            self._names = {i: str(n) for i, n in enumerate(names) if str(n).strip()}
        log.info("YOLO ready (%d classes): %s", len(self._names), list(self._names.values())[:12])

    @property
    def names(self) -> dict[int, str]:
        return self._names

    def _infer_kwargs(self) -> dict:
        kwargs = dict(
            imgsz=self.cfg.imgsz,
            conf=self.cfg.conf_threshold,
            iou=self.cfg.iou_threshold,
            max_det=self.cfg.max_det,
            device=self.device,
            augment=self.cfg.augment,
            verbose=False,
        )
        return kwargs

    def detect(self, frame: np.ndarray) -> list[BBox]:
        assert self._model is not None
        try:
            import torch
            with torch.inference_mode():
                results = self._model.predict(
                    source=frame,
                    stream=False,
                    **self._infer_kwargs(),
                )
        except ImportError:
            results = self._model.predict(
                source=frame,
                stream=False,
                **self._infer_kwargs(),
            )
        return self._parse(results, frame.shape)

    def track(self, frame: np.ndarray, tracker_yaml: str = "bytetrack.yaml") -> list[BBox]:
        """YOLO + ByteTrack / BoT-SORT in one call (GPU)."""
        assert self._model is not None
        results = self._model.track(
            source=frame,
            tracker=tracker_yaml,
            persist=True,
            **self._infer_kwargs(),
        )
        return self._parse(results, frame.shape, with_id=True)

    def _parse(
        self,
        results,
        frame_shape: tuple[int, ...] | None = None,
        with_id: bool = False,
    ) -> list[BBox]:
        boxes: list[BBox] = []
        if not results:
            return boxes
        r0 = results[0]
        if r0.boxes is None or len(r0.boxes) == 0:
            return boxes

        h = w = None
        if frame_shape is not None:
            h, w = frame_shape[:2]
            frame_area = float(h * w)
        else:
            frame_area = None

        xyxy = r0.boxes.xyxy.detach().cpu().numpy()
        confs = r0.boxes.conf.detach().cpu().numpy()
        clss = r0.boxes.cls.detach().cpu().numpy().astype(int)
        ids = None
        if with_id and r0.boxes.id is not None:
            ids = r0.boxes.id.detach().cpu().numpy().astype(int)

        class_filter: Sequence[int] = ()
        if self.cfg.mode == "coco":
            class_filter = self.cfg.class_filter

        for i in range(len(xyxy)):
            cls_id = int(clss[i])
            label = self._names.get(cls_id, str(cls_id)).strip()
            if not label:
                continue  # skip background prompt hits
            if class_filter and cls_id not in class_filter:
                continue

            x1, y1, x2, y2 = map(float, xyxy[i])
            bw, bh = max(0.0, x2 - x1), max(0.0, y2 - y1)
            area = bw * bh
            if frame_area and frame_area > 0:
                frac = area / frame_area
                if frac < self.cfg.min_box_area_frac or frac > self.cfg.max_box_area_frac:
                    continue

            # Prefer aerial labels; still keep if world returned a threat synonym
            tid = int(ids[i]) if ids is not None else -1
            boxes.append(
                BBox(
                    x1=x1,
                    y1=y1,
                    x2=x2,
                    y2=y2,
                    conf=float(confs[i]),
                    cls_id=cls_id,
                    track_id=tid,
                    label=label,
                )
            )

        # Prefer higher confidence first (helps lock click / HUD)
        boxes.sort(key=lambda b: b.conf, reverse=True)
        return boxes
