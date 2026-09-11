"""Run custom aerial weights and COCO vehicle classes, then fuse boxes."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import numpy as np

from config import (
    COCO_VEHICLE_CLASS_IDS,
    CONFIG,
    DetectionConfig,
)
from detection.yolo_detector import YOLODetector
from utils.helpers import BBox
from utils.logger import setup_logger

log = setup_logger("cuas.dual")


def _nms(boxes: list[BBox], iou_thr: float = 0.55) -> list[BBox]:
    if len(boxes) <= 1:
        return boxes
    ordered = sorted(boxes, key=lambda b: b.conf, reverse=True)
    keep: list[BBox] = []
    for box in ordered:
        drop = False
        for kept in keep:
            same_family = (box.family or "") == (kept.family or "") or not box.family or not kept.family
            iou = box.iou(kept)
            if same_family and iou >= iou_thr:
                drop = True
                break
            if not same_family and iou >= 0.72:
                drop = True
                break
        if not drop:
            keep.append(box)
    return keep


class DualDetector:
    """Aerial expert (custom / world) + ground expert (COCO vehicles)."""

    def __init__(self, cfg: DetectionConfig | None = None) -> None:
        self.cfg = cfg or CONFIG.detection
        self.aerial: YOLODetector | None = None
        self.ground: YOLODetector | None = None
        self._error: str | None = None

    def apply_config(self, cfg: DetectionConfig) -> None:
        prev_custom = str(getattr(self.cfg, "custom_weights", ""))
        prev_mode = getattr(self.cfg, "mode", None)
        self.cfg = cfg
        if prev_custom != str(cfg.custom_weights) or prev_mode != cfg.mode:
            self.aerial = None
            self.ground = None
            self._error = None

    def ensure(self) -> None:
        if self._error is not None:
            raise RuntimeError(self._error)
        if self.aerial is None:
            try:
                aerial_cfg = self._aerial_cfg()
                log.info("Loading aerial detector mode=%s", aerial_cfg.mode)
                self.aerial = YOLODetector(aerial_cfg, family="aerial")
            except Exception as exc:
                log.warning("Aerial detector unavailable: %s", exc)
                self.aerial = None
        if self.ground is None:
            try:
                ground_cfg = self._ground_cfg()
                log.info("Loading ground/COCO vehicle detector")
                self.ground = YOLODetector(ground_cfg, family="ground")
            except Exception as exc:
                log.warning("Ground detector unavailable: %s", exc)
                self.ground = None
        if self.aerial is None and self.ground is None:
            self._error = "YOLO unavailable: neither aerial nor ground detector loaded"
            raise RuntimeError(self._error)

    def _aerial_cfg(self) -> DetectionConfig:
        custom = Path(self.cfg.custom_weights)
        mode = self.cfg.mode
        if mode == "coco" and custom.is_file():
            mode = "custom"
        elif mode == "custom" and not custom.is_file():
            mode = "coco"
        conf = float(getattr(self.cfg, "aerial_conf", self.cfg.conf_threshold))
        class_filter: tuple[int, ...] = ()
        if mode == "coco":
            class_filter = tuple(self.cfg.class_filter or ())
        return replace(
            self.cfg,
            mode=mode,
            conf_threshold=conf,
            class_filter=class_filter,
            max_box_area_frac=float(self.cfg.max_box_area_frac),
        )

    def _ground_cfg(self) -> DetectionConfig:
        veh = tuple(getattr(self.cfg, "vehicle_class_filter", None) or COCO_VEHICLE_CLASS_IDS)
        conf = float(getattr(self.cfg, "vehicle_conf", 0.22))
        max_frac = float(getattr(self.cfg, "ground_max_box_area_frac", 0.85))
        return replace(
            self.cfg,
            mode="coco",
            conf_threshold=conf,
            class_filter=veh,
            max_box_area_frac=max_frac,
        )

    def detect(
        self,
        frame: np.ndarray,
        families: tuple[str, ...] | None = None,
        allow_aerial_crop: bool = False,
    ) -> list[BBox]:
        self.ensure()
        want = set(families or ("aerial", "ground"))
        boxes: list[BBox] = []
        h, w = frame.shape[:2]

        if "aerial" in want and self.aerial is not None:
            aerial = self.aerial.detect(frame)
            if allow_aerial_crop and not aerial:
                aerial = self._aerial_crop_pass(frame)
            boxes.extend(aerial)

        if "ground" in want and self.ground is not None:
            ground = self.ground.detect(frame)
            boxes.extend(self._filter_ground_priors(ground, w, h))

        merged = _nms(boxes, iou_thr=float(self.cfg.iou_threshold or 0.45))
        merged.sort(key=lambda b: b.conf, reverse=True)
        return merged

    def _filter_ground_priors(self, boxes: list[BBox], frame_w: int, frame_h: int) -> list[BBox]:
        min_cy = float(getattr(self.cfg, "ground_min_cy_frac", 0.30)) * float(frame_h)
        min_frac = max(float(self.cfg.min_box_area_frac), 0.0008)
        area = float(max(1, frame_w * frame_h))
        kept: list[BBox] = []
        for b in boxes:
            if b.cy < min_cy:
                continue
            if (b.area / area) < min_frac:
                continue
            kept.append(b)
        return kept

    def _aerial_crop_pass(self, frame: np.ndarray) -> list[BBox]:
        """Second pass on a center/upper crop for small distant drones."""
        if self.aerial is None:
            return []
        h, w = frame.shape[:2]
        x0, y0 = int(w * 0.15), int(h * 0.05)
        x1, y1 = int(w * 0.85), int(h * 0.72)
        crop = frame[y0:y1, x0:x1]
        if crop.size == 0:
            return []
        dets = self.aerial.detect(crop)
        shifted: list[BBox] = []
        for b in dets:
            shifted.append(
                BBox(
                    x1=b.x1 + x0,
                    y1=b.y1 + y0,
                    x2=b.x2 + x0,
                    y2=b.y2 + y0,
                    conf=b.conf,
                    cls_id=b.cls_id,
                    track_id=b.track_id,
                    label=b.label,
                    family="aerial",
                )
            )
        return shifted
