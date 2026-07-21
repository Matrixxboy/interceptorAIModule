"""Persistent target database with image and evidence storage."""

from __future__ import annotations

import json
import shutil
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from database.target_profile import TargetProfile, TargetStatus

DATA_DIR = Path(__file__).resolve().parent.parent / "data"
TARGETS_DIR = DATA_DIR / "targets"
TARGETS_DIR.mkdir(parents=True, exist_ok=True)


class TargetStore:
    """Centralized target database for inspection, replay, and export."""

    def __init__(self, base_dir: Path | None = None) -> None:
        self.base_dir = base_dir or TARGETS_DIR
        self.base_dir.mkdir(parents=True, exist_ok=True)
        self._index_path = self.base_dir / "index.json"
        self._targets: dict[str, TargetProfile] = {}
        self._load_index()

    def _load_index(self) -> None:
        if not self._index_path.exists():
            self._save_index()
            return
        try:
            with open(self._index_path, encoding="utf-8") as f:
                index = json.load(f)
            for tid, meta_path in index.get("targets", {}).items():
                meta_file = self.base_dir / meta_path
                if meta_file.exists():
                    with open(meta_file, encoding="utf-8") as mf:
                        self._targets[tid] = TargetProfile.from_dict(json.load(mf))
        except (json.JSONDecodeError, OSError, TypeError, KeyError):
            self._targets = {}

    def _save_index(self) -> None:
        index = {
            "updated": time.time(),
            "targets": {tid: f"{tid}/profile.json" for tid in self._targets},
        }
        with open(self._index_path, "w", encoding="utf-8") as f:
            json.dump(index, f, indent=2)

    def _target_dir(self, target_id: str) -> Path:
        d = self.base_dir / target_id
        d.mkdir(parents=True, exist_ok=True)
        (d / "images").mkdir(exist_ok=True)
        (d / "snapshots").mkdir(exist_ok=True)
        return d

    def create_target(self, label: str = "unknown") -> TargetProfile:
        profile = TargetProfile(label=label, status=TargetStatus.DETECTED)
        profile.add_event("Target Detected", system_response="Detection pipeline active")
        self._targets[profile.target_id] = profile
        self._persist(profile)
        return profile

    def get(self, target_id: str) -> TargetProfile | None:
        return self._targets.get(target_id)

    def list_targets(
        self,
        status: TargetStatus | None = None,
        min_confidence: float = 0.0,
        search_id: str = "",
    ) -> list[TargetProfile]:
        results = list(self._targets.values())
        if status:
            results = [t for t in results if t.status == status]
        if min_confidence > 0:
            results = [t for t in results if t.confidence >= min_confidence]
        if search_id:
            q = search_id.upper()
            results = [t for t in results if q in t.target_id.upper()]
        return sorted(results, key=lambda t: t.detection_time, reverse=True)

    @staticmethod
    def crop_bbox(
        frame_bgr: np.ndarray,
        bbox_xywh: tuple[int, int, int, int] | tuple[float, float, float, float],
        pad_frac: float = 0.0,
    ) -> np.ndarray | None:
        """Extract only the pixels inside the target bounding box."""
        if frame_bgr is None or frame_bgr.size == 0:
            return None
        fh, fw = frame_bgr.shape[:2]
        x, y, w, h = (float(v) for v in bbox_xywh)
        if w < 2 or h < 2:
            return None

        pad_x = w * max(0.0, pad_frac)
        pad_y = h * max(0.0, pad_frac)
        x1 = int(max(0, np.floor(x - pad_x)))
        y1 = int(max(0, np.floor(y - pad_y)))
        x2 = int(min(fw, np.ceil(x + w + pad_x)))
        y2 = int(min(fh, np.ceil(y + h + pad_y)))
        if x2 <= x1 or y2 <= y1:
            return None
        return frame_bgr[y1:y2, x1:x2].copy()

    def lock_target(
        self,
        profile: TargetProfile,
        frame_bgr: np.ndarray,
        bbox_xywh: tuple[int, int, int, int],
        source: str = "manual",
    ) -> None:
        profile.status = TargetStatus.LOCKED
        profile.lock_time = time.time()
        profile.tracking_source = source
        profile.add_event(
            "Target Locked",
            confidence=profile.confidence,
            system_response=f"Lock acquired via {source}",
        )
        self._save_lock_images(profile, frame_bgr, bbox_xywh)
        self._persist(profile)

    def _save_lock_images(
        self,
        profile: TargetProfile,
        frame_bgr: np.ndarray,
        bbox_xywh: tuple[int, int, int, int],
    ) -> None:
        """Store only the target region inside the lock box (precise crop)."""
        tdir = self._target_dir(profile.target_id)
        img_dir = tdir / "images"

        crop = self.crop_bbox(frame_bgr, bbox_xywh, pad_frac=0.0)
        if crop is None:
            return

        # Primary lock image = exact box contents only
        lock_path = img_dir / "initial_lock.jpg"
        cv2.imwrite(str(lock_path), crop)
        rel_lock = str(lock_path.relative_to(self.base_dir))
        profile.image_paths["initial_lock"] = rel_lock
        profile.image_paths["reference_crop"] = rel_lock
        profile.image_paths["target_box"] = rel_lock
        profile.image_paths["best_quality"] = rel_lock
        profile.image_paths["latest"] = rel_lock

        # Second tight reference for multi-angle / reacquire matching
        ref_path = img_dir / "reference_01.jpg"
        cv2.imwrite(str(ref_path), crop)
        profile.image_paths["reference_01"] = str(ref_path.relative_to(self.base_dir))

        profile.last_bbox = tuple(float(v) for v in bbox_xywh)
        profile.object_width_px = float(bbox_xywh[2])
        profile.object_height_px = float(bbox_xywh[3])
        profile.add_event(
            "Images Captured",
            confidence=profile.confidence,
            system_response="Stored target-box crop only (precise tracking reference)",
        )

    def save_snapshot(
        self,
        profile: TargetProfile,
        frame_bgr: np.ndarray,
        tag: str = "latest",
        bbox_xywh: tuple[int, int, int, int] | tuple[float, float, float, float] | None = None,
    ) -> str | None:
        """Save a tracking snapshot — always the bounding-box crop when bbox is known."""
        bbox = bbox_xywh or profile.last_bbox
        if bbox is None:
            return None

        crop = self.crop_bbox(frame_bgr, bbox, pad_frac=0.0)
        if crop is None:
            return None

        tdir = self._target_dir(profile.target_id)
        snap_dir = tdir / "snapshots"
        filename = f"{tag}_{int(time.time() * 1000)}.jpg"
        path = snap_dir / filename
        cv2.imwrite(str(path), crop)
        rel = str(path.relative_to(self.base_dir))
        profile.image_paths[tag] = rel
        profile.image_paths["latest"] = rel
        profile.image_paths["target_box"] = rel

        # Keep best-quality crop when confidence is high and crop is reasonably large
        best_path = profile.image_paths.get("best_quality")
        if profile.confidence >= 0.55 and crop.size >= 400:
            best_file = tdir / "images" / "best_quality.jpg"
            cv2.imwrite(str(best_file), crop)
            profile.image_paths["best_quality"] = str(best_file.relative_to(self.base_dir))
            if best_path is None:
                profile.add_event(
                    "Best Quality Image",
                    confidence=profile.confidence,
                    system_response="Updated best target-box crop",
                )

        return rel

    def update_active(
        self,
        profile: TargetProfile,
        frame_bgr: np.ndarray | None = None,
        bbox_xywh: tuple[int, int, int, int] | tuple[float, float, float, float] | None = None,
        save_interval: int = 30,
    ) -> None:
        if profile.status in (TargetStatus.LOCKED, TargetStatus.REACQUIRED):
            profile.status = TargetStatus.TRACKING
        if (
            frame_bgr is not None
            and bbox_xywh is not None
            and profile.frames_processed % save_interval == 0
        ):
            self.save_snapshot(profile, frame_bgr, tag="tracking", bbox_xywh=bbox_xywh)
        self._persist(profile)

    def mark_lost(self, profile: TargetProfile) -> None:
        profile.status = TargetStatus.LOST
        profile.add_event(
            "Temporary Loss",
            confidence=profile.confidence,
            system_response="Tracking lost — reacquisition active",
        )
        self._persist(profile)

    def mark_reacquired(self, profile: TargetProfile) -> None:
        profile.status = TargetStatus.REACQUIRED
        profile.add_event(
            "Target Reacquired",
            confidence=profile.confidence,
            system_response="Automatic re-lock successful",
        )
        self._persist(profile)

    def finish_target(self, profile: TargetProfile) -> None:
        profile.status = TargetStatus.FINISHED
        profile.finish_time = time.time()
        profile.add_event(
            "Tracking Finished",
            confidence=profile.confidence,
            system_response="Session ended",
        )
        self._persist(profile)

    def delete_target(self, target_id: str) -> bool:
        if target_id not in self._targets:
            return False
        del self._targets[target_id]
        tdir = self.base_dir / target_id
        if tdir.exists():
            shutil.rmtree(tdir)
        self._save_index()
        return True

    def archive_target(self, target_id: str) -> bool:
        profile = self._targets.get(target_id)
        if not profile:
            return False
        profile.status = TargetStatus.ARCHIVED
        self._persist(profile)
        return True

    def _persist(self, profile: TargetProfile) -> None:
        tdir = self._target_dir(profile.target_id)
        meta_path = tdir / "profile.json"
        with open(meta_path, "w", encoding="utf-8") as f:
            json.dump(profile.to_dict(), f, indent=2)
        self._targets[profile.target_id] = profile
        self._save_index()

    def export_target_json(self, target_id: str, dest: Path | None = None) -> Path | None:
        profile = self.get(target_id)
        if not profile:
            return None
        dest = dest or (self.base_dir / f"{target_id}_export.json")
        with open(dest, "w", encoding="utf-8") as f:
            json.dump(profile.to_dict(), f, indent=2)
        return dest

    def export_target_csv(self, target_id: str, dest: Path | None = None) -> Path | None:
        import csv

        profile = self.get(target_id)
        if not profile:
            return None
        dest = dest or (self.base_dir / f"{target_id}_history.csv")
        with open(dest, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["frame", "confidence", "bbox_x", "bbox_y", "bbox_w", "bbox_h", "vx", "vy"])
            for i, bbox in enumerate(profile.bbox_history):
                conf = profile.confidence_history[i] if i < len(profile.confidence_history) else 0
                vx, vy = profile.velocity_history[i] if i < len(profile.velocity_history) else (0, 0)
                writer.writerow([i, conf, *bbox, vx, vy])
        return dest

    def get_image_path(self, target_id: str, key: str) -> Path | None:
        profile = self.get(target_id)
        if not profile or key not in profile.image_paths:
            return None
        path = self.base_dir / profile.image_paths[key]
        return path if path.exists() else None

    def stats(self) -> dict[str, Any]:
        statuses: dict[str, int] = {}
        for t in self._targets.values():
            statuses[t.status.value] = statuses.get(t.status.value, 0) + 1
        return {
            "total": len(self._targets),
            "by_status": statuses,
        }
