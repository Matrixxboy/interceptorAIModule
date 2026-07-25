"""Multi-scale template lock so bbox pixel size tracks distance changes.

Optimized for realtime: few scales, downscaled NCC, full search every N frames.
"""

from __future__ import annotations

import cv2
import numpy as np


class ScaleAwareLock:
    """Template lock that searches across scales so pixel size tracks distance."""

    def __init__(self, use_csrt: bool = False) -> None:
        self.use_csrt = use_csrt
        self._template: np.ndarray | None = None
        self._template_small: np.ndarray | None = None
        self._bbox: tuple[float, float, float, float] | None = None
        self._scale = 1.0
        self._misses = 0
        self._csrt = None
        self._last_score = 0.0
        self._frames = 0
        self._full_every = 2  # full multi-scale every N frames
        self._match_max_side = 96  # downscale template for faster NCC

    @property
    def scale(self) -> float:
        return self._scale

    @property
    def bbox(self) -> tuple[float, float, float, float] | None:
        return self._bbox

    @property
    def last_score(self) -> float:
        return self._last_score

    @property
    def locked(self) -> bool:
        return self._template is not None

    @staticmethod
    def _make_csrt():
        try:
            if hasattr(cv2, "TrackerCSRT_create"):
                return cv2.TrackerCSRT_create()
            if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerCSRT_create"):
                return cv2.legacy.TrackerCSRT_create()
        except Exception:
            return None
        return None

    def reset(self) -> None:
        self._template = None
        self._template_small = None
        self._bbox = None
        self._scale = 1.0
        self._misses = 0
        self._csrt = None
        self._last_score = 0.0
        self._frames = 0

    def _shrink_template(self, gray_patch: np.ndarray) -> np.ndarray:
        h, w = gray_patch.shape[:2]
        side = max(h, w)
        if side <= self._match_max_side:
            return gray_patch
        scale = self._match_max_side / float(side)
        nw = max(8, int(round(w * scale)))
        nh = max(8, int(round(h * scale)))
        return cv2.resize(gray_patch, (nw, nh), interpolation=cv2.INTER_AREA)

    def init(self, frame_bgr: np.ndarray, bbox_xywh: tuple[float, float, float, float]) -> bool:
        try:
            x, y, w, h = [int(round(float(v))) for v in bbox_xywh]
            fh, fw = frame_bgr.shape[:2]
            x = max(0, min(x, fw - 2))
            y = max(0, min(y, fh - 2))
            w = max(12, min(w, fw - x))
            h = max(12, min(h, fh - y))
            if w < 12 or h < 12:
                return False
            patch = frame_bgr[y : y + h, x : x + w]
            if patch is None or patch.size == 0 or patch.shape[0] < 8 or patch.shape[1] < 8:
                return False
            gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
            self._template = gray
            self._template_small = self._shrink_template(gray)
            self._bbox = (float(x), float(y), float(w), float(h))
            self._scale = 1.0
            self._misses = 0
            self._last_score = 1.0
            self._frames = 0
            self._csrt = None
            if self.use_csrt:
                self._csrt = self._make_csrt()
                if self._csrt is not None:
                    try:
                        self._csrt.init(frame_bgr, (x, y, w, h))
                    except Exception:
                        self._csrt = None
            return True
        except Exception:
            self.reset()
            return False

    def _match_scales(
        self,
        gray: np.ndarray,
        scales: list[float],
    ) -> tuple[float, float, float, float, float] | None:
        if self._template is None or self._bbox is None or self._template_small is None:
            return None
        x, y, w, h = self._bbox
        tw0, th0 = self._template.shape[1], self._template.shape[0]
        if tw0 < 4 or th0 < 4:
            return None

        # Match on downscaled template; map size back to original template scale
        ts = self._template_small
        tsw, tsh = ts.shape[1], ts.shape[0]
        shrink = tsw / float(tw0)

        pad = max(64, int(max(w, h) * 1.15))
        fh, fw = gray.shape[:2]
        x0 = max(0, int(x) - pad)
        y0 = max(0, int(y) - pad)
        x1 = min(fw, int(x + w) + pad)
        y1 = min(fh, int(y + h) + pad)
        roi_full = gray[y0:y1, x0:x1]
        if roi_full.size == 0 or roi_full.shape[0] < 16 or roi_full.shape[1] < 16:
            return None

        # Downscale ROI by same factor as template for faster NCC
        if shrink < 0.999:
            roi = cv2.resize(
                roi_full,
                (max(16, int(round(roi_full.shape[1] * shrink))),
                 max(16, int(round(roi_full.shape[0] * shrink)))),
                interpolation=cv2.INTER_AREA,
            )
            inv = 1.0 / shrink
        else:
            roi = roi_full
            inv = 1.0

        best = None
        for s in scales:
            tw = max(8, int(round(tsw * s)))
            th = max(8, int(round(tsh * s)))
            if tw >= roi.shape[1] - 2 or th >= roi.shape[0] - 2:
                continue
            try:
                tmpl = cv2.resize(ts, (tw, th), interpolation=cv2.INTER_AREA)
                res = cv2.matchTemplate(roi, tmpl, cv2.TM_CCOEFF_NORMED)
                _, max_val, _, max_loc = cv2.minMaxLoc(res)
            except Exception:
                continue
            # Map back to full-resolution coords / size
            nx = float(x0 + max_loc[0] * inv)
            ny = float(y0 + max_loc[1] * inv)
            nw = float(tw * inv)
            nh = float(th * inv)
            if best is None or max_val > best[0]:
                best = (float(max_val), nx, ny, nw, nh, float(s))

        if best is None or best[0] < 0.38:
            return None
        score, nx, ny, nw, nh, s = best
        self._scale = 0.60 * self._scale + 0.40 * s
        self._last_score = score
        return (nx, ny, nw, nh, score)

    def _multiscale_match(self, gray: np.ndarray, full: bool) -> tuple[float, float, float, float, float] | None:
        if full:
            lo = max(0.40, self._scale * 0.70)
            hi = min(2.4, self._scale * 1.40)
            scales = [float(s) for s in np.linspace(lo, hi, 7)]
        else:
            # Fast path: 3 local scales around current
            scales = [
                max(0.40, self._scale * 0.92),
                self._scale,
                min(2.4, self._scale * 1.08),
            ]
        return self._match_scales(gray, scales)

    def update(self, frame_bgr: np.ndarray) -> tuple[bool, tuple[float, float, float, float] | None]:
        if self._template is None:
            return False, None
        try:
            self._frames += 1
            gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
            full = (self._frames % self._full_every) == 0 or self._misses > 0
            ms = self._multiscale_match(gray, full=full)

            csrt_box = None
            if self._csrt is not None:
                try:
                    ok, box = self._csrt.update(frame_bgr)
                    if ok:
                        csrt_box = tuple(float(v) for v in box)
                except Exception:
                    self._csrt = None

            if ms is not None:
                nx, ny, nw, nh, score = ms
                if csrt_box is not None:
                    cx = 0.70 * (nx + nw * 0.5) + 0.30 * (csrt_box[0] + csrt_box[2] * 0.5)
                    cy = 0.70 * (ny + nh * 0.5) + 0.30 * (csrt_box[1] + csrt_box[3] * 0.5)
                    self._bbox = (cx - nw * 0.5, cy - nh * 0.5, nw, nh)
                else:
                    self._bbox = (nx, ny, nw, nh)
                self._misses = 0

                if score >= 0.75 and self._frames % 60 == 0 and self._bbox is not None:
                    bx, by, bw, bh = [int(round(v)) for v in self._bbox]
                    fh, fw = frame_bgr.shape[:2]
                    bx = max(0, min(bx, fw - 2))
                    by = max(0, min(by, fh - 2))
                    bw = max(12, min(bw, fw - bx))
                    bh = max(12, min(bh, fh - by))
                    patch = frame_bgr[by : by + bh, bx : bx + bw]
                    if patch.size > 0 and patch.shape[0] >= 8 and patch.shape[1] >= 8:
                        gray_p = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
                        self._template = gray_p
                        self._template_small = self._shrink_template(gray_p)
                        self._scale = 1.0

                return True, self._bbox

            if csrt_box is not None and self._bbox is not None:
                lx, ly, lw, lh = self._bbox
                cx = csrt_box[0] + csrt_box[2] * 0.5
                cy = csrt_box[1] + csrt_box[3] * 0.5
                cw, ch = csrt_box[2], csrt_box[3]
                if cw > 2.2 * lw or ch > 2.2 * lh or cw < 0.45 * lw or ch < 0.45 * lh:
                    self._bbox = (cx - lw * 0.5, cy - lh * 0.5, lw, lh)
                else:
                    nw = 0.85 * lw + 0.15 * cw
                    nh = 0.85 * lh + 0.15 * ch
                    self._bbox = (cx - nw * 0.5, cy - nh * 0.5, nw, nh)
                self._misses = 0
                return True, self._bbox

            self._misses += 1
            if self._misses > 30:
                self.reset()
                return False, None
            return True, self._bbox
        except Exception:
            self._misses += 1
            if self._misses > 30:
                self.reset()
                return False, None
            return True, self._bbox
