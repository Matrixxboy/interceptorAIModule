"""Multi-scale template lock so bbox pixel size tracks distance changes."""

from __future__ import annotations

import cv2
import numpy as np


class ScaleAwareLock:
    """Template lock that searches across scales so pixel size tracks distance.

    Size comes from multi-scale NCC. Optional CSRT is used for center only and
    is size-capped so it cannot inflate the box into the background.
    """

    def __init__(self, use_csrt: bool = False) -> None:
        self.use_csrt = use_csrt
        self._template: np.ndarray | None = None
        self._bbox: tuple[float, float, float, float] | None = None
        self._scale = 1.0
        self._misses = 0
        self._csrt = None
        self._last_score = 0.0
        self._frames = 0

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
        self._bbox = None
        self._scale = 1.0
        self._misses = 0
        self._csrt = None
        self._last_score = 0.0
        self._frames = 0

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
            self._template = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
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

    def _multiscale_match(self, gray: np.ndarray) -> tuple[float, float, float, float, float] | None:
        if self._template is None or self._bbox is None:
            return None
        x, y, w, h = self._bbox
        tw0, th0 = self._template.shape[1], self._template.shape[0]
        if tw0 < 4 or th0 < 4:
            return None

        # Wide enough band to track approach/recede; still local around current scale
        lo = max(0.35, self._scale * 0.62)
        hi = min(2.8, self._scale * 1.55)
        scales = [float(s) for s in np.linspace(lo, hi, 13)]

        pad = max(120, int(max(w, h) * 1.6))
        fh, fw = gray.shape[:2]
        x0 = max(0, int(x) - pad)
        y0 = max(0, int(y) - pad)
        x1 = min(fw, int(x + w) + pad)
        y1 = min(fh, int(y + h) + pad)
        roi = gray[y0:y1, x0:x1]
        if roi.size == 0 or roi.shape[0] < 16 or roi.shape[1] < 16:
            return None

        best = None
        for s in scales:
            tw = max(8, int(round(tw0 * s)))
            th = max(8, int(round(th0 * s)))
            if tw >= roi.shape[1] - 2 or th >= roi.shape[0] - 2:
                continue
            try:
                tmpl = cv2.resize(self._template, (tw, th), interpolation=cv2.INTER_AREA)
                if tmpl.shape[0] >= roi.shape[0] or tmpl.shape[1] >= roi.shape[1]:
                    continue
                res = cv2.matchTemplate(roi, tmpl, cv2.TM_CCOEFF_NORMED)
                _, max_val, _, max_loc = cv2.minMaxLoc(res)
            except Exception:
                continue
            if best is None or max_val > best[0]:
                best = (float(max_val), float(x0 + max_loc[0]), float(y0 + max_loc[1]), float(tw), float(th), s)

        if best is None or best[0] < 0.38:
            return None
        score, nx, ny, nw, nh, s = best
        # Smooth scale but allow real approach/recede
        self._scale = 0.55 * self._scale + 0.45 * s
        self._last_score = score
        return (nx, ny, nw, nh, score)

    def update(self, frame_bgr: np.ndarray) -> tuple[bool, tuple[float, float, float, float] | None]:
        if self._template is None:
            return False, None
        try:
            self._frames += 1
            gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
            ms = self._multiscale_match(gray)

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
                # Center blend with CSRT only; size always from multi-scale match
                if csrt_box is not None:
                    cx = 0.70 * (nx + nw * 0.5) + 0.30 * (csrt_box[0] + csrt_box[2] * 0.5)
                    cy = 0.70 * (ny + nh * 0.5) + 0.30 * (csrt_box[1] + csrt_box[3] * 0.5)
                    self._bbox = (cx - nw * 0.5, cy - nh * 0.5, nw, nh)
                else:
                    self._bbox = (nx, ny, nw, nh)
                self._misses = 0

                # Refresh template occasionally when match is strong (reduces drift)
                if score >= 0.72 and self._frames % 45 == 0 and self._bbox is not None:
                    bx, by, bw, bh = [int(round(v)) for v in self._bbox]
                    fh, fw = frame_bgr.shape[:2]
                    bx = max(0, min(bx, fw - 2))
                    by = max(0, min(by, fh - 2))
                    bw = max(12, min(bw, fw - bx))
                    bh = max(12, min(bh, fh - by))
                    patch = frame_bgr[by : by + bh, bx : bx + bw]
                    if patch.size > 0 and patch.shape[0] >= 8 and patch.shape[1] >= 8:
                        self._template = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
                        self._scale = 1.0  # template is new base scale

                return True, self._bbox

            if csrt_box is not None and self._bbox is not None:
                lx, ly, lw, lh = self._bbox
                cx = csrt_box[0] + csrt_box[2] * 0.5
                cy = csrt_box[1] + csrt_box[3] * 0.5
                cw, ch = csrt_box[2], csrt_box[3]
                # Reject CSRT size explosion — keep last size, move center only
                if cw > 2.2 * lw or ch > 2.2 * lh or cw < 0.45 * lw or ch < 0.45 * lh:
                    self._bbox = (cx - lw * 0.5, cy - lh * 0.5, lw, lh)
                else:
                    # Mild size EMA toward CSRT when reasonable
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
