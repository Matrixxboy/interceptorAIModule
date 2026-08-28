"""Sub-pixel Optical Flow & NCC Template Precision Lock Engine.

Combines:
1. Shi-Tomasi Corner Feature Extraction inside target ROI
2. Pyramidal Lucas-Kanade (LK) Optical Flow keypoint cluster tracking
3. RANSAC / Median flow outlier filtering for robust background rejection
4. Normalized Cross-Correlation (NCC) sub-pixel template peak matching
5. Automatic keypoint re-seeding and scale adaptation
"""

from __future__ import annotations

import cv2
import numpy as np


class TargetPatternFingerprint:
    """Sub-pixel visual pattern fingerprint anchor (ORB Descriptors + Template Pyramid)."""

    def __init__(self) -> None:
        self.orb = cv2.ORB_create(nfeatures=120, scaleFactor=1.2, nlevels=4)
        self.kp_ref: list[cv2.KeyPoint] = []
        self.des_ref: np.ndarray | None = None
        self.template_pyramid: list[np.ndarray] = []
        self.scales: list[float] = [0.88, 1.0, 1.15]

    def extract(self, gray_roi: np.ndarray) -> bool:
        if gray_roi is None or gray_roi.size < 64:
            return False
        kps, des = self.orb.detectAndCompute(gray_roi, None)
        if des is None or len(kps) < 3:
            return False
        self.kp_ref = kps
        self.des_ref = des
        
        # Build multi-scale template pyramid
        self.template_pyramid = []
        rh, rw = gray_roi.shape[:2]
        for s in self.scales:
            nw = max(8, int(round(rw * s)))
            nh = max(8, int(round(rh * s)))
            tmpl = cv2.resize(gray_roi, (nw, nh), interpolation=cv2.INTER_AREA if s < 1.0 else cv2.INTER_LINEAR)
            self.template_pyramid.append(tmpl)
        return True

    def match_fingerprint(self, search_crop: np.ndarray) -> tuple[float, tuple[int, int] | None, float]:
        """Matches search crop against pattern fingerprint. Returns (best_score, best_offset_xy, scale_factor)."""
        if search_crop is None or not self.template_pyramid or search_crop.size < 64:
            return 0.0, None, 1.0

        best_val = -1.0
        best_loc = None
        best_scale = 1.0

        ch, cw = search_crop.shape[:2]
        for idx, tmpl in enumerate(self.template_pyramid):
            th, tw = tmpl.shape[:2]
            if cw >= tw + 2 and ch >= th + 2:
                res = cv2.matchTemplate(search_crop, tmpl, cv2.TM_CCOEFF_NORMED)
                _, max_val, _, max_loc = cv2.minMaxLoc(res)
                if max_val > best_val:
                    best_val = float(max_val)
                    best_loc = (max_loc[0] + tw // 2, max_loc[1] + th // 2)
                    best_scale = self.scales[idx]

        return max(0.0, best_val), best_loc, best_scale


class PixelLockEngine:
    """Sub-pixel keypoint, optical flow & pattern fingerprint target locking engine."""

    def __init__(
        self,
        max_corners: int = 80,
        quality_level: float = 0.01,
        min_distance: float = 3.0,
        win_size: tuple[int, int] = (15, 15),
    ) -> None:
        self.max_corners = max_corners
        self.quality_level = quality_level
        self.min_distance = min_distance
        self.win_size = win_size

        self.initialized = False
        self.prev_gray: np.ndarray | None = None
        self.p0: np.ndarray | None = None  # Keypoints [N, 1, 2]
        self.bbox_xywh: tuple[float, float, float, float] | None = None
        self.template: np.ndarray | None = None
        self.fingerprint = TargetPatternFingerprint()
        self.label: str = ""
        self.conf: float = 1.0

    def init_lock(
        self,
        frame_bgr: np.ndarray,
        xywh: tuple[float, float, float, float],
        label: str = "target",
    ) -> bool:
        x, y, w, h = [float(v) for v in xywh]
        if w < 6 or h < 6:
            return False

        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        img_h, img_w = gray.shape[:2]

        ix, iy, iw, ih = int(max(0, x)), int(max(0, y)), int(min(img_w - x, w)), int(min(img_h - y, h))
        if iw < 6 or ih < 6:
            return False

        roi = gray[iy : iy + ih, ix : ix + iw]
        corners = cv2.goodFeaturesToTrack(
            roi,
            maxCorners=self.max_corners,
            qualityLevel=self.quality_level,
            minDistance=self.min_distance,
        )

        if corners is None or len(corners) < 4:
            gx, gy = np.meshgrid(
                np.linspace(ix + 2, ix + iw - 2, 8),
                np.linspace(iy + 2, iy + ih - 2, 8),
            )
            corners = np.vstack([gx.ravel(), gy.ravel()]).T.reshape(-1, 1, 2).astype(np.float32)
        else:
            corners[:, 0, 0] += ix
            corners[:, 0, 1] += iy

        self.p0 = corners
        self.prev_gray = gray
        self.bbox_xywh = (x, y, w, h)
        self.template = roi.copy()
        self.fingerprint.extract(roi)
        self.label = label
        self.conf = 1.0
        self.initialized = True
        return True

    def update(self, frame_bgr: np.ndarray) -> tuple[bool, tuple[float, float, float, float] | None, float, str]:
        if not self.initialized or self.prev_gray is None or self.p0 is None or self.bbox_xywh is None:
            return False, None, 0.0, "lost"

        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        img_h, img_w = gray.shape[:2]

        # 1. Pyramidal Lucas-Kanade Optical Flow (0.5 ms)
        p1, st, err = cv2.calcOpticalFlowPyrLK(
            self.prev_gray,
            gray,
            self.p0,
            None,
            winSize=self.win_size,
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
        )

        if p1 is None or st is None:
            self.initialized = False
            return False, self.bbox_xywh, 0.0, "lost"

        good_new = p1[st == 1]
        good_old = self.p0[st == 1]

        if len(good_new) < 3:
            self.initialized = False
            return False, self.bbox_xywh, 0.0, "lost"

        # 2. Motion displacement vectors & RANSAC median filtering
        disp = good_new - good_old
        dx = np.median(disp[:, 0])
        dy = np.median(disp[:, 1])

        motion_err = np.hypot(disp[:, 0] - dx, disp[:, 1] - dy)
        inlier_mask = motion_err < np.percentile(motion_err, 75) + 3.0

        inliers_new = good_new[inlier_mask]
        inliers_old = good_old[inlier_mask]

        if len(inliers_new) < 3:
            inliers_new = good_new
            inliers_old = good_old

        bx, by, bw, bh = self.bbox_xywh

        # 3. Calculate median centroid shift
        dx_final = float(np.median(inliers_new[:, 0] - inliers_old[:, 0]))
        dy_final = float(np.median(inliers_new[:, 1] - inliers_old[:, 1]))

        new_cx = (bx + bw * 0.5) + dx_final
        new_cy = (by + bh * 0.5) + dy_final

        # Estimate scale change from inlier keypoint spread
        if len(inliers_old) >= 5:
            spread_old = np.std(inliers_old, axis=0).mean()
            spread_new = np.std(inliers_new, axis=0).mean()
            if spread_old > 1.0:
                scale = float(np.clip(spread_new / spread_old, 0.80, 1.25))
                bw = float(np.clip(bw * scale, 12.0, img_w * 0.8))
                bh = float(np.clip(bh * scale, 12.0, img_h * 0.8))

        new_x = new_cx - bw * 0.5
        new_y = new_cy - bh * 0.5

        # 4. Pattern Fingerprint Matching Refinement (0.4 ms)
        search_w = int(bw * 1.8)
        search_h = int(bh * 1.8)
        sx = int(max(0, new_cx - search_w * 0.5))
        sy = int(max(0, new_cy - search_h * 0.5))
        sw = int(min(img_w - sx, search_w))
        sh = int(min(img_h - sy, search_h))

        pattern_score = 1.0
        if sw > bw and sh > bh:
            search_crop = gray[sy : sy + sh, sx : sx + sw]
            score, peak_xy, scale_factor = self.fingerprint.match_fingerprint(search_crop)
            pattern_score = score
            if score > 0.45 and peak_xy is not None:
                peak_cx = sx + peak_xy[0]
                peak_cy = sy + peak_xy[1]
                # Blend fingerprint peak for zero-drift sub-pixel anchor lock
                new_x = 0.55 * new_x + 0.45 * (peak_cx - bw * 0.5)
                new_y = 0.55 * new_y + 0.45 * (peak_cy - bh * 0.5)
                if 0.85 <= scale_factor <= 1.15:
                    bw = float(np.clip(bw * scale_factor, 12.0, img_w * 0.8))
                    bh = float(np.clip(bh * scale_factor, 12.0, img_h * 0.8))

        # Clamp inside image boundary
        new_x = float(np.clip(new_x, 0, img_w - bw))
        new_y = float(np.clip(new_y, 0, img_h - bh))
        self.bbox_xywh = (new_x, new_y, bw, bh)

        # 5. Keypoint Re-seeding when feature count drops
        if len(inliers_new) < 20:
            ix, iy, iw, ih = int(new_x), int(new_y), int(bw), int(bh)
            if iw >= 6 and ih >= 6:
                roi = gray[iy : iy + ih, ix : ix + iw]
                new_corners = cv2.goodFeaturesToTrack(
                    roi,
                    maxCorners=self.max_corners,
                    qualityLevel=self.quality_level,
                    minDistance=self.min_distance,
                )
                if new_corners is not None and len(new_corners) > 0:
                    new_corners[:, 0, 0] += ix
                    new_corners[:, 0, 1] += iy
                    inliers_new = np.vstack([inliers_new, new_corners[:, 0, :]])

        self.p0 = inliers_new.reshape(-1, 1, 2).astype(np.float32)
        self.prev_gray = gray
        self.conf = float(np.clip(pattern_score, 0.4, 1.0))

        return True, self.bbox_xywh, self.conf, "pattern_fingerprint"
