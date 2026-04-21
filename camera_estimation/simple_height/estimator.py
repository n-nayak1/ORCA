from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from .vision import MotionEstimate, detect_and_describe, estimate_motion


@dataclass
class EstimateOutput:
    delta_height_rel: float
    raw_delta_rel: float
    matches: int
    inliers: int
    inlier_ratio: float
    tracking_ok: bool


class SimpleRelativeHeightEstimator:
    """Very simple estimator:
    - Baseline at first accepted frame (delta=0)
    - Frame-to-frame homography scale accumulation
    - Minimal gating and EMA smoothing
    """

    def __init__(
        self,
        orb: Any,
        cx: float,
        cy: float,
        ratio_test: float,
        ransac_thresh_px: float,
        ema_alpha: float,
    ) -> None:
        self.orb = orb
        self.cx = cx
        self.cy = cy
        self.ratio_test = ratio_test
        self.ransac_thresh_px = ransac_thresh_px
        self.ema_alpha = float(np.clip(ema_alpha, 0.0, 1.0))

        self.prev_kp: list[Any] | None = None
        self.prev_desc: np.ndarray | None = None
        self.cumulative_log_scale = 0.0
        self.delta_smooth = 0.0
        self.initialized = False

    def reset(self) -> None:
        self.prev_kp = None
        self.prev_desc = None
        self.cumulative_log_scale = 0.0
        self.delta_smooth = 0.0
        self.initialized = False

    def _bootstrap(self, gray: np.ndarray) -> bool:
        kp, desc = detect_and_describe(gray, self.orb)
        if desc is None or len(kp) < 60:
            return False
        self.prev_kp = kp
        self.prev_desc = desc
        self.initialized = True
        return True

    def update(self, gray: np.ndarray) -> EstimateOutput | None:
        if not self.initialized:
            ok = self._bootstrap(gray)
            if not ok:
                return None
            return EstimateOutput(0.0, 0.0, 0, 0, 0.0, True)

        if self.prev_kp is None or self.prev_desc is None:
            self.reset()
            return None

        curr_kp, curr_desc = detect_and_describe(gray, self.orb)
        m = estimate_motion(
            prev_kp=self.prev_kp,
            prev_desc=self.prev_desc,
            curr_kp=curr_kp,
            curr_desc=curr_desc,
            cx=self.cx,
            cy=self.cy,
            ratio_test=self.ratio_test,
            ransac_thresh_px=self.ransac_thresh_px,
        )

        # Always refresh previous frame so we do not get stuck.
        if curr_desc is not None and len(curr_kp) > 0:
            self.prev_kp = curr_kp
            self.prev_desc = curr_desc

        if m is None:
            return EstimateOutput(
                delta_height_rel=float(self.delta_smooth),
                raw_delta_rel=float(self.delta_smooth),
                matches=0,
                inliers=0,
                inlier_ratio=0.0,
                tracking_ok=False,
            )

        # For flat-plane + near-downward view: image scale ~ h_prev/h_curr.
        step_log = float(np.log(max(m.scale, 1e-8)))
        self.cumulative_log_scale += step_log
        raw_delta = float(np.exp(-self.cumulative_log_scale) - 1.0)
        self.delta_smooth = (
            self.ema_alpha * raw_delta + (1.0 - self.ema_alpha) * self.delta_smooth
        )

        tracking_ok = m.matches >= 8 and m.inliers >= 6 and m.inlier_ratio >= 0.15
        return EstimateOutput(
            delta_height_rel=float(self.delta_smooth),
            raw_delta_rel=raw_delta,
            matches=m.matches,
            inliers=m.inliers,
            inlier_ratio=float(m.inlier_ratio),
            tracking_ok=tracking_ok,
        )
