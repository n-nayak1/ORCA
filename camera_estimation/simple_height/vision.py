from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np


@dataclass
class MotionEstimate:
    homography: np.ndarray
    scale: float
    matches: int
    inliers: int
    inlier_ratio: float


def create_orb(nfeatures: int) -> Any:
    orb_create = getattr(cv2, "ORB_create")
    return orb_create(nfeatures=nfeatures, scaleFactor=1.2, nlevels=8)


def detect_and_describe(
    gray: np.ndarray, orb: Any
) -> tuple[list[Any], np.ndarray | None]:
    kp, desc = orb.detectAndCompute(gray, None)
    return list(kp), desc


def match_descriptors(
    desc_a: np.ndarray | None, desc_b: np.ndarray | None, ratio: float
) -> list[Any]:
    if desc_a is None or desc_b is None:
        return []
    if len(desc_a) < 2 or len(desc_b) < 2:
        return []

    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    knn = matcher.knnMatch(desc_a, desc_b, k=2)

    good = []
    for pair in knn:
        if len(pair) != 2:
            continue
        m, n = pair
        if m.distance < ratio * n.distance:
            good.append(m)
    return good


def _project(h: np.ndarray, x: float, y: float) -> np.ndarray:
    p = np.array([x, y, 1.0], dtype=np.float64)
    q = h @ p
    if abs(q[2]) < 1e-9:
        return np.array([np.nan, np.nan], dtype=np.float64)
    return q[:2] / q[2]


def local_scale(h: np.ndarray, cx: float, cy: float, eps: float = 2.0) -> float | None:
    p0 = _project(h, cx, cy)
    px = _project(h, cx + eps, cy)
    py = _project(h, cx, cy + eps)
    if np.isnan(p0).any() or np.isnan(px).any() or np.isnan(py).any():
        return None

    jx = (px - p0) / eps
    jy = (py - p0) / eps
    j = np.stack([jx, jy], axis=1)
    det = float(np.linalg.det(j))
    if det <= 1e-10 or not np.isfinite(det):
        return None
    return float(np.sqrt(det))


def estimate_motion(
    prev_kp: list[Any],
    prev_desc: np.ndarray | None,
    curr_kp: list[Any],
    curr_desc: np.ndarray | None,
    cx: float,
    cy: float,
    ratio_test: float,
    ransac_thresh_px: float,
) -> MotionEstimate | None:
    matches = match_descriptors(prev_desc, curr_desc, ratio=ratio_test)
    if len(matches) < 8:
        return None

    src = np.array([prev_kp[m.queryIdx].pt for m in matches], dtype=np.float32)
    dst = np.array([curr_kp[m.trainIdx].pt for m in matches], dtype=np.float32)

    h, mask = cv2.findHomography(src, dst, cv2.RANSAC, ransac_thresh_px)
    if h is None or mask is None:
        return None

    inlier_mask = np.asarray(mask).ravel().astype(bool)
    inliers = int(np.count_nonzero(inlier_mask))
    if inliers < 6:
        return None

    s = local_scale(h, cx, cy)
    if s is None:
        return None

    return MotionEstimate(
        homography=h,
        scale=s,
        matches=len(matches),
        inliers=inliers,
        inlier_ratio=inliers / max(len(matches), 1),
    )
