from __future__ import annotations

import cv2
import numpy as np

from .estimator import EstimateOutput


def draw_overlay(frame: np.ndarray, est: EstimateOutput | None, fps: float) -> None:
    y = 28
    step = 22
    cv2.putText(
        frame,
        "Simple Relative Height",
        (20, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.72,
        (240, 240, 240),
        2,
    )
    y += step

    if est is None:
        cv2.putText(
            frame,
            "waiting for baseline frame...",
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (0, 180, 255),
            2,
        )
    else:
        color = (0, 220, 0) if est.tracking_ok else (0, 180, 255)
        cv2.putText(
            frame,
            f"delta_height_rel: {est.delta_height_rel:+.4f}",
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.67,
            color,
            2,
        )
        y += step
        cv2.putText(
            frame,
            f"raw: {est.raw_delta_rel:+.4f}",
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (220, 220, 220),
            1,
        )
        y += step
        cv2.putText(
            frame,
            f"matches: {est.matches} inliers: {est.inliers} ratio: {est.inlier_ratio:.2f}",
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (220, 220, 220),
            1,
        )

    cv2.putText(
        frame,
        f"FPS: {fps:.1f}",
        (20, frame.shape[0] - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (240, 240, 240),
        2,
    )
    cv2.putText(
        frame,
        "Keys: q quit | r reset baseline",
        (max(10, frame.shape[1] - 320), frame.shape[0] - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (240, 240, 240),
        1,
    )
