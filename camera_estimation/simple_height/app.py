from __future__ import annotations

import argparse
import time

import cv2

from .config import RuntimeConfig, load_calibration
from .estimator import SimpleRelativeHeightEstimator
from .ui import draw_overlay
from .vision import create_orb


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Simple realtime relative camera-floor height estimator"
    )
    p.add_argument("--config", required=True, help="YAML calibration file")
    p.add_argument("--camera", type=int, default=0)
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--features", type=int, default=2000)
    p.add_argument("--ratio-test", type=float, default=0.82)
    p.add_argument("--ransac-thresh", type=float, default=4.0)
    p.add_argument("--ema-alpha", type=float, default=0.2)
    return p.parse_args()


def run() -> None:
    args = parse_args()
    cfg = RuntimeConfig(
        camera_index=args.camera,
        width=args.width,
        height=args.height,
        features=args.features,
        ratio_test=args.ratio_test,
        ransac_thresh_px=args.ransac_thresh,
        ema_alpha=args.ema_alpha,
    )
    calib = load_calibration(args.config)

    cap = cv2.VideoCapture(cfg.camera_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.height)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {cfg.camera_index}")

    orb = create_orb(cfg.features)
    estimator: SimpleRelativeHeightEstimator | None = None

    prev_t = time.perf_counter()
    while True:
        ok, frame = cap.read()
        if not ok:
            break

        undist = cv2.undistort(frame, calib.k, calib.dist)
        gray = cv2.cvtColor(undist, cv2.COLOR_BGR2GRAY)

        h_img, w_img = gray.shape[:2]
        cx = float(calib.k[0, 2])
        cy = float(calib.k[1, 2])
        if not (0.0 <= cx < w_img and 0.0 <= cy < h_img):
            cx = w_img / 2.0
            cy = h_img / 2.0

        if estimator is None:
            estimator = SimpleRelativeHeightEstimator(
                orb=orb,
                cx=cx,
                cy=cy,
                ratio_test=cfg.ratio_test,
                ransac_thresh_px=cfg.ransac_thresh_px,
                ema_alpha=cfg.ema_alpha,
            )

        est = estimator.update(gray)

        now = time.perf_counter()
        fps = 1.0 / max(now - prev_t, 1e-6)
        prev_t = now

        draw_overlay(undist, est, fps)
        cv2.imshow("Simple Relative Height", undist)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("r") and estimator is not None:
            estimator.reset()

    cap.release()
    cv2.destroyAllWindows()
