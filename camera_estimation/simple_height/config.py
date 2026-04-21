from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import yaml


@dataclass
class Calibration:
    k: np.ndarray
    dist: np.ndarray


@dataclass
class RuntimeConfig:
    camera_index: int = 0
    width: int = 1280
    height: int = 720
    features: int = 2000
    ratio_test: float = 0.82
    ransac_thresh_px: float = 4.0
    ema_alpha: float = 0.2


def load_calibration(path: str) -> Calibration:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    k = np.asarray(data["camera_matrix"], dtype=np.float64)
    dist = np.asarray(
        data.get("dist_coeffs", [0.0, 0.0, 0.0, 0.0, 0.0]), dtype=np.float64
    )

    if k.shape != (3, 3):
        raise ValueError("camera_matrix must be 3x3")

    return Calibration(k=k, dist=dist.reshape(-1))
