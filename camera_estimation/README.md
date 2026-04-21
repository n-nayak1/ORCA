# Camera Estimation (Simple)

This version is intentionally minimal and split into small files.

- Goal: realtime **relative** camera-floor height
- Baseline: first accepted frame is `0.0`
- Output: `delta_height_rel` (unitless, not meters)

## File layout

- `height_estimator.py` - tiny entrypoint
- `simple_height/config.py` - calibration loading and runtime config
- `simple_height/vision.py` - ORB features, matching, homography scale
- `simple_height/estimator.py` - minimal stateful estimator
- `simple_height/ui.py` - overlay drawing
- `simple_height/app.py` - webcam loop and controls

## Install

```bash
pip install opencv-python numpy pyyaml
```

## Calibration YAML

Use your calibrated camera values:

```yaml
camera_matrix:
  - [fx, 0.0, cx]
  - [0.0, fy, cy]
  - [0.0, 0.0, 1.0]
dist_coeffs: [k1, k2, p1, p2, k3]
```

## Run

From `camera_estimation/`:

```bash
python height_estimator.py --config camera_params_example.yaml
```

Camera source selection:

```bash
# Default webcam (index 0 by default)
python height_estimator.py --config camera_params_example.yaml --camera-source default

# Apple iPhone Continuity Camera (index 1 by default)
python height_estimator.py --config camera_params_example.yaml --camera-source iphone
```

Useful flags:

- `--features 3000` for weak-texture floors
- `--ratio-test 0.85`
- `--ransac-thresh 5.0`
- `--ema-alpha 0.15`
- `--camera-source default|iphone`
- `--camera 0` (default source index)
- `--iphone-camera 1` (iPhone source index)

## Controls

- `q` quit
- `r` reset baseline to current frame

## Simplified algorithm

1. Undistort frame
2. Detect ORB features on previous and current frame
3. Estimate homography with RANSAC
4. Extract local homography scale at image center
5. Accumulate scale over time and convert to relative height
6. Smooth with EMA

## Notes

- Keep floor texture visible and move slowly.
- Avoid glare, strong blur, and large yaw rotations.
- If this still fails, the most likely blocker is bad calibration values.
