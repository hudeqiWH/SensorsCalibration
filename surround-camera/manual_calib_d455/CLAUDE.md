# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a C++ GUI tool for manually calibrating a front-view 3-camera system: left fisheye (FL), front Ocam (F), and right fisheye (FR). It performs fisheye/Ocam undistortion, perspective projection onto a front wall, and interactive 6-DOF extrinsic fine-tuning via a Pangolin-based GUI.

## Build

```bash
cd build && cmake .. && make -j4
```

The output binary is `bin/run_front3_calib_v4`. There are older binaries from prior iterations in `bin/` that are not maintained (e.g. `run_front3_calib`, `run_front3_calib_v2`, `run_front3_calib_v3`). The canonical source file is `src/run_front3_calib_v4.cpp`.

## Run

```bash
./bin/run_front3_calib_v4 <image_dir> <intrinsics_dir> <extrinsics.json> [wall_distance_m]
```

Example:
```bash
./bin/run_front3_calib_v4 ./1_h1_front/imgs ./1_h1_front/param ./1_h1_front/param/extrinsics.json 1.8
```

The optional `wall_distance` changes the front wall projection plane distance from the vehicle frame origin (default 1.8m).

## Dependencies

Dependency locations are hardcoded absolute paths in `CMakeLists.txt`:
- **Pangolin**: `third_party/Pangolin` (build and include paths)
- **Eigen3**: `third_party/SensorsCalibration/factory_calib/3rdparty/eigen3`
- **JsonCpp**: `third_party/jsoncpp` (static lib at `build/lib/libjsoncpp.a`)
- **OpenCV** and **Boost**: found via `find_package`

If the repo is relocated, update these absolute paths in `CMakeLists.txt` before building.

## Input File Formats

Images (`<image_dir>`):
- `front_left.png` — left fisheye image
- `front.png`    — front Ocam image
- `front_right.png` — right fisheye image

Intrinsics (`<intrinsics_dir>`):
- `front_left.json` / `front_right.json` — OpenCV fisheye 4-parameter model (`camera_matrix`, `distortion_coefficients` [k1,k2,k3,k4])
- `park_front.json` — Ocam polynomial model (`world2cam` coefficients, `affine_c/d/e`, principal point)

Extrinsics (JSON, `<extrinsics.json>`):
```json
{
  "extrinsic_param": {
    "front_left":  { "rotation": [rx, ry, rz], "translation": [tx, ty, tz] },
    "park_front":  { "rotation": [rx, ry, rz], "translation": [tx, ty, tz] },
    "front_right": { "rotation": [rx, ry, rz], "translation": [tx, ty, tz] }
  }
}
```
- `rotation`: Rodrigues rotation vector (rad)
- `translation`: camera position in vehicle body frame (meters)
- The program uses `T_body_to_cam` internally but outputs `T_cam_to_body` (same convention as input).

## Source Architecture

- **`src/run_front3_calib_v4.cpp`** — Main program (~1000+ LOC). Defines:
  - `FrontCameraConfig` struct holding intrinsics, extrinsics, and images for each of the 3 cameras
  - `CameraType` enum: `FISHEYE_OPENCV = 0`, `OCAM = 1`
  - `ViewMode` enum: `VIEW_ALL = 0`, `VIEW_FL = 1`, `VIEW_F = 2`, `VIEW_FR = 3`, `VIEW_FL_FR = 4`, `VIEW_FL_F = 5`, `VIEW_F_FR = 6`
  - Global `cameras[3]` indexed: 0=FL, 1=F, 2=FR with corresponding `camera_names` and `json_names` lookups
  - Pangolin GUI with sliders for `cali_frame` (camera selector), `cali_scale_degree_` (rot step, default 0.5°), `cali_scale_trans_` (trans step, default 0.05m = 5cm)
- **`src/calibration_metrics.hpp/cpp`** — Quality metrics (photometric loss, SSIM, edge alignment, feature matching, line preservation). Invoked on Save button press.
- **`src/ocam_verify.cpp`** — Standalone Ocam model verification utility (not linked into the main binary).

## GUI Controls

Keyboard shortcuts (active while GUI has focus):
- `q/a` — +/- X rotation (Roll)
- `w/s` — +/- Y rotation (Pitch)
- `e/d` — +/- Z rotation (Yaw)
- `r/f` — +/- X translation
- `t/g` — +/- Y translation
- `y/h` — +/- Z translation

Buttons:
- **Reset** — restore current camera extrinsics to initial values
- **Save**  — save all calibration results and compute quality metrics

## Outputs (on Save)

1. `calibration_front_left.txt` / `calibration_front.txt` / `calibration_front_right.txt` — per-camera calibration text
2. `front3_extrinsics_calibrated.json` — full extrinsics JSON (same format as input)
3. `front3_stitched.png` — stitched result image
4. `front3_calibration_metrics.txt` — quality evaluation report

## Quality Metrics

Metrics are computed for overlapping pairs (FL-F and F-FR). Key thresholds:
- Photometric Loss (RMSE): < 5 excellent, > 30 poor
- SSIM: > 0.9 excellent, < 0.5 poor
- Edge Alignment (IoU): > 0.75 excellent, < 0.35 poor
- Feature Matching (inliers): > 50 excellent, < 10 poor
- Overall score: 80-100 Excellent, 60-79 Good, 40-59 Fair, < 40 Poor

A Python visualization script is provided at `scripts/analyze_metrics.py`:
```bash
python scripts/analyze_metrics.py front3_calibration_metrics.txt [output.png]
```

## Wall Projection Parameters

The default front wall projection parameters (in meters) are:
- `wall_distance` = 1.8 (CLI overridable)
- `wall_width` = 12.0
- `wall_height` = 6.0
- `wall_center_y` = 0.0
- Output resolution: 2400×1200 (`output_width` × `output_height`)
- `meter_per_pixel` derived from `wall_width / output_width`

These affect the bird's-eye-style stitch visualization. If the projected image looks too zoomed in/out for a given scene, adjust `wall_distance` via the 4th CLI argument.
