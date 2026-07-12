# ASCEND Software System Design

## Requirements Implemented

The IRoC-U 2026 elimination rulebook requires an autonomous micro-UAV to ingest 3 to 5 reference feature images, survey a 35 ft by 25 ft arena without GNSS or external navigation aids, capture HD imagery at 1280x720 or better, downsample to 128x128 for matching, report local feature coordinates relative to the base station, land only at the home/base station after takeoff, demonstrate autonomous charging, transfer data, and validate results without human intervention.

This repository implements the software side as a companion-computer stack:

1. ArduPilot on Pixhawk/Cube remains responsible for attitude control, motor mixing, EKF, arming, and firmware failsafes.
2. The companion computer runs mission state, waypoint survey, boundary/geofence checks, image matching, local mapping, storage, and base-station transfer.
3. The base station accepts seed images, start commands, reports, and image uploads over HTTP/Wi-Fi. Wired transfer can mirror the same files after docking.
4. The base station can independently validate transferred HD evidence against seed images instead of trusting the onboard report.
5. The companion can fetch seed images from the base station before takeoff, so the reference-ingestion step can be part of the autonomous workflow.
6. The companion reads BMS/charging telemetry after docking and records whether charging was autonomously confirmed.

## Runtime Modes

- `sim`: no hardware required. Uses a simulated flight controller and optionally a directory of images.
- `bench`: same software path as `sim`, but intended for live camera/base-station/power tests without arming.
- `flight`: targets the Jetson CUDA companion profile and uses `pymavlink` to command ArduPilot in guided local-NED mode.

## Boundary Strip Logic

The seniors' dot-product idea is the right primitive. The arena is represented as a counter-clockwise polygon. Each edge gets an inward unit normal. For every point:

```text
signed_distance_to_edge = dot(point - edge_start, inward_normal)
```

If all signed distances are positive, the drone is inside the polygon. The smallest distance tells us how close the drone is to the boundary strip. If that value falls below the configured action margin, the safety monitor commands a hold before the vehicle crosses the strip. If it becomes negative, the system treats the vehicle as outside and returns/lands depending on severity.

## Vision Pipeline

The pipeline follows the rulebook's HD-to-LR matching requirement:

1. Capture or load a full-resolution frame.
2. Generate 128x128 LR variants using area, Lanczos, Gaussian-area, and center-crop-area methods.
3. Run ORB by default for fast onboard matching. SIFT can be enabled for stricter validation if the compute budget allows it.
4. Use ratio-tested descriptor matches and RANSAC homography inliers to reject false positives.
5. Run whole-frame matching plus overlapping tile search, which is the practical version of HORS for this build.
6. Convert the detected pixel center to local arena coordinates using the nadir camera FOV and current vehicle pose.

## Map and Memory Model

`SurveyMap` stores coverage in `uint16` and confidence in `float32`, exported as compressed NPZ plus JSON. Full HD frames are not kept in RAM; only the current frame is processed, and accepted detection images are written to disk immediately with checksums available at transfer time.

## Flight Control Boundary

The companion sends local-NED position targets to ArduPilot. It does not try to replace the flight controller's attitude loop. This is intentional: Pixhawk/Cube runs hard real-time control, while the companion is allowed to be slower and more failure-tolerant.

The repository does not implement LQR inner-loop attitude control. See [CONSISTENCY_AUDIT.md](CONSISTENCY_AUDIT.md) for the explicit control-scope decision.

## Data Products

Each mission run writes:

- `mission_report.json`: state transitions, safety events, detections, paths.
- `charging_confirmed` and `final_power` in `mission_report.json`: proof of charging contact/current/SOC telemetry after docking.
- `survey_map.json`: compact map metadata and detections.
- `survey_map.npz`: coverage and confidence arrays.
- `detections/<seed>/<frame>.jpg`: HD evidence image.
- `detections/<seed>/<frame>_lr.jpg`: 128x128 validation image.

## Base-Station Validation

The base station re-runs seed matching over transferred HD evidence with overlapping tiles:

```powershell
python -m iroc.cli base-validate --seed-dir path\to\seeds --report path\to\mission_report.json --image-dir path\to\uploaded_images
```

The HTTP server exposes the same behavior at `POST /validate/latest` after seeds, report, and images have been uploaded.
