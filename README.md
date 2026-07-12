# IRoC-U ASCEND Drone Software

This repository contains the companion-computer and base-station software for the IRoC-U 2026 ASCEND challenge. It is built around the constraints from the rulebook and team drafts:

- GNSS-denied autonomous takeoff, survey, coordinate logging, return, docking, charging, transfer, and validation.
- HD image capture at 1280x720 or higher, downsampled to 128x128 for matching.
- Local feature coordinates reported relative to the base station.
- Boundary-strip/geofence protection using a dot-product half-space check against the arena polygon.
- Pixhawk/Cube running ArduPilot for real-time stabilization, with a Jetson/RPi companion running mission logic, vision, mapping, and transfer.

The code can run in three modes:

- `sim`: no flight hardware; useful for vision and mission dry runs.
- `bench`: camera/network/base station checks without arming motors.
- `flight`: real MAVLink control via `pymavlink` on the companion computer.

## Quick Start

```powershell
python -m pip install -e .[dev]
python -m pytest
python -m iroc.cli preflight --config config/vehicle.example.yaml
python -m iroc.cli base-station --config config/vehicle.example.yaml
```

To test matching on folders of images:

```powershell
python -m iroc.cli validate --seed-dir path\to\seeds --capture-dir path\to\captures --out runs\validation
```

To run a dry mission over a directory of frames:

```powershell
python -m iroc.cli companion --config config/vehicle.example.yaml --seed-dir path\to\seeds --frame-dir path\to\frames --dry-run
```

To fetch seed images from the base station instead of using a local seed folder:

```powershell
python -m iroc.cli companion --config config/vehicle.example.yaml --fetch-seeds --dry-run
```

For bench checks before flight:

```powershell
python -m iroc.cli camera-check --config config/vehicle.example.yaml --frames 30
python -m iroc.cli mavlink-check --config config/vehicle.example.yaml --seconds 5
python -m iroc.cli power-check --config config/vehicle.example.yaml --seconds 5
```

To independently validate a transferred mission report on the base station:

```powershell
python -m iroc.cli base-validate --config config/vehicle.example.yaml --seed-dir path\to\seeds --report runs\mission_x\mission_report.json --image-dir runs\base_station\images
```

Read [docs/SYSTEM_DESIGN.md](docs/SYSTEM_DESIGN.md), [docs/CONSISTENCY_AUDIT.md](docs/CONSISTENCY_AUDIT.md), [docs/ARDUPILOT_INTEGRATION.md](docs/ARDUPILOT_INTEGRATION.md), and [docs/BRINGUP_CHECKLIST.md](docs/BRINGUP_CHECKLIST.md) before connecting to real hardware.
