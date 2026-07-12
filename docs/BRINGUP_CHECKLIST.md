# Bring-Up Checklist

## 1. Software-Only

```powershell
python -m pip install -e .[dev]
python -m pytest
python -m iroc.cli preflight --config config/vehicle.example.yaml --allow-warnings
python tools/generate_synthetic_dataset.py --out runs/synthetic
python -m iroc.cli validate --seed-dir runs/synthetic/seeds --capture-dir runs/synthetic/frames --out runs/synthetic/validation
python -m iroc.cli camera-check --config config/vehicle.example.yaml --frame-dir runs/synthetic/frames --frames 3
python -m iroc.cli power-check --config config/vehicle.example.yaml --seconds 2
python -m iroc.cli companion --config config/vehicle.example.yaml --seed-dir runs/synthetic/seeds --frame-dir runs/synthetic/frames --dry-run --no-transfer --max-waypoints 6
```

Expected result: tests pass, preflight prints only missing real-flight package warnings on a laptop, validation accepts the synthetic target frames, and the dry mission writes a report under `runs/mission_*`.

After a mission report exists, validate it the same way the base station will:

```powershell
python -m iroc.cli base-validate --config config/vehicle.example.yaml --seed-dir runs/synthetic/seeds --report runs/mission_<id>/mission_report.json --image-dir runs/mission_<id>/detections
```

## 2. Base Station

```powershell
python -m iroc.cli base-station --config config/vehicle.example.yaml
```

Check:

- `GET /health` returns `ok: true`.
- Seed images can be uploaded through `POST /seed/<name>`.
- Seed images can be downloaded by the companion through `GET /seed/<name>/download`, or automatically with `--fetch-seeds`.
- Reports arrive under `runs/base_station/reports`.
- Image evidence arrives under `runs/base_station/images`.
- `POST /validate/latest` independently validates the latest report against uploaded seeds and evidence images.

## 3. Companion Bench

Set `camera.source: opencv`, verify camera index, and run:

```powershell
python -m iroc.cli companion --config config/vehicle.example.yaml --seed-dir path\to\seeds --dry-run --no-transfer --max-waypoints 3
```

Check:

- Camera produces 1280x720 or better.
- Detections are written only when confidence/inliers pass thresholds.
- CPU temperature and memory remain stable for at least one full sortie duration.

## 4. Pixhawk/Cube Integration

Install real-flight dependencies on the companion computer:

```bash
python -m pip install -r requirements.txt
```

Set:

```yaml
flight:
  mode: flight
  mavlink_url: /dev/ttyAMA0
  baud: 921600
```

Before props:

- Confirm ArduPilot heartbeat.
- Confirm `LOCAL_POSITION_NED`, `ATTITUDE`, and `BATTERY_STATUS` are received.
- Run `python -m iroc.cli mavlink-check --config config/vehicle.example.yaml --seconds 5`.
- Run `python -m iroc.cli camera-check --config config/vehicle.example.yaml --frames 30` and confirm HD resolution.
- Run `python -m iroc.cli power-check --config config/vehicle.example.yaml --seconds 5` and confirm contact/current/SOC telemetry.
- Confirm optical flow/rangefinder/EKF source configuration in Mission Planner or MAVProxy.
- Confirm emergency kill switch and RC/manual override.
- Confirm `preflight` has no flight-mode dependency warnings.

## 5. Tethered Low-Risk Flight

- Use prop guards/net.
- Start at 2 m minimum survey altitude as required by the rulebook.
- Use `max_waypoints: 1` for the first autonomous movement.
- Confirm the boundary monitor issues hold/return events before the configured strip.
- Confirm report/image transfer and `charging_confirmed: true` after landing.

## 6. Competition Rehearsal

- Record one continuous fixed-camera video.
- Show autonomous takeoff, survey, coordinate determination, landing, charging indication, transfer, validation, and result report.
- Keep seed images, HD captures, LR images, report JSON, and map files for audit.
