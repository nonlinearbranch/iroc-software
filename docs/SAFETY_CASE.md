# Safety Case

## Safety Responsibilities

ArduPilot/Pixhawk or Cube:

- Attitude stabilization.
- Motor mixing and ESC outputs.
- EKF and sensor health.
- Firmware-level failsafes.
- Manual/RC emergency intervention.

Companion computer:

- Mission state machine.
- Local-NED waypoint commands.
- Boundary strip/geofence decisions.
- Vision and coordinate logging.
- Data preservation and transfer.
- High-level emergency decisions.

## Companion Failsafe Triggers

- Critical battery voltage or percent: land now.
- Low battery voltage or percent: return home and land.
- Stale base-station/telemetry link: return home.
- Stale or unhealthy estimator: hold.
- Excessive velocity: hold.
- Inside configured boundary action strip: hold.
- Outside arena polygon: return home or land depending on available control.

## Notes for Real Flight

This software is not a substitute for ArduPilot pre-arm checks, prop safety, a netted test arena, RC override, or a physical kill switch. First real tests should use very short autonomous segments and low horizontal speed until the local coordinate frame, optical flow/rangefinder, and camera geometry are calibrated.
