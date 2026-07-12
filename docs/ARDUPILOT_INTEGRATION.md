# ArduPilot Integration Notes

## Control Boundary

Pixhawk/Cube with ArduPilot owns:

- attitude stabilization
- motor outputs
- EKF and sensor fusion
- firmware-level failsafes
- arming and disarming safety

The companion owns:

- local mission state
- seed/image matching
- local survey map
- polygon arena boundary checks
- high-level `SET_POSITION_TARGET_LOCAL_NED` commands
- transfer, validation, and charging confirmation

## MAVLink Messages Used

- `HEARTBEAT`: armed state, mode, link freshness.
- `LOCAL_POSITION_NED`: local position, velocity, estimator freshness.
- `ATTITUDE`: yaw for camera pixel-to-local coordinate projection.
- `BATTERY_STATUS`: battery voltage and remaining percent.
- `SET_POSITION_TARGET_LOCAL_NED`: local waypoint targets in guided mode.
- `MAV_CMD_NAV_TAKEOFF`: takeoff command.

## Pre-Flight Parameters to Verify

These are setup checks, not a blindly flashable parameter file:

- `SERIALx_PROTOCOL` for the companion telemetry port is MAVLink.
- `SERIALx_BAUD` matches `flight.baud`.
- `EK3_ENABLE=1` if EKF3 is used.
- Optical flow and rangefinder sources are configured and visible to the EKF.
- GPS is not used as a navigation aid for the challenge run.
- Battery failsafe and RC/manual emergency action are enabled.
- ArduPilot geofence/altitude failsafes are configured as backup limits.

The precise arena boundary is handled by the companion polygon geofence, because the rulebook arena is smaller and more rectangular than a broad native radius fence.

## Props-Off MAVLink Check

```powershell
python -m iroc.cli mavlink-check --config config/vehicle.example.yaml --seconds 5
```

Expected:

- heartbeat samples arrive
- `LOCAL_POSITION_NED` updates
- link age remains low
- estimator age remains low
- mode and battery status are populated
