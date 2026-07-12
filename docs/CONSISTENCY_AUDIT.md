# Consistency Audit Against Drafts

This file records the decisions made to keep the deployable software honest against the proposal drafts and rulebook.

## 1. Companion Compute: Jetson CUDA vs Raspberry Pi 5

Decision: the flight configuration targets `jetson_orin_nano` with `cuda` acceleration.

Reason: the final draft describes CUDA-optimized monocular depth. Raspberry Pi 5 does not provide CUDA, so it cannot be the default flight target for that claim. The repo still runs in `sim` or `bench` mode on Windows/RPi-class machines, but `flight` mode preflight warns when the configured CUDA target is not detected.

Relevant config:

```yaml
companion:
  platform: jetson_orin_nano
  accelerator: cuda
  require_accelerator_in_flight: true
  depth_model_enabled: false
```

`depth_model_enabled` remains false until the actual CUDA model file exists. Enabling it without `depth_model_path` is reported by preflight.

## 2. Geofence Authority

Decision: the companion polygon geofence is the precise arena geofence; ArduPilot native fences are a backup failsafe, not the primary boundary-strip controller.

Reason: the rulebook arena is approximately 35 ft by 25 ft, and the stop strip is a local polygon problem. A generic ArduPilot circular/radius fence is too coarse for this arena. The companion computes the exact signed boundary distance using dot-product half-spaces. If the drone reaches the configured action strip, the safety monitor issues a high-severity hold; the mission runner stops the survey loop and returns/lands. If the vehicle is outside the polygon, the safety monitor commands return-home.

Required hardware setup:

- Keep ArduPilot low-level failsafes enabled for link/battery/altitude.
- Do not rely on a large `FENCE_RADIUS` as the competition boundary.
- Validate companion geofence behavior with `max_waypoints: 1` and a netted/tethered arena before full runs.

## 3. Inner-Loop Control: LQR vs ArduPilot PID

Decision: this repository does not implement inner-loop LQR attitude control.

Reason: standard ArduPilot Copter uses cascaded PID attitude/rate loops. A true LQR inner loop would require either a custom ArduPilot firmware fork or a carefully validated offboard low-level rate controller, both of which are higher-risk than needed for the elimination software loop. This repo intentionally sends high-level local-NED targets over MAVLink and leaves real-time attitude stabilization to Pixhawk/Cube firmware.

Report implication: do not claim deployable LQR flight control unless the team adds and tests a firmware fork or low-level controller. The current defensible claim is:

> ArduPilot handles hard real-time stabilization and failsafes; the companion computer handles mission logic, perception, mapping, geofencing, transfer, validation, and charging confirmation.

## 4. SLAM, HORS, and RFA*

Decision: the current deployable stack uses coverage planning plus overlapping tile search, not full visual SLAM or RFA*.

Reason: the rulebook requires local coordinate reporting and feature matching, not necessarily full SLAM. The current map is a compact local survey map indexed in the base-station frame. The HORS-like behavior is implemented as overlapping tile search over HD frames. A small A* helper exists for future obstacle rerouting, but it is not the active planner.

Report implication: if the design report keeps the SLAM/RFA* language, the team must either implement those modules or label them as planned/future work. The current code-supported wording should be "local survey map + lawnmower coverage + tiled feature search."

## 5. Charging and BMS

Decision: charging confirmation is now a mission state backed by a power-monitor abstraction.

The repo supports:

- `sim`: immediate simulated charging/contact telemetry.
- `serial`: newline-delimited JSON telemetry from STM32/BMS over UART.
- `disabled`: explicit no-power-monitor mode.

The mission report records `charging_confirmed` and `final_power`.
