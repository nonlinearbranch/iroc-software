from __future__ import annotations

import importlib
import logging
import math
import time
from typing import Protocol

from iroc.config import FlightConfig
from iroc.navigation.planner import Waypoint
from iroc.types import PoseNED, Telemetry, VelocityNED

LOG = logging.getLogger(__name__)


class FlightController(Protocol):
    def connect(self) -> None: ...

    def telemetry(self) -> Telemetry: ...

    def arm(self) -> None: ...

    def takeoff(self, altitude_m: float) -> None: ...

    def goto(self, waypoint: Waypoint, timeout_s: float | None = None) -> None: ...

    def hold(self) -> None: ...

    def return_home(self) -> None: ...

    def land(self) -> None: ...

    def close(self) -> None: ...


class SimFlightController:
    """Deterministic local controller used for bench tests and mission dry runs."""

    def __init__(self, config: FlightConfig) -> None:
        self.config = config
        self.pose = PoseNED()
        self.velocity = VelocityNED()
        self.armed = False
        self.mode = "SIM"
        self.connected = False
        self.battery_v = 24.0
        self.battery_pct = 100.0

    def connect(self) -> None:
        self.connected = True
        LOG.info("Sim flight controller connected")

    def telemetry(self) -> Telemetry:
        return Telemetry(
            pose=self.pose,
            velocity=self.velocity,
            battery_v=self.battery_v,
            battery_remaining_pct=self.battery_pct,
            armed=self.armed,
            mode=self.mode,
            timestamp_s=time.time(),
        )

    def arm(self) -> None:
        self.armed = True
        self.mode = "GUIDED"

    def takeoff(self, altitude_m: float) -> None:
        self.pose = PoseNED(self.pose.x_m, self.pose.y_m, -abs(altitude_m), self.pose.yaw_rad)
        self.mode = "GUIDED"

    def goto(self, waypoint: Waypoint, timeout_s: float | None = None) -> None:
        distance = math.dist((self.pose.x_m, self.pose.y_m, self.pose.z_m), (waypoint.x_m, waypoint.y_m, waypoint.z_m))
        duration = max(0.05, distance / max(0.1, self.config.max_xy_velocity_mps))
        self.velocity = VelocityNED(
            (waypoint.x_m - self.pose.x_m) / duration,
            (waypoint.y_m - self.pose.y_m) / duration,
            (waypoint.z_m - self.pose.z_m) / duration,
        )
        self.pose = PoseNED(waypoint.x_m, waypoint.y_m, waypoint.z_m, waypoint.yaw_rad)
        self.battery_pct = max(0.0, self.battery_pct - 0.15)
        self.battery_v = max(19.0, 20.4 + 3.6 * (self.battery_pct / 100.0))
        self.velocity = VelocityNED()

    def hold(self) -> None:
        self.mode = "HOLD"
        self.velocity = VelocityNED()

    def return_home(self) -> None:
        self.mode = "RTL"
        self.goto(Waypoint(0.0, 0.0, self.pose.z_m, self.pose.yaw_rad), self.config.goto_timeout_s)

    def land(self) -> None:
        self.pose = PoseNED(self.pose.x_m, self.pose.y_m, 0.0, self.pose.yaw_rad)
        self.mode = "LAND"
        self.armed = False

    def close(self) -> None:
        self.connected = False


class PymavlinkFlightController:
    """MAVLink position-target adapter for ArduPilot guided operation.

    This class intentionally keeps Pixhawk/Cube attitude control inside ArduPilot.
    The companion only sends high-level local-NED position targets and mode/failsafe
    commands.
    """

    def __init__(self, config: FlightConfig) -> None:
        self.config = config
        self.master = None
        self._mavutil = None
        self._last_telemetry = Telemetry()

    def connect(self) -> None:
        try:
            self._mavutil = importlib.import_module("pymavlink.mavutil")
        except ImportError as exc:
            raise RuntimeError("Install pymavlink on the companion computer for real flight mode") from exc
        self.master = self._mavutil.mavlink_connection(self.config.mavlink_url, baud=self.config.baud)
        self.master.wait_heartbeat(timeout=10)
        LOG.info("MAVLink heartbeat from system %s component %s", self.master.target_system, self.master.target_component)

    def telemetry(self) -> Telemetry:
        if self.master is None:
            raise RuntimeError("MAVLink controller is not connected")
        end = time.time() + 0.03
        while time.time() < end:
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                break
            msg_type = msg.get_type()
            if msg_type == "LOCAL_POSITION_NED":
                self._last_telemetry.pose = PoseNED(float(msg.x), float(msg.y), float(msg.z), self._last_telemetry.pose.yaw_rad)
                self._last_telemetry.velocity = VelocityNED(float(msg.vx), float(msg.vy), float(msg.vz))
            elif msg_type == "ATTITUDE":
                self._last_telemetry.pose.yaw_rad = float(msg.yaw)
            elif msg_type == "BATTERY_STATUS":
                voltages = [v for v in msg.voltages if 0 < v < 65535]
                if voltages:
                    self._last_telemetry.battery_v = sum(voltages) / 1000.0
                if msg.battery_remaining >= 0:
                    self._last_telemetry.battery_remaining_pct = float(msg.battery_remaining)
            elif msg_type == "HEARTBEAT":
                self._last_telemetry.armed = bool(msg.base_mode & self._mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self._last_telemetry.mode = self._mavutil.mode_string_v10(msg)
                self._last_telemetry.link_age_s = 0.0
        self._last_telemetry.timestamp_s = time.time()
        return self._last_telemetry

    def arm(self) -> None:
        self._require_master()
        self.master.arducopter_arm()
        self.master.motors_armed_wait(timeout=10)

    def takeoff(self, altitude_m: float) -> None:
        self._require_master()
        self._set_mode("GUIDED")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            self._mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            float(altitude_m),
        )
        time.sleep(0.5)

    def goto(self, waypoint: Waypoint, timeout_s: float | None = None) -> None:
        self._require_master()
        timeout = timeout_s or self.config.goto_timeout_s
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._send_position_target(waypoint)
            telemetry = self.telemetry()
            distance_xy = math.dist((telemetry.pose.x_m, telemetry.pose.y_m), (waypoint.x_m, waypoint.y_m))
            distance_z = abs(telemetry.pose.z_m - waypoint.z_m)
            if distance_xy <= self.config.waypoint_tolerance_m and distance_z <= 0.4:
                return
            time.sleep(0.2)
        raise TimeoutError(f"Timed out reaching waypoint {waypoint}")

    def hold(self) -> None:
        self._set_mode("LOITER")

    def return_home(self) -> None:
        self._set_mode("RTL")

    def land(self) -> None:
        self._set_mode("LAND")

    def close(self) -> None:
        self.master = None

    def _send_position_target(self, waypoint: Waypoint) -> None:
        self.master.mav.set_position_target_local_ned_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            self.master.target_system,
            self.master.target_component,
            self._mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,
            float(waypoint.x_m),
            float(waypoint.y_m),
            float(waypoint.z_m),
            0,
            0,
            0,
            0,
            0,
            0,
            float(waypoint.yaw_rad),
            0,
        )

    def _set_mode(self, mode: str) -> None:
        self._require_master()
        if mode not in self.master.mode_mapping():
            raise RuntimeError(f"Mode {mode!r} is not available from vehicle")
        self.master.set_mode(self.master.mode_mapping()[mode])

    def _require_master(self) -> None:
        if self.master is None or self._mavutil is None:
            raise RuntimeError("MAVLink controller is not connected")


def make_flight_controller(config: FlightConfig) -> FlightController:
    if config.mode.lower() in {"sim", "dry-run", "dry_run", "bench"}:
        return SimFlightController(config)
    if config.mode.lower() in {"flight", "mavlink"}:
        return PymavlinkFlightController(config)
    raise ValueError(f"Unknown flight mode: {config.mode}")
