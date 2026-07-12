from __future__ import annotations

import math

from iroc.config import SafetyConfig
from iroc.navigation.arena import ArenaBoundary
from iroc.types import SafetyAction, SafetyEvent, Telemetry


class SafetyMonitor:
    def __init__(self, config: SafetyConfig, boundary: ArenaBoundary) -> None:
        self.config = config
        self.boundary = boundary

    def evaluate(self, telemetry: Telemetry) -> SafetyEvent:
        if telemetry.battery_v and telemetry.battery_v <= self.config.critical_battery_v:
            return SafetyEvent(SafetyAction.LAND_NOW, f"critical battery voltage {telemetry.battery_v:.2f} V", 3)
        if telemetry.battery_remaining_pct and telemetry.battery_remaining_pct <= self.config.critical_battery_pct:
            return SafetyEvent(SafetyAction.LAND_NOW, f"critical battery {telemetry.battery_remaining_pct:.1f}%", 3)
        if telemetry.battery_v and telemetry.battery_v <= self.config.min_battery_v:
            return SafetyEvent(SafetyAction.RETURN_HOME, f"low battery voltage {telemetry.battery_v:.2f} V", 2)
        if telemetry.battery_remaining_pct and telemetry.battery_remaining_pct <= self.config.min_battery_pct:
            return SafetyEvent(SafetyAction.RETURN_HOME, f"low battery {telemetry.battery_remaining_pct:.1f}%", 2)
        if telemetry.link_age_s > self.config.max_link_age_s:
            return SafetyEvent(SafetyAction.RETURN_HOME, f"link stale for {telemetry.link_age_s:.2f} s", 2)
        if not telemetry.estimator_ok or telemetry.estimator_age_s > self.config.max_estimator_age_s:
            return SafetyEvent(SafetyAction.HOLD, "estimator unhealthy or stale", 2)

        speed = math.sqrt(
            telemetry.velocity.vx_mps**2 + telemetry.velocity.vy_mps**2 + telemetry.velocity.vz_mps**2
        )
        if speed > self.config.max_speed_mps:
            return SafetyEvent(SafetyAction.HOLD, f"speed {speed:.2f} m/s exceeds limit", 1)

        boundary = self.boundary.check(telemetry.pose.xy(), self.config.boundary_action_margin_m)
        if not boundary.inside:
            return SafetyEvent(SafetyAction.RETURN_HOME, f"outside arena by {-boundary.min_distance_m:.2f} m", 3)
        if boundary.in_stop_strip:
            return SafetyEvent(
                SafetyAction.HOLD,
                f"boundary strip reached: {boundary.min_distance_m:.2f} m from edge {boundary.nearest_edge_index}",
                2,
            )
        return SafetyEvent(SafetyAction.NONE, "ok", 0)
