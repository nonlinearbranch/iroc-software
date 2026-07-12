from __future__ import annotations

import math

from iroc.types import PoseNED


class DownwardCameraModel:
    """Simple nadir-camera projection for local feature coordinate estimates."""

    def __init__(self, width_px: int, height_px: int, h_fov_deg: float, v_fov_deg: float) -> None:
        self.width_px = width_px
        self.height_px = height_px
        self.h_fov_rad = math.radians(h_fov_deg)
        self.v_fov_rad = math.radians(v_fov_deg)

    def pixel_to_local_xy(self, pixel: tuple[float, float], pose: PoseNED) -> tuple[float, float]:
        altitude = max(0.05, pose.altitude_m)
        footprint_x = 2.0 * altitude * math.tan(self.v_fov_rad / 2.0)
        footprint_y = 2.0 * altitude * math.tan(self.h_fov_rad / 2.0)
        px, py = pixel
        forward_body = -((py - self.height_px / 2.0) / self.height_px) * footprint_x
        right_body = ((px - self.width_px / 2.0) / self.width_px) * footprint_y
        cos_yaw = math.cos(pose.yaw_rad)
        sin_yaw = math.sin(pose.yaw_rad)
        north = forward_body * cos_yaw - right_body * sin_yaw
        east = forward_body * sin_yaw + right_body * cos_yaw
        return pose.x_m + north, pose.y_m + east
