from __future__ import annotations

from dataclasses import asdict, dataclass, field, fields, is_dataclass
from pathlib import Path
from typing import Any, TypeVar

import yaml


T = TypeVar("T")


@dataclass(slots=True)
class ArenaConfig:
    width_m: float = 10.668
    height_m: float = 7.62
    origin_x_m: float = 0.0
    origin_y_m: float = 0.0
    boundary_margin_m: float = 0.75
    map_resolution_m: float = 0.20
    lane_spacing_m: float = 1.25
    home_x_m: float = 0.75
    home_y_m: float = 0.75


@dataclass(slots=True)
class CameraConfig:
    source: str = "opencv"
    device_index: int = 0
    frame_dir: str = ""
    width_px: int = 1280
    height_px: int = 720
    fps: int = 30
    h_fov_deg: float = 62.2
    v_fov_deg: float = 48.8


@dataclass(slots=True)
class VisionConfig:
    detector: str = "ORB"
    lr_size_px: int = 128
    ratio_test: float = 0.75
    min_good_matches: int = 18
    min_inliers: int = 8
    min_score: float = 0.70
    tile_size_px: int = 420
    tile_overlap_px: int = 140
    max_detections_per_seed: int = 3
    methods: tuple[str, ...] = ("area", "lanczos", "gaussian_area", "center_crop_area")


@dataclass(slots=True)
class FlightConfig:
    mode: str = "sim"
    mavlink_url: str = "udpin:127.0.0.1:14550"
    baud: int = 921600
    takeoff_altitude_m: float = 2.5
    survey_altitude_m: float = 3.0
    max_xy_velocity_mps: float = 0.8
    goto_timeout_s: float = 18.0
    waypoint_tolerance_m: float = 0.35
    land_timeout_s: float = 45.0


@dataclass(slots=True)
class CommsConfig:
    base_url: str = "http://127.0.0.1:5050"
    bind_host: str = "0.0.0.0"
    port: int = 5050
    transfer_dir: str = "runs/base_station"
    timeout_s: float = 5.0


@dataclass(slots=True)
class SafetyConfig:
    min_battery_v: float = 21.6
    critical_battery_v: float = 20.4
    min_battery_pct: float = 25.0
    critical_battery_pct: float = 15.0
    max_link_age_s: float = 2.0
    max_estimator_age_s: float = 0.75
    max_speed_mps: float = 1.8
    boundary_action_margin_m: float = 0.35


@dataclass(slots=True)
class PowerConfig:
    mode: str = "sim"
    serial_url: str = "COM5"
    baud: int = 9600
    read_timeout_s: float = 1.0
    charge_confirm_timeout_s: float = 20.0
    min_charge_current_a: float = 0.25
    min_soc_increase_pct: float = 0.2


@dataclass(slots=True)
class StorageConfig:
    run_root: str = "runs"
    keep_full_hd: bool = True
    jpeg_quality: int = 92
    max_cached_frames: int = 12


@dataclass(slots=True)
class SystemConfig:
    arena: ArenaConfig = field(default_factory=ArenaConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    vision: VisionConfig = field(default_factory=VisionConfig)
    flight: FlightConfig = field(default_factory=FlightConfig)
    comms: CommsConfig = field(default_factory=CommsConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    power: PowerConfig = field(default_factory=PowerConfig)
    storage: StorageConfig = field(default_factory=StorageConfig)


def _merge_dataclass(instance: T, data: dict[str, Any]) -> T:
    if not is_dataclass(instance):
        raise TypeError(f"{instance!r} is not a dataclass")
    values = asdict(instance)
    field_types = {field.name: getattr(instance, field.name) for field in fields(instance)}
    for key, value in data.items():
        if key not in values:
            raise KeyError(f"Unknown config key: {key}")
        current = field_types[key]
        if is_dataclass(current) and isinstance(value, dict):
            values[key] = _merge_dataclass(current, value)
        elif isinstance(current, tuple) and isinstance(value, list):
            values[key] = tuple(value)
        else:
            values[key] = value
    return type(instance)(**values)


def load_config(path: str | Path | None = None) -> SystemConfig:
    config = SystemConfig()
    if not path:
        return config
    config_path = Path(path)
    if not config_path.exists():
        raise FileNotFoundError(config_path)
    with config_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    return _merge_dataclass(config, data)


def write_example_config(path: str | Path) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(asdict(SystemConfig()), handle, sort_keys=False)
