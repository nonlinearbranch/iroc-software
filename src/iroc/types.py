from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any


class FrameStatus(str, Enum):
    OK = "ok"
    END_OF_STREAM = "end_of_stream"
    CAMERA_ERROR = "camera_error"


class MissionState(str, Enum):
    IDLE = "idle"
    PREFLIGHT = "preflight"
    SEEDING = "seeding"
    TAKEOFF = "takeoff"
    SURVEY = "survey"
    RETURN_HOME = "return_home"
    LANDING = "landing"
    TRANSFER = "transfer"
    CHARGING = "charging"
    COMPLETE = "complete"
    ABORT = "abort"


class SafetyAction(str, Enum):
    NONE = "none"
    HOLD = "hold"
    RETURN_HOME = "return_home"
    LAND_NOW = "land_now"
    ABORT = "abort"


@dataclass(slots=True)
class PoseNED:
    """Local pose relative to the base station/home frame.

    ArduPilot local coordinates are NED. `z_m` is negative above the arena. Helper
    methods expose altitude as a positive value.
    """

    x_m: float = 0.0
    y_m: float = 0.0
    z_m: float = 0.0
    yaw_rad: float = 0.0

    @property
    def altitude_m(self) -> float:
        return max(0.0, -self.z_m)

    def xy(self) -> tuple[float, float]:
        return (self.x_m, self.y_m)


@dataclass(slots=True)
class VelocityNED:
    vx_mps: float = 0.0
    vy_mps: float = 0.0
    vz_mps: float = 0.0


@dataclass(slots=True)
class Telemetry:
    pose: PoseNED = field(default_factory=PoseNED)
    velocity: VelocityNED = field(default_factory=VelocityNED)
    battery_v: float = 0.0
    battery_remaining_pct: float = 0.0
    armed: bool = False
    mode: str = "UNKNOWN"
    estimator_ok: bool = True
    estimator_age_s: float = 0.0
    link_age_s: float = 0.0
    timestamp_s: float = 0.0


@dataclass(slots=True)
class PowerStatus:
    voltage_v: float = 0.0
    current_a: float = 0.0
    soc_pct: float = 0.0
    temperature_c: float = 0.0
    charging: bool = False
    contact_detected: bool = False
    timestamp_s: float = 0.0
    source: str = "unknown"


@dataclass(slots=True)
class FramePacket:
    status: FrameStatus
    image: Any | None = None
    frame_id: str = ""
    timestamp_s: float = 0.0
    source_path: Path | None = None


@dataclass(slots=True)
class MatchResult:
    seed_name: str
    score: float
    inliers: int
    good_matches: int
    method: str
    similarity: float = 0.0
    pixel_center: tuple[float, float] | None = None
    tile: tuple[int, int, int, int] | None = None
    homography: list[list[float]] | None = None


@dataclass(slots=True)
class Detection:
    seed_name: str
    confidence: float
    local_x_m: float
    local_y_m: float
    altitude_m: float
    image_path: str
    lr_path: str
    frame_id: str
    timestamp_s: float
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass(slots=True)
class SafetyEvent:
    action: SafetyAction
    reason: str
    severity: int = 0

    @property
    def active(self) -> bool:
        return self.action is not SafetyAction.NONE


@dataclass(slots=True)
class MissionReport:
    mission_id: str
    state: MissionState
    detections: list[Detection] = field(default_factory=list)
    map_path: str = ""
    events: list[dict[str, Any]] = field(default_factory=list)
    charging_confirmed: bool = False
    final_power: PowerStatus | None = None
    started_s: float = 0.0
    finished_s: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["state"] = self.state.value
        return data
