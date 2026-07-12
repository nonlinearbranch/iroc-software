from __future__ import annotations

import importlib.util
from dataclasses import asdict
from pathlib import Path

import cv2
import numpy as np

from iroc.config import SystemConfig
from iroc.navigation.arena import ArenaBoundary


def preflight_report(config: SystemConfig) -> dict:
    packages = {
        "numpy": np.__version__,
        "opencv": cv2.__version__,
        "yaml": bool(importlib.util.find_spec("yaml")),
        "flask": bool(importlib.util.find_spec("flask")),
        "pymavlink": bool(importlib.util.find_spec("pymavlink")),
        "serial": bool(importlib.util.find_spec("serial")),
        "picamera2": bool(importlib.util.find_spec("picamera2")),
    }
    boundary = ArenaBoundary.from_config(config.arena)
    home_check = boundary.check((config.arena.home_x_m, config.arena.home_y_m))
    warnings: list[str] = []
    notes: list[str] = []
    if config.flight.mode.lower() in {"flight", "mavlink"} and not packages["pymavlink"]:
        warnings.append("pymavlink is required for real Pixhawk/Cube flight mode")
    if config.power.mode.lower() in {"serial", "uart", "bms"} and not packages["serial"]:
        warnings.append("pyserial is required for serial STM32/BMS power monitoring")
    if not home_check.inside:
        warnings.append("configured home point is outside the arena")
    if home_check.in_stop_strip:
        notes.append("configured home point is inside the boundary stop strip; this is normal if the base station is near the arena corner")
    if config.camera.width_px < 1280 or config.camera.height_px < 720:
        warnings.append("camera config is below the rulebook HD minimum of 1280x720")
    run_root = Path(config.storage.run_root)
    run_root.mkdir(parents=True, exist_ok=True)
    return {
        "ok": not warnings,
        "warnings": warnings,
        "notes": notes,
        "packages": packages,
        "config": asdict(config),
    }
