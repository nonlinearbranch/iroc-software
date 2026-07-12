from __future__ import annotations

import importlib
import json
import time
from typing import Protocol

from iroc.config import PowerConfig
from iroc.types import PowerStatus


class PowerMonitor(Protocol):
    def connect(self) -> None: ...

    def read_status(self) -> PowerStatus: ...

    def close(self) -> None: ...


class SimPowerMonitor:
    def __init__(self, config: PowerConfig) -> None:
        self.config = config
        self.connected = False
        self.soc_pct = 62.0
        self._reads = 0

    def connect(self) -> None:
        self.connected = True

    def read_status(self) -> PowerStatus:
        self._reads += 1
        if self.connected:
            self.soc_pct = min(100.0, self.soc_pct + 0.15)
        return PowerStatus(
            voltage_v=22.8 + (self.soc_pct - 50.0) * 0.018,
            current_a=1.4 if self.connected else 0.0,
            soc_pct=self.soc_pct,
            temperature_c=31.0,
            charging=self.connected,
            contact_detected=self.connected,
            timestamp_s=time.time(),
            source="sim",
        )

    def close(self) -> None:
        self.connected = False


class DisabledPowerMonitor:
    def connect(self) -> None:
        return None

    def read_status(self) -> PowerStatus:
        return PowerStatus(timestamp_s=time.time(), source="disabled")

    def close(self) -> None:
        return None


class SerialPowerMonitor:
    """Reads newline-delimited JSON telemetry from the STM32/BMS UART.

    Expected keys are intentionally simple and firmware-friendly:
    voltage_v/current_a/soc_pct/temperature_c/charging/contact_detected.
    Short aliases such as voltage, current, soc, temp, and contact are accepted.
    """

    def __init__(self, config: PowerConfig) -> None:
        self.config = config
        self.serial = None

    def connect(self) -> None:
        try:
            serial_module = importlib.import_module("serial")
        except ImportError as exc:
            raise RuntimeError("Install pyserial for serial BMS power monitoring") from exc
        self.serial = serial_module.Serial(
            self.config.serial_url,
            self.config.baud,
            timeout=self.config.read_timeout_s,
        )

    def read_status(self) -> PowerStatus:
        if self.serial is None:
            raise RuntimeError("Serial BMS monitor is not connected")
        line = self.serial.readline().decode("utf-8", errors="replace").strip()
        if not line:
            return PowerStatus(timestamp_s=time.time(), source="serial_timeout")
        try:
            payload = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(f"Invalid BMS JSON line: {line!r}") from exc
        return PowerStatus(
            voltage_v=float(payload.get("voltage_v", payload.get("voltage", 0.0))),
            current_a=float(payload.get("current_a", payload.get("current", 0.0))),
            soc_pct=float(payload.get("soc_pct", payload.get("soc", 0.0))),
            temperature_c=float(payload.get("temperature_c", payload.get("temp", 0.0))),
            charging=bool(payload.get("charging", False)),
            contact_detected=bool(payload.get("contact_detected", payload.get("contact", False))),
            timestamp_s=time.time(),
            source="serial",
        )

    def close(self) -> None:
        if self.serial is not None:
            self.serial.close()
            self.serial = None


def make_power_monitor(config: PowerConfig) -> PowerMonitor:
    mode = config.mode.lower()
    if mode in {"sim", "bench"}:
        return SimPowerMonitor(config)
    if mode in {"serial", "uart", "bms"}:
        return SerialPowerMonitor(config)
    if mode in {"disabled", "none", "off"}:
        return DisabledPowerMonitor()
    raise ValueError(f"Unknown power monitor mode: {config.mode}")


def charging_confirmed(status: PowerStatus, config: PowerConfig, initial_soc_pct: float | None = None) -> bool:
    if not status.contact_detected:
        return False
    current_ok = status.charging or status.current_a >= config.min_charge_current_a
    soc_ok = (
        initial_soc_pct is not None
        and status.soc_pct > 0.0
        and status.soc_pct - initial_soc_pct >= config.min_soc_increase_pct
    )
    return bool(current_ok or soc_ok)
