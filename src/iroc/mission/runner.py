from __future__ import annotations

import logging
import time
from pathlib import Path

from iroc.comms.client import BaseStationClient
from iroc.config import SystemConfig
from iroc.flight.controller import FlightController
from iroc.navigation.arena import ArenaBoundary
from iroc.navigation.map import SurveyMap
from iroc.navigation.planner import CoveragePlanner, Waypoint
from iroc.power.monitor import PowerMonitor, charging_confirmed
from iroc.safety.monitor import SafetyMonitor
from iroc.storage import atomic_write_json, ensure_dir
from iroc.types import FrameStatus, MissionReport, MissionState, SafetyAction, SafetyEvent
from iroc.vision.camera import CameraSource
from iroc.vision.detector import SurveyDetector

LOG = logging.getLogger(__name__)


class MissionRunner:
    def __init__(
        self,
        config: SystemConfig,
        flight: FlightController,
        camera: CameraSource,
        detector: SurveyDetector,
        run_dir: str | Path,
        client: BaseStationClient | None = None,
        power: PowerMonitor | None = None,
    ) -> None:
        self.config = config
        self.flight = flight
        self.camera = camera
        self.detector = detector
        self.power = power
        self.run_dir = ensure_dir(run_dir)
        self.boundary = ArenaBoundary.from_config(config.arena)
        self.safety = SafetyMonitor(config.safety, self.boundary)
        self.map = SurveyMap(config.arena)
        self.planner = CoveragePlanner(config.arena, self.boundary)
        self.client = client
        self.report = MissionReport(
            mission_id=self.run_dir.name,
            state=MissionState.IDLE,
            started_s=time.time(),
        )

    def run(self, target_count: int | None = None, max_waypoints: int | None = None) -> MissionReport:
        target_total = target_count or max(1, len(self.detector.matcher.seed_names))
        try:
            self._state(MissionState.PREFLIGHT)
            self.flight.connect()
            if self.power is not None:
                self.power.connect()

            self._state(MissionState.TAKEOFF)
            self.flight.arm()
            self.flight.takeoff(self.config.flight.takeoff_altitude_m)

            self._state(MissionState.SURVEY)
            waypoints = self.planner.lawnmower(self.config.flight.survey_altitude_m)
            if max_waypoints is not None:
                waypoints = waypoints[:max_waypoints]

            for waypoint in waypoints:
                self.flight.goto(waypoint, self.config.flight.goto_timeout_s)
                telemetry = self.flight.telemetry()
                safety_event = self.safety.evaluate(telemetry)
                if safety_event.active:
                    self._event("safety", safety_event.reason, action=safety_event.action.value)
                    if self._handle_safety(safety_event):
                        break

                self.map.mark_pose(telemetry.pose)
                packet = self.camera.read()
                if packet.status is FrameStatus.END_OF_STREAM:
                    self._event("camera", "end of frame stream")
                    break
                if packet.status is not FrameStatus.OK:
                    self._event("camera", f"camera read failed: {packet.status.value}")
                    continue

                detections = self.detector.scan_frame(packet, telemetry.pose)
                for detection in detections:
                    if not self.boundary.contains((detection.local_x_m, detection.local_y_m)):
                        self._event(
                            "detection_rejected",
                            f"{detection.seed_name} estimated outside arena",
                            x_m=detection.local_x_m,
                            y_m=detection.local_y_m,
                            confidence=detection.confidence,
                        )
                        continue
                    self.map.add_detection(detection)
                    self.report.detections.append(detection)
                    self._event(
                        "detection",
                        f"{detection.seed_name} at ({detection.local_x_m:.2f}, {detection.local_y_m:.2f})",
                        confidence=detection.confidence,
                    )
                if len({d.seed_name for d in self.report.detections}) >= target_total:
                    self._event("mission", "all seed classes detected")
                    break

            self._return_and_land()
            self._state(MissionState.TRANSFER)
            self._finalize_and_transfer()
            self._state(MissionState.CHARGING)
            self._confirm_charging()
            self._state(MissionState.COMPLETE)
        except Exception as exc:
            LOG.exception("Mission aborted")
            self._event("abort", str(exc))
            self._state(MissionState.ABORT)
            try:
                self.flight.hold()
                self.flight.land()
            except Exception:
                LOG.exception("Failsafe landing command failed")
        finally:
            self.report.finished_s = time.time()
            self.camera.close()
            self.flight.close()
            if self.power is not None:
                self.power.close()
            atomic_write_json(self.run_dir / "mission_report.json", self.report.to_dict())
        return self.report

    def _return_and_land(self) -> None:
        self._state(MissionState.RETURN_HOME)
        home = Waypoint(
            self.config.arena.home_x_m,
            self.config.arena.home_y_m,
            -abs(self.config.flight.takeoff_altitude_m),
            0.0,
        )
        self.flight.goto(home, self.config.flight.goto_timeout_s)
        self._state(MissionState.LANDING)
        self.flight.land()

    def _finalize_and_transfer(self) -> None:
        map_path = self.map.export(self.run_dir)
        self.report.map_path = str(map_path)
        atomic_write_json(self.run_dir / "mission_report.json", self.report.to_dict())
        if self.client is None:
            return
        try:
            self.client.upload_report(self.report.to_dict())
            for detection in self.report.detections:
                image_path = Path(detection.image_path)
                if image_path.exists():
                    self.client.upload_image(image_path)
        except Exception as exc:
            self._event("transfer", f"base station transfer failed: {exc}")

    def _confirm_charging(self) -> None:
        if self.power is None:
            self._event("charging", "power monitor not configured")
            return
        deadline = time.time() + self.config.power.charge_confirm_timeout_s
        initial = self.power.read_status()
        initial_soc = initial.soc_pct if initial.soc_pct > 0.0 else None
        self.report.final_power = initial
        while time.time() <= deadline:
            status = self.power.read_status()
            self.report.final_power = status
            if charging_confirmed(status, self.config.power, initial_soc):
                self.report.charging_confirmed = True
                self._event(
                    "charging",
                    "charging confirmed",
                    voltage_v=status.voltage_v,
                    current_a=status.current_a,
                    soc_pct=status.soc_pct,
                    contact_detected=status.contact_detected,
                )
                return
            time.sleep(0.5)
        self._event(
            "charging",
            "charging not confirmed before timeout",
            voltage_v=self.report.final_power.voltage_v if self.report.final_power else 0.0,
            current_a=self.report.final_power.current_a if self.report.final_power else 0.0,
            soc_pct=self.report.final_power.soc_pct if self.report.final_power else 0.0,
        )

    def _handle_safety(self, event: SafetyEvent) -> bool:
        if event.action is SafetyAction.HOLD:
            self.flight.hold()
            return event.severity >= 2
        if event.action is SafetyAction.RETURN_HOME:
            self._return_and_land()
            return True
        if event.action is SafetyAction.LAND_NOW:
            self.flight.land()
            return True
        if event.action is SafetyAction.ABORT:
            raise RuntimeError(event.reason)
        return False

    def _state(self, state: MissionState) -> None:
        self.report.state = state
        self._event("state", state.value)
        LOG.info("Mission state: %s", state.value)

    def _event(self, kind: str, message: str, **extra) -> None:
        payload = {"time_s": time.time(), "kind": kind, "message": message}
        payload.update(extra)
        self.report.events.append(payload)
