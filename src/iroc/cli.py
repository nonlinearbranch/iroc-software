from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict
from pathlib import Path

import cv2

from iroc.comms.base_station import run_base_station
from iroc.comms.client import BaseStationClient
from iroc.comms.validation import validate_report
from iroc.config import load_config, write_example_config
from iroc.diagnostics import preflight_report
from iroc.flight.controller import make_flight_controller
from iroc.logging_utils import configure_logging
from iroc.mission.runner import MissionRunner
from iroc.power.monitor import charging_confirmed, make_power_monitor
from iroc.types import FramePacket, FrameStatus, PoseNED
from iroc.storage import atomic_write_json, ensure_dir, write_image
from iroc.vision.camera import DirectoryCamera, make_camera
from iroc.vision.detector import SurveyDetector
from iroc.vision.features import FeatureMatcher, load_seed_images


def build_matcher(config, seed_dir: str | Path) -> FeatureMatcher:
    matcher = FeatureMatcher(
        detector=config.vision.detector,
        lr_size_px=config.vision.lr_size_px,
        methods=config.vision.methods,
        ratio_test=config.vision.ratio_test,
        min_good_matches=config.vision.min_good_matches,
        min_inliers=config.vision.min_inliers,
        min_score=config.vision.min_score,
    )
    for name, image in load_seed_images(seed_dir).items():
        matcher.add_seed(name, image)
    return matcher


def command_preflight(args) -> int:
    config = load_config(args.config)
    report = preflight_report(config)
    print(json.dumps(report, indent=2))
    return 0 if report["ok"] or args.allow_warnings else 2


def command_base_station(args) -> int:
    config = load_config(args.config)
    run_base_station(config.comms, config.vision)
    return 0


def command_base_validate(args) -> int:
    config = load_config(args.config)
    result = validate_report(args.seed_dir, args.report, args.image_dir, config.vision, args.out or None)
    print(json.dumps(result, indent=2))
    return 0 if result["valid"] else 1


def command_camera_check(args) -> int:
    config = load_config(args.config)
    camera = make_camera(config.camera, args.frame_dir)
    out_dir = ensure_dir(args.out or Path(config.storage.run_root) / f"camera_check_{int(time.time())}")
    samples = []
    start = time.time()
    ok_frames = 0
    first_shape = None
    try:
        for index in range(args.frames):
            packet = camera.read()
            if packet.status is not FrameStatus.OK or packet.image is None:
                continue
            ok_frames += 1
            first_shape = first_shape or packet.image.shape
            if index in {0, args.frames - 1}:
                sample_path = write_image(out_dir / f"sample_{index:03d}.jpg", packet.image, config.storage.jpeg_quality)
                samples.append(str(sample_path))
    finally:
        camera.close()
    elapsed = max(1e-6, time.time() - start)
    height, width = (first_shape[:2] if first_shape is not None else (0, 0))
    report = {
        "ok": ok_frames > 0 and width >= 1280 and height >= 720,
        "frames_requested": args.frames,
        "frames_ok": ok_frames,
        "measured_fps": ok_frames / elapsed,
        "width_px": int(width),
        "height_px": int(height),
        "samples": samples,
    }
    atomic_write_json(out_dir / "camera_check.json", report)
    print(json.dumps(report, indent=2))
    return 0 if report["ok"] else 1


def command_mavlink_check(args) -> int:
    config = load_config(args.config)
    config.flight.mode = "flight"
    flight = make_flight_controller(config.flight)
    samples = []
    try:
        flight.connect()
        deadline = time.time() + args.seconds
        while time.time() < deadline:
            telemetry = flight.telemetry()
            samples.append(asdict(telemetry))
            time.sleep(args.interval)
    finally:
        flight.close()
    report = {
        "ok": bool(samples),
        "samples": samples[-args.keep :],
    }
    print(json.dumps(report, indent=2))
    return 0 if report["ok"] else 1


def command_power_check(args) -> int:
    config = load_config(args.config)
    monitor = make_power_monitor(config.power)
    samples = []
    try:
        monitor.connect()
        initial = monitor.read_status()
        samples.append(asdict(initial))
        deadline = time.time() + args.seconds
        while time.time() < deadline:
            status = monitor.read_status()
            samples.append(asdict(status))
            time.sleep(args.interval)
    finally:
        monitor.close()
    last = samples[-1] if samples else {}
    initial_soc = samples[0]["soc_pct"] if samples and samples[0].get("soc_pct", 0.0) > 0.0 else None
    status_obj = None
    if last:
        from iroc.types import PowerStatus

        status_obj = PowerStatus(**last)
    report = {
        "ok": bool(samples),
        "charging_confirmed": charging_confirmed(status_obj, config.power, initial_soc) if status_obj else False,
        "samples": samples[-args.keep :],
    }
    print(json.dumps(report, indent=2))
    return 0 if report["ok"] else 1


def command_validate(args) -> int:
    config = load_config(args.config)
    out_dir = ensure_dir(args.out or Path(config.storage.run_root) / f"validation_{int(time.time())}")
    matcher = build_matcher(config, args.seed_dir)
    detector = SurveyDetector(matcher, config.vision, config.camera, config.storage, out_dir)
    pose = PoseNED(config.arena.width_m / 2.0, config.arena.height_m / 2.0, -abs(config.flight.survey_altitude_m), 0.0)
    results = []
    for path in sorted(Path(args.capture_dir).iterdir()):
        if path.suffix.lower() not in {".jpg", ".jpeg", ".png", ".bmp", ".webp"}:
            continue
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        packet = FramePacket(FrameStatus.OK, image, path.stem, time.time(), path)
        detections = detector.scan_frame(packet, pose)
        results.append(
            {
                "image": str(path),
                "detections": [asdict(detection) for detection in detections],
                "accepted": bool(detections),
            }
        )
    atomic_write_json(out_dir / "validation_results.json", {"results": results})
    accepted = sum(1 for item in results if item["accepted"])
    print(f"validated {len(results)} images, accepted {accepted}; wrote {out_dir}")
    return 0


def command_companion(args) -> int:
    config = load_config(args.config)
    if args.dry_run:
        config.flight.mode = "sim"
    run_dir = ensure_dir(Path(config.storage.run_root) / f"mission_{int(time.time())}")
    transfer_client = None if args.no_transfer else BaseStationClient(config.comms)
    seed_dir = args.seed_dir
    if args.fetch_seeds:
        seed_client = transfer_client or BaseStationClient(config.comms)
        seed_dir = str(run_dir / "seeds")
        seed_client.download_seeds(seed_dir)
    if not seed_dir:
        raise SystemExit("--seed-dir is required unless --fetch-seeds is used")
    matcher = build_matcher(config, seed_dir)
    camera = DirectoryCamera(args.frame_dir) if args.frame_dir else make_camera(config.camera)
    flight = make_flight_controller(config.flight)
    detector = SurveyDetector(matcher, config.vision, config.camera, config.storage, run_dir)
    power = make_power_monitor(config.power)
    report = MissionRunner(config, flight, camera, detector, run_dir, transfer_client, power).run(
        target_count=args.target_count,
        max_waypoints=args.max_waypoints,
    )
    print(json.dumps(report.to_dict(), indent=2))
    return 0 if report.state.value == "complete" else 1


def command_init_config(args) -> int:
    write_example_config(args.output)
    print(f"wrote {args.output}")
    return 0


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="iroc", description="IRoC-U ASCEND drone software")
    p.add_argument("--log-level", default="INFO")
    sub = p.add_subparsers(dest="command", required=True)

    init_cfg = sub.add_parser("init-config", help="write a default YAML config")
    init_cfg.add_argument("--output", default="config/vehicle.example.yaml")
    init_cfg.set_defaults(func=command_init_config)

    preflight = sub.add_parser("preflight", help="check local software and config")
    preflight.add_argument("--config", default="config/vehicle.example.yaml")
    preflight.add_argument("--allow-warnings", action="store_true")
    preflight.set_defaults(func=command_preflight)

    base = sub.add_parser("base-station", help="run the base-station HTTP server")
    base.add_argument("--config", default="config/vehicle.example.yaml")
    base.set_defaults(func=command_base_station)

    base_validate = sub.add_parser("base-validate", help="validate a mission report against seed and evidence images")
    base_validate.add_argument("--config", default="config/vehicle.example.yaml")
    base_validate.add_argument("--seed-dir", required=True)
    base_validate.add_argument("--report", required=True)
    base_validate.add_argument("--image-dir", required=True)
    base_validate.add_argument("--out", default="")
    base_validate.set_defaults(func=command_base_validate)

    camera_check = sub.add_parser("camera-check", help="verify camera or frame-directory resolution and sample FPS")
    camera_check.add_argument("--config", default="config/vehicle.example.yaml")
    camera_check.add_argument("--frame-dir", default="")
    camera_check.add_argument("--frames", type=int, default=30)
    camera_check.add_argument("--out", default="")
    camera_check.set_defaults(func=command_camera_check)

    mavlink_check = sub.add_parser("mavlink-check", help="connect to Pixhawk/Cube and print recent telemetry")
    mavlink_check.add_argument("--config", default="config/vehicle.example.yaml")
    mavlink_check.add_argument("--seconds", type=float, default=5.0)
    mavlink_check.add_argument("--interval", type=float, default=0.25)
    mavlink_check.add_argument("--keep", type=int, default=5)
    mavlink_check.set_defaults(func=command_mavlink_check)

    power_check = sub.add_parser("power-check", help="read BMS/charging telemetry")
    power_check.add_argument("--config", default="config/vehicle.example.yaml")
    power_check.add_argument("--seconds", type=float, default=5.0)
    power_check.add_argument("--interval", type=float, default=0.5)
    power_check.add_argument("--keep", type=int, default=8)
    power_check.set_defaults(func=command_power_check)

    validate = sub.add_parser("validate", help="match seed images against captured images")
    validate.add_argument("--config", default="config/vehicle.example.yaml")
    validate.add_argument("--seed-dir", required=True)
    validate.add_argument("--capture-dir", required=True)
    validate.add_argument("--out", default="")
    validate.set_defaults(func=command_validate)

    companion = sub.add_parser("companion", help="run companion mission logic")
    companion.add_argument("--config", default="config/vehicle.example.yaml")
    companion.add_argument("--seed-dir", default="")
    companion.add_argument("--fetch-seeds", action="store_true")
    companion.add_argument("--frame-dir", default="")
    companion.add_argument("--dry-run", action="store_true")
    companion.add_argument("--no-transfer", action="store_true")
    companion.add_argument("--target-count", type=int, default=None)
    companion.add_argument("--max-waypoints", type=int, default=None)
    companion.set_defaults(func=command_companion)
    return p


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    configure_logging(args.log_level)
    return int(args.func(args))


if __name__ == "__main__":
    sys.exit(main())
