import cv2
import numpy as np

from iroc.config import SystemConfig
from iroc.flight.controller import SimFlightController
from iroc.mission.runner import MissionRunner
from iroc.power.monitor import SimPowerMonitor
from iroc.vision.camera import DirectoryCamera
from iroc.vision.detector import SurveyDetector
from iroc.vision.features import FeatureMatcher


def make_patch() -> np.ndarray:
    rng = np.random.default_rng(11)
    patch = rng.integers(0, 255, (220, 220, 3), dtype=np.uint8)
    for index in range(16):
        color = tuple(int(x) for x in rng.integers(0, 255, 3))
        cv2.rectangle(patch, (10 + index * 5, 20 + index * 3), (90 + index * 4, 120 + index * 2), color, 2)
    cv2.putText(patch, "A1", (55, 125), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (255, 255, 255), 4)
    return patch


def test_dryrun_mission_writes_report(tmp_path):
    config = SystemConfig()
    config.storage.run_root = str(tmp_path)
    config.vision.min_good_matches = 8
    config.vision.min_inliers = 4
    config.vision.min_score = 0.2
    config.vision.methods = ("area", "lanczos")

    seed = make_patch()
    seed_dir = tmp_path / "seeds"
    frame_dir = tmp_path / "frames"
    seed_dir.mkdir()
    frame_dir.mkdir()
    cv2.imwrite(str(seed_dir / "feature_a.png"), seed)

    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    frame[:] = (50, 80, 90)
    frame[220:440, 500:720] = seed
    cv2.imwrite(str(frame_dir / "frame_000.jpg"), frame)

    matcher = FeatureMatcher(
        detector=config.vision.detector,
        methods=config.vision.methods,
        min_good_matches=config.vision.min_good_matches,
        min_inliers=config.vision.min_inliers,
        min_score=config.vision.min_score,
    )
    matcher.add_seed("feature_a", seed)

    run_dir = tmp_path / "mission"
    detector = SurveyDetector(matcher, config.vision, config.camera, config.storage, run_dir)
    report = MissionRunner(
        config,
        SimFlightController(config.flight),
        DirectoryCamera(frame_dir),
        detector,
        run_dir,
        client=None,
        power=SimPowerMonitor(config.power),
    ).run(target_count=1, max_waypoints=1)

    assert report.state.value == "complete"
    assert report.charging_confirmed
    assert report.final_power is not None
    assert (run_dir / "mission_report.json").exists()
    assert (run_dir / "survey_map.json").exists()
    assert report.detections
