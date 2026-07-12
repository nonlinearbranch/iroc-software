from pathlib import Path

import numpy as np

from iroc.config import CameraConfig, StorageConfig, VisionConfig
from iroc.vision.detector import SurveyDetector


class DummyMatcher:
    @property
    def seed_names(self):
        return {"target"}

    def is_accepted(self, result):
        return True


def test_lr_center_is_scaled_to_full_frame_pixels(tmp_path):
    detector = SurveyDetector(
        DummyMatcher(),
        VisionConfig(lr_size_px=128),
        CameraConfig(width_px=1280, height_px=720),
        StorageConfig(),
        Path(tmp_path),
    )

    assert detector._lr_center_to_source((64, 64), source_width=1280, source_height=720, offset_x=0, offset_y=0) == (
        640.0,
        360.0,
    )
    assert detector._lr_center_to_source((64, 64), source_width=420, source_height=420, offset_x=280, offset_y=140) == (
        490.0,
        350.0,
    )
