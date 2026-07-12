from __future__ import annotations

from dataclasses import asdict
from pathlib import Path

import numpy as np

from iroc.config import ArenaConfig
from iroc.storage import atomic_write_json
from iroc.types import Detection, PoseNED


class SurveyMap:
    """Compact local map for coverage, confidence, and feature logs."""

    def __init__(self, config: ArenaConfig) -> None:
        self.config = config
        self.resolution_m = config.map_resolution_m
        self.width_cells = max(1, int(np.ceil(config.width_m / self.resolution_m)))
        self.height_cells = max(1, int(np.ceil(config.height_m / self.resolution_m)))
        self.coverage = np.zeros((self.height_cells, self.width_cells), dtype=np.uint16)
        self.confidence = np.zeros((self.height_cells, self.width_cells), dtype=np.float32)
        self.detections: list[Detection] = []

    def world_to_cell(self, x_m: float, y_m: float) -> tuple[int, int]:
        col = int((x_m - self.config.origin_x_m) / self.resolution_m)
        row = int((y_m - self.config.origin_y_m) / self.resolution_m)
        col = min(max(col, 0), self.width_cells - 1)
        row = min(max(row, 0), self.height_cells - 1)
        return row, col

    def mark_pose(self, pose: PoseNED, footprint_radius_m: float = 0.45) -> None:
        row, col = self.world_to_cell(pose.x_m, pose.y_m)
        radius_cells = max(1, int(np.ceil(footprint_radius_m / self.resolution_m)))
        r0 = max(0, row - radius_cells)
        r1 = min(self.height_cells, row + radius_cells + 1)
        c0 = max(0, col - radius_cells)
        c1 = min(self.width_cells, col + radius_cells + 1)
        patch = self.coverage[r0:r1, c0:c1].astype(np.uint32) + 1
        self.coverage[r0:r1, c0:c1] = np.minimum(patch, np.iinfo(np.uint16).max).astype(np.uint16)

    def add_detection(self, detection: Detection) -> None:
        self.detections.append(detection)
        row, col = self.world_to_cell(detection.local_x_m, detection.local_y_m)
        self.confidence[row, col] = max(self.confidence[row, col], float(detection.confidence))

    def least_covered_cell(self) -> tuple[int, int]:
        index = int(np.argmin(self.coverage))
        return np.unravel_index(index, self.coverage.shape)

    def export(self, output_dir: str | Path) -> Path:
        directory = Path(output_dir)
        directory.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            directory / "survey_map.npz",
            coverage=self.coverage,
            confidence=self.confidence,
            resolution_m=np.array([self.resolution_m], dtype=np.float32),
        )
        payload = {
            "resolution_m": self.resolution_m,
            "width_cells": self.width_cells,
            "height_cells": self.height_cells,
            "detections": [asdict(detection) for detection in self.detections],
        }
        return atomic_write_json(directory / "survey_map.json", payload)
