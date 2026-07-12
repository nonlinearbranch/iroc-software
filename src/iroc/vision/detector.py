from __future__ import annotations

import time
from pathlib import Path

import cv2
import numpy as np

from iroc.config import CameraConfig, StorageConfig, VisionConfig
from iroc.storage import write_image
from iroc.types import Detection, FramePacket, MatchResult, PoseNED
from iroc.vision.downsample import resize_lr
from iroc.vision.features import FeatureMatcher
from iroc.vision.geometry import DownwardCameraModel


class SurveyDetector:
    def __init__(
        self,
        matcher: FeatureMatcher,
        vision: VisionConfig,
        camera: CameraConfig,
        storage: StorageConfig,
        run_dir: str | Path,
    ) -> None:
        self.matcher = matcher
        self.vision = vision
        self.storage = storage
        self.run_dir = Path(run_dir)
        self.camera_model = DownwardCameraModel(
            camera.width_px,
            camera.height_px,
            camera.h_fov_deg,
            camera.v_fov_deg,
        )
        self._accepted_by_seed: dict[str, int] = {}

    def scan_frame(self, packet: FramePacket, pose: PoseNED) -> list[Detection]:
        if packet.image is None:
            return []
        frame = packet.image
        matches = self._candidate_matches(frame)
        detections: list[Detection] = []
        seen_in_frame: set[str] = set()
        for match in matches:
            if not self.matcher.is_accepted(match):
                continue
            if match.seed_name in seen_in_frame:
                continue
            count = self._accepted_by_seed.get(match.seed_name, 0)
            if count >= self.vision.max_detections_per_seed:
                continue
            detection = self._match_to_detection(match, packet, pose)
            detections.append(detection)
            seen_in_frame.add(match.seed_name)
            self._accepted_by_seed[match.seed_name] = count + 1
        return detections

    def _candidate_matches(self, frame: np.ndarray) -> list[MatchResult]:
        height, width = frame.shape[:2]
        candidates: list[MatchResult] = []
        whole = self.matcher.match(frame)
        if whole:
            whole.pixel_center = self._lr_center_to_source(
                whole.pixel_center,
                source_width=width,
                source_height=height,
                offset_x=0,
                offset_y=0,
            )
            whole.tile = (0, 0, width, height)
            candidates.append(whole)

        step = max(32, self.vision.tile_size_px - self.vision.tile_overlap_px)
        for y0 in range(0, max(1, height - self.vision.tile_size_px + 1), step):
            for x0 in range(0, max(1, width - self.vision.tile_size_px + 1), step):
                x1 = min(width, x0 + self.vision.tile_size_px)
                y1 = min(height, y0 + self.vision.tile_size_px)
                tile = frame[y0:y1, x0:x1]
                result = self.matcher.match(tile)
                if not result:
                    continue
                result.pixel_center = self._lr_center_to_source(
                    result.pixel_center,
                    source_width=x1 - x0,
                    source_height=y1 - y0,
                    offset_x=x0,
                    offset_y=y0,
                )
                result.tile = (x0, y0, x1, y1)
                candidates.append(result)

        candidates.sort(key=lambda item: item.score, reverse=True)
        return self._non_max_suppress(candidates)

    def _lr_center_to_source(
        self,
        center: tuple[float, float] | None,
        *,
        source_width: int,
        source_height: int,
        offset_x: int,
        offset_y: int,
    ) -> tuple[float, float]:
        if center is None:
            return (float(offset_x + source_width / 2.0), float(offset_y + source_height / 2.0))
        lr_size = float(self.vision.lr_size_px)
        x = offset_x + (float(center[0]) / lr_size) * source_width
        y = offset_y + (float(center[1]) / lr_size) * source_height
        return (float(x), float(y))

    def _non_max_suppress(self, matches: list[MatchResult]) -> list[MatchResult]:
        accepted: list[MatchResult] = []
        for candidate in matches:
            if candidate.pixel_center is None:
                accepted.append(candidate)
                continue
            too_close = False
            for existing in accepted:
                if existing.seed_name != candidate.seed_name or existing.pixel_center is None:
                    continue
                dx = existing.pixel_center[0] - candidate.pixel_center[0]
                dy = existing.pixel_center[1] - candidate.pixel_center[1]
                if dx * dx + dy * dy < 80 * 80:
                    too_close = True
                    break
            if not too_close:
                accepted.append(candidate)
        return accepted

    def _match_to_detection(self, match: MatchResult, packet: FramePacket, pose: PoseNED) -> Detection:
        frame_id = packet.frame_id or f"frame_{int(time.time() * 1000)}"
        pixel = match.pixel_center or (0.0, 0.0)
        local_x, local_y = self.camera_model.pixel_to_local_xy(pixel, pose)
        image_dir = self.run_dir / "detections" / match.seed_name
        image_path = image_dir / f"{frame_id}.jpg"
        lr_path = image_dir / f"{frame_id}_lr.jpg"
        if packet.image is not None:
            if self.storage.keep_full_hd:
                write_image(image_path, packet.image, self.storage.jpeg_quality)
            else:
                image_path = lr_path
            write_image(lr_path, resize_lr(packet.image, 128, "area"), self.storage.jpeg_quality)
        return Detection(
            seed_name=match.seed_name,
            confidence=match.score,
            local_x_m=float(local_x),
            local_y_m=float(local_y),
            altitude_m=pose.altitude_m,
            image_path=str(image_path),
            lr_path=str(lr_path),
            frame_id=frame_id,
            timestamp_s=packet.timestamp_s,
            metadata={
                "inliers": match.inliers,
                "good_matches": match.good_matches,
                "method": match.method,
                "pixel_center": match.pixel_center,
                "tile": match.tile,
            },
        )
