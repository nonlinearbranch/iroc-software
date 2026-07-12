from __future__ import annotations

import time
from pathlib import Path
from typing import Protocol

import cv2

from iroc.config import CameraConfig
from iroc.types import FramePacket, FrameStatus


class CameraSource(Protocol):
    def read(self) -> FramePacket: ...

    def close(self) -> None: ...


class OpenCVCamera:
    def __init__(self, config: CameraConfig) -> None:
        self.config = config
        self.capture = cv2.VideoCapture(config.device_index)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, config.width_px)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, config.height_px)
        self.capture.set(cv2.CAP_PROP_FPS, config.fps)
        self.count = 0

    def read(self) -> FramePacket:
        ok, frame = self.capture.read()
        if not ok or frame is None:
            return FramePacket(FrameStatus.CAMERA_ERROR, frame_id=f"cam_{self.count:06d}", timestamp_s=time.time())
        self.count += 1
        return FramePacket(FrameStatus.OK, frame, f"cam_{self.count:06d}", time.time())

    def close(self) -> None:
        self.capture.release()


class DirectoryCamera:
    def __init__(self, frame_dir: str | Path) -> None:
        directory = Path(frame_dir)
        if not directory.exists():
            raise FileNotFoundError(directory)
        self.paths = [
            path
            for path in sorted(directory.iterdir())
            if path.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
        ]
        self.index = 0

    def read(self) -> FramePacket:
        if self.index >= len(self.paths):
            return FramePacket(FrameStatus.END_OF_STREAM, frame_id=f"dir_{self.index:06d}", timestamp_s=time.time())
        path = self.paths[self.index]
        self.index += 1
        frame = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if frame is None:
            return FramePacket(FrameStatus.CAMERA_ERROR, frame_id=path.stem, timestamp_s=time.time(), source_path=path)
        return FramePacket(FrameStatus.OK, frame, path.stem, time.time(), source_path=path)

    def close(self) -> None:
        return None


def make_camera(config: CameraConfig, frame_dir: str = "") -> CameraSource:
    if frame_dir:
        return DirectoryCamera(frame_dir)
    if config.source == "directory":
        return DirectoryCamera(config.frame_dir)
    return OpenCVCamera(config)
