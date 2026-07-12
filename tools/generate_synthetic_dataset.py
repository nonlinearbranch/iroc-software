from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def textured_patch(seed: int, size: int = 220) -> np.ndarray:
    rng = np.random.default_rng(seed)
    patch = rng.integers(0, 255, (size, size, 3), dtype=np.uint8)
    for _ in range(18):
        color = tuple(int(x) for x in rng.integers(0, 255, 3))
        center = tuple(int(x) for x in rng.integers(20, size - 20, 2))
        radius = int(rng.integers(6, 28))
        cv2.circle(patch, center, radius, color, -1)
    cv2.putText(patch, f"S{seed}", (20, size // 2), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (255, 255, 255), 3)
    return patch


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="runs/synthetic")
    args = parser.parse_args()
    root = Path(args.out)
    seed_dir = root / "seeds"
    frame_dir = root / "frames"
    seed_dir.mkdir(parents=True, exist_ok=True)
    frame_dir.mkdir(parents=True, exist_ok=True)

    patches = [textured_patch(i) for i in range(3)]
    for idx, patch in enumerate(patches):
        cv2.imwrite(str(seed_dir / f"feature_{idx}.png"), patch)

    rng = np.random.default_rng(42)
    for frame_idx in range(8):
        frame = rng.integers(30, 150, (720, 1280, 3), dtype=np.uint8)
        cv2.GaussianBlur(frame, (7, 7), 0, dst=frame)
        if frame_idx < len(patches):
            patch = patches[frame_idx]
            x = 160 + frame_idx * 310
            y = 220 + frame_idx * 70
            frame[y : y + patch.shape[0], x : x + patch.shape[1]] = patch
        cv2.imwrite(str(frame_dir / f"frame_{frame_idx:03d}.jpg"), frame, [int(cv2.IMWRITE_JPEG_QUALITY), 94])
    print(root)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
