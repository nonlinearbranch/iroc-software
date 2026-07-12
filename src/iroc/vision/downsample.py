from __future__ import annotations

from collections.abc import Iterable

import cv2
import numpy as np


def center_square(image: np.ndarray) -> np.ndarray:
    height, width = image.shape[:2]
    side = min(height, width)
    y0 = (height - side) // 2
    x0 = (width - side) // 2
    return image[y0 : y0 + side, x0 : x0 + side]


def resize_lr(image: np.ndarray, size_px: int = 128, method: str = "area") -> np.ndarray:
    if image is None or image.size == 0:
        raise ValueError("Cannot downsample an empty image")

    source = image
    interpolation = cv2.INTER_AREA

    if method == "area":
        interpolation = cv2.INTER_AREA
    elif method == "lanczos":
        interpolation = cv2.INTER_LANCZOS4
    elif method == "linear":
        interpolation = cv2.INTER_LINEAR
    elif method == "gaussian_area":
        source = cv2.GaussianBlur(image, (5, 5), 0)
        interpolation = cv2.INTER_AREA
    elif method == "center_crop_area":
        source = center_square(image)
        interpolation = cv2.INTER_AREA
    else:
        raise ValueError(f"Unknown downsample method: {method}")

    return cv2.resize(source, (size_px, size_px), interpolation=interpolation)


def lr_variants(
    image: np.ndarray,
    size_px: int = 128,
    methods: Iterable[str] = ("area", "lanczos", "gaussian_area", "center_crop_area"),
) -> dict[str, np.ndarray]:
    return {method: resize_lr(image, size_px=size_px, method=method) for method in methods}


def hsv_histogram(image: np.ndarray) -> np.ndarray:
    resized = resize_lr(image, size_px=128, method="area")
    hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1, 2], None, [8, 8, 8], [0, 180, 0, 256, 0, 256])
    cv2.normalize(hist, hist)
    return hist.flatten().astype(np.float32)
