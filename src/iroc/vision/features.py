from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import cv2
import numpy as np

from iroc.types import MatchResult
from iroc.vision.downsample import hsv_histogram, lr_variants, resize_lr


@dataclass(slots=True)
class SeedFeature:
    name: str
    method: str
    image_lr: np.ndarray
    keypoints: tuple[cv2.KeyPoint, ...]
    descriptors: np.ndarray | None
    histogram: np.ndarray


class FeatureMatcher:
    """Two-stage matcher for 128x128 LR feature validation.

    ORB is the default because it is fast on RPi/Jetson-class CPUs. SIFT is also
    supported when OpenCV was built with it, and can be enabled for stricter
    post-flight validation.
    """

    def __init__(
        self,
        detector: str = "ORB",
        lr_size_px: int = 128,
        methods: Iterable[str] = ("area", "lanczos", "gaussian_area", "center_crop_area"),
        ratio_test: float = 0.75,
        min_good_matches: int = 18,
        min_inliers: int = 8,
        min_score: float = 0.42,
    ) -> None:
        self.detector_name = detector.upper()
        self.lr_size_px = lr_size_px
        self.methods = tuple(methods)
        self.ratio_test = ratio_test
        self.min_good_matches = min_good_matches
        self.min_inliers = min_inliers
        self.min_score = min_score
        self._detector, self._norm = self._make_detector(self.detector_name)
        self._matcher = cv2.BFMatcher(self._norm, crossCheck=False)
        self._seeds: list[SeedFeature] = []

    @property
    def seed_names(self) -> set[str]:
        return {seed.name for seed in self._seeds}

    def _make_detector(self, name: str):
        if name == "SIFT":
            if not hasattr(cv2, "SIFT_create"):
                raise RuntimeError("OpenCV SIFT is unavailable in this environment")
            return cv2.SIFT_create(nfeatures=400), cv2.NORM_L2
        if name == "ORB":
            return (
                cv2.ORB_create(
                    nfeatures=650,
                    scaleFactor=1.2,
                    nlevels=8,
                    edgeThreshold=12,
                    patchSize=31,
                    fastThreshold=12,
                ),
                cv2.NORM_HAMMING,
            )
        raise ValueError(f"Unsupported detector: {name}")

    def _features(self, image: np.ndarray) -> tuple[tuple[cv2.KeyPoint, ...], np.ndarray | None]:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        keypoints, descriptors = self._detector.detectAndCompute(gray, None)
        return tuple(keypoints or ()), descriptors

    def add_seed(self, name: str, image: np.ndarray) -> None:
        for method, variant in lr_variants(image, self.lr_size_px, self.methods).items():
            keypoints, descriptors = self._features(variant)
            self._seeds.append(
                SeedFeature(
                    name=name,
                    method=method,
                    image_lr=variant,
                    keypoints=keypoints,
                    descriptors=descriptors,
                    histogram=hsv_histogram(variant),
                )
            )

    def match(self, image: np.ndarray) -> MatchResult | None:
        if not self._seeds:
            raise RuntimeError("No seed images loaded")

        best: MatchResult | None = None
        frame_variants = lr_variants(image, self.lr_size_px, self.methods)
        frame_hist = hsv_histogram(image)

        for frame_method, frame_lr in frame_variants.items():
            frame_kp, frame_desc = self._features(frame_lr)
            for seed in self._seeds:
                candidate = self._score_pair(seed, frame_lr, frame_kp, frame_desc, frame_hist, frame_method)
                if candidate and (best is None or candidate.score > best.score):
                    best = candidate

        if best and self.is_accepted(best):
            return best
        return best

    def is_accepted(self, result: MatchResult | None) -> bool:
        if result is None:
            return False
        return (
            result.score >= self.min_score
            and result.good_matches >= self.min_good_matches
            and result.inliers >= self.min_inliers
        )

    def _score_pair(
        self,
        seed: SeedFeature,
        frame_lr: np.ndarray,
        frame_kp: tuple[cv2.KeyPoint, ...],
        frame_desc: np.ndarray | None,
        frame_hist: np.ndarray,
        frame_method: str,
    ) -> MatchResult | None:
        similarity = self._lr_similarity(seed.image_lr, frame_lr)
        if seed.descriptors is None or frame_desc is None:
            hist_score = float(cv2.compareHist(seed.histogram, frame_hist, cv2.HISTCMP_CORREL))
            score = 0.60 * similarity + 0.40 * max(0.0, hist_score)
            return MatchResult(seed.name, float(score), 0, 0, "histogram", similarity=similarity)

        raw_matches = self._matcher.knnMatch(seed.descriptors, frame_desc, k=2)
        good = []
        for pair in raw_matches:
            if len(pair) != 2:
                continue
            first, second = pair
            if first.distance < self.ratio_test * second.distance:
                good.append(first)

        inliers = 0
        homography = None
        center: tuple[float, float] | None = None
        if len(good) >= 4:
            src = np.float32([seed.keypoints[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
            dst = np.float32([frame_kp[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
            homography_np, mask = cv2.findHomography(src, dst, cv2.RANSAC, 4.0)
            if homography_np is not None and mask is not None:
                inliers = int(mask.ravel().sum())
                homography = homography_np.tolist()
                corners = np.float32(
                    [[0, 0], [self.lr_size_px - 1, 0], [self.lr_size_px - 1, self.lr_size_px - 1], [0, self.lr_size_px - 1]]
                ).reshape(-1, 1, 2)
                projected = cv2.perspectiveTransform(corners, homography_np).reshape(-1, 2)
                center_arr = projected.mean(axis=0)
                center = (float(center_arr[0]), float(center_arr[1]))

        hist_score = float(cv2.compareHist(seed.histogram, frame_hist, cv2.HISTCMP_CORREL))
        hist_score = max(0.0, min(1.0, hist_score))
        match_score = min(1.0, len(good) / max(1, self.min_good_matches * 2))
        inlier_score = min(1.0, inliers / max(1, self.min_inliers * 2))
        score = 0.35 * inlier_score + 0.25 * match_score + 0.25 * similarity + 0.15 * hist_score

        return MatchResult(
            seed_name=seed.name,
            score=float(score),
            inliers=inliers,
            good_matches=len(good),
            method=f"{self.detector_name}:{seed.method}->{frame_method}",
            similarity=similarity,
            pixel_center=center,
            homography=homography,
        )

    @staticmethod
    def _lr_similarity(seed_lr: np.ndarray, frame_lr: np.ndarray) -> float:
        seed_gray = cv2.cvtColor(seed_lr, cv2.COLOR_BGR2GRAY) if seed_lr.ndim == 3 else seed_lr
        frame_gray = cv2.cvtColor(frame_lr, cv2.COLOR_BGR2GRAY) if frame_lr.ndim == 3 else frame_lr
        value = float(cv2.matchTemplate(frame_gray, seed_gray, cv2.TM_CCOEFF_NORMED)[0][0])
        if np.isnan(value):
            return 0.0
        return max(0.0, min(1.0, (value + 1.0) / 2.0))


def load_seed_images(seed_dir) -> dict[str, np.ndarray]:
    from pathlib import Path

    directory = Path(seed_dir)
    if not directory.exists():
        raise FileNotFoundError(directory)
    images: dict[str, np.ndarray] = {}
    for path in sorted(directory.iterdir()):
        if path.suffix.lower() not in {".jpg", ".jpeg", ".png", ".bmp", ".webp"}:
            continue
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        images[path.stem] = image
    if not images:
        raise RuntimeError(f"No seed images found in {directory}")
    return images


def load_matcher_from_dir(seed_dir, *, detector: str = "ORB", lr_size_px: int = 128, methods=()):
    matcher = FeatureMatcher(detector=detector, lr_size_px=lr_size_px, methods=methods or ("area", "lanczos"))
    for name, image in load_seed_images(seed_dir).items():
        matcher.add_seed(name, image)
    return matcher
