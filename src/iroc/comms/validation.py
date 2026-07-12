from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import cv2

from iroc.config import VisionConfig
from iroc.storage import atomic_write_json
from iroc.vision.features import FeatureMatcher, load_seed_images


def build_validation_matcher(seed_dir: str | Path, vision: VisionConfig) -> FeatureMatcher:
    matcher = FeatureMatcher(
        detector=vision.detector,
        lr_size_px=vision.lr_size_px,
        methods=vision.methods,
        ratio_test=vision.ratio_test,
        min_good_matches=vision.min_good_matches,
        min_inliers=vision.min_inliers,
        min_score=vision.min_score,
    )
    for name, image in load_seed_images(seed_dir).items():
        matcher.add_seed(name, image)
    return matcher


def validate_report(
    seed_dir: str | Path,
    report_path: str | Path,
    image_dir: str | Path,
    vision: VisionConfig,
    out_path: str | Path | None = None,
) -> dict[str, Any]:
    report_file = Path(report_path)
    payload = json.loads(report_file.read_text(encoding="utf-8"))
    matcher = build_validation_matcher(seed_dir, vision)
    image_root = Path(image_dir)
    results = []
    for detection in payload.get("detections", []):
        expected_seed = detection.get("seed_name", "")
        source_name = Path(detection.get("image_path", "")).name
        candidates = [image_root / source_name]
        original = Path(detection.get("image_path", ""))
        if original.exists():
            candidates.append(original)
        image_path = next((path for path in candidates if path.exists()), None)
        if image_path is None:
            results.append(
                {
                    "seed_name": expected_seed,
                    "image": source_name,
                    "valid": False,
                    "reason": "evidence image not found",
                }
            )
            continue
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            results.append(
                {
                    "seed_name": expected_seed,
                    "image": str(image_path),
                    "valid": False,
                    "reason": "evidence image unreadable",
                }
            )
            continue
        match = best_tiled_match(matcher, image, vision)
        valid = bool(match and matcher.is_accepted(match) and match.seed_name == expected_seed)
        results.append(
            {
                "seed_name": expected_seed,
                "image": str(image_path),
                "valid": valid,
                "matched_seed": match.seed_name if match else None,
                "score": match.score if match else 0.0,
                "inliers": match.inliers if match else 0,
                "good_matches": match.good_matches if match else 0,
                "method": match.method if match else "",
            }
        )
    output = {
        "mission_id": payload.get("mission_id", ""),
        "valid": bool(results) and all(item["valid"] for item in results),
        "results": results,
    }
    if out_path:
        atomic_write_json(out_path, output)
    return output


def best_tiled_match(matcher: FeatureMatcher, image, vision: VisionConfig):
    height, width = image.shape[:2]
    candidates = []
    whole = matcher.match(image)
    if whole:
        candidates.append(whole)
    step = max(32, vision.tile_size_px - vision.tile_overlap_px)
    for y0 in range(0, max(1, height - vision.tile_size_px + 1), step):
        for x0 in range(0, max(1, width - vision.tile_size_px + 1), step):
            x1 = min(width, x0 + vision.tile_size_px)
            y1 = min(height, y0 + vision.tile_size_px)
            result = matcher.match(image[y0:y1, x0:x1])
            if result:
                result.tile = (x0, y0, x1, y1)
                candidates.append(result)
    if not candidates:
        return None
    candidates.sort(key=lambda item: item.score, reverse=True)
    return candidates[0]
