import cv2
import numpy as np

from iroc.vision.features import FeatureMatcher


def textured_patch(seed: int = 7, size: int = 220):
    rng = np.random.default_rng(seed)
    patch = rng.integers(0, 255, (size, size, 3), dtype=np.uint8)
    for index in range(20):
        color = tuple(int(x) for x in rng.integers(0, 255, 3))
        center = tuple(int(x) for x in rng.integers(20, size - 20, 2))
        cv2.circle(patch, center, 8 + index % 20, color, -1)
    cv2.putText(patch, "IRoC", (24, 112), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (255, 255, 255), 3)
    return patch


def test_feature_matcher_accepts_same_seed_image():
    seed = textured_patch()
    matcher = FeatureMatcher(
        detector="ORB",
        methods=("area", "lanczos"),
        min_good_matches=8,
        min_inliers=4,
        min_score=0.2,
    )
    matcher.add_seed("feature_a", seed)

    result = matcher.match(seed)

    assert result is not None
    assert result.seed_name == "feature_a"
    assert matcher.is_accepted(result)
