import numpy as np

from iroc.vision.downsample import lr_variants, resize_lr


def test_downsample_variants_are_128_square():
    image = np.zeros((720, 1280, 3), dtype=np.uint8)
    image[100:260, 400:620] = (30, 180, 220)

    output = resize_lr(image, 128, "area")
    assert output.shape == (128, 128, 3)

    variants = lr_variants(image, 128, ("area", "lanczos", "gaussian_area", "center_crop_area"))
    assert set(variants) == {"area", "lanczos", "gaussian_area", "center_crop_area"}
    assert all(value.shape == (128, 128, 3) for value in variants.values())
