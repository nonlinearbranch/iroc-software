from iroc.config import ArenaConfig
from iroc.navigation.arena import ArenaBoundary


def test_dot_product_boundary_check_inside_strip_and_outside():
    boundary = ArenaBoundary.from_config(ArenaConfig(width_m=10.0, height_m=5.0, boundary_margin_m=0.75))

    center = boundary.check((5.0, 2.5))
    assert center.inside
    assert not center.in_stop_strip
    assert center.min_distance_m == 2.5

    strip = boundary.check((0.2, 2.5))
    assert strip.inside
    assert strip.in_stop_strip

    outside = boundary.check((-0.1, 2.5))
    assert not outside.inside
    assert outside.min_distance_m < 0
