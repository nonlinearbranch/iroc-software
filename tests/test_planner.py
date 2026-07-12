from iroc.config import ArenaConfig
from iroc.navigation.arena import ArenaBoundary
from iroc.navigation.planner import CoveragePlanner


def test_lawnmower_waypoints_stay_inside_boundary():
    config = ArenaConfig(width_m=10.0, height_m=6.0, boundary_margin_m=0.75, lane_spacing_m=1.5)
    boundary = ArenaBoundary.from_config(config)
    planner = CoveragePlanner(config, boundary)

    waypoints = planner.lawnmower(altitude_m=3.0)

    assert len(waypoints) >= 6
    assert all(boundary.contains(point.xy()) for point in waypoints)
    assert all(point.z_m == -3.0 for point in waypoints)
