from __future__ import annotations

import math
from dataclasses import dataclass

from iroc.config import ArenaConfig


Point2 = tuple[float, float]


@dataclass(slots=True)
class BoundaryCheck:
    inside: bool
    min_distance_m: float
    nearest_edge_index: int
    in_stop_strip: bool


class ArenaBoundary:
    """Convex polygon geofence using dot-product half-space checks.

    For a counter-clockwise polygon, every edge has an inward normal. A point is
    inside the arena when dot(point - edge_start, inward_normal) >= 0 for every
    edge. Dividing that dot product by the edge length gives signed distance to
    that boundary edge.
    """

    def __init__(self, vertices: list[Point2], stop_margin_m: float) -> None:
        if len(vertices) < 3:
            raise ValueError("Arena polygon needs at least three vertices")
        self.vertices = vertices
        self.stop_margin_m = stop_margin_m
        self._edges = self._build_edges(vertices)

    @classmethod
    def from_config(cls, config: ArenaConfig) -> "ArenaBoundary":
        x0 = config.origin_x_m
        y0 = config.origin_y_m
        vertices = [
            (x0, y0),
            (x0 + config.width_m, y0),
            (x0 + config.width_m, y0 + config.height_m),
            (x0, y0 + config.height_m),
        ]
        return cls(vertices, config.boundary_margin_m)

    def _build_edges(self, vertices: list[Point2]):
        edges = []
        for idx, start in enumerate(vertices):
            end = vertices[(idx + 1) % len(vertices)]
            ex = end[0] - start[0]
            ey = end[1] - start[1]
            length = math.hypot(ex, ey)
            if length <= 0:
                raise ValueError("Duplicate arena vertices are not allowed")
            inward = (-ey / length, ex / length)
            edges.append((start, end, inward, length))
        return edges

    def signed_distances(self, point: Point2) -> list[float]:
        distances = []
        px, py = point
        for start, _end, inward, _length in self._edges:
            vx = px - start[0]
            vy = py - start[1]
            distances.append(vx * inward[0] + vy * inward[1])
        return distances

    def check(self, point: Point2, action_margin_m: float | None = None) -> BoundaryCheck:
        distances = self.signed_distances(point)
        min_distance = min(distances)
        edge_index = distances.index(min_distance)
        margin = self.stop_margin_m if action_margin_m is None else action_margin_m
        return BoundaryCheck(
            inside=min_distance >= 0.0,
            min_distance_m=float(min_distance),
            nearest_edge_index=edge_index,
            in_stop_strip=min_distance <= margin,
        )

    def contains(self, point: Point2) -> bool:
        return self.check(point).inside

    def clamp_rectangle(self, point: Point2, margin_m: float = 0.0) -> Point2:
        xs = [p[0] for p in self.vertices]
        ys = [p[1] for p in self.vertices]
        x = min(max(point[0], min(xs) + margin_m), max(xs) - margin_m)
        y = min(max(point[1], min(ys) + margin_m), max(ys) - margin_m)
        return (x, y)

    def bounds(self) -> tuple[float, float, float, float]:
        xs = [p[0] for p in self.vertices]
        ys = [p[1] for p in self.vertices]
        return min(xs), min(ys), max(xs), max(ys)
