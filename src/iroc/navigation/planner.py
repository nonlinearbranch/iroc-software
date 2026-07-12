from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

import numpy as np

from iroc.config import ArenaConfig
from iroc.navigation.arena import ArenaBoundary, Point2
from iroc.navigation.map import SurveyMap


@dataclass(frozen=True, slots=True)
class Waypoint:
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float = 0.0

    def xy(self) -> Point2:
        return (self.x_m, self.y_m)


class CoveragePlanner:
    def __init__(self, arena: ArenaConfig, boundary: ArenaBoundary) -> None:
        self.arena = arena
        self.boundary = boundary

    def lawnmower(self, altitude_m: float, lane_spacing_m: float | None = None) -> list[Waypoint]:
        spacing = lane_spacing_m or self.arena.lane_spacing_m
        min_x, min_y, max_x, max_y = self.boundary.bounds()
        margin = self.arena.boundary_margin_m
        x_values = self._frange(min_x + margin, max_x - margin, spacing)
        y_min = min_y + margin
        y_max = max_y - margin
        waypoints: list[Waypoint] = []
        for index, x in enumerate(x_values):
            lane = [(x, y_min), (x, y_max)] if index % 2 == 0 else [(x, y_max), (x, y_min)]
            for point in lane:
                safe = self.boundary.clamp_rectangle(point, margin)
                if self.boundary.contains(safe):
                    waypoints.append(Waypoint(safe[0], safe[1], -abs(altitude_m), 0.0))
        return waypoints

    def next_frontier(self, survey_map: SurveyMap, altitude_m: float) -> Waypoint:
        row, col = survey_map.least_covered_cell()
        x = self.arena.origin_x_m + (col + 0.5) * self.arena.map_resolution_m
        y = self.arena.origin_y_m + (row + 0.5) * self.arena.map_resolution_m
        x, y = self.boundary.clamp_rectangle((x, y), self.arena.boundary_margin_m)
        return Waypoint(x, y, -abs(altitude_m), 0.0)

    @staticmethod
    def _frange(start: float, stop: float, step: float) -> list[float]:
        if stop < start:
            return [start]
        values = []
        current = start
        while current <= stop + 1e-9:
            values.append(current)
            current += step
        if values and values[-1] < stop:
            values.append(stop)
        return values


class AStarGridPlanner:
    """Small A* helper for future obstacle-aware rerouting on the occupancy map."""

    def __init__(self, blocked: np.ndarray) -> None:
        self.blocked = blocked.astype(bool)
        self.height, self.width = self.blocked.shape

    def plan(self, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
        if self.blocked[start] or self.blocked[goal]:
            return []
        frontier: list[tuple[float, tuple[int, int]]] = [(0.0, start)]
        came_from: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
        cost: dict[tuple[int, int], float] = {start: 0.0}

        while frontier:
            _priority, current = heapq.heappop(frontier)
            if current == goal:
                break
            for nxt in self._neighbors(current):
                new_cost = cost[current] + math.dist(current, nxt)
                if nxt not in cost or new_cost < cost[nxt]:
                    cost[nxt] = new_cost
                    priority = new_cost + math.dist(nxt, goal)
                    heapq.heappush(frontier, (priority, nxt))
                    came_from[nxt] = current

        if goal not in came_from:
            return []
        path = [goal]
        current = goal
        while came_from[current] is not None:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def _neighbors(self, cell: tuple[int, int]):
        row, col = cell
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nr = row + dr
            nc = col + dc
            if 0 <= nr < self.height and 0 <= nc < self.width and not self.blocked[nr, nc]:
                yield (nr, nc)
