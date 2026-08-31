"""A* pathfinding on a 2-D grid.

This module implements the A* search algorithm with Manhattan-distance
heuristic for 4-directional movement.  It is a *pure planning* module:
it depends only on the Grid (environment) and Position (spatial type),
not on any robot, simulation, or visualisation code.

Usage::

    from swarmos.planning.astar import find_path

    path = find_path(grid, start, goal)
    if path is None:
        print("No path exists")
"""

from __future__ import annotations

import heapq
from typing import TYPE_CHECKING

from swarmos.planning.path import Path
from swarmos.warehouse.cell import Position

if TYPE_CHECKING:
    from swarmos.warehouse.grid import Grid


def find_path(grid: Grid, start: Position, goal: Position) -> Path | None:
    """Compute a shortest path from *start* to *goal* using A*.

    Parameters:
        grid:  The warehouse grid (provides bounds, traversability,
               and neighbour queries).
        start: The starting position.
        goal:  The target position.

    Returns:
        A :class:`~swarmos.planning.path.Path` containing the ordered
        waypoints from *start* to *goal* (inclusive), or ``None`` if no
        path exists.

    Raises:
        ValueError: If *start* or *goal* is out of bounds or
                    non-traversable.
    """
    # --- validate inputs ---
    _validate_endpoint(grid, start, "start")
    _validate_endpoint(grid, goal, "goal")

    # --- trivial case ---
    if start == goal:
        return Path([start])

    # --- A* search ---
    # Each entry in the open set is (f_cost, tiebreaker, position).
    # The tiebreaker is a monotonically increasing counter to ensure
    # deterministic ordering when f-costs are equal.
    open_set: list[tuple[int, int, Position]] = []
    counter = 0

    g_cost: dict[Position, int] = {start: 0}
    came_from: dict[Position, Position] = {}

    h_start = start.manhattan_distance(goal)
    heapq.heappush(open_set, (h_start, counter, start))
    counter += 1

    closed_set: set[Position] = set()

    while open_set:
        _f, _tie, current = heapq.heappop(open_set)

        if current == goal:
            return _reconstruct_path(came_from, current)

        if current in closed_set:
            continue
        closed_set.add(current)

        current_g = g_cost[current]

        for neighbor in grid.get_neighbors(current):
            if neighbor in closed_set:
                continue

            tentative_g = current_g + 1  # uniform edge cost

            if tentative_g < g_cost.get(neighbor, float("inf")):  # type: ignore[arg-type]
                g_cost[neighbor] = tentative_g
                came_from[neighbor] = current
                f = tentative_g + neighbor.manhattan_distance(goal)
                heapq.heappush(open_set, (f, counter, neighbor))
                counter += 1

    # Goal unreachable.
    return None


# ------------------------------------------------------------------
# Internal helpers
# ------------------------------------------------------------------


def _validate_endpoint(grid: Grid, pos: Position, label: str) -> None:
    """Raise if *pos* is invalid as a start/goal."""
    if not grid.in_bounds(pos):
        raise ValueError(
            f"A* {label} position {pos} is out of bounds "
            f"(grid is {grid.width}×{grid.height})"
        )
    if not grid.is_traversable(pos):
        raise ValueError(
            f"A* {label} position {pos} is not traversable"
        )


def _reconstruct_path(
    came_from: dict[Position, Position],
    current: Position,
) -> Path:
    """Walk the *came_from* map backwards to build the full path."""
    waypoints: list[Position] = [current]
    while current in came_from:
        current = came_from[current]
        waypoints.append(current)
    waypoints.reverse()
    return Path(waypoints)
