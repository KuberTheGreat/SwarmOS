"""Tests for the A* pathfinding algorithm."""

from __future__ import annotations

import pytest

from swarmos.planning.astar import find_path
from swarmos.warehouse.cell import Position
from swarmos.warehouse.grid import Grid


class TestAStarSimplePath:
    """Basic pathfinding on an open grid."""

    def test_straight_horizontal(self) -> None:
        grid = Grid(10, 1)
        path = find_path(grid, Position(0, 0), Position(9, 0))
        assert path is not None
        assert path.start == Position(0, 0)
        assert path.goal == Position(9, 0)
        assert path.length == 10  # 0..9 inclusive

    def test_straight_vertical(self) -> None:
        grid = Grid(1, 10)
        path = find_path(grid, Position(0, 0), Position(0, 9))
        assert path is not None
        assert path.length == 10

    def test_diagonal_like_manhattan(self) -> None:
        grid = Grid(5, 5)
        path = find_path(grid, Position(0, 0), Position(4, 4))
        assert path is not None
        # Manhattan distance = 8, so optimal path length = 9 waypoints
        assert path.length == 9

    def test_adjacent_cells(self) -> None:
        grid = Grid(5, 5)
        path = find_path(grid, Position(2, 2), Position(3, 2))
        assert path is not None
        assert path.length == 2


class TestAStarObstacles:
    """Pathfinding around obstacles."""

    def test_path_around_single_obstacle(self) -> None:
        grid = Grid(5, 3)
        grid.add_obstacle(Position(2, 1))
        path = find_path(grid, Position(0, 1), Position(4, 1))
        assert path is not None
        # Must detour around the obstacle
        assert Position(2, 1) not in path.waypoints

    def test_path_around_wall(self) -> None:
        # 5x5 grid with a vertical wall at x=2 from y=0 to y=3
        grid = Grid(5, 5)
        for y in range(4):
            grid.add_obstacle(Position(2, y))
        path = find_path(grid, Position(0, 0), Position(4, 0))
        assert path is not None
        # Path must go around the bottom of the wall
        assert any(p.y == 4 for p in path.waypoints)

    def test_optimal_path_length_with_obstacle(self) -> None:
        # Simple 3x3 grid with center blocked
        grid = Grid(3, 3)
        grid.add_obstacle(Position(1, 1))
        path = find_path(grid, Position(0, 0), Position(2, 2))
        assert path is not None
        # Manhattan = 4, but must detour, so length = 5 waypoints
        assert path.length == 5


class TestAStarEdgeCases:
    """Edge cases and error handling."""

    def test_start_equals_goal(self) -> None:
        grid = Grid(5, 5)
        path = find_path(grid, Position(2, 2), Position(2, 2))
        assert path is not None
        assert path.length == 1
        assert path.start == path.goal == Position(2, 2)

    def test_unreachable_goal(self) -> None:
        # Completely wall off the goal
        grid = Grid(5, 5)
        # Surround (4,4) with obstacles
        grid.add_obstacle(Position(3, 4))
        grid.add_obstacle(Position(4, 3))
        path = find_path(grid, Position(0, 0), Position(4, 4))
        assert path is None

    def test_invalid_start_out_of_bounds(self) -> None:
        grid = Grid(5, 5)
        with pytest.raises(ValueError, match="start"):
            find_path(grid, Position(-1, 0), Position(4, 4))

    def test_invalid_goal_out_of_bounds(self) -> None:
        grid = Grid(5, 5)
        with pytest.raises(ValueError, match="goal"):
            find_path(grid, Position(0, 0), Position(10, 10))

    def test_start_on_obstacle(self) -> None:
        grid = Grid(5, 5)
        grid.add_obstacle(Position(0, 0))
        with pytest.raises(ValueError, match="start"):
            find_path(grid, Position(0, 0), Position(4, 4))

    def test_goal_on_obstacle(self) -> None:
        grid = Grid(5, 5)
        grid.add_obstacle(Position(4, 4))
        with pytest.raises(ValueError, match="goal"):
            find_path(grid, Position(0, 0), Position(4, 4))

    def test_no_path_isolated_island(self) -> None:
        # Create two disconnected regions
        grid = Grid(5, 5)
        for y in range(5):
            grid.add_obstacle(Position(2, y))
        path = find_path(grid, Position(0, 0), Position(4, 4))
        assert path is None

    def test_1x1_grid(self) -> None:
        grid = Grid(1, 1)
        path = find_path(grid, Position(0, 0), Position(0, 0))
        assert path is not None
        assert path.length == 1


class TestAStarPathCorrectness:
    """Verify that paths are contiguous and valid."""

    def test_path_is_contiguous(self) -> None:
        grid = Grid(10, 10)
        grid.add_obstacle(Position(5, 3))
        grid.add_obstacle(Position(5, 4))
        grid.add_obstacle(Position(5, 5))
        path = find_path(grid, Position(0, 0), Position(9, 9))
        assert path is not None
        waypoints = path.waypoints
        for i in range(len(waypoints) - 1):
            a, b = waypoints[i], waypoints[i + 1]
            dist = abs(a.x - b.x) + abs(a.y - b.y)
            assert dist == 1, f"Non-adjacent step from {a} to {b}"

    def test_path_avoids_obstacles(self) -> None:
        grid = Grid(10, 10)
        obstacles = [Position(5, y) for y in range(8)]
        grid.add_obstacles(obstacles)
        path = find_path(grid, Position(0, 0), Position(9, 0))
        assert path is not None
        for obs in obstacles:
            assert obs not in path.waypoints
