"""Tests for the warehouse Grid and supporting types."""

from __future__ import annotations

import pytest

from swarmos.warehouse.cell import CellType, Position
from swarmos.warehouse.grid import Grid


# ==================================================================
# Position tests
# ==================================================================


class TestPosition:
    def test_equality(self) -> None:
        assert Position(3, 4) == Position(3, 4)

    def test_inequality(self) -> None:
        assert Position(3, 4) != Position(4, 3)

    def test_hashable(self) -> None:
        s = {Position(1, 2), Position(1, 2), Position(3, 4)}
        assert len(s) == 2

    def test_manhattan_distance(self) -> None:
        assert Position(0, 0).manhattan_distance(Position(3, 4)) == 7

    def test_manhattan_distance_same(self) -> None:
        assert Position(5, 5).manhattan_distance(Position(5, 5)) == 0

    def test_repr(self) -> None:
        assert "Position" in repr(Position(1, 2))


# ==================================================================
# CellType tests
# ==================================================================


class TestCellType:
    def test_empty_is_traversable(self) -> None:
        assert CellType.EMPTY.is_traversable is True

    def test_obstacle_is_not_traversable(self) -> None:
        assert CellType.OBSTACLE.is_traversable is False


# ==================================================================
# Grid tests
# ==================================================================


class TestGrid:
    def test_creation(self) -> None:
        grid = Grid(10, 8)
        assert grid.width == 10
        assert grid.height == 8

    def test_invalid_dimensions(self) -> None:
        with pytest.raises(ValueError):
            Grid(0, 5)
        with pytest.raises(ValueError):
            Grid(5, -1)

    def test_valid_position_in_bounds(self) -> None:
        grid = Grid(10, 10)
        assert grid.in_bounds(Position(0, 0)) is True
        assert grid.in_bounds(Position(9, 9)) is True
        assert grid.in_bounds(Position(5, 5)) is True

    def test_out_of_bounds_positions(self) -> None:
        grid = Grid(10, 10)
        assert grid.in_bounds(Position(-1, 0)) is False
        assert grid.in_bounds(Position(0, -1)) is False
        assert grid.in_bounds(Position(10, 0)) is False
        assert grid.in_bounds(Position(0, 10)) is False
        assert grid.in_bounds(Position(100, 100)) is False

    def test_empty_cell_is_traversable(self) -> None:
        grid = Grid(5, 5)
        assert grid.is_traversable(Position(2, 2)) is True

    def test_obstacle_is_not_traversable(self) -> None:
        grid = Grid(5, 5)
        grid.add_obstacle(Position(2, 2))
        assert grid.is_traversable(Position(2, 2)) is False

    def test_out_of_bounds_is_not_traversable(self) -> None:
        grid = Grid(5, 5)
        assert grid.is_traversable(Position(-1, 0)) is False

    def test_add_and_remove_obstacle(self) -> None:
        grid = Grid(5, 5)
        pos = Position(3, 3)
        grid.add_obstacle(pos)
        assert grid.get_cell(pos) is CellType.OBSTACLE
        grid.remove_obstacle(pos)
        assert grid.get_cell(pos) is CellType.EMPTY

    def test_add_obstacles_batch(self) -> None:
        grid = Grid(5, 5)
        obstacles = [Position(0, 0), Position(1, 1), Position(2, 2)]
        grid.add_obstacles(obstacles)
        for pos in obstacles:
            assert grid.get_cell(pos) is CellType.OBSTACLE

    def test_get_cell_out_of_bounds_raises(self) -> None:
        grid = Grid(5, 5)
        with pytest.raises(IndexError):
            grid.get_cell(Position(10, 10))

    def test_neighbors_open_grid(self) -> None:
        grid = Grid(5, 5)
        # Center cell should have 4 neighbours.
        neighbors = grid.get_neighbors(Position(2, 2))
        assert len(neighbors) == 4
        expected = {Position(3, 2), Position(1, 2), Position(2, 3), Position(2, 1)}
        assert set(neighbors) == expected

    def test_neighbors_corner(self) -> None:
        grid = Grid(5, 5)
        neighbors = grid.get_neighbors(Position(0, 0))
        assert len(neighbors) == 2
        assert set(neighbors) == {Position(1, 0), Position(0, 1)}

    def test_neighbors_edge(self) -> None:
        grid = Grid(5, 5)
        neighbors = grid.get_neighbors(Position(0, 2))
        assert len(neighbors) == 3

    def test_neighbors_blocked(self) -> None:
        grid = Grid(5, 5)
        # Block all neighbours of (2,2)
        grid.add_obstacle(Position(3, 2))
        grid.add_obstacle(Position(1, 2))
        grid.add_obstacle(Position(2, 3))
        grid.add_obstacle(Position(2, 1))
        neighbors = grid.get_neighbors(Position(2, 2))
        assert neighbors == []

    def test_repr(self) -> None:
        grid = Grid(10, 8)
        grid.add_obstacle(Position(0, 0))
        r = repr(grid)
        assert "10" in r
        assert "8" in r
        assert "obstacles=1" in r
