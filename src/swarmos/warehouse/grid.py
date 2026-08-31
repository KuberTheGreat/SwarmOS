"""Warehouse grid — the spatial model of the environment.

The Grid is purely an *environment* abstraction.  It knows nothing about
path-planning algorithms or robot decision-making.  Its job is to answer
spatial queries:

* Is a position within bounds?
* Is a cell traversable?
* What are the traversable neighbours of a position?

This keeps the warehouse model reusable across different planners,
multiple robots, and headless benchmark runs.
"""

from __future__ import annotations

from typing import Sequence

from swarmos.warehouse.cell import CellType, Position

# 4-directional movement offsets: right, left, down, up.
_DIRECTION_OFFSETS: list[tuple[int, int]] = [
    (1, 0),   # right
    (-1, 0),  # left
    (0, 1),   # down
    (0, -1),  # up
]


class Grid:
    """A 2-D rectangular grid of cells representing the warehouse floor.

    Parameters:
        width:  Number of columns.
        height: Number of rows.

    Internally the grid is stored as a flat list in row-major order
    (``index = y * width + x``).  All cells start as ``CellType.EMPTY``.
    """

    __slots__ = ("_width", "_height", "_cells")

    def __init__(self, width: int, height: int) -> None:
        if width <= 0 or height <= 0:
            raise ValueError(
                f"Grid dimensions must be positive, got {width}×{height}"
            )
        self._width = width
        self._height = height
        self._cells: list[CellType] = [CellType.EMPTY] * (width * height)

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def width(self) -> int:
        """Number of columns."""
        return self._width

    @property
    def height(self) -> int:
        """Number of rows."""
        return self._height

    # ------------------------------------------------------------------
    # Spatial queries
    # ------------------------------------------------------------------

    def in_bounds(self, pos: Position) -> bool:
        """Return ``True`` if *pos* lies within the grid."""
        return 0 <= pos.x < self._width and 0 <= pos.y < self._height

    def _index(self, pos: Position) -> int:
        """Convert a position to a flat-array index (unchecked)."""
        return pos.y * self._width + pos.x

    def get_cell(self, pos: Position) -> CellType:
        """Return the :class:`CellType` at *pos*.

        Raises:
            IndexError: If *pos* is out of bounds.
        """
        if not self.in_bounds(pos):
            raise IndexError(f"Position {pos} is out of bounds for {self._width}×{self._height} grid")
        return self._cells[self._index(pos)]

    def is_traversable(self, pos: Position) -> bool:
        """Return ``True`` if *pos* is in-bounds and traversable."""
        return self.in_bounds(pos) and self._cells[self._index(pos)].is_traversable

    def set_cell(self, pos: Position, cell_type: CellType) -> None:
        """Set the cell at *pos* to *cell_type*.

        Raises:
            IndexError: If *pos* is out of bounds.
        """
        if not self.in_bounds(pos):
            raise IndexError(f"Position {pos} is out of bounds for {self._width}×{self._height} grid")
        self._cells[self._index(pos)] = cell_type

    def add_obstacle(self, pos: Position) -> None:
        """Mark *pos* as an obstacle."""
        self.set_cell(pos, CellType.OBSTACLE)

    def remove_obstacle(self, pos: Position) -> None:
        """Reset *pos* to empty (remove an obstacle)."""
        self.set_cell(pos, CellType.EMPTY)

    def add_obstacles(self, positions: Sequence[Position]) -> None:
        """Mark multiple positions as obstacles."""
        for pos in positions:
            self.add_obstacle(pos)

    def get_neighbors(self, pos: Position) -> list[Position]:
        """Return the traversable 4-directional neighbours of *pos*.

        Only positions that are in-bounds **and** traversable are
        returned.  The order is deterministic: right, left, down, up.
        """
        neighbors: list[Position] = []
        for dx, dy in _DIRECTION_OFFSETS:
            neighbor = Position(pos.x + dx, pos.y + dy)
            if self.is_traversable(neighbor):
                neighbors.append(neighbor)
        return neighbors

    def get_all_neighbors(self, pos: Position) -> list[Position]:
        """Return all in-bounds 4-directional neighbours regardless of
        traversability.  Useful for visualization and debugging."""
        neighbors: list[Position] = []
        for dx, dy in _DIRECTION_OFFSETS:
            neighbor = Position(pos.x + dx, pos.y + dy)
            if self.in_bounds(neighbor):
                neighbors.append(neighbor)
        return neighbors

    # ------------------------------------------------------------------
    # Representation
    # ------------------------------------------------------------------

    def __repr__(self) -> str:
        obstacle_count = sum(
            1 for c in self._cells if c is CellType.OBSTACLE
        )
        return (
            f"Grid(width={self._width}, height={self._height}, "
            f"obstacles={obstacle_count})"
        )
