"""Cell types and position representation for the warehouse grid.

This module defines the fundamental spatial types used throughout SwarmOS:
- Position: an immutable (x, y) coordinate on the grid
- CellType: an enum describing what occupies a grid cell

Design note: Position is a NamedTuple rather than a dataclass so it is
immutable, hashable, and usable as a dict key / set member without any
extra boilerplate.  CellType is an enum designed for forward-compatible
extension — adding new variants (e.g. CHARGING_STATION, PICKUP_POINT)
requires zero changes to the grid or planner.
"""

from __future__ import annotations

from enum import Enum, auto
from typing import NamedTuple


class Position(NamedTuple):
    """Immutable 2-D grid coordinate.

    Attributes:
        x: Column index (0-indexed, increases rightward).
        y: Row index (0-indexed, increases downward).
    """

    x: int
    y: int

    def __repr__(self) -> str:
        return f"Position(x={self.x}, y={self.y})"

    def manhattan_distance(self, other: Position) -> int:
        """Return the Manhattan distance to *other*."""
        return abs(self.x - other.x) + abs(self.y - other.y)


class CellType(Enum):
    """Classification of a single grid cell.

    Phase 1 uses only EMPTY and OBSTACLE.  Future phases will extend
    this enum with additional cell semantics (pickup points, charging
    stations, intersections, etc.) without modifying existing code that
    only cares about traversability.
    """

    EMPTY = auto()
    OBSTACLE = auto()

    # ----- future cell types (commented stubs for documentation) -----
    # PICKUP_POINT = auto()
    # DROP_POINT = auto()
    # CHARGING_STATION = auto()
    # INTERSECTION = auto()

    @property
    def is_traversable(self) -> bool:
        """Return ``True`` if a robot can move through this cell type."""
        return self is not CellType.OBSTACLE
