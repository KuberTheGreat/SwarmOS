"""Conflict and collision data models.

This module defines the types used to represent detected conflicts
between robot paths and actual runtime collisions.

KEY DISTINCTION:
    - Conflict:  A planned trajectory overlap (detected statically from
                 paths before or during simulation).
    - Collision: Two robots actually occupying the same cell at the same
                 simulation tick (detected at runtime).

These concepts are kept separate because:
    1. Conflicts can be detected before execution starts.
    2. Collisions can only be detected during execution.
    3. Future conflict resolution will try to prevent collisions by
       responding to conflicts.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto

from swarmos.warehouse.cell import Position


class ConflictType(Enum):
    """Classification of a path conflict.

    Phase 2 types:
        NODE: Two robots planned to occupy the same cell at the same
              timestep.
        EDGE: Two robots planned to swap positions (traverse the same
              edge in opposite directions) at the same timestep.

    Future types (commented for documentation):
        FOLLOWING:     Robot B occupies the cell Robot A just left.
        INTERSECTION:  Multiple robots converge on a shared corridor.
        DEADLOCK:      Circular wait among robots.
    """

    NODE = auto()
    EDGE = auto()

    # FOLLOWING = auto()
    # INTERSECTION = auto()
    # DEADLOCK = auto()


@dataclass(frozen=True)
class Conflict:
    """A detected conflict between two robot paths.

    Attributes:
        conflict_type: The kind of conflict (NODE or EDGE).
        robot_a_id:    ID of the first robot.
        robot_b_id:    ID of the second robot.
        position:      The grid position where the conflict occurs.
                       For EDGE conflicts, this is the position that
                       robot_a moves TO (and robot_b moves FROM).
        timestep:      The discrete timestep at which the conflict occurs.
    """

    conflict_type: ConflictType
    robot_a_id: str
    robot_b_id: str
    position: Position
    timestep: int

    def __repr__(self) -> str:
        return (
            f"Conflict({self.conflict_type.name}, "
            f"{self.robot_a_id!r} ↔ {self.robot_b_id!r}, "
            f"pos={self.position}, t={self.timestep})"
        )


@dataclass(frozen=True)
class Collision:
    """A runtime collision — two robots occupying the same cell.

    Attributes:
        robot_a_id: ID of the first robot.
        robot_b_id: ID of the second robot.
        position:   The cell where the collision occurred.
        tick:       The simulation tick when the collision was detected.
    """

    robot_a_id: str
    robot_b_id: str
    position: Position
    tick: int

    def __repr__(self) -> str:
        return (
            f"Collision({self.robot_a_id!r} ↔ {self.robot_b_id!r}, "
            f"pos={self.position}, tick={self.tick})"
        )
