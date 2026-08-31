"""Robot state definitions.

The state machine is intentionally minimal for Phase 1 but designed
so that future states (CHARGING, WAITING, COMMUNICATING, FAILED, etc.)
can be added without modifying consuming code.
"""

from __future__ import annotations

from enum import Enum, auto


class RobotState(Enum):
    """High-level operational state of an AMR.

    Phase 1 states:
        IDLE:       Waiting for a task / path assignment.
        MOVING:     Executing a planned path.
        ARRIVED:    Reached its goal.

    Future states (commented for documentation):
        PLANNING:   Computing / re-computing a path.
        WAITING:    Yielding to another robot (conflict resolution).
        CHARGING:   At a charging station.
        FAILED:     Unrecoverable error.
    """

    IDLE = auto()
    MOVING = auto()
    ARRIVED = auto()

    # PLANNING = auto()
    # WAITING = auto()
    # CHARGING = auto()
    # FAILED = auto()
