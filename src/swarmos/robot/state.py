"""Robot state definitions.

The state machine is designed so that future states can be added
without modifying consuming code.

Phase 1:  IDLE, MOVING, ARRIVED
Phase 3:  WAITING (yielding to another robot via stop-and-wait policy)
"""

from __future__ import annotations

from enum import Enum, auto


class RobotState(Enum):
    """High-level operational state of an AMR.

    States:
        IDLE:       Waiting for a task / path assignment.
        MOVING:     Executing a planned path.
        ARRIVED:    Reached its goal.
        WAITING:    Yielding to another robot (stop-and-wait policy).

    Future states (commented for documentation):
        PLANNING:   Computing / re-computing a path.
        CHARGING:   At a charging station.
        FAILED:     Unrecoverable error.
    """

    IDLE = auto()
    MOVING = auto()
    ARRIVED = auto()
    WAITING = auto()

    # PLANNING = auto()
    # CHARGING = auto()
    # FAILED = auto()
