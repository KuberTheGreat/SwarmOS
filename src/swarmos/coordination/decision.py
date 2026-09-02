"""Decision types for coordination policies.

This module defines the data types used by coordination policies to
make per-robot movement decisions.  The types are intentionally simple
and decoupled from the robot implementation so that policies can be
tested in isolation.

KEY TYPES:
    Decision:       MOVE or WAIT — the output of a policy for one robot.
    RobotSnapshot:  A frozen view of a robot's state at a single instant,
                    used as input to the policy.  Snapshots ensure that
                    all robots are evaluated against the *same* world
                    state (simultaneous-update semantics).
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto

from swarmos.robot.state import RobotState
from swarmos.warehouse.cell import Position


class Decision(Enum):
    """The outcome of a coordination policy for a single robot.

    MOVE:  The robot is cleared to advance one step along its path.
    WAIT:  The robot must remain at its current position this tick.
    """

    MOVE = auto()
    WAIT = auto()


@dataclass(frozen=True)
class RobotSnapshot:
    """An immutable snapshot of a robot's state for policy evaluation.

    Attributes:
        robot_id:      Unique identifier of the robot.
        position:      Current grid position.
        intended_next: The position the robot *wants* to move to.
                       Equal to ``position`` if the robot is not moving
                       (IDLE, ARRIVED, WAITING, or path complete).
        state:         The robot's current operational state.
    """

    robot_id: str
    position: Position
    intended_next: Position
    state: RobotState
