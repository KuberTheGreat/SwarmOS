"""Communication message models.

Defines the payload structure for peer-to-peer state exchange.

Phase 4: position, intended_next, state, timestamp.
Phase 5: adds ``intent`` (IntentAction) — typed movement intention.
"""

from __future__ import annotations

from dataclasses import dataclass

from swarmos.communication.intent import IntentAction
from swarmos.robot.state import RobotState
from swarmos.warehouse.cell import Position


@dataclass(frozen=True)
class RobotStateMessage:
    """A broadcast message containing a robot's local state and intent.

    Attributes:
        sender_id:     The ID of the robot sending the message.
        position:      The robot's current grid position.
        intended_next: The cell the robot intends to occupy next.
        state:         The robot's current activity state (e.g. MOVING).
        timestamp:     The simulation tick when the message was sent.
        intent:        The robot's typed movement intention for this tick.
    """

    sender_id: str
    position: Position
    intended_next: Position
    state: RobotState
    timestamp: int
    intent: IntentAction = IntentAction.IDLE

    def __repr__(self) -> str:
        return (
            f"RobotStateMessage({self.sender_id!r}, "
            f"pos={self.position}, next={self.intended_next}, "
            f"state={self.state.name}, intent={self.intent.name}, "
            f"t={self.timestamp})"
        )

