"""AMR Communication Interface.

Provides the local interface that a robot uses to send and receive messages.
Each AMR owns one CommunicationInterface instance, which maintains its
local knowledge of the fleet (peer_state).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from swarmos.communication.message import RobotStateMessage
from swarmos.robot.state import RobotState
from swarmos.warehouse.cell import Position

if TYPE_CHECKING:
    from swarmos.communication.transport import SimulatedTransport


class CommunicationInterface:
    """The networking interface for a single robot.

    Maintains a local table of known peer states and provides methods
    to broadcast the owner's state and receive incoming messages.

    Parameters:
        robot_id:  The ID of the robot owning this interface.
        transport: The simulated transport layer to use for sending.
    """

    __slots__ = ("_robot_id", "_transport", "_peer_state")

    def __init__(self, robot_id: str, transport: SimulatedTransport) -> None:
        self._robot_id = robot_id
        self._transport = transport
        self._peer_state: dict[str, RobotStateMessage] = {}

        # Register with the transport so we can receive messages.
        self._transport.register(self)

    @property
    def robot_id(self) -> str:
        return self._robot_id

    @property
    def peer_state(self) -> dict[str, RobotStateMessage]:
        """Return the local knowledge of other robots.

        Returns a dictionary mapping peer robot IDs to their latest
        known state messages.
        """
        return self._peer_state.copy()

    def broadcast_state(
        self,
        position: Position,
        intended_next: Position,
        state: RobotState,
        tick: int,
    ) -> None:
        """Broadcast the robot's current state to all peers.

        Parameters:
            position:      Current grid position.
            intended_next: Intended next grid position.
            state:         Current activity state.
            tick:          Simulation timestamp.
        """
        message = RobotStateMessage(
            sender_id=self._robot_id,
            position=position,
            intended_next=intended_next,
            state=state,
            timestamp=tick,
        )
        self._transport.broadcast(message)

    def receive(self, message: RobotStateMessage) -> None:
        """Process an incoming message from the transport.

        Updates the local peer state ONLY if the message is newer than
        the currently known state for that peer. Stale or out-of-order
        messages are ignored.
        """
        if message.sender_id == self._robot_id:
            # A robot should never receive its own broadcast from the
            # transport, but this is a defensive check.
            return

        existing = self._peer_state.get(message.sender_id)
        if existing is None or message.timestamp > existing.timestamp:
            self._peer_state[message.sender_id] = message
