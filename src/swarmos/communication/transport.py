"""Simulated Transport Layer.

Models the delivery of messages between AMRs, allowing for deterministic
delivery with optional simulated delays.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from swarmos.communication.interface import CommunicationInterface
    from swarmos.communication.message import RobotStateMessage


class SimulatedTransport:
    """Delivers messages between registered communication interfaces.

    Provides deterministic delivery. Messages queued via `broadcast()`
    are delivered when `deliver_messages()` is called. If `delay` is
    configured, messages are held until the appropriate simulation tick.

    Parameters:
        delay: The number of simulation ticks a message takes to arrive
               (default 0 means delivery on the same tick it was sent).
    """

    __slots__ = (
        "_delay",
        "_interfaces",
        "_in_flight",
        "_metrics_sent",
        "_metrics_delivered",
    )

    def __init__(self, delay: int = 0) -> None:
        self._delay = delay
        self._interfaces: dict[str, CommunicationInterface] = {}
        # List of (delivery_tick, message)
        self._in_flight: list[tuple[int, RobotStateMessage]] = []

        self._metrics_sent: int = 0
        self._metrics_delivered: int = 0

    @property
    def metrics_sent(self) -> int:
        return self._metrics_sent

    @property
    def metrics_delivered(self) -> int:
        return self._metrics_delivered

    @property
    def interfaces(self) -> dict[str, CommunicationInterface]:
        return self._interfaces

    def register(self, interface: CommunicationInterface) -> None:
        """Register a communication interface to receive messages."""
        self._interfaces[interface.robot_id] = interface

    def broadcast(self, message: RobotStateMessage) -> None:
        """Queue a message for broadcast to all registered peers."""
        delivery_tick = message.timestamp + self._delay
        self._in_flight.append((delivery_tick, message))
        self._metrics_sent += 1

    def deliver_messages(self, current_tick: int) -> None:
        """Deliver all messages scheduled for delivery by `current_tick`.

        Called by the simulation engine during the communication phase.
        """
        if not self._in_flight:
            return

        # Separate messages that are ready to deliver from those still delayed.
        deliver_now = [m for t, m in self._in_flight if t <= current_tick]
        self._in_flight = [(t, m) for t, m in self._in_flight if t > current_tick]

        for msg in deliver_now:
            # Deliver to all interfaces EXCEPT the sender.
            for robot_id, interface in self._interfaces.items():
                if robot_id != msg.sender_id:
                    interface.receive(msg)
                    self._metrics_delivered += 1
