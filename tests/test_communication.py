"""Unit tests for Phase 4: P2P Communication.

Verifies the message structure, the local communication interface,
the deterministic simulated transport, and timestamp/stale handling.
"""

from __future__ import annotations

import pytest

from swarmos.communication.interface import CommunicationInterface
from swarmos.communication.message import RobotStateMessage
from swarmos.communication.transport import SimulatedTransport
from swarmos.robot.state import RobotState
from swarmos.warehouse.cell import Position


# ==================================================================
# 1. Message Structure
# ==================================================================

class TestRobotStateMessage:
    def test_message_initialisation(self) -> None:
        msg = RobotStateMessage(
            sender_id="AMR-01",
            position=Position(1, 1),
            intended_next=Position(1, 2),
            state=RobotState.MOVING,
            timestamp=42,
        )
        assert msg.sender_id == "AMR-01"
        assert msg.position == Position(1, 1)
        assert msg.intended_next == Position(1, 2)
        assert msg.state is RobotState.MOVING
        assert msg.timestamp == 42

    def test_message_is_frozen(self) -> None:
        msg = RobotStateMessage("R1", Position(0, 0), Position(0, 0), RobotState.IDLE, 0)
        with pytest.raises(Exception):
            msg.timestamp = 1  # type: ignore[misc]


# ==================================================================
# 2. Local Peer State (CommunicationInterface)
# ==================================================================

class TestCommunicationInterface:
    def setup_method(self) -> None:
        self.transport = SimulatedTransport(delay=0)
        self.interface = CommunicationInterface("AMR-01", self.transport)

    def test_initial_peer_state_is_empty(self) -> None:
        assert len(self.interface.peer_state) == 0

    def test_receive_new_peer_adds_to_state(self) -> None:
        msg = RobotStateMessage("AMR-02", Position(2, 2), Position(2, 3), RobotState.MOVING, 10)
        self.interface.receive(msg)
        
        state = self.interface.peer_state
        assert len(state) == 1
        assert "AMR-02" in state
        assert state["AMR-02"] is msg

    def test_receive_newer_message_updates_state(self) -> None:
        msg1 = RobotStateMessage("AMR-02", Position(2, 2), Position(2, 3), RobotState.MOVING, 10)
        msg2 = RobotStateMessage("AMR-02", Position(2, 3), Position(2, 4), RobotState.MOVING, 15)
        
        self.interface.receive(msg1)
        self.interface.receive(msg2)
        
        assert self.interface.peer_state["AMR-02"] is msg2

    def test_receive_stale_message_is_ignored(self) -> None:
        msg_new = RobotStateMessage("AMR-02", Position(2, 3), Position(2, 4), RobotState.MOVING, 15)
        msg_old = RobotStateMessage("AMR-02", Position(2, 2), Position(2, 3), RobotState.MOVING, 10)
        
        self.interface.receive(msg_new)
        self.interface.receive(msg_old)
        
        assert self.interface.peer_state["AMR-02"] is msg_new

    def test_receive_duplicate_message_is_ignored(self) -> None:
        msg1 = RobotStateMessage("AMR-02", Position(2, 2), Position(2, 3), RobotState.MOVING, 10)
        msg2 = RobotStateMessage("AMR-02", Position(2, 2), Position(2, 3), RobotState.MOVING, 10)
        
        self.interface.receive(msg1)
        self.interface.receive(msg2)
        
        # Exact identity check since it should keep the first one
        assert self.interface.peer_state["AMR-02"] is msg1

    def test_robot_ignores_its_own_messages(self) -> None:
        msg = RobotStateMessage("AMR-01", Position(1, 1), Position(1, 1), RobotState.IDLE, 5)
        self.interface.receive(msg)
        assert len(self.interface.peer_state) == 0

    def test_broadcast_sends_to_transport(self) -> None:
        self.interface.broadcast_state(Position(5, 5), Position(5, 6), RobotState.MOVING, 20)
        assert self.transport.metrics_sent == 1


# ==================================================================
# 3. Transport and Delivery
# ==================================================================

class TestSimulatedTransport:
    def test_zero_delay_delivery(self) -> None:
        transport = SimulatedTransport(delay=0)
        iface1 = CommunicationInterface("R1", transport)
        iface2 = CommunicationInterface("R2", transport)
        
        iface1.broadcast_state(Position(1, 1), Position(1, 1), RobotState.IDLE, 5)
        
        # Message is queued but not delivered yet
        assert len(iface2.peer_state) == 0
        
        # Deliver at current tick
        transport.deliver_messages(5)
        
        assert len(iface2.peer_state) == 1
        assert "R1" in iface2.peer_state
        assert transport.metrics_sent == 1
        assert transport.metrics_delivered == 1

    def test_delayed_delivery(self) -> None:
        transport = SimulatedTransport(delay=3)
        iface1 = CommunicationInterface("R1", transport)
        iface2 = CommunicationInterface("R2", transport)
        
        iface1.broadcast_state(Position(1, 1), Position(1, 1), RobotState.IDLE, 5)
        
        # Tick 5, 6, 7 — should not be delivered
        transport.deliver_messages(5)
        transport.deliver_messages(6)
        transport.deliver_messages(7)
        assert len(iface2.peer_state) == 0
        
        # Tick 8 — should be delivered (5 + 3 = 8)
        transport.deliver_messages(8)
        assert len(iface2.peer_state) == 1

    def test_broadcast_reaches_multiple_peers(self) -> None:
        transport = SimulatedTransport(delay=0)
        iface1 = CommunicationInterface("R1", transport)
        iface2 = CommunicationInterface("R2", transport)
        iface3 = CommunicationInterface("R3", transport)
        
        iface1.broadcast_state(Position(1, 1), Position(1, 1), RobotState.IDLE, 10)
        transport.deliver_messages(10)
        
        assert "R1" in iface2.peer_state
        assert "R1" in iface3.peer_state
        assert "R1" not in iface1.peer_state
        assert transport.metrics_delivered == 2

    def test_multiple_messages_queued(self) -> None:
        transport = SimulatedTransport(delay=2)
        iface1 = CommunicationInterface("R1", transport)
        iface2 = CommunicationInterface("R2", transport)
        
        # Send at tick 10 (arrives 12)
        iface1.broadcast_state(Position(1, 1), Position(1, 1), RobotState.MOVING, 10)
        # Send at tick 11 (arrives 13)
        iface1.broadcast_state(Position(1, 2), Position(1, 2), RobotState.MOVING, 11)
        
        transport.deliver_messages(11)
        assert len(iface2.peer_state) == 0
        
        transport.deliver_messages(12)
        assert iface2.peer_state["R1"].timestamp == 10
        
        transport.deliver_messages(13)
        assert iface2.peer_state["R1"].timestamp == 11
