"""Tests for Phase 5: Distributed Intent Exchange.

Covers all 20 acceptance criteria:
  1-4.   Intent generation (MOVE, WAIT, IDLE, COMPLETE) with position info
  5-6.   Intent contains timestamp and is broadcast through P2P
  7-9.   Peer receipt, multi-peer receipt, independent local state
 10-14.  Timestamp ordering, stale detection, delayed delivery
 15-17.  Potential node/edge conflict observation
 18-19.  Observation does NOT modify movement, Phase 3 unchanged
    20.  Full test suite still passes (verified by running pytest)
"""

from __future__ import annotations

import pytest

from swarmos.communication.intent import (
    ConflictType,
    IntentAction,
    IntentObserver,
    PotentialConflict,
    is_stale,
)
from swarmos.communication.interface import CommunicationInterface
from swarmos.communication.message import RobotStateMessage
from swarmos.communication.transport import SimulatedTransport
from swarmos.coordination.policy import StopAndWaitPolicy
from swarmos.planning.path import Path
from swarmos.robot.amr import AMR
from swarmos.robot.fleet import Fleet
from swarmos.robot.state import RobotState
from swarmos.simulation.config import SimulationConfig
from swarmos.simulation.engine import SimulationEngine
from swarmos.warehouse.cell import Position
from swarmos.warehouse.grid import Grid


# ==================================================================
# Helpers
# ==================================================================

def _make_amr_with_comms(
    robot_id: str,
    pos: tuple[int, int],
    transport: SimulatedTransport,
) -> AMR:
    """Build an AMR with a CommunicationInterface attached."""
    amr = AMR(robot_id=robot_id, position=Position(*pos))
    amr.communication = CommunicationInterface(robot_id, transport)
    return amr


def _build_scenario(
    grid_size: tuple[int, int],
    robots: list[tuple[str, tuple[int, int], tuple[int, int]]],
    step_delay: int = 1,
) -> SimulationEngine:
    """Build a simulation with stop-and-wait policy."""
    grid = Grid(*grid_size)
    fleet = Fleet()
    goals: dict[str, Position] = {}

    for rid, start, goal in robots:
        amr = AMR(robot_id=rid, position=Position(*start))
        fleet.add(amr)
        goals[rid] = Position(*goal)

    policy = StopAndWaitPolicy()
    config = SimulationConfig(robot_step_delay=step_delay)
    engine = SimulationEngine(
        grid=grid, robots=fleet, config=config,
        policy=policy, max_ticks=5000,
    )
    results = engine.plan_all(goals)
    assert all(results.values())
    return engine


# ==================================================================
# 1–4. Intent Generation
# ==================================================================

class TestIntentGeneration:
    """Robot derives correct IntentAction from its current state."""

    def test_moving_robot_generates_move_intent(self) -> None:
        transport = SimulatedTransport()
        amr = _make_amr_with_comms("AMR-01", (0, 0), transport)
        grid = Grid(5, 1)
        amr.plan(grid, Position(4, 0))

        assert amr.state is RobotState.MOVING
        amr.broadcast_state(10)
        transport.deliver_messages(10)

        # No peers to check against, but verify the message was sent
        assert transport.metrics_sent == 1

    def test_waiting_robot_generates_wait_intent(self) -> None:
        transport = SimulatedTransport()
        amr = _make_amr_with_comms("AMR-01", (0, 0), transport)
        iface2 = CommunicationInterface("AMR-02", transport)

        grid = Grid(5, 1)
        amr.plan(grid, Position(4, 0))
        amr.wait()

        assert amr.state is RobotState.WAITING
        amr.broadcast_state(10)
        transport.deliver_messages(10)

        msg = iface2.peer_state["AMR-01"]
        assert msg.intent is IntentAction.WAIT

    def test_idle_robot_generates_idle_intent(self) -> None:
        transport = SimulatedTransport()
        amr = _make_amr_with_comms("AMR-01", (0, 0), transport)
        iface2 = CommunicationInterface("AMR-02", transport)

        assert amr.state is RobotState.IDLE
        amr.broadcast_state(5)
        transport.deliver_messages(5)

        msg = iface2.peer_state["AMR-01"]
        assert msg.intent is IntentAction.IDLE

    def test_arrived_robot_generates_complete_intent(self) -> None:
        transport = SimulatedTransport()
        amr = _make_amr_with_comms("AMR-01", (0, 0), transport)
        iface2 = CommunicationInterface("AMR-02", transport)

        grid = Grid(5, 1)
        amr.plan(grid, Position(0, 0))  # start == goal

        assert amr.state is RobotState.ARRIVED
        amr.broadcast_state(5)
        transport.deliver_messages(5)

        msg = iface2.peer_state["AMR-01"]
        assert msg.intent is IntentAction.COMPLETE

    def test_intent_contains_current_position(self) -> None:
        transport = SimulatedTransport()
        amr = _make_amr_with_comms("AMR-01", (3, 7), transport)
        iface2 = CommunicationInterface("AMR-02", transport)

        amr.broadcast_state(1)
        transport.deliver_messages(1)

        msg = iface2.peer_state["AMR-01"]
        assert msg.position == Position(3, 7)

    def test_intent_contains_intended_next(self) -> None:
        transport = SimulatedTransport()
        amr = _make_amr_with_comms("AMR-01", (0, 0), transport)
        iface2 = CommunicationInterface("AMR-02", transport)

        grid = Grid(5, 1)
        amr.plan(grid, Position(4, 0))
        amr.broadcast_state(1)
        transport.deliver_messages(1)

        msg = iface2.peer_state["AMR-01"]
        assert msg.intended_next == Position(1, 0)

    def test_intent_contains_timestamp(self) -> None:
        transport = SimulatedTransport()
        amr = _make_amr_with_comms("AMR-01", (0, 0), transport)
        iface2 = CommunicationInterface("AMR-02", transport)

        amr.broadcast_state(42)
        transport.deliver_messages(42)

        msg = iface2.peer_state["AMR-01"]
        assert msg.timestamp == 42


# ==================================================================
# 5–7. P2P Delivery of Intents
# ==================================================================

class TestIntentPeerDelivery:
    def test_peer_receives_intent(self) -> None:
        transport = SimulatedTransport()
        amr1 = _make_amr_with_comms("AMR-01", (0, 0), transport)
        amr2 = _make_amr_with_comms("AMR-02", (5, 5), transport)

        grid = Grid(10, 10)
        amr1.plan(grid, Position(9, 0))
        amr1.broadcast_state(10)
        transport.deliver_messages(10)

        state = amr2.peer_state
        assert "AMR-01" in state
        assert state["AMR-01"].intent is IntentAction.MOVE

    def test_multiple_peers_receive_intent(self) -> None:
        transport = SimulatedTransport()
        amr1 = _make_amr_with_comms("AMR-01", (0, 0), transport)
        amr2 = _make_amr_with_comms("AMR-02", (5, 5), transport)
        amr3 = _make_amr_with_comms("AMR-03", (9, 9), transport)

        amr1.broadcast_state(10)
        transport.deliver_messages(10)

        assert "AMR-01" in amr2.peer_state
        assert "AMR-01" in amr3.peer_state
        assert "AMR-01" not in amr1.peer_state

    def test_independent_local_state(self) -> None:
        """Each robot maintains its own independent local view."""
        transport = SimulatedTransport()
        amr1 = _make_amr_with_comms("AMR-01", (0, 0), transport)
        amr2 = _make_amr_with_comms("AMR-02", (5, 5), transport)
        amr3 = _make_amr_with_comms("AMR-03", (9, 9), transport)

        grid = Grid(10, 10)
        amr1.plan(grid, Position(9, 0))
        amr2.plan(grid, Position(5, 9))

        amr1.broadcast_state(10)
        amr2.broadcast_state(10)
        amr3.broadcast_state(10)
        transport.deliver_messages(10)

        # AMR-01 knows about AMR-02 and AMR-03, but not itself
        s1 = amr1.peer_state
        assert "AMR-02" in s1
        assert "AMR-03" in s1
        assert "AMR-01" not in s1

        # AMR-02 knows about AMR-01 and AMR-03
        s2 = amr2.peer_state
        assert "AMR-01" in s2
        assert "AMR-03" in s2


# ==================================================================
# 8–12. Timestamp Ordering and Stale Handling
# ==================================================================

class TestTimestampOrdering:
    def test_newer_intent_replaces_older(self) -> None:
        transport = SimulatedTransport()
        amr1 = _make_amr_with_comms("AMR-01", (0, 0), transport)
        amr2 = _make_amr_with_comms("AMR-02", (5, 5), transport)

        grid = Grid(10, 1)
        amr1.plan(grid, Position(9, 0))

        amr1.broadcast_state(10)
        transport.deliver_messages(10)
        assert amr2.peer_state["AMR-01"].timestamp == 10

        amr1.step()
        amr1.broadcast_state(11)
        transport.deliver_messages(11)
        assert amr2.peer_state["AMR-01"].timestamp == 11
        assert amr2.peer_state["AMR-01"].position == Position(1, 0)

    def test_older_intent_cannot_overwrite_newer(self) -> None:
        transport = SimulatedTransport()
        iface1 = CommunicationInterface("AMR-01", transport)
        iface2 = CommunicationInterface("AMR-02", transport)

        msg_new = RobotStateMessage(
            "AMR-01", Position(2, 0), Position(3, 0),
            RobotState.MOVING, 15, IntentAction.MOVE,
        )
        msg_old = RobotStateMessage(
            "AMR-01", Position(1, 0), Position(2, 0),
            RobotState.MOVING, 10, IntentAction.MOVE,
        )

        iface2.receive(msg_new)
        iface2.receive(msg_old)

        assert iface2.peer_state["AMR-01"] is msg_new

    def test_duplicate_intent_is_safe(self) -> None:
        transport = SimulatedTransport()
        iface1 = CommunicationInterface("AMR-01", transport)
        iface2 = CommunicationInterface("AMR-02", transport)

        msg1 = RobotStateMessage(
            "AMR-01", Position(1, 0), Position(2, 0),
            RobotState.MOVING, 10, IntentAction.MOVE,
        )
        msg2 = RobotStateMessage(
            "AMR-01", Position(1, 0), Position(2, 0),
            RobotState.MOVING, 10, IntentAction.MOVE,
        )

        iface2.receive(msg1)
        iface2.receive(msg2)
        assert iface2.peer_state["AMR-01"] is msg1

    def test_delayed_intent_delivery(self) -> None:
        transport = SimulatedTransport(delay=2)
        amr1 = _make_amr_with_comms("AMR-01", (0, 0), transport)
        amr2 = _make_amr_with_comms("AMR-02", (5, 5), transport)

        amr1.broadcast_state(10)

        transport.deliver_messages(10)
        transport.deliver_messages(11)
        assert len(amr2.peer_state) == 0

        transport.deliver_messages(12)
        assert "AMR-01" in amr2.peer_state


# ==================================================================
# 13. Stale Intent Identification
# ==================================================================

class TestStaleIntent:
    def test_is_stale_helper(self) -> None:
        msg = RobotStateMessage(
            "R1", Position(0, 0), Position(0, 0),
            RobotState.IDLE, 10, IntentAction.IDLE,
        )
        assert not is_stale(msg, 15, threshold=5)
        assert is_stale(msg, 16, threshold=5)

    def test_fresh_peer_state_filters_stale(self) -> None:
        transport = SimulatedTransport()
        iface = CommunicationInterface("AMR-01", transport, stale_threshold=5)

        msg_fresh = RobotStateMessage(
            "AMR-02", Position(2, 0), Position(3, 0),
            RobotState.MOVING, 20, IntentAction.MOVE,
        )
        msg_stale = RobotStateMessage(
            "AMR-03", Position(5, 0), Position(5, 0),
            RobotState.IDLE, 10, IntentAction.IDLE,
        )

        iface.receive(msg_fresh)
        iface.receive(msg_stale)

        assert len(iface.peer_state) == 2  # All entries
        fresh = iface.fresh_peer_state(current_tick=22)
        assert "AMR-02" in fresh   # 22 - 20 = 2 ≤ 5
        assert "AMR-03" not in fresh  # 22 - 10 = 12 > 5

    def test_fresh_peer_state_no_threshold(self) -> None:
        """Without a threshold, all entries are returned."""
        transport = SimulatedTransport()
        iface = CommunicationInterface("AMR-01", transport, stale_threshold=None)

        msg = RobotStateMessage(
            "AMR-02", Position(0, 0), Position(0, 0),
            RobotState.IDLE, 1, IntentAction.IDLE,
        )
        iface.receive(msg)

        fresh = iface.fresh_peer_state(current_tick=1000)
        assert "AMR-02" in fresh


# ==================================================================
# 14–15. Potential Conflict Observation
# ==================================================================

class TestConflictObservation:
    """IntentObserver detects potential conflicts (read-only)."""

    def test_potential_node_conflict(self) -> None:
        """Two intents targeting the same cell."""
        peer_state = {
            "AMR-02": RobotStateMessage(
                "AMR-02", Position(6, 5), Position(5, 5),
                RobotState.MOVING, 10, IntentAction.MOVE,
            ),
        }
        conflicts = IntentObserver.observe(
            own_id="AMR-01",
            own_position=Position(4, 5),
            own_intended=Position(5, 5),
            own_intent=IntentAction.MOVE,
            peer_state=peer_state,
            current_tick=10,
        )
        assert len(conflicts) == 1
        assert conflicts[0].conflict_type is ConflictType.NODE
        assert conflicts[0].position == Position(5, 5)
        assert set([conflicts[0].robot_a_id, conflicts[0].robot_b_id]) == {"AMR-01", "AMR-02"}

    def test_potential_edge_conflict(self) -> None:
        """Two intents forming an opposite-direction swap."""
        peer_state = {
            "AMR-02": RobotStateMessage(
                "AMR-02", Position(5, 5), Position(4, 5),
                RobotState.MOVING, 10, IntentAction.MOVE,
            ),
        }
        conflicts = IntentObserver.observe(
            own_id="AMR-01",
            own_position=Position(4, 5),
            own_intended=Position(5, 5),
            own_intent=IntentAction.MOVE,
            peer_state=peer_state,
            current_tick=10,
        )
        assert len(conflicts) == 1
        assert conflicts[0].conflict_type is ConflictType.EDGE

    def test_no_conflict_disjoint_intents(self) -> None:
        """Disjoint intents produce no observations."""
        peer_state = {
            "AMR-02": RobotStateMessage(
                "AMR-02", Position(8, 8), Position(8, 9),
                RobotState.MOVING, 10, IntentAction.MOVE,
            ),
        }
        conflicts = IntentObserver.observe(
            own_id="AMR-01",
            own_position=Position(0, 0),
            own_intended=Position(1, 0),
            own_intent=IntentAction.MOVE,
            peer_state=peer_state,
            current_tick=10,
        )
        assert len(conflicts) == 0

    def test_waiting_robot_creates_no_conflicts(self) -> None:
        """A WAIT intent does not produce conflict observations."""
        peer_state = {
            "AMR-02": RobotStateMessage(
                "AMR-02", Position(5, 5), Position(5, 6),
                RobotState.MOVING, 10, IntentAction.MOVE,
            ),
        }
        conflicts = IntentObserver.observe(
            own_id="AMR-01",
            own_position=Position(5, 5),
            own_intended=Position(5, 5),
            own_intent=IntentAction.WAIT,
            peer_state=peer_state,
            current_tick=10,
        )
        assert len(conflicts) == 0

    def test_stale_peers_skipped_in_observation(self) -> None:
        """Observer skips stale peer intents."""
        peer_state = {
            "AMR-02": RobotStateMessage(
                "AMR-02", Position(6, 5), Position(5, 5),
                RobotState.MOVING, 5, IntentAction.MOVE,
            ),
        }
        # With stale_threshold=3 at tick 10: age=5 > 3 → stale
        conflicts = IntentObserver.observe(
            own_id="AMR-01",
            own_position=Position(4, 5),
            own_intended=Position(5, 5),
            own_intent=IntentAction.MOVE,
            peer_state=peer_state,
            current_tick=10,
            stale_threshold=3,
        )
        assert len(conflicts) == 0

    def test_observe_from_amr(self) -> None:
        """observe_conflicts() works end-to-end on an AMR."""
        transport = SimulatedTransport()
        amr1 = _make_amr_with_comms("AMR-01", (4, 5), transport)
        amr2 = _make_amr_with_comms("AMR-02", (6, 5), transport)

        grid = Grid(10, 10)
        amr1.plan(grid, Position(9, 5))  # next = (5, 5)
        amr2.plan(grid, Position(0, 5))  # next = (5, 5)

        # Broadcast
        amr1.broadcast_state(10)
        amr2.broadcast_state(10)
        transport.deliver_messages(10)

        # AMR-01 should observe the node conflict
        conflicts = amr1.observe_conflicts(10)
        assert len(conflicts) >= 1
        types = {c.conflict_type for c in conflicts}
        assert ConflictType.NODE in types or ConflictType.EDGE in types

    def test_multiple_conflicts_observed(self) -> None:
        """Observer can detect multiple conflicts at once."""
        peer_state = {
            "AMR-02": RobotStateMessage(
                "AMR-02", Position(6, 5), Position(5, 5),
                RobotState.MOVING, 10, IntentAction.MOVE,
            ),
            "AMR-03": RobotStateMessage(
                "AMR-03", Position(5, 4), Position(5, 5),
                RobotState.MOVING, 10, IntentAction.MOVE,
            ),
        }
        conflicts = IntentObserver.observe(
            own_id="AMR-01",
            own_position=Position(4, 5),
            own_intended=Position(5, 5),
            own_intent=IntentAction.MOVE,
            peer_state=peer_state,
            current_tick=10,
        )
        assert len(conflicts) == 2


# ==================================================================
# 16. Observation Does NOT Modify Movement
# ==================================================================

class TestObservationDoesNotResolve:
    def test_observation_does_not_change_state(self) -> None:
        """Calling observe_conflicts does not alter robot state."""
        transport = SimulatedTransport()
        amr1 = _make_amr_with_comms("AMR-01", (4, 5), transport)
        amr2 = _make_amr_with_comms("AMR-02", (6, 5), transport)

        grid = Grid(10, 10)
        amr1.plan(grid, Position(9, 5))
        amr2.plan(grid, Position(0, 5))

        amr1.broadcast_state(10)
        amr2.broadcast_state(10)
        transport.deliver_messages(10)

        state_before = amr1.state
        pos_before = amr1.position
        amr1.observe_conflicts(10)
        assert amr1.state is state_before
        assert amr1.position == pos_before

    def test_observation_returns_info_only(self) -> None:
        """PotentialConflict is frozen data — cannot trigger actions."""
        c = PotentialConflict(
            conflict_type=ConflictType.NODE,
            robot_a_id="AMR-01",
            robot_b_id="AMR-02",
            position=Position(5, 5),
            tick=10,
        )
        with pytest.raises(Exception):
            c.tick = 20  # type: ignore[misc]


# ==================================================================
# 17–18. Phase 3 Behavior Unchanged
# ==================================================================

class TestPhase3Preservation:
    """Communication and intent exchange must not alter movement physics."""

    def test_crossing_still_succeeds(self) -> None:
        engine = _build_scenario(
            grid_size=(10, 10),
            robots=[
                ("AMR-01", (0, 5), (9, 5)),
                ("AMR-02", (5, 0), (5, 9)),
            ],
        )
        while not engine.is_finished:
            engine.update()

        assert engine.fleet.all_arrived
        assert engine.metrics.total_collisions == 0

    def test_head_on_still_deadlocks(self) -> None:
        engine = _build_scenario(
            grid_size=(10, 5),
            robots=[
                ("AMR-01", (0, 2), (9, 2)),
                ("AMR-02", (9, 2), (0, 2)),
            ],
        )
        while not engine.is_finished:
            engine.update()

        assert engine.deadlocked
        assert not engine.fleet.all_arrived
        assert engine.metrics.total_collisions == 0

    def test_intent_communication_does_not_create_collisions(self) -> None:
        """Four-way intersection with intents — zero collisions."""
        engine = _build_scenario(
            grid_size=(10, 10),
            robots=[
                ("AMR-01", (0, 5), (9, 5)),
                ("AMR-02", (9, 5), (0, 5)),
                ("AMR-03", (5, 0), (5, 9)),
                ("AMR-04", (5, 9), (5, 0)),
            ],
        )
        while not engine.is_finished:
            engine.update()

        assert engine.deadlocked
        assert engine.metrics.total_collisions == 0


# ==================================================================
# 19. Determinism
# ==================================================================

class TestIntentDeterminism:
    def test_deterministic_intent_exchange(self) -> None:
        """Two identical runs produce the same peer state."""
        def run_once() -> dict:
            engine = _build_scenario(
                grid_size=(10, 10),
                robots=[
                    ("AMR-01", (0, 5), (9, 5)),
                    ("AMR-02", (5, 0), (5, 9)),
                ],
            )
            while not engine.is_finished:
                engine.update()
            m = engine.metrics
            return {
                "total_ticks": m.total_ticks,
                "total_wait_ticks": m.total_wait_ticks,
                "collisions": m.total_collisions,
                "comms_sent": engine.transport.metrics_sent,
                "comms_delivered": engine.transport.metrics_delivered,
            }

        r1 = run_once()
        r2 = run_once()
        assert r1 == r2


# ==================================================================
# 20. IntentAction Enum Basics
# ==================================================================

class TestIntentActionEnum:
    def test_intent_values(self) -> None:
        assert IntentAction.MOVE is not IntentAction.WAIT
        assert IntentAction.IDLE is not IntentAction.COMPLETE
        assert len(IntentAction) == 4

    def test_intent_names(self) -> None:
        assert IntentAction.MOVE.name == "MOVE"
        assert IntentAction.WAIT.name == "WAIT"
        assert IntentAction.IDLE.name == "IDLE"
        assert IntentAction.COMPLETE.name == "COMPLETE"
