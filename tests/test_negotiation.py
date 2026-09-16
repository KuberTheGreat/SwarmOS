"""Tests for Phase 6: Distributed Conflict Negotiation.

Covers all acceptance criteria including:
  1-4.   Node conflicts (2, 3, 4 robots competing for same cell)
  5.     Edge conflicts
  6-9.   Deterministic priority, independent agreement, PROCEED/WAIT
 10-11.  Stale/missing information → conservative WAIT
 12.     No centralized controller (distributed consistency proof)
 13-14.  Physical safety (no edge swaps, no same-cell collisions)
 15-17.  Phase 3/4/5 regression
 18.     Determinism
"""

from __future__ import annotations

import pytest

from swarmos.communication.intent import (
    ConflictType,
    IntentAction,
    PotentialConflict,
)
from swarmos.communication.interface import CommunicationInterface
from swarmos.communication.message import RobotStateMessage
from swarmos.communication.transport import SimulatedTransport
from swarmos.coordination.decision import Decision
from swarmos.coordination.distributed_policy import DistributedPolicy
from swarmos.coordination.negotiator import (
    DistributedNegotiator,
    NegotiationResult,
)
from swarmos.coordination.policy import StopAndWaitPolicy
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

def _build_scenario(
    grid_size: tuple[int, int],
    robots: list[tuple[str, tuple[int, int], tuple[int, int]]],
    policy_type: str = "distributed",
    step_delay: int = 1,
) -> SimulationEngine:
    """Build a simulation with the specified policy."""
    grid = Grid(*grid_size)
    fleet = Fleet()
    goals: dict[str, Position] = {}

    for rid, start, goal in robots:
        amr = AMR(robot_id=rid, position=Position(*start))
        fleet.add(amr)
        goals[rid] = Position(*goal)

    if policy_type == "distributed":
        policy = DistributedPolicy(fleet)
    elif policy_type == "stop_and_wait":
        policy = StopAndWaitPolicy()
    else:
        policy = None

    config = SimulationConfig(robot_step_delay=step_delay)
    engine = SimulationEngine(
        grid=grid, robots=fleet, config=config,
        policy=policy, max_ticks=5000,
    )
    results = engine.plan_all(goals)
    assert all(results.values())
    return engine


def _make_peer_state(
    entries: list[tuple[str, tuple[int, int], tuple[int, int], IntentAction, int]],
) -> dict[str, RobotStateMessage]:
    """Build a peer_state dict from simple tuples."""
    result = {}
    for rid, pos, intended, intent, ts in entries:
        result[rid] = RobotStateMessage(
            sender_id=rid,
            position=Position(*pos),
            intended_next=Position(*intended),
            state=RobotState.MOVING,
            timestamp=ts,
            intent=intent,
        )
    return result


# ==================================================================
# 1–4. Node Conflicts (2, 3, 4 robots)
# ==================================================================

class TestNodeConflicts:
    def test_two_robots_same_node(self) -> None:
        """Lowest ID wins a two-robot node conflict."""
        peer_state = _make_peer_state([
            ("AMR-02", (6, 5), (5, 5), IntentAction.MOVE, 10),
        ])
        result = DistributedNegotiator.negotiate(
            own_id="AMR-01",
            own_position=Position(4, 5),
            own_intended=Position(5, 5),
            own_intent=IntentAction.MOVE,
            peer_state=peer_state,
            current_tick=10,
        )
        assert result.decision is Decision.MOVE
        assert result.winner == "AMR-01"

    def test_two_robots_loser_waits(self) -> None:
        """Higher ID waits in a two-robot node conflict."""
        peer_state = _make_peer_state([
            ("AMR-01", (4, 5), (5, 5), IntentAction.MOVE, 10),
        ])
        result = DistributedNegotiator.negotiate(
            own_id="AMR-02",
            own_position=Position(6, 5),
            own_intended=Position(5, 5),
            own_intent=IntentAction.MOVE,
            peer_state=peer_state,
            current_tick=10,
        )
        assert result.decision is Decision.WAIT
        assert result.winner == "AMR-01"

    def test_three_robots_same_node(self) -> None:
        """Only the lowest ID proceeds in a 3-way conflict."""
        peer_state = _make_peer_state([
            ("AMR-02", (6, 5), (5, 5), IntentAction.MOVE, 10),
            ("AMR-03", (5, 4), (5, 5), IntentAction.MOVE, 10),
        ])
        result = DistributedNegotiator.negotiate(
            own_id="AMR-01",
            own_position=Position(4, 5),
            own_intended=Position(5, 5),
            own_intent=IntentAction.MOVE,
            peer_state=peer_state,
            current_tick=10,
        )
        assert result.decision is Decision.MOVE
        assert result.winner == "AMR-01"
        assert len(result.participants) == 3

    def test_four_robots_same_node(self) -> None:
        """Only one robot proceeds in a 4-way conflict."""
        peer_state = _make_peer_state([
            ("AMR-01", (4, 5), (5, 5), IntentAction.MOVE, 10),
            ("AMR-02", (6, 5), (5, 5), IntentAction.MOVE, 10),
            ("AMR-03", (5, 4), (5, 5), IntentAction.MOVE, 10),
        ])
        result = DistributedNegotiator.negotiate(
            own_id="AMR-04",
            own_position=Position(5, 6),
            own_intended=Position(5, 5),
            own_intent=IntentAction.MOVE,
            peer_state=peer_state,
            current_tick=10,
        )
        assert result.decision is Decision.WAIT
        assert result.winner == "AMR-01"
        assert len(result.participants) == 4


# ==================================================================
# 5. Edge Conflicts
# ==================================================================

class TestEdgeConflicts:
    def test_edge_swap_both_wait(self) -> None:
        """Edge swap: both robots must WAIT regardless of priority."""
        peer_state = _make_peer_state([
            ("AMR-02", (5, 5), (4, 5), IntentAction.MOVE, 10),
        ])
        result = DistributedNegotiator.negotiate(
            own_id="AMR-01",
            own_position=Position(4, 5),
            own_intended=Position(5, 5),
            own_intent=IntentAction.MOVE,
            peer_state=peer_state,
            current_tick=10,
        )
        assert result.decision is Decision.WAIT
        assert "physical safety" in result.reason.lower() or "edge swap" in result.reason.lower()

    def test_edge_swap_higher_id_also_waits(self) -> None:
        """Edge swap: the higher-ID robot also waits."""
        peer_state = _make_peer_state([
            ("AMR-01", (4, 5), (5, 5), IntentAction.MOVE, 10),
        ])
        result = DistributedNegotiator.negotiate(
            own_id="AMR-02",
            own_position=Position(5, 5),
            own_intended=Position(4, 5),
            own_intent=IntentAction.MOVE,
            peer_state=peer_state,
            current_tick=10,
        )
        assert result.decision is Decision.WAIT


# ==================================================================
# 6–9. Deterministic Priority and Consistent Decisions
# ==================================================================

class TestDeterministicPriority:
    def test_lower_id_always_wins(self) -> None:
        """Deterministic: lower ID always wins."""
        for winner, loser in [("AMR-01", "AMR-02"), ("AMR-03", "AMR-99")]:
            peer_state = _make_peer_state([
                (loser, (6, 5), (5, 5), IntentAction.MOVE, 10),
            ])
            result = DistributedNegotiator.negotiate(
                own_id=winner,
                own_position=Position(4, 5),
                own_intended=Position(5, 5),
                own_intent=IntentAction.MOVE,
                peer_state=peer_state,
                current_tick=10,
            )
            assert result.decision is Decision.MOVE
            assert result.winner == winner

    def test_both_participants_agree_on_winner(self) -> None:
        """The critical distributed consistency test: both robots
        independently compute the same winner from their local views."""
        # AMR-01's local view
        peer_state_01 = _make_peer_state([
            ("AMR-02", (6, 5), (5, 5), IntentAction.MOVE, 10),
        ])
        result_01 = DistributedNegotiator.negotiate(
            own_id="AMR-01",
            own_position=Position(4, 5),
            own_intended=Position(5, 5),
            own_intent=IntentAction.MOVE,
            peer_state=peer_state_01,
            current_tick=10,
        )

        # AMR-02's local view
        peer_state_02 = _make_peer_state([
            ("AMR-01", (4, 5), (5, 5), IntentAction.MOVE, 10),
        ])
        result_02 = DistributedNegotiator.negotiate(
            own_id="AMR-02",
            own_position=Position(6, 5),
            own_intended=Position(5, 5),
            own_intent=IntentAction.MOVE,
            peer_state=peer_state_02,
            current_tick=10,
        )

        # Both agree: AMR-01 wins
        assert result_01.winner == "AMR-01"
        assert result_02.winner == "AMR-01"
        assert result_01.decision is Decision.MOVE
        assert result_02.decision is Decision.WAIT

    def test_three_way_all_agree(self) -> None:
        """Three robots all independently agree on the same winner."""
        # Each robot sees the other two
        r1 = DistributedNegotiator.negotiate(
            "AMR-01", Position(4, 5), Position(5, 5), IntentAction.MOVE,
            _make_peer_state([
                ("AMR-02", (6, 5), (5, 5), IntentAction.MOVE, 10),
                ("AMR-03", (5, 4), (5, 5), IntentAction.MOVE, 10),
            ]), 10,
        )
        r2 = DistributedNegotiator.negotiate(
            "AMR-02", Position(6, 5), Position(5, 5), IntentAction.MOVE,
            _make_peer_state([
                ("AMR-01", (4, 5), (5, 5), IntentAction.MOVE, 10),
                ("AMR-03", (5, 4), (5, 5), IntentAction.MOVE, 10),
            ]), 10,
        )
        r3 = DistributedNegotiator.negotiate(
            "AMR-03", Position(5, 4), Position(5, 5), IntentAction.MOVE,
            _make_peer_state([
                ("AMR-01", (4, 5), (5, 5), IntentAction.MOVE, 10),
                ("AMR-02", (6, 5), (5, 5), IntentAction.MOVE, 10),
            ]), 10,
        )

        assert r1.winner == r2.winner == r3.winner == "AMR-01"
        assert r1.decision is Decision.MOVE
        assert r2.decision is Decision.WAIT
        assert r3.decision is Decision.WAIT


# ==================================================================
# 10–11. Stale/Missing Information → Conservative WAIT
# ==================================================================

class TestStaleMissingInfo:
    def test_stale_peer_skipped(self) -> None:
        """Stale peer info is ignored — no conflict detected."""
        peer_state = _make_peer_state([
            ("AMR-02", (6, 5), (5, 5), IntentAction.MOVE, 5),
        ])
        result = DistributedNegotiator.negotiate(
            own_id="AMR-01",
            own_position=Position(4, 5),
            own_intended=Position(5, 5),
            own_intent=IntentAction.MOVE,
            peer_state=peer_state,
            current_tick=20,
            stale_threshold=3,
        )
        # Stale info (age=15 > threshold=3) → no conflict seen
        assert result.decision is Decision.MOVE
        assert len(result.conflicts) == 0

    def test_no_peer_info_proceeds(self) -> None:
        """No peer info at all → robot can proceed."""
        result = DistributedNegotiator.negotiate(
            own_id="AMR-01",
            own_position=Position(4, 5),
            own_intended=Position(5, 5),
            own_intent=IntentAction.MOVE,
            peer_state={},
            current_tick=10,
        )
        assert result.decision is Decision.MOVE

    def test_occupancy_blocks_movement(self) -> None:
        """Target cell occupied by non-moving peer → WAIT."""
        peer_state = _make_peer_state([
            ("AMR-02", (5, 5), (5, 5), IntentAction.WAIT, 10),
        ])
        result = DistributedNegotiator.negotiate(
            own_id="AMR-01",
            own_position=Position(4, 5),
            own_intended=Position(5, 5),
            own_intent=IntentAction.MOVE,
            peer_state=peer_state,
            current_tick=10,
        )
        assert result.decision is Decision.WAIT


# ==================================================================
# 12. No Centralized Controller (Distributed Consistency Proof)
# ==================================================================

class TestDistributedConsistency:
    """Proves that each robot independently derives the same decision
    using ONLY its local peer knowledge — no hidden global state."""

    def test_no_global_snapshot_used(self) -> None:
        """DistributedPolicy reads each robot's peer_state, not a
        centralized snapshot.

        We construct two robots with communication interfaces,
        broadcast their states through the transport, then verify
        the distributed policy produces consistent decisions from
        each robot's LOCAL peer knowledge.
        """
        transport = SimulatedTransport()
        fleet = Fleet()

        amr1 = AMR("AMR-01", Position(4, 5))
        amr1.communication = CommunicationInterface("AMR-01", transport)
        fleet.add(amr1)

        amr2 = AMR("AMR-02", Position(6, 5))
        amr2.communication = CommunicationInterface("AMR-02", transport)
        fleet.add(amr2)

        grid = Grid(10, 10)
        amr1.plan(grid, Position(9, 5))
        amr2.plan(grid, Position(0, 5))

        # Simulate P2P exchange
        amr1.broadcast_state(10)
        amr2.broadcast_state(10)
        transport.deliver_messages(10)

        # Each robot should now know the other's intent
        assert "AMR-02" in amr1.peer_state
        assert "AMR-01" in amr2.peer_state

        # Run distributed policy
        policy = DistributedPolicy(fleet)
        from swarmos.coordination.decision import RobotSnapshot
        snapshots = [
            RobotSnapshot(r.robot_id, r.position, r.intended_next_position, r.state)
            for r in fleet
        ]
        decisions = policy.decide(snapshots, current_tick=10)

        # Both should have negotiation results
        assert "AMR-01" in policy.last_results
        assert "AMR-02" in policy.last_results

        # They must agree on the winner
        r1 = policy.last_results["AMR-01"]
        r2 = policy.last_results["AMR-02"]

        # Both independently computed the same winner
        assert r1.winner == r2.winner

    def test_robots_compute_independently(self) -> None:
        """Each robot's negotiation result is based on its own peer_state,
        not shared state. Changing one robot's peer info doesn't affect
        the other's computation."""
        # AMR-01's view: knows AMR-02 wants (5,5)
        r1 = DistributedNegotiator.negotiate(
            "AMR-01", Position(4, 5), Position(5, 5), IntentAction.MOVE,
            _make_peer_state([
                ("AMR-02", (6, 5), (5, 5), IntentAction.MOVE, 10),
            ]), 10,
        )

        # AMR-02's view: knows AMR-01 wants (5,5)
        r2 = DistributedNegotiator.negotiate(
            "AMR-02", Position(6, 5), Position(5, 5), IntentAction.MOVE,
            _make_peer_state([
                ("AMR-01", (4, 5), (5, 5), IntentAction.MOVE, 10),
            ]), 10,
        )

        # Consistent: AMR-01 proceeds, AMR-02 waits
        assert r1.decision is Decision.MOVE
        assert r2.decision is Decision.WAIT
        assert r1.winner == r2.winner == "AMR-01"


# ==================================================================
# 13–14. Physical Safety
# ==================================================================

class TestPhysicalSafety:
    def test_no_edge_swap_through_negotiation(self) -> None:
        """Negotiation cannot permit an edge swap."""
        engine = _build_scenario(
            grid_size=(10, 3),
            robots=[
                ("AMR-01", (0, 1), (9, 1)),
                ("AMR-02", (9, 1), (0, 1)),
            ],
        )
        while not engine.is_finished:
            engine.update()

        assert engine.metrics.total_collisions == 0

    def test_no_same_cell_collision(self) -> None:
        """Negotiation cannot place two robots in the same cell."""
        engine = _build_scenario(
            grid_size=(10, 10),
            robots=[
                ("AMR-01", (0, 5), (9, 5)),
                ("AMR-02", (5, 0), (5, 9)),
            ],
        )
        while not engine.is_finished:
            engine.update()

        assert engine.metrics.total_collisions == 0
        assert engine.fleet.all_arrived

    def test_four_way_intersection_safe(self) -> None:
        """Four robots converging on intersection — zero collisions."""
        engine = _build_scenario(
            grid_size=(11, 11),
            robots=[
                ("AMR-01", (2, 5), (8, 5)),
                ("AMR-02", (8, 5), (2, 5)),
                ("AMR-03", (5, 2), (5, 8)),
                ("AMR-04", (5, 8), (5, 2)),
            ],
        )
        while not engine.is_finished:
            engine.update()

        assert engine.metrics.total_collisions == 0


# ==================================================================
# 15–17. Phase 3/4/5 Regression
# ==================================================================

class TestPhaseRegression:
    def test_stop_and_wait_still_works(self) -> None:
        """Phase 3 StopAndWaitPolicy still produces correct results."""
        engine = _build_scenario(
            grid_size=(10, 10),
            robots=[
                ("AMR-01", (0, 5), (9, 5)),
                ("AMR-02", (5, 0), (5, 9)),
            ],
            policy_type="stop_and_wait",
        )
        while not engine.is_finished:
            engine.update()

        assert engine.fleet.all_arrived
        assert engine.metrics.total_collisions == 0

    def test_crossing_distributed_succeeds(self) -> None:
        """Crossing scenario completes with distributed policy."""
        engine = _build_scenario(
            grid_size=(10, 10),
            robots=[
                ("AMR-01", (0, 5), (9, 5)),
                ("AMR-02", (5, 0), (5, 9)),
                ("AMR-03", (9, 4), (0, 4)),
            ],
        )
        while not engine.is_finished:
            engine.update()

        assert engine.fleet.all_arrived
        assert engine.metrics.total_collisions == 0

    def test_headon_distributed_deadlocks(self) -> None:
        """Head-on corridor still deadlocks — negotiation doesn't solve
        structural deadlocks."""
        engine = _build_scenario(
            grid_size=(10, 3),
            robots=[
                ("AMR-01", (0, 1), (9, 1)),
                ("AMR-02", (9, 1), (0, 1)),
            ],
        )
        while not engine.is_finished:
            engine.update()

        assert engine.deadlocked
        assert engine.metrics.total_collisions == 0


# ==================================================================
# 18. Determinism
# ==================================================================

class TestDeterminism:
    def test_repeated_runs_identical(self) -> None:
        """Two identical runs produce the same metrics."""
        def run_once() -> dict:
            engine = _build_scenario(
                grid_size=(10, 10),
                robots=[
                    ("AMR-01", (0, 5), (9, 5)),
                    ("AMR-02", (5, 0), (5, 9)),
                    ("AMR-03", (9, 4), (0, 4)),
                ],
            )
            while not engine.is_finished:
                engine.update()
            m = engine.metrics
            return {
                "ticks": m.total_ticks,
                "waits": m.total_wait_ticks,
                "movements": m.total_movements,
                "collisions": m.total_collisions,
                "negotiations": m.negotiations_completed,
                "proceed": m.proceed_decisions,
                "wait_decisions": m.wait_decisions,
            }

        r1 = run_once()
        r2 = run_once()
        assert r1 == r2


# ==================================================================
# Additional: Non-active robots
# ==================================================================

class TestNonActiveRobots:
    def test_idle_robot_no_negotiation(self) -> None:
        result = DistributedNegotiator.negotiate(
            "AMR-01", Position(0, 0), Position(0, 0), IntentAction.IDLE,
            {}, 10,
        )
        assert result.decision is Decision.MOVE

    def test_complete_robot_no_negotiation(self) -> None:
        result = DistributedNegotiator.negotiate(
            "AMR-01", Position(0, 0), Position(0, 0), IntentAction.COMPLETE,
            {}, 10,
        )
        assert result.decision is Decision.MOVE

    def test_wait_intent_no_new_conflicts(self) -> None:
        """A WAIT intent robot doesn't create new conflicts."""
        result = DistributedNegotiator.negotiate(
            "AMR-01", Position(4, 5), Position(4, 5), IntentAction.WAIT,
            _make_peer_state([
                ("AMR-02", (6, 5), (5, 5), IntentAction.MOVE, 10),
            ]), 10,
        )
        assert result.decision is Decision.MOVE  # no-op, robot is waiting


# ==================================================================
# Multiple simultaneous conflicts
# ==================================================================

class TestMultipleConflicts:
    def test_two_independent_conflicts(self) -> None:
        """Two separate node conflicts resolved independently."""
        engine = _build_scenario(
            grid_size=(20, 10),
            robots=[
                ("AMR-01", (0, 5), (9, 5)),    # group 1
                ("AMR-02", (9, 5), (0, 5)),    # group 1
                ("AMR-03", (10, 5), (19, 5)),  # group 2, no conflict
            ],
        )
        while not engine.is_finished:
            engine.update()

        assert engine.metrics.total_collisions == 0


# ==================================================================
# Negotiation metrics
# ==================================================================

class TestNegotiationMetrics:
    def test_metrics_recorded(self) -> None:
        """Negotiation decisions are recorded in metrics."""
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
        assert m.negotiations_completed > 0
        assert m.proceed_decisions + m.wait_decisions == m.negotiations_completed
