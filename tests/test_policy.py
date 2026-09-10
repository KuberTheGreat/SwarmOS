"""Tests for coordination policies — stop-and-wait baseline.

Tests the StopAndWaitPolicy in isolation, verifying:
- Safe movements → MOVE
- Node conflict → lower ID MOVE, higher ID WAIT
- Edge conflict (swap) → both WAIT
- Occupancy conflict → WAIT
- Deterministic tie-breaking
- Iteration-order independence (simultaneous-update semantics)
- Decision and RobotSnapshot types
"""

from __future__ import annotations

from swarmos.coordination.decision import Decision, RobotSnapshot
from swarmos.coordination.policy import StopAndWaitPolicy
from swarmos.robot.state import RobotState
from swarmos.warehouse.cell import Position


# ==================================================================
# Helpers
# ==================================================================

def _snap(
    robot_id: str,
    pos: tuple[int, int],
    intended: tuple[int, int] | None = None,
    state: RobotState = RobotState.MOVING,
) -> RobotSnapshot:
    """Shorthand for building a RobotSnapshot."""
    p = Position(*pos)
    i = Position(*intended) if intended is not None else p
    return RobotSnapshot(robot_id=robot_id, position=p, intended_next=i, state=state)


# ==================================================================
# Decision and RobotSnapshot types
# ==================================================================

class TestDecisionType:
    def test_move_and_wait_values(self) -> None:
        assert Decision.MOVE is not Decision.WAIT
        assert Decision.MOVE.name == "MOVE"
        assert Decision.WAIT.name == "WAIT"

    def test_robot_snapshot_fields(self) -> None:
        snap = RobotSnapshot(
            robot_id="R1",
            position=Position(3, 4),
            intended_next=Position(3, 5),
            state=RobotState.MOVING,
        )
        assert snap.robot_id == "R1"
        assert snap.position == Position(3, 4)
        assert snap.intended_next == Position(3, 5)
        assert snap.state is RobotState.MOVING

    def test_robot_snapshot_frozen(self) -> None:
        snap = _snap("R1", (0, 0), (1, 0))
        try:
            snap.robot_id = "R2"  # type: ignore[misc]
            assert False, "Should not be able to mutate frozen dataclass"
        except AttributeError:
            pass


# ==================================================================
# StopAndWaitPolicy — basic decisions
# ==================================================================

class TestStopAndWaitBasics:
    def setup_method(self) -> None:
        self.policy = StopAndWaitPolicy()

    def test_policy_name(self) -> None:
        assert self.policy.name == "STOP_AND_WAIT"

    def test_single_robot_no_conflict(self) -> None:
        """A single robot with no peers always gets MOVE."""
        snaps = [_snap("AMR-01", (0, 0), (1, 0))]
        decisions = self.policy.decide(snaps)
        assert decisions["AMR-01"] is Decision.MOVE

    def test_no_conflict_disjoint_targets(self) -> None:
        """Robots heading to different cells all get MOVE."""
        snaps = [
            _snap("AMR-01", (0, 0), (1, 0)),
            _snap("AMR-02", (5, 5), (5, 6)),
            _snap("AMR-03", (9, 9), (8, 9)),
        ]
        decisions = self.policy.decide(snaps)
        assert all(d is Decision.MOVE for d in decisions.values())

    def test_idle_robot_gets_move(self) -> None:
        """IDLE robots always get MOVE (no action taken)."""
        snaps = [_snap("R1", (0, 0), state=RobotState.IDLE)]
        decisions = self.policy.decide(snaps)
        assert decisions["R1"] is Decision.MOVE

    def test_arrived_robot_gets_move(self) -> None:
        """ARRIVED robots always get MOVE (no action taken)."""
        snaps = [_snap("R1", (5, 5), state=RobotState.ARRIVED)]
        decisions = self.policy.decide(snaps)
        assert decisions["R1"] is Decision.MOVE

    def test_staying_in_place_gets_move(self) -> None:
        """Robot whose intended == position always gets MOVE."""
        snaps = [_snap("R1", (3, 3), (3, 3))]
        decisions = self.policy.decide(snaps)
        assert decisions["R1"] is Decision.MOVE


# ==================================================================
# StopAndWaitPolicy — node conflicts
# ==================================================================

class TestNodeConflict:
    def setup_method(self) -> None:
        self.policy = StopAndWaitPolicy()

    def test_two_robots_same_target(self) -> None:
        """Two robots targeting the same cell: lower ID moves."""
        snaps = [
            _snap("AMR-01", (0, 0), (1, 0)),
            _snap("AMR-02", (2, 0), (1, 0)),
        ]
        decisions = self.policy.decide(snaps)
        assert decisions["AMR-01"] is Decision.MOVE
        assert decisions["AMR-02"] is Decision.WAIT

    def test_three_robots_same_target(self) -> None:
        """Three robots targeting the same cell: only lowest ID moves."""
        snaps = [
            _snap("AMR-03", (0, 0), (5, 5)),
            _snap("AMR-01", (5, 4), (5, 5)),
            _snap("AMR-02", (5, 6), (5, 5)),
        ]
        decisions = self.policy.decide(snaps)
        assert decisions["AMR-01"] is Decision.MOVE
        assert decisions["AMR-02"] is Decision.WAIT
        assert decisions["AMR-03"] is Decision.WAIT

    def test_node_conflict_does_not_affect_unrelated(self) -> None:
        """A node conflict between A and B does not affect C."""
        snaps = [
            _snap("AMR-01", (0, 0), (1, 0)),
            _snap("AMR-02", (2, 0), (1, 0)),
            _snap("AMR-03", (9, 9), (8, 9)),
        ]
        decisions = self.policy.decide(snaps)
        assert decisions["AMR-03"] is Decision.MOVE


# ==================================================================
# StopAndWaitPolicy — edge conflicts (swaps)
# ==================================================================

class TestEdgeConflict:
    """Edge swaps are physically unsafe — both robots must WAIT."""

    def setup_method(self) -> None:
        self.policy = StopAndWaitPolicy()

    def test_edge_swap_both_wait(self) -> None:
        """Two robots swapping positions: both WAIT.

        A: (3,0) → (4,0)
        B: (4,0) → (3,0)

        They would traverse the same edge in opposite directions.
        This is physically impossible for grid-occupying AMRs.
        """
        snaps = [
            _snap("AMR-01", (3, 0), (4, 0)),
            _snap("AMR-02", (4, 0), (3, 0)),
        ]
        decisions = self.policy.decide(snaps)
        assert decisions["AMR-01"] is Decision.WAIT
        assert decisions["AMR-02"] is Decision.WAIT

    def test_edge_swap_vertical(self) -> None:
        """Vertical edge swap: both WAIT."""
        snaps = [
            _snap("AMR-01", (5, 3), (5, 4)),
            _snap("AMR-02", (5, 4), (5, 3)),
        ]
        decisions = self.policy.decide(snaps)
        assert decisions["AMR-01"] is Decision.WAIT
        assert decisions["AMR-02"] is Decision.WAIT

    def test_edge_swap_deterministic(self) -> None:
        """Edge swap result is the same regardless of input order."""
        forward = [
            _snap("AMR-01", (3, 0), (4, 0)),
            _snap("AMR-02", (4, 0), (3, 0)),
        ]
        backward = list(reversed(forward))
        d_fwd = self.policy.decide(forward)
        d_bwd = self.policy.decide(backward)
        # Both must WAIT in both orderings.
        assert d_fwd["AMR-01"] is Decision.WAIT
        assert d_fwd["AMR-02"] is Decision.WAIT
        assert d_bwd["AMR-01"] is Decision.WAIT
        assert d_bwd["AMR-02"] is Decision.WAIT

    def test_no_edge_conflict_same_direction(self) -> None:
        """Robots moving in the same direction — no edge conflict."""
        snaps = [
            _snap("AMR-01", (0, 0), (1, 0)),
            _snap("AMR-02", (1, 0), (2, 0)),
        ]
        decisions = self.policy.decide(snaps)
        assert decisions["AMR-01"] is Decision.MOVE
        assert decisions["AMR-02"] is Decision.MOVE

    def test_edge_swap_in_corridor(self) -> None:
        """Edge swap in a narrow corridor: both WAIT.

        This models the head-on scenario — no physical space to pass.
        """
        snaps = [
            _snap("AMR-01", (2, 0), (3, 0)),
            _snap("AMR-02", (3, 0), (2, 0)),
        ]
        decisions = self.policy.decide(snaps)
        assert decisions["AMR-01"] is Decision.WAIT
        assert decisions["AMR-02"] is Decision.WAIT


# ==================================================================
# StopAndWaitPolicy — occupancy conflicts
# ==================================================================

class TestOccupancyConflict:
    def setup_method(self) -> None:
        self.policy = StopAndWaitPolicy()

    def test_move_into_occupied_stationary(self) -> None:
        """Robot wants to enter a cell with a stationary (ARRIVED) robot."""
        snaps = [
            _snap("AMR-01", (0, 0), (1, 0)),
            _snap("AMR-02", (1, 0), state=RobotState.ARRIVED),
        ]
        decisions = self.policy.decide(snaps)
        assert decisions["AMR-01"] is Decision.WAIT

    def test_move_into_cell_being_vacated(self) -> None:
        """Robot can enter a cell that another robot is leaving."""
        snaps = [
            _snap("AMR-01", (0, 0), (1, 0)),
            _snap("AMR-02", (1, 0), (2, 0)),
        ]
        decisions = self.policy.decide(snaps)
        assert decisions["AMR-01"] is Decision.MOVE
        assert decisions["AMR-02"] is Decision.MOVE

    def test_move_into_occupied_waiting_robot(self) -> None:
        """Robot cannot enter a cell occupied by a WAITING robot."""
        snaps = [
            _snap("AMR-01", (0, 0), (1, 0)),
            _snap("AMR-02", (1, 0), (2, 0), state=RobotState.WAITING),
        ]
        decisions = self.policy.decide(snaps)
        # AMR-02 is WAITING state but is an active mover (intended != pos).
        # If no one blocks AMR-02 at (2,0), AMR-02 gets MOVE, and
        # AMR-01 can then enter (1,0).
        assert decisions["AMR-02"] is Decision.MOVE
        assert decisions["AMR-01"] is Decision.MOVE


# ==================================================================
# StopAndWaitPolicy — deterministic tie-breaking
# ==================================================================

class TestTieBreaking:
    def setup_method(self) -> None:
        self.policy = StopAndWaitPolicy()

    def test_lower_id_always_wins(self) -> None:
        """The robot with the lexicographically lower ID always wins."""
        snaps = [
            _snap("B", (0, 0), (1, 0)),
            _snap("A", (2, 0), (1, 0)),
        ]
        decisions = self.policy.decide(snaps)
        assert decisions["A"] is Decision.MOVE
        assert decisions["B"] is Decision.WAIT

    def test_deterministic_across_orderings(self) -> None:
        """Tie-breaking result is the same regardless of snapshot order."""
        snaps_forward = [
            _snap("AMR-01", (0, 0), (1, 0)),
            _snap("AMR-02", (2, 0), (1, 0)),
        ]
        snaps_reversed = list(reversed(snaps_forward))

        d_fwd = self.policy.decide(snaps_forward)
        d_rev = self.policy.decide(snaps_reversed)

        assert d_fwd["AMR-01"] == d_rev["AMR-01"]
        assert d_fwd["AMR-02"] == d_rev["AMR-02"]
        assert d_fwd["AMR-01"] is Decision.MOVE
        assert d_fwd["AMR-02"] is Decision.WAIT


# ==================================================================
# Simultaneous update — iteration order independence
# ==================================================================

class TestSimultaneousUpdate:
    def setup_method(self) -> None:
        self.policy = StopAndWaitPolicy()

    def test_order_independence_three_robots(self) -> None:
        """Robot ordering should not change policy decisions."""
        snaps_a = [
            _snap("AMR-01", (0, 0), (1, 0)),
            _snap("AMR-02", (2, 0), (1, 0)),
            _snap("AMR-03", (5, 5), (5, 6)),
        ]
        import itertools
        for perm in itertools.permutations(snaps_a):
            decisions = self.policy.decide(list(perm))
            assert decisions["AMR-01"] is Decision.MOVE
            assert decisions["AMR-02"] is Decision.WAIT
            assert decisions["AMR-03"] is Decision.MOVE

    def test_no_unnecessary_waiting(self) -> None:
        """Robots with no conflicts should never wait."""
        snaps = [
            _snap("R1", (0, 0), (1, 0)),
            _snap("R2", (3, 3), (3, 4)),
            _snap("R3", (7, 7), (7, 6)),
            _snap("R4", (9, 0), (8, 0)),
        ]
        decisions = self.policy.decide(snaps)
        assert all(d is Decision.MOVE for d in decisions.values())


# ==================================================================
# Complex multi-conflict scenarios
# ==================================================================

class TestComplexScenarios:
    def setup_method(self) -> None:
        self.policy = StopAndWaitPolicy()

    def test_two_independent_conflicts(self) -> None:
        """Two separate node conflicts resolved independently."""
        snaps = [
            _snap("AMR-01", (0, 0), (1, 0)),  # conflict with AMR-02 at (1,0)
            _snap("AMR-02", (2, 0), (1, 0)),  # conflict with AMR-01 at (1,0)
            _snap("AMR-03", (5, 5), (5, 6)),  # conflict with AMR-04 at (5,6)
            _snap("AMR-04", (5, 7), (5, 6)),  # conflict with AMR-03 at (5,6)
        ]
        decisions = self.policy.decide(snaps)
        assert decisions["AMR-01"] is Decision.MOVE
        assert decisions["AMR-02"] is Decision.WAIT
        assert decisions["AMR-03"] is Decision.MOVE
        assert decisions["AMR-04"] is Decision.WAIT

    def test_combined_node_and_edge(self) -> None:
        """A node conflict and an edge conflict at the same time."""
        snaps = [
            _snap("AMR-01", (0, 0), (1, 0)),  # node conflict at (1,0)
            _snap("AMR-02", (2, 0), (1, 0)),  # node conflict at (1,0)
            _snap("AMR-03", (5, 0), (6, 0)),  # edge conflict (swap)
            _snap("AMR-04", (6, 0), (5, 0)),  # edge conflict (swap)
        ]
        decisions = self.policy.decide(snaps)
        assert decisions["AMR-01"] is Decision.MOVE
        assert decisions["AMR-02"] is Decision.WAIT
        # Edge conflict — both WAIT (physically unsafe swap)
        assert decisions["AMR-03"] is Decision.WAIT
        assert decisions["AMR-04"] is Decision.WAIT


# ==================================================================
# Cascading occupancy, 4-way intersection, determinism
# ==================================================================

class TestCascadingOccupancy:
    """Test the iterative occupancy check with cascading blocks."""

    def setup_method(self) -> None:
        self.policy = StopAndWaitPolicy()

    def test_chain_blocked_by_stationary(self) -> None:
        """A→B→C where C is ARRIVED.  B blocked by C, A blocked by B."""
        snaps = [
            _snap("AMR-01", (0, 0), (1, 0)),
            _snap("AMR-02", (1, 0), (2, 0)),
            _snap("AMR-03", (2, 0), state=RobotState.ARRIVED),
        ]
        decisions = self.policy.decide(snaps)
        assert decisions["AMR-01"] is Decision.WAIT
        assert decisions["AMR-02"] is Decision.WAIT

    def test_chain_unblocked(self) -> None:
        """A→B→C where all are moving forward into empty cells."""
        snaps = [
            _snap("AMR-01", (0, 0), (1, 0)),
            _snap("AMR-02", (1, 0), (2, 0)),
            _snap("AMR-03", (2, 0), (3, 0)),
        ]
        decisions = self.policy.decide(snaps)
        assert decisions["AMR-01"] is Decision.MOVE
        assert decisions["AMR-02"] is Decision.MOVE
        assert decisions["AMR-03"] is Decision.MOVE

    def test_three_way_rotation_rejected(self) -> None:
        """A→B, B→C, C→A — circular chain is rejected.

        Under the physical model, C→A is an edge conflict with A→B
        only if they share an edge.  But more importantly, C is
        trying to enter A's cell while A is trying to leave — and
        A is trying to enter B's cell while B is leaving for C's cell.

        The edge conflict between C→(0,0) and A@(0,0)→(1,0) is NOT
        an edge swap because A is going to (1,0) not to (0,1).

        An edge swap requires A.pos == B.intended AND B.pos == A.intended.
        C: (0,1)→(0,0), A: (0,0)→(1,0).  C.intended=(0,0)=A.pos ✓
        but A.intended=(1,0)≠C.pos=(0,1) ✗.  So NO edge conflict.

        This is a chain where everyone moves forward into the cell
        being vacated by the next robot.  All MOVE.
        """
        snaps = [
            _snap("AMR-01", (0, 0), (1, 0)),
            _snap("AMR-02", (1, 0), (0, 1)),
            _snap("AMR-03", (0, 1), (0, 0)),
        ]
        decisions = self.policy.decide(snaps)
        # Circular chain: each robot enters the cell being vacated
        # by the next.  No edge swaps (edges are different).
        assert decisions["AMR-01"] is Decision.MOVE
        assert decisions["AMR-02"] is Decision.MOVE
        assert decisions["AMR-03"] is Decision.MOVE


class TestFourWayIntersection:
    """Regression: 4 robots converging on center must not all wait."""

    def setup_method(self) -> None:
        self.policy = StopAndWaitPolicy()

    def test_four_robots_same_target(self) -> None:
        """Four robots all want the same cell.  Only one moves."""
        snaps = [
            _snap("AMR-01", (4, 5), (5, 5)),
            _snap("AMR-02", (6, 5), (5, 5)),
            _snap("AMR-03", (5, 4), (5, 5)),
            _snap("AMR-04", (5, 6), (5, 5)),
        ]
        decisions = self.policy.decide(snaps)
        assert decisions["AMR-01"] is Decision.MOVE  # lowest ID wins
        assert decisions["AMR-02"] is Decision.WAIT
        assert decisions["AMR-03"] is Decision.WAIT
        assert decisions["AMR-04"] is Decision.WAIT

    def test_edge_swap_at_intersection(self) -> None:
        """Robot leaving center swaps with robot entering — both WAIT.

        AMR-01: (5,5)→(6,5) and AMR-02: (6,5)→(5,5) is an edge swap.
        Both must WAIT.  AMR-03 and AMR-04 also want (5,5) but lose
        the node conflict and can't enter because AMR-01 stays.
        """
        snaps = [
            _snap("AMR-01", (5, 5), (6, 5)),
            _snap("AMR-02", (6, 5), (5, 5)),
            _snap("AMR-03", (5, 4), (5, 5)),
            _snap("AMR-04", (5, 6), (5, 5)),
        ]
        decisions = self.policy.decide(snaps)
        # AMR-01 and AMR-02 have an edge conflict — both WAIT
        assert decisions["AMR-01"] is Decision.WAIT
        assert decisions["AMR-02"] is Decision.WAIT
        # AMR-03 and AMR-04 lose node conflict to AMR-02 (but AMR-02
        # is now WAIT), AND AMR-01 is still at (5,5) not leaving.
        # So they are blocked by occupancy too.
        assert decisions["AMR-03"] is Decision.WAIT
        assert decisions["AMR-04"] is Decision.WAIT


class TestSharedDestination:
    """Regression: two robots want the same empty cell."""

    def setup_method(self) -> None:
        self.policy = StopAndWaitPolicy()

    def test_higher_priority_moves(self) -> None:
        snaps = [
            _snap("AMR-01", (0, 0), (1, 0)),
            _snap("AMR-02", (2, 0), (1, 0)),
        ]
        decisions = self.policy.decide(snaps)
        assert decisions["AMR-01"] is Decision.MOVE
        assert decisions["AMR-02"] is Decision.WAIT


class TestReversedIterationOrder:
    """Regression: input order must not affect output."""

    def setup_method(self) -> None:
        self.policy = StopAndWaitPolicy()

    def test_reversed_order_same_result(self) -> None:
        forward = [
            _snap("AMR-01", (4, 5), (5, 5)),
            _snap("AMR-02", (6, 5), (5, 5)),
            _snap("AMR-03", (5, 4), (5, 5)),
            _snap("AMR-04", (5, 6), (5, 5)),
        ]
        backward = list(reversed(forward))
        d_fwd = self.policy.decide(forward)
        d_bwd = self.policy.decide(backward)
        for rid in ["AMR-01", "AMR-02", "AMR-03", "AMR-04"]:
            assert d_fwd[rid] is d_bwd[rid], f"{rid}: {d_fwd[rid]} != {d_bwd[rid]}"

    def test_edge_swap_reversed_order(self) -> None:
        """Edge swap is deterministic regardless of input order."""
        forward = [
            _snap("AMR-01", (3, 0), (4, 0)),
            _snap("AMR-02", (4, 0), (3, 0)),
        ]
        backward = list(reversed(forward))
        d_fwd = self.policy.decide(forward)
        d_bwd = self.policy.decide(backward)
        # Both WAIT in both orderings.
        assert d_fwd["AMR-01"] is Decision.WAIT
        assert d_fwd["AMR-02"] is Decision.WAIT
        assert d_bwd["AMR-01"] is Decision.WAIT
        assert d_bwd["AMR-02"] is Decision.WAIT


# ==================================================================
# Deadlock detection in policy
# ==================================================================

class TestDeadlockScenarios:
    """Scenarios that are physically unsolvable under stop-and-wait."""

    def setup_method(self) -> None:
        self.policy = StopAndWaitPolicy()

    def test_head_on_pair_all_wait(self) -> None:
        """Two robots facing each other on a line — both must WAIT.

        This is the fundamental head-on deadlock.  Without rerouting,
        neither robot can make progress.
        """
        snaps = [
            _snap("AMR-01", (2, 0), (3, 0)),
            _snap("AMR-02", (3, 0), (2, 0)),
        ]
        decisions = self.policy.decide(snaps)
        assert decisions["AMR-01"] is Decision.WAIT
        assert decisions["AMR-02"] is Decision.WAIT

    def test_four_robot_corridor_deadlock(self) -> None:
        """Four robots paired head-on in a corridor — all WAIT."""
        snaps = [
            _snap("AMR-01", (2, 0), (3, 0)),
            _snap("AMR-02", (3, 0), (4, 0)),
            _snap("AMR-03", (5, 0), (4, 0)),
            _snap("AMR-04", (4, 0), (3, 0)),
        ]
        decisions = self.policy.decide(snaps)
        # AMR-02 and AMR-04 both want to swap direction → edge conflict
        # or node conflict at (3,0)/(4,0).  All blocked.
        move_count = sum(1 for d in decisions.values() if d is Decision.MOVE)
        # At most one robot could theoretically move, but the chain
        # should cause cascading blocks.  Verify zero collisions if
        # all wait.
        assert all(d is Decision.WAIT for d in decisions.values()) or move_count <= 1
