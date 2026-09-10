"""Coordination policies — pluggable movement-decision strategies.

A CoordinationPolicy examines a *snapshot* of every robot's intended
movement and returns a per-robot Decision (MOVE or WAIT).

IMPORTANT — This is NOT a centralized controller:
    The policy is a *strategy* that each robot conceptually applies to
    the shared snapshot.  The simulation engine provides the plumbing
    for snapshot-based simultaneous evaluation, but the decision logic
    is local: each robot's outcome depends only on its own state and
    the observable positions of its peers.

Phase 3 provides a single, deliberately naive policy:

    StopAndWaitPolicy
        If a robot's intended next position would create a conflict,
        the robot with the higher ID waits.  No negotiation, no
        rerouting, no deadlock resolution.

This baseline exists solely to establish a measurable comparison
point for future decentralized coordination algorithms.

Future policies will implement the same interface:

    CoordinationPolicy
        ├── StopAndWaitPolicy       (Phase 3)
        └── DecentralizedPolicy     (future)
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from swarmos.coordination.decision import Decision, RobotSnapshot
from swarmos.robot.state import RobotState


class CoordinationPolicy(ABC):
    """Abstract base class for robot coordination policies.

    Subclasses implement :meth:`decide`, which receives a list of
    :class:`RobotSnapshot` objects (one per robot) and returns a
    mapping of ``robot_id → Decision``.

    The snapshot list represents the state of *all* robots at the
    same instant, ensuring simultaneous-update semantics.
    """

    @property
    @abstractmethod
    def name(self) -> str:
        """Human-readable name of the policy (e.g. ``'STOP_AND_WAIT'``)."""

    @abstractmethod
    def decide(
        self,
        snapshots: list[RobotSnapshot],
    ) -> dict[str, Decision]:
        """Determine MOVE or WAIT for every robot.

        Parameters:
            snapshots: Frozen view of every robot's current state and
                       intended next position.

        Returns:
            A dict mapping each ``robot_id`` to its :class:`Decision`.
        """


class StopAndWaitPolicy(CoordinationPolicy):
    """Naive stop-and-wait collision avoidance baseline.

    Decision rules (evaluated for each robot independently):

    1. **Not active** — robot is IDLE or ARRIVED → Decision.MOVE
       (no-op; the robot won't actually move).

    2. **Staying in place** — intended_next == position → Decision.MOVE
       (path is complete or robot has no path).

    3. **Node conflict** — two (or more) robots intend to move to the
       *same* cell.  The robot with the *lowest* ID (lexicographic)
       moves; all others wait.

    4. **Edge conflict** — two robots intend to swap adjacent cells
       (A→B, B→A).  This is physically unsafe: two AMRs cannot pass
       through each other on a single grid edge.  Both robots wait.

    5. **Occupancy conflict** (iterative) — a robot wants to move into
       a cell currently occupied by another robot that is *not* moving
       away from that cell.  The moving robot waits.  This check
       iterates until no decisions change, handling cascading blocks.

    Tie-breaking:
        Lower robot ID wins (Python's default string ``<`` comparison).
        This is deterministic, generic, and deliberately naive.

    Limitations (by design):
        - No priority negotiation
        - No deadlock resolution beyond simple tie-breaking
        - No rerouting
        - No reservation or communication
    """


    @property
    def name(self) -> str:
        return "STOP_AND_WAIT"

    def decide(
        self,
        snapshots: list[RobotSnapshot],
    ) -> dict[str, Decision]:
        """Evaluate all robots against the shared snapshot."""
        decisions: dict[str, Decision] = {}

        # Index snapshots by robot ID for fast lookup.
        by_id: dict[str, RobotSnapshot] = {s.robot_id: s for s in snapshots}

        # ── Phase 1: default every robot to MOVE ──────────────────
        active_movers: list[RobotSnapshot] = []
        for snap in snapshots:
            if snap.state not in (RobotState.MOVING, RobotState.WAITING):
                # IDLE or ARRIVED — nothing to decide.
                decisions[snap.robot_id] = Decision.MOVE
            elif snap.intended_next == snap.position:
                # Path complete or staying in place.
                decisions[snap.robot_id] = Decision.MOVE
            else:
                active_movers.append(snap)
                decisions[snap.robot_id] = Decision.MOVE  # tentative

        # ── Phase 2: detect node conflicts among active movers ────
        # Build an index: intended_next → list of robots heading there.
        target_map: dict[tuple[int, int], list[RobotSnapshot]] = {}
        for snap in active_movers:
            key = (snap.intended_next.x, snap.intended_next.y)
            target_map.setdefault(key, []).append(snap)

        # If multiple robots target the same cell, only the one with
        # the lowest ID moves; all others wait.
        for cell_key, claimants in target_map.items():
            if len(claimants) > 1:
                claimants.sort(key=lambda s: s.robot_id)
                for loser in claimants[1:]:
                    decisions[loser.robot_id] = Decision.WAIT

        # ── Phase 3: detect edge conflicts (swaps) ───────────────
        # Two robots attempting to swap adjacent cells (A→B, B→A)
        # is physically unsafe — they would traverse the same edge
        # in opposite directions.  Both must wait.
        #
        # Build a lookup: (from, to) → robot_id for robots still MOVE.
        move_edges: dict[tuple[tuple[int, int], tuple[int, int]], str] = {}
        for snap in active_movers:
            if decisions[snap.robot_id] is Decision.MOVE:
                from_key = (snap.position.x, snap.position.y)
                to_key = (snap.intended_next.x, snap.intended_next.y)
                move_edges[(from_key, to_key)] = snap.robot_id

        for snap in active_movers:
            if decisions[snap.robot_id] is not Decision.MOVE:
                continue
            from_key = (snap.position.x, snap.position.y)
            to_key = (snap.intended_next.x, snap.intended_next.y)
            # Check if any other robot is traversing the reverse edge.
            reverse = (to_key, from_key)
            if reverse in move_edges:
                other_id = move_edges[reverse]
                if other_id != snap.robot_id:
                    # Edge conflict: both robots wait.
                    decisions[snap.robot_id] = Decision.WAIT
                    decisions[other_id] = Decision.WAIT

        # ── Phase 4: occupancy conflicts (iterative) ─────────────
        # A robot wants to enter a cell occupied by another robot
        # that is NOT approved to move away from that cell.
        #
        # The check is iterative because a cascade can occur:
        #   C blocks B → B can't leave → A can't enter B's cell
        # Each time a robot is downgraded to WAIT, we must recheck
        # all MOVE robots whose targets depended on the newly-waiting
        # robot vacating.

        # Build occupied-position index (constant across iterations).
        occupied: dict[tuple[int, int], list[RobotSnapshot]] = {}
        for snap in snapshots:
            key = (snap.position.x, snap.position.y)
            occupied.setdefault(key, []).append(snap)

        changed = True
        while changed:
            changed = False

            for snap in active_movers:
                if decisions[snap.robot_id] is Decision.WAIT:
                    continue  # already waiting

                target_key = (snap.intended_next.x, snap.intended_next.y)
                if target_key not in occupied:
                    continue  # target cell is empty

                for blocker in occupied[target_key]:
                    if blocker.robot_id == snap.robot_id:
                        continue  # self

                    # Is the blocker approved to leave this cell?
                    blocker_leaving = (
                        decisions.get(blocker.robot_id) is Decision.MOVE
                        and blocker.intended_next != blocker.position
                    )
                    if not blocker_leaving:
                        decisions[snap.robot_id] = Decision.WAIT
                        changed = True
                        break

        return decisions

