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

    4. **Edge conflict** — two robots would swap positions (A→B and
       B→A).  The robot with the *lowest* ID moves; the other waits.

    5. **Occupancy conflict** — a robot wants to move into a cell
       currently occupied by another robot that is *not* moving away
       from that cell.  The moving robot waits.

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

        # ── Phase 2: detect conflicts among active movers ─────────
        # Build an index: intended_next → list of robots heading there.
        target_map: dict[tuple[int, int], list[RobotSnapshot]] = {}
        for snap in active_movers:
            key = (snap.intended_next.x, snap.intended_next.y)
            target_map.setdefault(key, []).append(snap)

        # --- Node conflicts ---
        # If multiple robots target the same cell, only the one with
        # the lowest ID moves; all others wait.
        for cell_key, claimants in target_map.items():
            if len(claimants) > 1:
                # Sort by robot_id — lowest wins.
                claimants.sort(key=lambda s: s.robot_id)
                for loser in claimants[1:]:
                    decisions[loser.robot_id] = Decision.WAIT

        # --- Edge conflicts ---
        # Two robots A and B would swap positions: A→B_pos and B→A_pos.
        # In stop-and-wait with fixed paths, BOTH must wait because:
        #   - If only B waits, A moves into B's cell → collision
        #   - If only A waits, B moves into A's cell → collision
        # This is a known limitation — deadlock without rerouting.
        checked: set[tuple[str, str]] = set()
        for snap_a in active_movers:
            for snap_b in active_movers:
                if snap_a.robot_id >= snap_b.robot_id:
                    continue
                pair_key = (snap_a.robot_id, snap_b.robot_id)
                if pair_key in checked:
                    continue
                checked.add(pair_key)

                if (snap_a.intended_next == snap_b.position
                        and snap_b.intended_next == snap_a.position):
                    # Edge conflict — both must wait in stop-and-wait.
                    # Lower ID gets to move first in future ticks when
                    # the situation changes (e.g., on open grids).
                    decisions[snap_a.robot_id] = Decision.WAIT
                    decisions[snap_b.robot_id] = Decision.WAIT

        # --- Occupancy conflicts ---
        # A robot wants to enter a cell occupied by another robot that
        # is NOT moving away from that cell.
        # Build a set of positions being vacated by active movers who
        # are still allowed to move.
        vacating: set[tuple[int, int]] = set()
        for snap in active_movers:
            if (decisions[snap.robot_id] is Decision.MOVE
                    and snap.intended_next != snap.position):
                vacating.add((snap.position.x, snap.position.y))

        # Build a set of all currently occupied positions.
        occupied: dict[tuple[int, int], list[RobotSnapshot]] = {}
        for snap in snapshots:
            key = (snap.position.x, snap.position.y)
            occupied.setdefault(key, []).append(snap)

        for snap in active_movers:
            if decisions[snap.robot_id] is Decision.WAIT:
                continue  # already waiting

            target_key = (snap.intended_next.x, snap.intended_next.y)
            if target_key in occupied:
                # Someone is sitting there — are they leaving?
                blockers = occupied[target_key]
                for blocker in blockers:
                    if blocker.robot_id == snap.robot_id:
                        continue  # self

                    blocker_leaving = (
                        target_key in vacating
                        and decisions.get(blocker.robot_id) is Decision.MOVE
                        and blocker.intended_next != blocker.position
                    )
                    if not blocker_leaving:
                        decisions[snap.robot_id] = Decision.WAIT
                        break

        return decisions


