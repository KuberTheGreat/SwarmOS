"""Distributed coordination policy.

Phase 6 — Distributed Conflict Negotiation.

Implements the CoordinationPolicy interface using per-robot distributed
negotiation.  Each robot's decision is computed from its OWN local peer
knowledge (received through P2P communication), not from a centralized
global snapshot.

The simulation engine still calls ``decide(snapshots)`` for interface
compatibility, but the ``DistributedPolicy`` does NOT use the snapshot
data for decision-making.  Instead, it accesses each robot's
``CommunicationInterface.peer_state`` to make truly distributed
decisions.

A post-decision physical safety verification pass ensures that no
edge swaps or occupancy violations escape (defense in depth).

"Phase 6 introduces decentralized conflict negotiation.  Each AMR
independently determines whether it should proceed or wait based on
exchanged peer intents and deterministic coordination rules.  No
centralized conflict arbiter is used."
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from swarmos.communication.intent import IntentAction
from swarmos.coordination.decision import Decision, RobotSnapshot
from swarmos.coordination.negotiator import DistributedNegotiator, NegotiationResult
from swarmos.coordination.policy import CoordinationPolicy
from swarmos.robot.state import RobotState

if TYPE_CHECKING:
    from swarmos.robot.fleet import Fleet


# Map RobotState to IntentAction.
_INTENT_MAP = {
    RobotState.MOVING:  IntentAction.MOVE,
    RobotState.WAITING: IntentAction.WAIT,
    RobotState.IDLE:    IntentAction.IDLE,
    RobotState.ARRIVED: IntentAction.COMPLETE,
}


class DistributedPolicy(CoordinationPolicy):
    """Distributed conflict negotiation policy.

    Each robot independently computes its PROCEED/WAIT decision from
    its local peer knowledge.  No centralized arbiter is used.

    Parameters:
        fleet: Reference to the robot fleet, used to access each
               robot's CommunicationInterface for local peer state.
    """

    __slots__ = ("_fleet", "_last_results")

    def __init__(self, fleet: Fleet) -> None:
        self._fleet = fleet
        self._last_results: dict[str, NegotiationResult] = {}

    @property
    def name(self) -> str:
        return "DISTRIBUTED"

    @property
    def last_results(self) -> dict[str, NegotiationResult]:
        """The negotiation results from the most recent decide() call."""
        return self._last_results

    def decide(
        self,
        snapshots: list[RobotSnapshot],
        current_tick: int = 0,
    ) -> dict[str, Decision]:
        """Compute per-robot decisions using distributed negotiation.

        The ``snapshots`` parameter is accepted for interface
        compatibility but is NOT used for decision-making.  Each
        robot's decision is computed from its own local peer state.

        Parameters:
            snapshots:    Robot snapshots (used only for bookkeeping).
            current_tick: The current simulation tick for stale checks.

        Returns:
            A dict mapping each ``robot_id`` to its :class:`Decision`.
        """
        decisions: dict[str, Decision] = {}
        self._last_results = {}

        # --- Phase 1: Each robot negotiates independently ---
        for robot in self._fleet:
            if robot.state not in (RobotState.MOVING, RobotState.WAITING):
                decisions[robot.robot_id] = Decision.MOVE  # no-op
                continue

            if robot.intended_next_position == robot.position:
                decisions[robot.robot_id] = Decision.MOVE  # staying
                continue

            own_intent = _INTENT_MAP.get(robot.state, IntentAction.IDLE)

            # Get this robot's LOCAL peer knowledge.
            peer_state = {}
            if robot.communication is not None:
                peer_state = robot.communication.peer_state
                stale_threshold = robot.communication.stale_threshold
            else:
                stale_threshold = None

            result = DistributedNegotiator.negotiate(
                own_id=robot.robot_id,
                own_position=robot.position,
                own_intended=robot.intended_next_position,
                own_intent=own_intent,
                peer_state=peer_state,
                current_tick=current_tick,
                stale_threshold=stale_threshold,
            )

            decisions[robot.robot_id] = result.decision
            self._last_results[robot.robot_id] = result

        # --- Phase 2: Physical safety verification (defense in depth) ---
        # Even after distributed negotiation, verify that no approved
        # movements would create edge swaps or occupancy violations.
        self._verify_edge_safety(decisions)
        self._verify_occupancy_safety(decisions)

        return decisions

    def _verify_edge_safety(
        self,
        decisions: dict[str, Decision],
    ) -> None:
        """Ensure no approved movements form edge swaps."""
        # Build move-edge index for robots approved to MOVE.
        move_edges: dict[
            tuple[tuple[int, int], tuple[int, int]], str
        ] = {}

        for robot in self._fleet:
            if decisions.get(robot.robot_id) is not Decision.MOVE:
                continue
            if robot.intended_next_position == robot.position:
                continue

            from_key = (robot.position.x, robot.position.y)
            to_key = (
                robot.intended_next_position.x,
                robot.intended_next_position.y,
            )
            move_edges[(from_key, to_key)] = robot.robot_id

        # Check for reverse edges.
        for robot in self._fleet:
            if decisions.get(robot.robot_id) is not Decision.MOVE:
                continue
            if robot.intended_next_position == robot.position:
                continue

            from_key = (robot.position.x, robot.position.y)
            to_key = (
                robot.intended_next_position.x,
                robot.intended_next_position.y,
            )
            reverse = (to_key, from_key)
            if reverse in move_edges:
                other_id = move_edges[reverse]
                if other_id != robot.robot_id:
                    # Edge swap detected — both must WAIT.
                    decisions[robot.robot_id] = Decision.WAIT
                    decisions[other_id] = Decision.WAIT

    def _verify_occupancy_safety(
        self,
        decisions: dict[str, Decision],
    ) -> None:
        """Ensure no robot moves into a cell occupied by a non-leaving peer."""
        # Build occupied-position index.
        occupied: dict[tuple[int, int], list[str]] = {}
        for robot in self._fleet:
            key = (robot.position.x, robot.position.y)
            occupied.setdefault(key, []).append(robot.robot_id)

        # Iterative check (cascading blocks).
        changed = True
        while changed:
            changed = False
            for robot in self._fleet:
                if decisions.get(robot.robot_id) is not Decision.MOVE:
                    continue
                if robot.intended_next_position == robot.position:
                    continue

                target_key = (
                    robot.intended_next_position.x,
                    robot.intended_next_position.y,
                )
                if target_key not in occupied:
                    continue

                for blocker_id in occupied[target_key]:
                    if blocker_id == robot.robot_id:
                        continue

                    # Is the blocker approved to leave?
                    blocker = self._fleet.get(blocker_id)
                    if blocker is None:
                        continue
                    blocker_leaving = (
                        decisions.get(blocker_id) is Decision.MOVE
                        and blocker.intended_next_position != blocker.position
                    )
                    if not blocker_leaving:
                        decisions[robot.robot_id] = Decision.WAIT
                        changed = True
                        break
