"""Distributed conflict negotiation.

Phase 6 — Distributed Conflict Negotiation.

Each AMR independently evaluates its local peer knowledge and
determines whether it should PROCEED or WAIT.  No centralized
arbiter is involved.

Provides:
    NegotiationResult   — the outcome of a single robot's local
                          negotiation computation.
    DistributedNegotiator — stateless negotiation logic that a robot
                            invokes with its own state + local peer
                            knowledge.

Deterministic priority rule:
    Lowest robot ID (lexicographic) wins any given conflict.

Physical safety:
    Edge swaps (A→B, B→A) force BOTH robots to WAIT regardless of
    priority.  Safety cannot be overridden by negotiation.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from swarmos.communication.intent import (
    ConflictType,
    IntentAction,
    PotentialConflict,
    is_stale,
)
from swarmos.coordination.decision import Decision
from swarmos.warehouse.cell import Position

if TYPE_CHECKING:
    from swarmos.communication.message import RobotStateMessage


@dataclass(frozen=True)
class NegotiationResult:
    """Outcome of a single robot's local negotiation.

    Attributes:
        decision:     MOVE (proceed) or WAIT.
        conflicts:    Potential conflicts that were evaluated.
        participants: IDs of all robots involved in any conflict.
        winner:       The robot ID that was granted PROCEED, or None
                      if no conflict was detected or if the robot is
                      not actively moving.
        reason:       Human-readable explanation for the decision.
    """

    decision: Decision
    conflicts: list[PotentialConflict] = field(default_factory=list)
    participants: list[str] = field(default_factory=list)
    winner: str | None = None
    reason: str = ""


class DistributedNegotiator:
    """Stateless distributed negotiation logic.

    Each AMR calls :meth:`negotiate` with its own state and the peer
    knowledge it has received through the P2P layer.  The method returns
    a :class:`NegotiationResult` telling the robot whether to PROCEED
    or WAIT.

    Because every robot applies the same deterministic rules to the same
    observable information, two robots involved in a conflict will
    independently arrive at consistent decisions.

    There is NO centralized conflict controller.  This class reads only
    the information available to the calling robot.
    """

    @staticmethod
    def negotiate(
        own_id: str,
        own_position: Position,
        own_intended: Position,
        own_intent: IntentAction,
        peer_state: dict[str, RobotStateMessage],
        current_tick: int,
        stale_threshold: int | None = None,
    ) -> NegotiationResult:
        """Compute a local PROCEED/WAIT decision.

        Parameters:
            own_id:           This robot's ID.
            own_position:     This robot's current position.
            own_intended:     This robot's intended next position.
            own_intent:       This robot's intent action.
            peer_state:       Local peer knowledge (peer_id → message).
            current_tick:     Current simulation tick.
            stale_threshold:  If set, ignore peer messages older than
                              this many ticks.

        Returns:
            A :class:`NegotiationResult` with the decision.
        """
        # --- Non-active robots: no negotiation needed ---
        if own_intent is not IntentAction.MOVE:
            return NegotiationResult(
                decision=Decision.MOVE,
                reason="not actively moving",
            )

        # Robot staying in place — no conflict possible.
        if own_intended == own_position:
            return NegotiationResult(
                decision=Decision.MOVE,
                reason="staying in place",
            )

        # --- Collect relevant conflicts ---
        node_conflicts: list[PotentialConflict] = []
        edge_conflicts: list[PotentialConflict] = []
        occupancy_blockers: list[str] = []

        for peer_id, msg in peer_state.items():
            # Skip stale messages.
            if stale_threshold is not None and is_stale(
                msg, current_tick, stale_threshold
            ):
                continue

            # --- Edge conflict: opposite-direction swap ---
            # Physical safety: BOTH robots must WAIT for edge swaps
            # regardless of priority.
            if (msg.intent is IntentAction.MOVE
                    and own_intended == msg.position
                    and msg.intended_next == own_position):
                edge_conflicts.append(PotentialConflict(
                    conflict_type=ConflictType.EDGE,
                    robot_a_id=own_id,
                    robot_b_id=peer_id,
                    position=own_intended,
                    tick=current_tick,
                ))

            # --- Node conflict: both target the same cell ---
            if (msg.intent is IntentAction.MOVE
                    and own_intended == msg.intended_next):
                node_conflicts.append(PotentialConflict(
                    conflict_type=ConflictType.NODE,
                    robot_a_id=own_id,
                    robot_b_id=peer_id,
                    position=own_intended,
                    tick=current_tick,
                ))

            # --- Occupancy conflict ---
            # Target cell is occupied by a peer that is NOT moving away.
            peer_pos = (msg.position.x, msg.position.y)
            target_pos = (own_intended.x, own_intended.y)
            if peer_pos == target_pos:
                peer_is_leaving = (
                    msg.intent is IntentAction.MOVE
                    and msg.intended_next != msg.position
                )
                if not peer_is_leaving:
                    occupancy_blockers.append(peer_id)

        all_conflicts = edge_conflicts + node_conflicts

        # --- Decision: Edge conflicts → always WAIT (physical safety) ---
        if edge_conflicts:
            participants = sorted(
                {own_id} | {c.robot_b_id for c in edge_conflicts}
            )
            return NegotiationResult(
                decision=Decision.WAIT,
                conflicts=all_conflicts,
                participants=participants,
                winner=None,
                reason="edge swap — physical safety",
            )

        # --- Decision: Node conflicts → deterministic priority ---
        if node_conflicts:
            # Gather all participants including self.
            participant_set = {own_id}
            for c in node_conflicts:
                participant_set.add(c.robot_b_id)
            participants = sorted(participant_set)

            # Deterministic priority: lowest robot ID wins.
            winner = participants[0]

            if own_id == winner:
                # Check if winner's target is blocked by occupancy.
                if occupancy_blockers:
                    return NegotiationResult(
                        decision=Decision.WAIT,
                        conflicts=all_conflicts,
                        participants=participants,
                        winner=winner,
                        reason=f"node conflict winner but blocked by occupancy ({occupancy_blockers})",
                    )
                return NegotiationResult(
                    decision=Decision.MOVE,
                    conflicts=all_conflicts,
                    participants=participants,
                    winner=winner,
                    reason="node conflict winner (lowest ID)",
                )
            else:
                return NegotiationResult(
                    decision=Decision.WAIT,
                    conflicts=all_conflicts,
                    participants=participants,
                    winner=winner,
                    reason=f"node conflict loser (winner={winner})",
                )

        # --- Decision: Occupancy conflict (no node/edge) ---
        if occupancy_blockers:
            return NegotiationResult(
                decision=Decision.WAIT,
                reason=f"target cell occupied by {occupancy_blockers}",
            )

        # --- No conflict: PROCEED ---
        return NegotiationResult(
            decision=Decision.MOVE,
            reason="no conflict",
        )
