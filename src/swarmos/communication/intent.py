"""Intent model and conflict observation.

Phase 5 — Distributed Intent Exchange.

Provides:
    IntentAction      — what a robot intends to do next tick.
    PotentialConflict  — a read-only observation of two intents that may
                         conflict (node or edge).
    IntentObserver     — derives potential conflicts from local peer
                         knowledge.  Strictly read-only — never modifies
                         movement decisions.
    is_stale()         — checks whether a peer message has expired.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto

from swarmos.warehouse.cell import Position

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from swarmos.communication.message import RobotStateMessage


# ------------------------------------------------------------------
# Intent action enum
# ------------------------------------------------------------------

class IntentAction(Enum):
    """What a robot intends to do on its next movement tick.

    Values:
        MOVE:     Advance to ``intended_next``.
        WAIT:     Hold current position (yielding).
        IDLE:     No path assigned / inactive.
        COMPLETE: Goal reached — no further movement.
    """

    MOVE = auto()
    WAIT = auto()
    IDLE = auto()
    COMPLETE = auto()


# ------------------------------------------------------------------
# Potential conflict (observation only)
# ------------------------------------------------------------------

class ConflictType(Enum):
    """Classification of an observed potential conflict."""

    NODE = auto()   # Two intents target the same cell.
    EDGE = auto()   # Two intents form an opposite-direction swap.


@dataclass(frozen=True)
class PotentialConflict:
    """A read-only observation of a potential conflict between two intents.

    This is purely informational.  It does NOT trigger any movement
    change.  Conflict resolution belongs to Phase 6+.

    Attributes:
        conflict_type: NODE or EDGE.
        robot_a_id:    First robot involved.
        robot_b_id:    Second robot involved.
        position:      The contested cell (node) or edge endpoint.
        tick:          The simulation tick when this was observed.
    """

    conflict_type: ConflictType
    robot_a_id: str
    robot_b_id: str
    position: Position
    tick: int

    def __repr__(self) -> str:
        return (
            f"PotentialConflict({self.conflict_type.name}, "
            f"{self.robot_a_id!r} ↔ {self.robot_b_id!r}, "
            f"pos={self.position}, t={self.tick})"
        )


# ------------------------------------------------------------------
# Stale-message helper
# ------------------------------------------------------------------

def is_stale(
    message: RobotStateMessage,
    current_tick: int,
    threshold: int,
) -> bool:
    """Return ``True`` if *message* is older than *threshold* ticks.

    A message sent at tick ``T`` is considered stale at tick ``C`` when
    ``C - T > threshold``.

    Parameters:
        message:      The peer state message to check.
        current_tick: The current simulation tick.
        threshold:    Maximum age (in ticks) before a message expires.
    """
    return (current_tick - message.timestamp) > threshold


# ------------------------------------------------------------------
# Intent observer (read-only)
# ------------------------------------------------------------------

class IntentObserver:
    """Derives potential conflicts from a robot's local peer knowledge.

    This class is strictly **read-only**.  It examines the owner robot's
    own intent and the intents reported by peers, then identifies pairs
    that may conflict.  It never modifies movement decisions.

    Typical usage (inside an AMR)::

        observer = IntentObserver()
        conflicts = observer.observe(
            own_id="AMR-01",
            own_position=Position(4, 5),
            own_intended=Position(5, 5),
            own_intent=IntentAction.MOVE,
            peer_state=self.peer_state,
            current_tick=tick,
        )
        # conflicts is a list[PotentialConflict] — information only.
    """

    @staticmethod
    def observe(
        own_id: str,
        own_position: Position,
        own_intended: Position,
        own_intent: IntentAction,
        peer_state: dict[str, RobotStateMessage],
        current_tick: int,
        stale_threshold: int | None = None,
    ) -> list[PotentialConflict]:
        """Observe potential conflicts between own intent and peers.

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
            A list of :class:`PotentialConflict` observations.
        """
        conflicts: list[PotentialConflict] = []

        # Only MOVE intents can create conflicts.
        if own_intent is not IntentAction.MOVE:
            return conflicts

        for peer_id, msg in peer_state.items():
            # Skip stale messages.
            if stale_threshold is not None and is_stale(
                msg, current_tick, stale_threshold
            ):
                continue

            # Only peers with MOVE intent can conflict.
            if msg.intent is not IntentAction.MOVE:
                continue

            # --- Node conflict: both target the same cell ---
            if own_intended == msg.intended_next:
                conflicts.append(PotentialConflict(
                    conflict_type=ConflictType.NODE,
                    robot_a_id=own_id,
                    robot_b_id=peer_id,
                    position=own_intended,
                    tick=current_tick,
                ))

            # --- Edge conflict: opposite-direction swap ---
            if (own_intended == msg.position
                    and msg.intended_next == own_position
                    and own_position != own_intended):
                conflicts.append(PotentialConflict(
                    conflict_type=ConflictType.EDGE,
                    robot_a_id=own_id,
                    robot_b_id=peer_id,
                    position=own_intended,
                    tick=current_tick,
                ))

        return conflicts
