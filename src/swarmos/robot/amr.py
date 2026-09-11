"""Autonomous Mobile Robot (AMR) model.

The AMR is an *independent agent* — it owns its own state and consumes
a planner to decide how to move.  In Phase 1 the robot is given a goal,
plans once via A*, and executes that path.  The class is structured so
that future phases can add:

* local world model
* peer communication
* re-planning on conflict
* battery / velocity / sensor state

without rewriting the core robot interface.

Phase 3 additions:
    wait()                   — transition to WAITING (stop-and-wait policy)
    resume()                 — transition from WAITING back to MOVING
    intended_next_position   — where the robot *wants* to go next
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from swarmos.planning.astar import find_path
from swarmos.planning.path import Path
from swarmos.robot.state import RobotState
from swarmos.warehouse.cell import Position

if TYPE_CHECKING:
    from swarmos.communication.interface import CommunicationInterface
    from swarmos.communication.message import RobotStateMessage
    from swarmos.warehouse.grid import Grid


class AMR:
    """A single autonomous mobile robot operating on a warehouse grid.

    Parameters:
        robot_id: Unique identifier string.
        position: Initial grid position.
    """

    __slots__ = (
        "_robot_id",
        "_position",
        "_goal",
        "_path",
        "_state",
        "_communication",
    )

    def __init__(self, robot_id: str, position: Position) -> None:
        self._robot_id = robot_id
        self._position = position
        self._goal: Position | None = None
        self._path: Path | None = None
        self._state = RobotState.IDLE
        self._communication: CommunicationInterface | None = None

    # ------------------------------------------------------------------
    # Properties (read-only public interface)
    # ------------------------------------------------------------------

    @property
    def robot_id(self) -> str:
        return self._robot_id

    @property
    def position(self) -> Position:
        return self._position

    @property
    def goal(self) -> Position | None:
        return self._goal

    @property
    def path(self) -> Path | None:
        return self._path

    @property
    def state(self) -> RobotState:
        return self._state

    @property
    def has_reached_goal(self) -> bool:
        return self._state is RobotState.ARRIVED

    @property
    def intended_next_position(self) -> Position:
        """Return the position this robot *wants* to move to next.

        If the robot is MOVING (or WAITING) and has remaining waypoints,
        returns the next waypoint.  Otherwise returns the current
        position (the robot intends to stay put).
        """
        if self._state in (RobotState.MOVING, RobotState.WAITING):
            if self._path is not None:
                nxt = self._path.peek_next()
                if nxt is not None:
                    return nxt
        return self._position

    @property
    def communication(self) -> CommunicationInterface | None:
        """The communication interface component, if attached."""
        return self._communication

    @communication.setter
    def communication(self, interface: CommunicationInterface) -> None:
        self._communication = interface

    @property
    def peer_state(self) -> dict[str, RobotStateMessage]:
        """Return the local knowledge of other robots' states."""
        if self._communication is not None:
            return self._communication.peer_state
        return {}

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------

    def plan(self, grid: Grid, goal: Position) -> bool:
        """Compute a path to *goal* using A* and transition to MOVING.

        Returns ``True`` if a valid path was found, ``False`` otherwise.
        On failure the robot remains IDLE.
        """
        self._goal = goal
        result = find_path(grid, self._position, goal)

        if result is None:
            self._path = None
            self._state = RobotState.IDLE
            return False

        self._path = result

        # If start == goal the path has length 1 — already there.
        if result.is_complete:
            self._state = RobotState.ARRIVED
        else:
            self._state = RobotState.MOVING

        return True

    # ------------------------------------------------------------------
    # Execution
    # ------------------------------------------------------------------

    def step(self) -> None:
        """Advance the robot by one waypoint along its planned path.

        Does nothing if the robot is not in the MOVING state.
        """
        if self._state is not RobotState.MOVING:
            return

        if self._path is None:
            return

        next_pos = self._path.advance()
        if next_pos is not None:
            self._position = next_pos

        if self._path.is_complete:
            self._state = RobotState.ARRIVED

    def wait(self) -> None:
        """Transition to WAITING — the robot yields for this tick.

        Only meaningful when the robot is MOVING or WAITING.
        Does nothing for IDLE or ARRIVED robots.
        """
        if self._state in (RobotState.MOVING, RobotState.WAITING):
            self._state = RobotState.WAITING

    def resume(self) -> None:
        """Transition from WAITING back to MOVING.

        Only meaningful when the robot is WAITING.
        Does nothing for other states.
        """
        if self._state is RobotState.WAITING:
            self._state = RobotState.MOVING

    # ------------------------------------------------------------------
    # Communication
    # ------------------------------------------------------------------

    def broadcast_state(self, tick: int) -> None:
        """Broadcast current state to peers via the communication interface."""
        if self._communication is not None:
            self._communication.broadcast_state(
                position=self._position,
                intended_next=self.intended_next_position,
                state=self._state,
                tick=tick,
            )

    # ------------------------------------------------------------------
    # Representation
    # ------------------------------------------------------------------

    def __repr__(self) -> str:
        return (
            f"AMR(id={self._robot_id!r}, pos={self._position}, "
            f"state={self._state.name})"
        )

