"""Path abstraction — an ordered sequence of waypoints.

A Path encapsulates the result of a planning algorithm and provides a
clean interface for a robot to execute the path step-by-step.

Design rationale: keeping path representation separate from path
*planning* and path *execution* lets each concern evolve independently.
The planner produces a Path; the robot consumes it.
"""

from __future__ import annotations

from swarmos.warehouse.cell import Position


class Path:
    """An ordered sequence of :class:`Position` waypoints.

    The path tracks a *cursor* so a robot can advance through waypoints
    one at a time.

    Parameters:
        waypoints: The full list of positions from start to goal
                   (inclusive of both endpoints).
    """

    __slots__ = ("_waypoints", "_cursor")

    def __init__(self, waypoints: list[Position]) -> None:
        if not waypoints:
            raise ValueError("A Path must contain at least one waypoint")
        self._waypoints = list(waypoints)  # defensive copy
        self._cursor: int = 0

    # ------------------------------------------------------------------
    # Query interface
    # ------------------------------------------------------------------

    @property
    def waypoints(self) -> list[Position]:
        """Return a copy of the full waypoint list."""
        return list(self._waypoints)

    @property
    def start(self) -> Position:
        """The first waypoint."""
        return self._waypoints[0]

    @property
    def goal(self) -> Position:
        """The last waypoint."""
        return self._waypoints[-1]

    @property
    def current(self) -> Position:
        """The waypoint the cursor currently points to."""
        return self._waypoints[self._cursor]

    @property
    def is_complete(self) -> bool:
        """Return ``True`` if the cursor has reached the final waypoint."""
        return self._cursor >= len(self._waypoints) - 1

    @property
    def remaining(self) -> list[Position]:
        """Return the waypoints from the current cursor to the goal."""
        return list(self._waypoints[self._cursor:])

    @property
    def length(self) -> int:
        """Total number of waypoints (including start and goal)."""
        return len(self._waypoints)

    # ------------------------------------------------------------------
    # Mutation interface
    # ------------------------------------------------------------------

    def peek_next(self) -> Position | None:
        """Return the next waypoint without advancing, or ``None`` if
        the path is complete."""
        if self.is_complete:
            return None
        return self._waypoints[self._cursor + 1]

    def advance(self) -> Position | None:
        """Advance the cursor to the next waypoint and return it.

        Returns ``None`` if the path was already complete.
        """
        if self.is_complete:
            return None
        self._cursor += 1
        return self._waypoints[self._cursor]

    def reset(self) -> None:
        """Reset the cursor back to the start."""
        self._cursor = 0

    # ------------------------------------------------------------------
    # Representation
    # ------------------------------------------------------------------

    def __len__(self) -> int:
        return len(self._waypoints)

    def __repr__(self) -> str:
        return (
            f"Path(length={len(self._waypoints)}, "
            f"cursor={self._cursor}, "
            f"start={self.start}, goal={self.goal})"
        )
