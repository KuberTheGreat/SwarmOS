"""Fleet — a container for multiple AMRs.

The Fleet is a *state container*, not a controller.  It holds the
collection of active robots and provides query/iteration operations.
It does NOT decide paths, resolve conflicts, or coordinate movement.

This distinction will be critical in later phases: the Fleet is shared
state that the simulation engine reads, while each robot independently
owns its own planning and decision-making.
"""

from __future__ import annotations

from typing import Iterator

from swarmos.robot.amr import AMR


class Fleet:
    """An ordered collection of AMRs with unique IDs.

    Robots are stored in insertion order for deterministic iteration.
    """

    __slots__ = ("_robots",)

    def __init__(self) -> None:
        self._robots: dict[str, AMR] = {}

    # ------------------------------------------------------------------
    # Mutation
    # ------------------------------------------------------------------

    def add(self, robot: AMR) -> None:
        """Add *robot* to the fleet.

        Raises:
            ValueError: If a robot with the same ID already exists.
        """
        if robot.robot_id in self._robots:
            raise ValueError(
                f"Duplicate robot ID: {robot.robot_id!r} already in fleet"
            )
        self._robots[robot.robot_id] = robot

    def remove(self, robot_id: str) -> AMR:
        """Remove and return the robot with *robot_id*.

        Raises:
            KeyError: If no robot with that ID exists.
        """
        if robot_id not in self._robots:
            raise KeyError(f"Robot {robot_id!r} not in fleet")
        return self._robots.pop(robot_id)

    # ------------------------------------------------------------------
    # Query
    # ------------------------------------------------------------------

    def get(self, robot_id: str) -> AMR:
        """Return the robot with *robot_id*.

        Raises:
            KeyError: If no robot with that ID exists.
        """
        if robot_id not in self._robots:
            raise KeyError(f"Robot {robot_id!r} not in fleet")
        return self._robots[robot_id]

    def __contains__(self, robot_id: str) -> bool:
        return robot_id in self._robots

    def __len__(self) -> int:
        return len(self._robots)

    def __iter__(self) -> Iterator[AMR]:
        """Iterate robots in insertion order (deterministic)."""
        return iter(self._robots.values())

    @property
    def robot_ids(self) -> list[str]:
        """Return the list of robot IDs in insertion order."""
        return list(self._robots.keys())

    @property
    def all_arrived(self) -> bool:
        """Return ``True`` if every robot has reached its goal."""
        return all(robot.has_reached_goal for robot in self._robots.values())

    @property
    def active_robots(self) -> list[AMR]:
        """Return robots that have not yet reached their goal."""
        return [r for r in self._robots.values() if not r.has_reached_goal]

    @property
    def arrived_robots(self) -> list[AMR]:
        """Return robots that have reached their goal."""
        return [r for r in self._robots.values() if r.has_reached_goal]

    # ------------------------------------------------------------------
    # Representation
    # ------------------------------------------------------------------

    def __repr__(self) -> str:
        return f"Fleet(robots={len(self._robots)}, ids={self.robot_ids})"
