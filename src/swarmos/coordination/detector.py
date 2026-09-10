"""Conflict and collision detection.

This module provides two categories of detection:

1. **Path conflict detection** (static analysis):
   Given the planned paths of all robots, detect temporal overlaps
   (node conflicts and edge conflicts) *before* or *during* execution.

2. **Collision detection** (runtime):
   Given the current positions of all robots at a specific simulation
   tick, detect actual same-cell occupancy.

IMPORTANT: This module *detects* problems.  It does NOT resolve them.
Resolution will be introduced in later phases via negotiation protocols.
"""

from __future__ import annotations

from itertools import combinations
from typing import TYPE_CHECKING

from swarmos.coordination.conflict import Collision, Conflict, ConflictType
from swarmos.warehouse.cell import Position

if TYPE_CHECKING:
    from swarmos.robot.amr import AMR
    from swarmos.robot.fleet import Fleet


# ------------------------------------------------------------------
# Timed trajectory extraction
# ------------------------------------------------------------------

def _extract_timed_trajectory(robot: AMR) -> list[Position]:
    """Return the full waypoint list from a robot's path.

    The index in the returned list corresponds to the discrete timestep.
    If the robot has no path, returns a single-element list with its
    current position (it stays in place).

    After the path ends, the robot remains at its final position.
    """
    if robot.path is None:
        return [robot.position]
    return robot.path.waypoints


def _get_position_at(trajectory: list[Position], timestep: int) -> Position:
    """Return the position at *timestep*, clamping to the final position
    if the trajectory is shorter."""
    if timestep >= len(trajectory):
        return trajectory[-1]
    return trajectory[timestep]


# ------------------------------------------------------------------
# Path conflict detection (static)
# ------------------------------------------------------------------

def detect_path_conflicts(fleet: Fleet) -> list[Conflict]:
    """Detect node and edge conflicts across all robot paths.

    Compares every pair of robots and checks, at each discrete timestep,
    whether their planned trajectories conflict.

    Returns a list of :class:`Conflict` instances, sorted by timestep.
    """
    conflicts: list[Conflict] = []

    robots = list(fleet)
    trajectories: dict[str, list[Position]] = {}
    for robot in robots:
        trajectories[robot.robot_id] = _extract_timed_trajectory(robot)

    # Determine the longest trajectory length.
    max_len = max((len(t) for t in trajectories.values()), default=0)

    for (robot_a, robot_b) in combinations(robots, 2):
        traj_a = trajectories[robot_a.robot_id]
        traj_b = trajectories[robot_b.robot_id]

        pair_max = max(len(traj_a), len(traj_b))

        for t in range(pair_max):
            pos_a = _get_position_at(traj_a, t)
            pos_b = _get_position_at(traj_b, t)

            # --- Node conflict ---
            if pos_a == pos_b:
                conflicts.append(Conflict(
                    conflict_type=ConflictType.NODE,
                    robot_a_id=robot_a.robot_id,
                    robot_b_id=robot_b.robot_id,
                    position=pos_a,
                    timestep=t,
                ))

            # --- Edge conflict ---
            # Check if robots swap positions between t and t+1.
            if t + 1 < pair_max:
                next_a = _get_position_at(traj_a, t + 1)
                next_b = _get_position_at(traj_b, t + 1)

                if pos_a == next_b and pos_b == next_a:
                    conflicts.append(Conflict(
                        conflict_type=ConflictType.EDGE,
                        robot_a_id=robot_a.robot_id,
                        robot_b_id=robot_b.robot_id,
                        position=next_a,
                        timestep=t,
                    ))

    conflicts.sort(key=lambda c: c.timestep)
    return conflicts


# ------------------------------------------------------------------
# Collision detection (runtime)
# ------------------------------------------------------------------

def detect_collisions(
    fleet: Fleet,
    tick: int,
    previous_positions: dict[str, Position] | None = None,
) -> list[Collision]:
    """Detect actual collisions at runtime.

    Checks for:
        1. **Node collisions** — two robots occupying the same cell.
        2. **Edge collisions** — two robots that swapped adjacent cells
           (traversed the same edge in opposite directions).

    Parameters:
        fleet:              The active fleet.
        tick:               The current simulation tick.
        previous_positions: Optional mapping of robot_id → position
                            *before* the current tick's movement.
                            Required for edge-swap detection.

    Returns:
        A list of :class:`Collision` instances for each violation.
    """
    collisions: list[Collision] = []
    robots = list(fleet)

    for i, robot_a in enumerate(robots):
        for robot_b in robots[i + 1:]:
            # --- Node collision ---
            if robot_a.position == robot_b.position:
                collisions.append(Collision(
                    robot_a_id=robot_a.robot_id,
                    robot_b_id=robot_b.robot_id,
                    position=robot_a.position,
                    tick=tick,
                ))

            # --- Edge collision (swap) ---
            if previous_positions is not None:
                prev_a = previous_positions.get(robot_a.robot_id)
                prev_b = previous_positions.get(robot_b.robot_id)
                if prev_a is not None and prev_b is not None:
                    # A was at prev_a, now at robot_a.position
                    # B was at prev_b, now at robot_b.position
                    # Swap: A moved to where B was, B moved to where A was
                    if (robot_a.position == prev_b
                            and robot_b.position == prev_a
                            and prev_a != prev_b):
                        collisions.append(Collision(
                            robot_a_id=robot_a.robot_id,
                            robot_b_id=robot_b.robot_id,
                            position=robot_a.position,
                            tick=tick,
                        ))

    return collisions

