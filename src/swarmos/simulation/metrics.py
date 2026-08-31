"""Simulation metrics — lightweight measurement system.

Tracks key simulation statistics independently from rendering.
This module will become the foundation for benchmarking comparisons
(e.g. stop-and-wait vs. decentralised coordination) in later phases.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from swarmos.coordination.conflict import Collision, Conflict


@dataclass
class RobotMetrics:
    """Per-robot metrics.

    Attributes:
        robot_id:        The robot's identifier.
        path_length:     Number of waypoints in the planned path.
        steps_taken:     Number of movement steps executed so far.
        completed:       Whether the robot has reached its goal.
        completion_tick: The simulation tick when the robot arrived
                         (None if not yet arrived).
    """

    robot_id: str
    path_length: int = 0
    steps_taken: int = 0
    completed: bool = False
    completion_tick: int | None = None


@dataclass
class SimulationMetrics:
    """Aggregate metrics for the entire simulation run.

    This class accumulates data as the simulation progresses.
    It is independent of rendering and can be serialised for
    benchmarking.
    """

    total_ticks: int = 0
    robots_completed: int = 0
    total_robots: int = 0
    total_collisions: int = 0
    total_path_conflicts: int = 0
    collisions: list[Collision] = field(default_factory=list)
    path_conflicts: list[Conflict] = field(default_factory=list)
    per_robot: dict[str, RobotMetrics] = field(default_factory=dict)

    # ------------------------------------------------------------------
    # Recording
    # ------------------------------------------------------------------

    def register_robot(self, robot_id: str, path_length: int) -> None:
        """Register a robot and its planned path length."""
        self.total_robots += 1
        self.per_robot[robot_id] = RobotMetrics(
            robot_id=robot_id,
            path_length=path_length,
        )

    def record_step(self, robot_id: str) -> None:
        """Record that *robot_id* took one movement step."""
        if robot_id in self.per_robot:
            self.per_robot[robot_id].steps_taken += 1

    def record_arrival(self, robot_id: str, tick: int) -> None:
        """Record that *robot_id* reached its goal at *tick*."""
        if robot_id in self.per_robot:
            rm = self.per_robot[robot_id]
            if not rm.completed:
                rm.completed = True
                rm.completion_tick = tick
                self.robots_completed += 1

    def record_collisions(self, new_collisions: list[Collision]) -> None:
        """Record newly detected collisions."""
        self.collisions.extend(new_collisions)
        self.total_collisions += len(new_collisions)

    def record_path_conflicts(self, conflicts: list[Conflict]) -> None:
        """Record statically detected path conflicts."""
        self.path_conflicts = list(conflicts)
        self.total_path_conflicts = len(conflicts)

    def set_total_ticks(self, ticks: int) -> None:
        """Update the total tick count."""
        self.total_ticks = ticks

    # ------------------------------------------------------------------
    # Reporting
    # ------------------------------------------------------------------

    def summary(self) -> str:
        """Return a human-readable summary string."""
        lines = [
            "═══ Simulation Metrics ═══",
            f"  Total ticks:          {self.total_ticks}",
            f"  Robots:               {self.robots_completed}/{self.total_robots} completed",
            f"  Path conflicts:       {self.total_path_conflicts}",
            f"  Runtime collisions:   {self.total_collisions}",
        ]
        for rm in self.per_robot.values():
            status = f"arrived at tick {rm.completion_tick}" if rm.completed else "in progress"
            lines.append(
                f"  {rm.robot_id:>10s}: "
                f"path={rm.path_length}, "
                f"steps={rm.steps_taken}, "
                f"{status}"
            )
        return "\n".join(lines)
