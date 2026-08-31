"""Simulation engine — coordinates the simulation loop.

The engine is responsible for:

* Holding the simulation state (warehouse, fleet, tick counter).
* Advancing the simulation by one discrete tick.
* Detecting collisions each tick.
* Tracking metrics.
* Reporting whether the simulation is finished.

It does **not** know about rendering.  This separation is critical:
it lets us later run thousands of headless benchmark simulations
without importing Pygame.

DESIGN NOTE on update ordering:
    All robots are stepped simultaneously within a single tick.
    The step order is deterministic (fleet insertion order), but
    robot B's movement does NOT depend on robot A's movement within
    the same tick.  This avoids accidental centralized priority.
"""

from __future__ import annotations

from swarmos.coordination.detector import detect_collisions, detect_path_conflicts
from swarmos.robot.amr import AMR
from swarmos.robot.fleet import Fleet
from swarmos.simulation.config import SimulationConfig
from swarmos.simulation.metrics import SimulationMetrics
from swarmos.warehouse.cell import Position
from swarmos.warehouse.grid import Grid


class SimulationEngine:
    """Manages the discrete-time simulation loop.

    Parameters:
        grid:   The warehouse grid.
        robots: Either a single AMR (Phase 1 compat) or a Fleet.
        config: Engine configuration.
    """

    __slots__ = (
        "_grid", "_fleet", "_config", "_tick", "_finished", "_metrics",
    )

    def __init__(
        self,
        grid: Grid,
        robots: AMR | Fleet,
        config: SimulationConfig | None = None,
    ) -> None:
        self._grid = grid

        # Accept either a single AMR or a Fleet for backward compat.
        if isinstance(robots, AMR):
            self._fleet = Fleet()
            self._fleet.add(robots)
        else:
            self._fleet = robots

        self._config = config or SimulationConfig()
        self._tick: int = 0
        self._finished: bool = False
        self._metrics = SimulationMetrics()

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def grid(self) -> Grid:
        return self._grid

    @property
    def fleet(self) -> Fleet:
        """The active fleet of robots."""
        return self._fleet

    @property
    def robot(self) -> AMR:
        """Return the first robot — Phase 1 backward compatibility."""
        return next(iter(self._fleet))

    @property
    def config(self) -> SimulationConfig:
        return self._config

    @property
    def tick(self) -> int:
        """Current simulation tick (0-indexed)."""
        return self._tick

    @property
    def is_finished(self) -> bool:
        return self._finished

    @property
    def metrics(self) -> SimulationMetrics:
        return self._metrics

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------

    def plan_robot(self, goal: Position) -> bool:
        """Instruct the first robot to plan a path to *goal*.

        Phase 1 backward compatibility.
        Returns ``True`` if planning succeeded.
        """
        return self.robot.plan(self._grid, goal)

    def plan_all(self, goals: dict[str, Position]) -> dict[str, bool]:
        """Instruct each robot to plan a path to its goal.

        Parameters:
            goals: Mapping of robot_id → goal position.

        Returns:
            Mapping of robot_id → whether planning succeeded.
        """
        results: dict[str, bool] = {}
        for robot in self._fleet:
            goal = goals.get(robot.robot_id)
            if goal is not None:
                success = robot.plan(self._grid, goal)
                results[robot.robot_id] = success
                if success and robot.path is not None:
                    self._metrics.register_robot(
                        robot.robot_id, robot.path.length
                    )
            else:
                results[robot.robot_id] = False
        return results

    def detect_initial_conflicts(self) -> None:
        """Run static path conflict detection and record in metrics."""
        conflicts = detect_path_conflicts(self._fleet)
        self._metrics.record_path_conflicts(conflicts)

    # ------------------------------------------------------------------
    # Simulation loop
    # ------------------------------------------------------------------

    def update(self) -> None:
        """Advance the simulation by one tick.

        Each robot moves one waypoint every ``config.robot_step_delay``
        ticks.  Collision detection runs after every movement step.
        """
        if self._finished:
            return

        self._tick += 1

        # Move all robots at the configured step cadence.
        if self._tick % self._config.robot_step_delay == 0:
            for robot in self._fleet:
                was_moving = not robot.has_reached_goal
                robot.step()
                if was_moving:
                    self._metrics.record_step(robot.robot_id)
                if robot.has_reached_goal and was_moving:
                    self._metrics.record_arrival(robot.robot_id, self._tick)

            # Detect collisions after all robots have moved.
            collisions = detect_collisions(self._fleet, self._tick)
            if collisions:
                self._metrics.record_collisions(collisions)

        # Simulation ends when all robots have arrived.
        if self._fleet.all_arrived:
            self._finished = True
            self._metrics.set_total_ticks(self._tick)
