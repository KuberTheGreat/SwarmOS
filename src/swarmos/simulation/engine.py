"""Simulation engine — coordinates the simulation loop.

The engine is responsible for:

* Holding the simulation state (warehouse, robots, tick counter).
* Advancing the simulation by one discrete tick.
* Reporting whether the simulation is finished.

It does **not** know about rendering.  This separation is critical:
it lets us later run thousands of headless benchmark simulations
without importing Pygame.
"""

from __future__ import annotations

from swarmos.robot.amr import AMR
from swarmos.simulation.config import SimulationConfig
from swarmos.warehouse.cell import Position
from swarmos.warehouse.grid import Grid


class SimulationEngine:
    """Manages the discrete-time simulation loop.

    Parameters:
        grid:   The warehouse grid.
        robot:  The AMR to simulate.
        config: Engine configuration.
    """

    __slots__ = ("_grid", "_robot", "_config", "_tick", "_finished")

    def __init__(
        self,
        grid: Grid,
        robot: AMR,
        config: SimulationConfig | None = None,
    ) -> None:
        self._grid = grid
        self._robot = robot
        self._config = config or SimulationConfig()
        self._tick: int = 0
        self._finished: bool = False

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def grid(self) -> Grid:
        return self._grid

    @property
    def robot(self) -> AMR:
        return self._robot

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

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------

    def plan_robot(self, goal: Position) -> bool:
        """Instruct the robot to plan a path to *goal*.

        Returns ``True`` if planning succeeded.
        """
        return self._robot.plan(self._grid, goal)

    # ------------------------------------------------------------------
    # Simulation loop
    # ------------------------------------------------------------------

    def update(self) -> None:
        """Advance the simulation by one tick.

        The robot moves one waypoint every ``config.robot_step_delay``
        ticks, producing visually smooth stepping.
        """
        if self._finished:
            return

        self._tick += 1

        # Move the robot at the configured step cadence.
        if self._tick % self._config.robot_step_delay == 0:
            self._robot.step()

        if self._robot.has_reached_goal:
            self._finished = True
