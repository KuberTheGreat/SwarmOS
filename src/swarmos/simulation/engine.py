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

DESIGN NOTE on update ordering (Phase 3):
    When a CoordinationPolicy is active, the engine uses snapshot-based
    simultaneous updates:
      1. Build a snapshot of every robot's state and intended next move.
      2. Pass the snapshot list to the policy's decide() method.
      3. Apply only approved movements — robots told to WAIT stay put.
      4. Record metrics (moves, waits, arrivals, collisions).
    This ensures that iteration order does NOT affect the outcome.

    When no policy is set (Phase 1/2 backward compatibility), all
    robots step unconditionally in insertion order.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from swarmos.coordination.decision import Decision, RobotSnapshot
from swarmos.coordination.detector import detect_collisions, detect_path_conflicts
from swarmos.robot.amr import AMR
from swarmos.robot.fleet import Fleet
from swarmos.robot.state import RobotState
from swarmos.simulation.config import SimulationConfig
from swarmos.simulation.metrics import SimulationMetrics
from swarmos.warehouse.cell import Position
from swarmos.warehouse.grid import Grid

if TYPE_CHECKING:
    from swarmos.coordination.policy import CoordinationPolicy


class SimulationEngine:
    """Manages the discrete-time simulation loop.

    Parameters:
        grid:   The warehouse grid.
        robots: Either a single AMR (Phase 1 compat) or a Fleet.
        config: Engine configuration.
        policy: Optional coordination policy (Phase 3+).
    """

    __slots__ = (
        "_grid", "_fleet", "_config", "_tick", "_finished",
        "_metrics", "_policy", "_max_ticks",
        "_consecutive_idle_ticks", "_deadlocked", "_last_total_steps",
        "_pre_move_positions",
    )

    def __init__(
        self,
        grid: Grid,
        robots: AMR | Fleet,
        config: SimulationConfig | None = None,
        policy: CoordinationPolicy | None = None,
        max_ticks: int = 50_000,
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
        self._policy = policy
        self._max_ticks = max_ticks
        self._consecutive_idle_ticks: int = 0
        self._deadlocked: bool = False
        self._last_total_steps: int = -1
        self._pre_move_positions: dict[str, Position] = {}

        if policy is not None:
            self._metrics.policy_name = policy.name

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
    def deadlocked(self) -> bool:
        """True if the simulation ended due to deadlock."""
        return self._deadlocked

    @property
    def metrics(self) -> SimulationMetrics:
        return self._metrics

    @property
    def policy(self) -> CoordinationPolicy | None:
        return self._policy

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

        When a CoordinationPolicy is active, movement decisions are
        made via snapshot-based simultaneous evaluation.  Otherwise
        all robots step unconditionally (Phase 1/2 compat).
        """
        if self._finished:
            return

        self._tick += 1

        # Safety limit — prevent infinite loops.
        if self._tick >= self._max_ticks:
            self._finished = True
            self._metrics.set_total_ticks(self._tick)
            return

        # Move all robots at the configured step cadence.
        if self._tick % self._config.robot_step_delay == 0:
            # Capture pre-move positions for edge-swap detection.
            self._pre_move_positions = {
                r.robot_id: r.position for r in self._fleet
            }

            if self._policy is not None:
                self._update_with_policy()
            else:
                self._update_without_policy()

            # Detect collisions (node + edge swap) after movement.
            collisions = detect_collisions(
                self._fleet, self._tick, self._pre_move_positions,
            )
            if collisions:
                self._metrics.record_collisions(collisions)

            # Deadlock detection: if no robot moved for enough
            # consecutive *movement* ticks, declare deadlock and stop.
            # This must be inside the step-delay block so that
            # non-movement ticks are not counted as idle.
            if self._policy is not None and not self._finished:
                active = sum(
                    1 for r in self._fleet
                    if r.state in (RobotState.MOVING, RobotState.WAITING)
                )
                if active > 0:
                    tick_movements = sum(
                        self._metrics.per_robot[r.robot_id].steps_taken
                        for r in self._fleet
                        if r.robot_id in self._metrics.per_robot
                    )
                    if self._last_total_steps >= 0:
                        if tick_movements == self._last_total_steps:
                            self._consecutive_idle_ticks += 1
                        else:
                            self._consecutive_idle_ticks = 0
                    self._last_total_steps = tick_movements

                    deadlock_threshold = max(active * 2, 4)
                    if self._consecutive_idle_ticks >= deadlock_threshold:
                        self._deadlocked = True
                        self._finished = True
                        self._metrics.set_total_ticks(self._tick)

        # Simulation ends when all robots have arrived.
        if not self._finished and self._fleet.all_arrived:
            self._finished = True
            self._metrics.set_total_ticks(self._tick)

    # ------------------------------------------------------------------
    # Phase 1/2 backward-compatible update (no policy)
    # ------------------------------------------------------------------

    def _update_without_policy(self) -> None:
        """Step all robots unconditionally (original Phase 2 behavior)."""
        for robot in self._fleet:
            was_moving = not robot.has_reached_goal
            robot.step()
            if was_moving:
                self._metrics.record_step(robot.robot_id)
            if robot.has_reached_goal and was_moving:
                self._metrics.record_arrival(robot.robot_id, self._tick)

    # ------------------------------------------------------------------
    # Phase 3+ snapshot-based update (with policy)
    # ------------------------------------------------------------------

    def _update_with_policy(self) -> None:
        """Snapshot → decide → apply → record.

        1. Build a snapshot of all robots' states and intended moves.
        2. Pass to the policy for simultaneous decision-making.
        3. Apply approved movements.
        4. Record metrics.
        """
        assert self._policy is not None

        # 1. Snapshot
        snapshots: list[RobotSnapshot] = []
        for robot in self._fleet:
            snapshots.append(RobotSnapshot(
                robot_id=robot.robot_id,
                position=robot.position,
                intended_next=robot.intended_next_position,
                state=robot.state,
            ))

        # 2. Decide
        decisions = self._policy.decide(snapshots)

        # 3. Apply
        for robot in self._fleet:
            decision = decisions.get(robot.robot_id, Decision.MOVE)
            was_active = robot.state in (RobotState.MOVING, RobotState.WAITING)

            if not was_active:
                continue

            if decision is Decision.WAIT:
                robot.wait()
                self._metrics.record_wait(robot.robot_id)
            else:
                # Ensure the robot is in MOVING state before stepping.
                robot.resume()
                was_at_goal = robot.has_reached_goal
                robot.step()
                self._metrics.record_step(robot.robot_id)
                self._metrics.record_movement(robot.robot_id)
                if robot.has_reached_goal and not was_at_goal:
                    self._metrics.record_arrival(robot.robot_id, self._tick)

