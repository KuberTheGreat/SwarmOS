"""Integration tests for the stop-and-wait baseline.

Tests the full simulation pipeline with StopAndWaitPolicy:
- End-to-end scenario runs
- Robot state transitions (MOVING → WAITING → MOVING → ARRIVED)
- Metrics validation (wait ticks, movements, completion time)
- Deterministic reproducibility (same scenario = same results)
- Zero runtime collisions under policy
- Backward compatibility (no policy = Phase 2 behavior)
"""

from __future__ import annotations

from swarmos.coordination.detector import detect_collisions
from swarmos.coordination.policy import StopAndWaitPolicy
from swarmos.planning.path import Path
from swarmos.robot.amr import AMR
from swarmos.robot.fleet import Fleet
from swarmos.robot.state import RobotState
from swarmos.simulation.config import SimulationConfig
from swarmos.simulation.engine import SimulationEngine
from swarmos.warehouse.cell import Position
from swarmos.warehouse.grid import Grid


# ==================================================================
# Helpers
# ==================================================================

def _build_scenario(
    grid_size: tuple[int, int],
    robots: list[tuple[str, tuple[int, int], tuple[int, int]]],
    obstacles: list[tuple[int, int]] | None = None,
    step_delay: int = 1,
    use_policy: bool = True,
) -> SimulationEngine:
    """Build a simulation engine for a test scenario.

    Parameters:
        grid_size: (width, height)
        robots:    List of (id, start, goal) tuples
        obstacles: Optional list of obstacle positions
        step_delay: Ticks between movements (1 = every tick)
        use_policy: Whether to use StopAndWaitPolicy
    """
    grid = Grid(*grid_size)
    if obstacles:
        for ox, oy in obstacles:
            grid.add_obstacle(Position(ox, oy))

    fleet = Fleet()
    goals: dict[str, Position] = {}
    for robot_id, start, goal in robots:
        amr = AMR(robot_id=robot_id, position=Position(*start))
        fleet.add(amr)
        goals[robot_id] = Position(*goal)

    policy = StopAndWaitPolicy() if use_policy else None
    config = SimulationConfig(robot_step_delay=step_delay)
    engine = SimulationEngine(
        grid=grid, robots=fleet, config=config,
        policy=policy, max_ticks=5000,
    )

    # Plan all paths
    results = engine.plan_all(goals)
    assert all(results.values()), f"Not all robots could find paths: {results}"

    return engine


def _run_to_completion(engine: SimulationEngine) -> None:
    """Run the simulation until finished."""
    while not engine.is_finished:
        engine.update()


# ==================================================================
# Robot state transitions
# ==================================================================

class TestRobotStateTransitions:
    def test_wait_transitions_to_waiting(self) -> None:
        """wait() moves robot from MOVING to WAITING."""
        amr = AMR(robot_id="R1", position=Position(0, 0))
        grid = Grid(5, 1)
        amr.plan(grid, Position(4, 0))
        assert amr.state is RobotState.MOVING
        amr.wait()
        assert amr.state is RobotState.WAITING

    def test_resume_transitions_to_moving(self) -> None:
        """resume() moves robot from WAITING back to MOVING."""
        amr = AMR(robot_id="R1", position=Position(0, 0))
        grid = Grid(5, 1)
        amr.plan(grid, Position(4, 0))
        amr.wait()
        assert amr.state is RobotState.WAITING
        amr.resume()
        assert amr.state is RobotState.MOVING

    def test_wait_does_nothing_when_idle(self) -> None:
        amr = AMR(robot_id="R1", position=Position(0, 0))
        assert amr.state is RobotState.IDLE
        amr.wait()
        assert amr.state is RobotState.IDLE

    def test_wait_does_nothing_when_arrived(self) -> None:
        amr = AMR(robot_id="R1", position=Position(0, 0))
        grid = Grid(5, 1)
        amr.plan(grid, Position(0, 0))  # start == goal
        assert amr.state is RobotState.ARRIVED
        amr.wait()
        assert amr.state is RobotState.ARRIVED

    def test_resume_does_nothing_when_moving(self) -> None:
        amr = AMR(robot_id="R1", position=Position(0, 0))
        grid = Grid(5, 1)
        amr.plan(grid, Position(4, 0))
        assert amr.state is RobotState.MOVING
        amr.resume()
        assert amr.state is RobotState.MOVING

    def test_step_does_nothing_when_waiting(self) -> None:
        """step() should not advance a WAITING robot."""
        amr = AMR(robot_id="R1", position=Position(0, 0))
        grid = Grid(5, 1)
        amr.plan(grid, Position(4, 0))
        amr.wait()
        pos_before = amr.position
        amr.step()
        assert amr.position == pos_before
        assert amr.state is RobotState.WAITING

    def test_intended_next_position_moving(self) -> None:
        amr = AMR(robot_id="R1", position=Position(0, 0))
        grid = Grid(5, 1)
        amr.plan(grid, Position(4, 0))
        assert amr.intended_next_position == Position(1, 0)

    def test_intended_next_position_idle(self) -> None:
        amr = AMR(robot_id="R1", position=Position(3, 3))
        assert amr.intended_next_position == Position(3, 3)

    def test_intended_next_position_arrived(self) -> None:
        amr = AMR(robot_id="R1", position=Position(0, 0))
        grid = Grid(5, 1)
        amr.plan(grid, Position(0, 0))
        assert amr.state is RobotState.ARRIVED
        assert amr.intended_next_position == Position(0, 0)


# ==================================================================
# End-to-end scenario tests
# ==================================================================

class TestEndToEnd:
    def test_single_robot_no_conflict(self) -> None:
        """Single robot reaches goal without waiting."""
        engine = _build_scenario(
            grid_size=(10, 1),
            robots=[("AMR-01", (0, 0), (9, 0))],
        )
        _run_to_completion(engine)

        assert engine.is_finished
        assert engine.fleet.all_arrived
        assert engine.metrics.total_collisions == 0
        assert engine.metrics.total_wait_ticks == 0
        assert engine.metrics.per_robot["AMR-01"].wait_ticks == 0

    def test_crossing_robots_zero_collisions(self) -> None:
        """Two crossing robots complete with zero collisions."""
        engine = _build_scenario(
            grid_size=(10, 10),
            robots=[
                ("AMR-01", (0, 5), (9, 5)),  # horizontal
                ("AMR-02", (5, 0), (5, 9)),  # vertical
            ],
        )
        _run_to_completion(engine)

        assert engine.is_finished
        assert engine.fleet.all_arrived
        assert engine.metrics.total_collisions == 0

    def test_head_on_corridor_deadlocks_safely(self) -> None:
        """Head-on in a 1-cell corridor deadlocks but has zero collisions.

        This is an expected limitation of stop-and-wait without rerouting.
        The simulation terminates via max_ticks. The important thing is
        that no collision ever occurs — the policy prevents movement.
        """
        engine = _build_scenario(
            grid_size=(6, 3),
            robots=[
                ("AMR-01", (0, 1), (5, 1)),
                ("AMR-02", (5, 1), (0, 1)),
            ],
            obstacles=[
                (0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (5, 0),
                (0, 2), (1, 2), (2, 2), (3, 2), (4, 2), (5, 2),
            ],
        )
        _run_to_completion(engine)

        assert engine.is_finished
        # Zero collisions even in deadlock — the policy prevents them.
        assert engine.metrics.total_collisions == 0
        # Wait ticks should accumulate due to the deadlock.
        assert engine.metrics.total_wait_ticks > 0

    def test_head_on_open_grid_deadlocks_safely(self) -> None:
        """Head-on on an open grid — A* gives same-row paths, deadlock.

        Even on an open grid, A* plans both robots along the same
        optimal row.  Stop-and-wait without rerouting deadlocks here.
        The important validation is: zero collisions.
        """
        engine = _build_scenario(
            grid_size=(10, 5),
            robots=[
                ("AMR-01", (0, 2), (9, 2)),
                ("AMR-02", (9, 2), (0, 2)),
            ],
        )
        _run_to_completion(engine)

        assert engine.is_finished
        assert engine.metrics.total_collisions == 0
        # Deadlock should have caused wait ticks
        assert engine.metrics.total_wait_ticks > 0

    def test_crossing_paths_all_arrive(self) -> None:
        """Two robots whose paths cross perpendicularly both arrive."""
        engine = _build_scenario(
            grid_size=(10, 10),
            robots=[
                ("AMR-01", (0, 5), (9, 5)),   # horizontal
                ("AMR-02", (5, 0), (5, 9)),   # vertical
            ],
        )
        _run_to_completion(engine)

        assert engine.is_finished
        assert engine.fleet.all_arrived
        assert engine.metrics.total_collisions == 0
        for robot in engine.fleet:
            assert robot.has_reached_goal, f"{robot.robot_id} did not arrive"

    def test_non_conflicting_robots_all_arrive(self) -> None:
        """Robots on separate parts of the grid all arrive."""
        engine = _build_scenario(
            grid_size=(10, 10),
            robots=[
                ("AMR-01", (0, 0), (4, 0)),
                ("AMR-02", (0, 9), (4, 9)),
                ("AMR-03", (9, 0), (9, 9)),
            ],
        )
        _run_to_completion(engine)

        assert engine.is_finished
        assert engine.fleet.all_arrived
        assert engine.metrics.total_collisions == 0
        assert engine.metrics.total_wait_ticks == 0


    def test_simulation_terminates(self) -> None:
        """Simulation doesn't hang — terminates within max_ticks."""
        engine = _build_scenario(
            grid_size=(5, 5),
            robots=[
                ("AMR-01", (0, 0), (4, 4)),
                ("AMR-02", (4, 4), (0, 0)),
            ],
        )
        _run_to_completion(engine)
        assert engine.is_finished


# ==================================================================
# Metrics validation
# ==================================================================

class TestMetrics:
    def test_wait_ticks_counted(self) -> None:
        """Wait ticks are recorded when conflicts occur."""
        engine = _build_scenario(
            grid_size=(5, 1),
            robots=[
                ("AMR-01", (0, 0), (4, 0)),
                ("AMR-02", (4, 0), (0, 0)),
            ],
        )
        _run_to_completion(engine)

        # In a corridor, one robot will wait while the other passes
        assert engine.metrics.total_wait_ticks > 0

    def test_movements_counted(self) -> None:
        """Movement ticks are recorded."""
        engine = _build_scenario(
            grid_size=(5, 1),
            robots=[("AMR-01", (0, 0), (4, 0))],
        )
        _run_to_completion(engine)

        assert engine.metrics.total_movements > 0
        assert engine.metrics.per_robot["AMR-01"].steps_taken == 4

    def test_completion_time_recorded(self) -> None:
        """Per-robot completion tick is recorded."""
        engine = _build_scenario(
            grid_size=(5, 1),
            robots=[("AMR-01", (0, 0), (4, 0))],
        )
        _run_to_completion(engine)

        rm = engine.metrics.per_robot["AMR-01"]
        assert rm.completed is True
        assert rm.completion_tick is not None
        assert rm.completion_tick > 0

    def test_policy_name_in_metrics(self) -> None:
        """Policy name is recorded in metrics."""
        engine = _build_scenario(
            grid_size=(5, 1),
            robots=[("AMR-01", (0, 0), (4, 0))],
        )
        assert engine.metrics.policy_name == "STOP_AND_WAIT"

    def test_summary_includes_wait_info(self) -> None:
        """Summary string includes wait tick information."""
        engine = _build_scenario(
            grid_size=(5, 1),
            robots=[
                ("AMR-01", (0, 0), (4, 0)),
                ("AMR-02", (4, 0), (0, 0)),
            ],
        )
        _run_to_completion(engine)
        summary = engine.metrics.summary()
        assert "STOP_AND_WAIT" in summary
        assert "wait" in summary.lower()


# ==================================================================
# Deterministic reproducibility
# ==================================================================

class TestDeterminism:
    def test_identical_runs_same_metrics(self) -> None:
        """Two identical runs produce the same metrics."""
        def run_once() -> dict:
            engine = _build_scenario(
                grid_size=(10, 10),
                robots=[
                    ("AMR-01", (0, 5), (9, 5)),
                    ("AMR-02", (5, 0), (5, 9)),
                    ("AMR-03", (9, 4), (0, 4)),
                ],
            )
            _run_to_completion(engine)
            m = engine.metrics
            return {
                "total_ticks": m.total_ticks,
                "total_wait_ticks": m.total_wait_ticks,
                "total_movements": m.total_movements,
                "total_collisions": m.total_collisions,
                "per_robot_waits": {
                    rid: rm.wait_ticks for rid, rm in m.per_robot.items()
                },
                "per_robot_completion": {
                    rid: rm.completion_tick for rid, rm in m.per_robot.items()
                },
            }

        run1 = run_once()
        run2 = run_once()
        assert run1 == run2, f"Non-deterministic results:\nRun1: {run1}\nRun2: {run2}"

    def test_determinism_with_conflicts(self) -> None:
        """Determinism holds even with many conflicts."""
        def run_once() -> dict:
            engine = _build_scenario(
                grid_size=(5, 1),
                robots=[
                    ("AMR-01", (0, 0), (4, 0)),
                    ("AMR-02", (4, 0), (0, 0)),
                ],
            )
            _run_to_completion(engine)
            m = engine.metrics
            return {
                "total_ticks": m.total_ticks,
                "total_wait_ticks": m.total_wait_ticks,
                "collisions": m.total_collisions,
            }

        run1 = run_once()
        run2 = run_once()
        assert run1 == run2


# ==================================================================
# Backward compatibility
# ==================================================================

class TestBackwardCompat:
    def test_no_policy_same_as_phase2(self) -> None:
        """Without a policy, the engine behaves like Phase 2."""
        engine = _build_scenario(
            grid_size=(10, 1),
            robots=[("AMR-01", (0, 0), (9, 0))],
            use_policy=False,
        )
        _run_to_completion(engine)

        assert engine.is_finished
        assert engine.fleet.all_arrived
        assert engine.metrics.policy_name == ""
        assert engine.metrics.total_wait_ticks == 0

    def test_no_policy_does_not_use_wait(self) -> None:
        """Without a policy, robots never enter WAITING state."""
        engine = _build_scenario(
            grid_size=(5, 1),
            robots=[
                ("AMR-01", (0, 0), (4, 0)),
                ("AMR-02", (4, 0), (0, 0)),
            ],
            use_policy=False,
        )
        # Run just a few ticks and check states
        for _ in range(20):
            engine.update()
        for robot in engine.fleet:
            assert robot.state is not RobotState.WAITING
