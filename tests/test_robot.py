"""Tests for the AMR robot model."""

from __future__ import annotations

from swarmos.planning.path import Path
from swarmos.robot.amr import AMR
from swarmos.robot.state import RobotState
from swarmos.warehouse.cell import Position
from swarmos.warehouse.grid import Grid


class TestAMRInitialisation:
    def test_initial_state(self) -> None:
        amr = AMR(robot_id="r1", position=Position(0, 0))
        assert amr.robot_id == "r1"
        assert amr.position == Position(0, 0)
        assert amr.state is RobotState.IDLE
        assert amr.goal is None
        assert amr.path is None
        assert amr.has_reached_goal is False

    def test_repr(self) -> None:
        amr = AMR(robot_id="r1", position=Position(3, 4))
        r = repr(amr)
        assert "r1" in r
        assert "IDLE" in r


class TestAMRPlanning:
    def test_successful_planning(self) -> None:
        grid = Grid(5, 5)
        amr = AMR(robot_id="r1", position=Position(0, 0))
        result = amr.plan(grid, Position(4, 4))
        assert result is True
        assert amr.state is RobotState.MOVING
        assert amr.path is not None
        assert amr.goal == Position(4, 4)

    def test_plan_unreachable_goal(self) -> None:
        grid = Grid(5, 5)
        # Wall off the goal
        grid.add_obstacle(Position(3, 4))
        grid.add_obstacle(Position(4, 3))
        amr = AMR(robot_id="r1", position=Position(0, 0))
        result = amr.plan(grid, Position(4, 4))
        assert result is False
        assert amr.state is RobotState.IDLE
        assert amr.path is None

    def test_plan_start_equals_goal(self) -> None:
        grid = Grid(5, 5)
        amr = AMR(robot_id="r1", position=Position(2, 2))
        result = amr.plan(grid, Position(2, 2))
        assert result is True
        assert amr.state is RobotState.ARRIVED


class TestAMRMovement:
    def test_step_advances_position(self) -> None:
        grid = Grid(5, 1)
        amr = AMR(robot_id="r1", position=Position(0, 0))
        amr.plan(grid, Position(4, 0))
        assert amr.state is RobotState.MOVING

        amr.step()
        assert amr.position == Position(1, 0)

        amr.step()
        assert amr.position == Position(2, 0)

    def test_full_path_traversal(self) -> None:
        grid = Grid(5, 1)
        amr = AMR(robot_id="r1", position=Position(0, 0))
        amr.plan(grid, Position(4, 0))

        while amr.state is RobotState.MOVING:
            amr.step()

        assert amr.position == Position(4, 0)
        assert amr.state is RobotState.ARRIVED
        assert amr.has_reached_goal is True

    def test_step_does_nothing_when_idle(self) -> None:
        amr = AMR(robot_id="r1", position=Position(0, 0))
        amr.step()
        assert amr.position == Position(0, 0)
        assert amr.state is RobotState.IDLE

    def test_step_does_nothing_when_arrived(self) -> None:
        grid = Grid(5, 5)
        amr = AMR(robot_id="r1", position=Position(2, 2))
        amr.plan(grid, Position(2, 2))
        assert amr.state is RobotState.ARRIVED
        amr.step()
        assert amr.position == Position(2, 2)
        assert amr.state is RobotState.ARRIVED

    def test_movement_with_obstacles(self) -> None:
        grid = Grid(5, 3)
        grid.add_obstacle(Position(2, 1))
        amr = AMR(robot_id="r1", position=Position(0, 1))
        amr.plan(grid, Position(4, 1))

        while amr.state is RobotState.MOVING:
            amr.step()

        assert amr.position == Position(4, 1)
        assert amr.has_reached_goal is True


class TestPath:
    def test_path_cursor(self) -> None:
        path = Path([Position(0, 0), Position(1, 0), Position(2, 0)])
        assert path.current == Position(0, 0)
        assert path.peek_next() == Position(1, 0)
        assert not path.is_complete

    def test_path_advance(self) -> None:
        path = Path([Position(0, 0), Position(1, 0), Position(2, 0)])
        next_pos = path.advance()
        assert next_pos == Position(1, 0)
        assert path.current == Position(1, 0)

    def test_path_completion(self) -> None:
        path = Path([Position(0, 0), Position(1, 0)])
        path.advance()
        assert path.is_complete is True
        assert path.advance() is None

    def test_path_remaining(self) -> None:
        path = Path([Position(0, 0), Position(1, 0), Position(2, 0)])
        path.advance()
        remaining = path.remaining
        assert remaining == [Position(1, 0), Position(2, 0)]

    def test_path_reset(self) -> None:
        path = Path([Position(0, 0), Position(1, 0)])
        path.advance()
        path.reset()
        assert path.current == Position(0, 0)
        assert not path.is_complete

    def test_single_waypoint_path(self) -> None:
        path = Path([Position(5, 5)])
        assert path.is_complete is True
        assert path.start == path.goal == Position(5, 5)
