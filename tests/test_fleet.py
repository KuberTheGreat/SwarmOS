"""Tests for the Fleet container."""

from __future__ import annotations

import pytest

from swarmos.robot.amr import AMR
from swarmos.robot.fleet import Fleet
from swarmos.robot.state import RobotState
from swarmos.warehouse.cell import Position
from swarmos.warehouse.grid import Grid


class TestFleetBasics:
    def test_empty_fleet(self) -> None:
        fleet = Fleet()
        assert len(fleet) == 0
        assert fleet.robot_ids == []

    def test_add_robot(self) -> None:
        fleet = Fleet()
        amr = AMR(robot_id="r1", position=Position(0, 0))
        fleet.add(amr)
        assert len(fleet) == 1
        assert "r1" in fleet

    def test_add_multiple_robots(self) -> None:
        fleet = Fleet()
        fleet.add(AMR(robot_id="r1", position=Position(0, 0)))
        fleet.add(AMR(robot_id="r2", position=Position(1, 0)))
        fleet.add(AMR(robot_id="r3", position=Position(2, 0)))
        assert len(fleet) == 3
        assert fleet.robot_ids == ["r1", "r2", "r3"]

    def test_duplicate_id_rejected(self) -> None:
        fleet = Fleet()
        fleet.add(AMR(robot_id="r1", position=Position(0, 0)))
        with pytest.raises(ValueError, match="Duplicate"):
            fleet.add(AMR(robot_id="r1", position=Position(1, 0)))

    def test_get_robot(self) -> None:
        fleet = Fleet()
        amr = AMR(robot_id="r1", position=Position(3, 4))
        fleet.add(amr)
        retrieved = fleet.get("r1")
        assert retrieved is amr
        assert retrieved.position == Position(3, 4)

    def test_get_missing_robot_raises(self) -> None:
        fleet = Fleet()
        with pytest.raises(KeyError):
            fleet.get("nonexistent")

    def test_remove_robot(self) -> None:
        fleet = Fleet()
        fleet.add(AMR(robot_id="r1", position=Position(0, 0)))
        removed = fleet.remove("r1")
        assert removed.robot_id == "r1"
        assert len(fleet) == 0
        assert "r1" not in fleet

    def test_remove_missing_robot_raises(self) -> None:
        fleet = Fleet()
        with pytest.raises(KeyError):
            fleet.remove("nonexistent")

    def test_iteration_order(self) -> None:
        fleet = Fleet()
        fleet.add(AMR(robot_id="c", position=Position(0, 0)))
        fleet.add(AMR(robot_id="a", position=Position(1, 0)))
        fleet.add(AMR(robot_id="b", position=Position(2, 0)))
        ids = [r.robot_id for r in fleet]
        assert ids == ["c", "a", "b"]  # insertion order

    def test_contains(self) -> None:
        fleet = Fleet()
        fleet.add(AMR(robot_id="r1", position=Position(0, 0)))
        assert "r1" in fleet
        assert "r2" not in fleet


class TestFleetStatus:
    def test_all_arrived_empty_fleet(self) -> None:
        fleet = Fleet()
        assert fleet.all_arrived is True  # vacuously true

    def test_all_arrived_none_arrived(self) -> None:
        fleet = Fleet()
        fleet.add(AMR(robot_id="r1", position=Position(0, 0)))
        assert fleet.all_arrived is False

    def test_all_arrived_all_arrived(self) -> None:
        grid = Grid(5, 5)
        fleet = Fleet()
        r1 = AMR(robot_id="r1", position=Position(2, 2))
        r1.plan(grid, Position(2, 2))  # start == goal → ARRIVED
        fleet.add(r1)
        assert fleet.all_arrived is True

    def test_active_and_arrived_robots(self) -> None:
        grid = Grid(5, 5)
        fleet = Fleet()

        r1 = AMR(robot_id="r1", position=Position(2, 2))
        r1.plan(grid, Position(2, 2))  # ARRIVED
        fleet.add(r1)

        r2 = AMR(robot_id="r2", position=Position(0, 0))
        r2.plan(grid, Position(4, 4))  # MOVING
        fleet.add(r2)

        assert len(fleet.active_robots) == 1
        assert fleet.active_robots[0].robot_id == "r2"
        assert len(fleet.arrived_robots) == 1
        assert fleet.arrived_robots[0].robot_id == "r1"

    def test_repr(self) -> None:
        fleet = Fleet()
        fleet.add(AMR(robot_id="r1", position=Position(0, 0)))
        r = repr(fleet)
        assert "r1" in r
        assert "Fleet" in r
