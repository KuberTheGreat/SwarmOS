"""Tests for the simulation metrics system."""

from __future__ import annotations

from swarmos.coordination.conflict import Collision, Conflict, ConflictType
from swarmos.simulation.metrics import SimulationMetrics
from swarmos.warehouse.cell import Position


class TestSimulationMetrics:
    def test_initial_state(self) -> None:
        m = SimulationMetrics()
        assert m.total_ticks == 0
        assert m.robots_completed == 0
        assert m.total_robots == 0
        assert m.total_collisions == 0
        assert m.total_path_conflicts == 0

    def test_register_robot(self) -> None:
        m = SimulationMetrics()
        m.register_robot("R1", path_length=10)
        assert m.total_robots == 1
        assert m.per_robot["R1"].path_length == 10
        assert m.per_robot["R1"].steps_taken == 0
        assert m.per_robot["R1"].completed is False

    def test_record_step(self) -> None:
        m = SimulationMetrics()
        m.register_robot("R1", path_length=5)
        m.record_step("R1")
        m.record_step("R1")
        assert m.per_robot["R1"].steps_taken == 2

    def test_record_arrival(self) -> None:
        m = SimulationMetrics()
        m.register_robot("R1", path_length=5)
        m.record_arrival("R1", tick=42)
        assert m.per_robot["R1"].completed is True
        assert m.per_robot["R1"].completion_tick == 42
        assert m.robots_completed == 1

    def test_duplicate_arrival_not_double_counted(self) -> None:
        m = SimulationMetrics()
        m.register_robot("R1", path_length=5)
        m.record_arrival("R1", tick=42)
        m.record_arrival("R1", tick=50)  # should not double-count
        assert m.robots_completed == 1
        assert m.per_robot["R1"].completion_tick == 42

    def test_record_collisions(self) -> None:
        m = SimulationMetrics()
        c1 = Collision("A", "B", Position(5, 5), tick=10)
        c2 = Collision("A", "C", Position(3, 3), tick=20)
        m.record_collisions([c1, c2])
        assert m.total_collisions == 2
        assert len(m.collisions) == 2

    def test_record_path_conflicts(self) -> None:
        m = SimulationMetrics()
        conflicts = [
            Conflict(ConflictType.NODE, "A", "B", Position(5, 5), timestep=3),
            Conflict(ConflictType.EDGE, "A", "C", Position(2, 2), timestep=7),
        ]
        m.record_path_conflicts(conflicts)
        assert m.total_path_conflicts == 2
        assert len(m.path_conflicts) == 2

    def test_summary_contains_key_info(self) -> None:
        m = SimulationMetrics()
        m.register_robot("R1", path_length=10)
        m.register_robot("R2", path_length=8)
        m.record_arrival("R1", tick=100)
        m.set_total_ticks(150)
        summary = m.summary()
        assert "150" in summary
        assert "R1" in summary
        assert "R2" in summary
        assert "1/2" in summary

    def test_multiple_robots_full_lifecycle(self) -> None:
        m = SimulationMetrics()
        m.register_robot("R1", path_length=5)
        m.register_robot("R2", path_length=8)
        m.register_robot("R3", path_length=3)

        for _ in range(5):
            m.record_step("R1")
        m.record_arrival("R1", tick=50)

        for _ in range(8):
            m.record_step("R2")
        m.record_arrival("R2", tick=80)

        for _ in range(3):
            m.record_step("R3")
        m.record_arrival("R3", tick=30)

        m.set_total_ticks(80)

        assert m.robots_completed == 3
        assert m.total_robots == 3
        assert m.per_robot["R1"].steps_taken == 5
        assert m.per_robot["R2"].steps_taken == 8
        assert m.per_robot["R3"].completion_tick == 30
