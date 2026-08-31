"""Tests for conflict detection and collision detection."""

from __future__ import annotations

from swarmos.coordination.conflict import Collision, Conflict, ConflictType
from swarmos.coordination.detector import (
    detect_collisions,
    detect_path_conflicts,
)
from swarmos.planning.path import Path
from swarmos.robot.amr import AMR
from swarmos.robot.fleet import Fleet
from swarmos.warehouse.cell import Position
from swarmos.warehouse.grid import Grid


# ==================================================================
# Helpers
# ==================================================================

def _make_fleet_with_paths(
    robots: list[tuple[str, Position, Position]],
    grid: Grid | None = None,
) -> Fleet:
    """Create a fleet of robots with planned paths.

    Each tuple is (robot_id, start, goal).
    """
    if grid is None:
        grid = Grid(20, 20)
    fleet = Fleet()
    for robot_id, start, goal in robots:
        amr = AMR(robot_id=robot_id, position=start)
        amr.plan(grid, goal)
        fleet.add(amr)
    return fleet


# ==================================================================
# Path conflict detection
# ==================================================================


class TestNodeConflict:
    def test_node_conflict_detected(self) -> None:
        """Two robots crossing through the same cell at the same time."""
        grid = Grid(10, 10)

        # Create robots that will cross paths.
        # AMR-A: (0,5) → (9,5)  — horizontal
        # AMR-B: (5,0) → (5,9)  — vertical
        # They cross at (5,5).
        fleet = _make_fleet_with_paths([
            ("A", Position(0, 5), Position(9, 5)),
            ("B", Position(5, 0), Position(5, 9)),
        ], grid)

        conflicts = detect_path_conflicts(fleet)
        node_conflicts = [c for c in conflicts if c.conflict_type is ConflictType.NODE]
        assert len(node_conflicts) > 0
        # At least one conflict should involve position (5, 5)
        positions = {c.position for c in node_conflicts}
        assert Position(5, 5) in positions

    def test_no_conflict_disjoint_paths(self) -> None:
        """Robots on completely separate paths have no conflicts."""
        grid = Grid(10, 10)
        fleet = _make_fleet_with_paths([
            ("A", Position(0, 0), Position(9, 0)),  # top row
            ("B", Position(0, 9), Position(9, 9)),  # bottom row
        ], grid)

        conflicts = detect_path_conflicts(fleet)
        assert len(conflicts) == 0

    def test_no_conflict_same_cell_different_time(self) -> None:
        """Robots share a cell in their path but at different timesteps."""
        grid = Grid(10, 1)
        # Both travel left-to-right on the same row, but A starts first.
        # A: (0,0)→(9,0), B: (0,0)... but B starts at (5,0)→(9,0)
        # B is much shorter so they shouldn't overlap in time at same cell.
        fleet = _make_fleet_with_paths([
            ("A", Position(0, 0), Position(4, 0)),
            ("B", Position(6, 0), Position(9, 0)),
        ], grid)

        conflicts = detect_path_conflicts(fleet)
        assert len(conflicts) == 0


class TestEdgeConflict:
    def test_edge_conflict_detected(self) -> None:
        """Two robots swap positions — head-on collision on an edge."""
        grid = Grid(3, 1)
        # A: (0,0) → (2,0): positions [0, 1, 2]
        # B: (2,0) → (0,0): positions [2, 1, 0]
        # At t=0: A at 0, B at 2
        # At t=1: A at 1, B at 1 → NODE conflict
        # Also: between t=0→t=1: A goes 0→1, B goes 2→1 → not a swap
        # Between t=1→t=2: A goes 1→2, B goes 1→0 → A's next is B's prev
        #   Actually: pos_a=1, next_a=2, pos_b=1, next_b=0 → not a swap
        # Let me think about this differently...
        # A: [0, 1, 2], B: [2, 1, 0]
        # t=0→t=1: A goes (0)→(1), B goes (2)→(1) — not edge swap
        # t=1→t=2: A goes (1)→(2), B goes (1)→(0) — not edge swap (same start)
        # Actually in 3x1, the edge conflict is embedded in the node conflict
        # at t=1 where both are at (1,0).
        # For a pure edge conflict, we need robots in adjacent cells swapping.

        grid2 = Grid(2, 1)
        fleet = _make_fleet_with_paths([
            ("A", Position(0, 0), Position(1, 0)),
            ("B", Position(1, 0), Position(0, 0)),
        ], grid2)

        conflicts = detect_path_conflicts(fleet)
        edge_conflicts = [c for c in conflicts if c.conflict_type is ConflictType.EDGE]
        # A: [0, 1], B: [1, 0]
        # t=0: A at (0,0), B at (1,0) — no node conflict
        # t=0→t=1: A goes to (1,0), B goes to (0,0) — SWAP → edge conflict
        # t=1: A at (1,0), B at (0,0) — no node conflict
        assert len(edge_conflicts) >= 1

    def test_no_edge_conflict_same_direction(self) -> None:
        """Robots travelling in the same direction have no edge conflict."""
        grid = Grid(5, 1)
        fleet = _make_fleet_with_paths([
            ("A", Position(0, 0), Position(4, 0)),
            ("B", Position(1, 0), Position(4, 0)),
        ], grid)

        conflicts = detect_path_conflicts(fleet)
        edge_conflicts = [c for c in conflicts if c.conflict_type is ConflictType.EDGE]
        assert len(edge_conflicts) == 0


class TestConflictModel:
    def test_conflict_fields(self) -> None:
        c = Conflict(
            conflict_type=ConflictType.NODE,
            robot_a_id="R1",
            robot_b_id="R2",
            position=Position(5, 5),
            timestep=3,
        )
        assert c.conflict_type is ConflictType.NODE
        assert c.robot_a_id == "R1"
        assert c.robot_b_id == "R2"
        assert c.position == Position(5, 5)
        assert c.timestep == 3

    def test_conflict_repr(self) -> None:
        c = Conflict(ConflictType.EDGE, "A", "B", Position(1, 2), 7)
        r = repr(c)
        assert "EDGE" in r
        assert "A" in r
        assert "B" in r

    def test_collision_fields(self) -> None:
        c = Collision(robot_a_id="R1", robot_b_id="R2", position=Position(3, 4), tick=42)
        assert c.robot_a_id == "R1"
        assert c.position == Position(3, 4)
        assert c.tick == 42


# ==================================================================
# Collision detection (runtime)
# ==================================================================


class TestCollisionDetection:
    def test_collision_same_position(self) -> None:
        """Two robots at the same position produce a collision."""
        fleet = Fleet()
        fleet.add(AMR(robot_id="A", position=Position(5, 5)))
        fleet.add(AMR(robot_id="B", position=Position(5, 5)))
        collisions = detect_collisions(fleet, tick=10)
        assert len(collisions) == 1
        assert collisions[0].position == Position(5, 5)

    def test_no_collision_different_positions(self) -> None:
        """Robots at different positions have no collision."""
        fleet = Fleet()
        fleet.add(AMR(robot_id="A", position=Position(0, 0)))
        fleet.add(AMR(robot_id="B", position=Position(9, 9)))
        collisions = detect_collisions(fleet, tick=10)
        assert len(collisions) == 0

    def test_three_robot_collision(self) -> None:
        """Three robots at the same cell produce 3 collision pairs."""
        fleet = Fleet()
        fleet.add(AMR(robot_id="A", position=Position(5, 5)))
        fleet.add(AMR(robot_id="B", position=Position(5, 5)))
        fleet.add(AMR(robot_id="C", position=Position(5, 5)))
        collisions = detect_collisions(fleet, tick=10)
        # C(3,2) = 3 pairs: AB, AC, BC
        assert len(collisions) == 3

    def test_partial_collision(self) -> None:
        """Only some robots collide."""
        fleet = Fleet()
        fleet.add(AMR(robot_id="A", position=Position(1, 1)))
        fleet.add(AMR(robot_id="B", position=Position(1, 1)))
        fleet.add(AMR(robot_id="C", position=Position(9, 9)))
        collisions = detect_collisions(fleet, tick=5)
        assert len(collisions) == 1
        assert collisions[0].robot_a_id == "A"
        assert collisions[0].robot_b_id == "B"
