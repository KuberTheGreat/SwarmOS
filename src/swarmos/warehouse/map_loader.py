"""Scenario loader — constructs a warehouse grid from configuration data.

Scenarios are stored as JSON files in the ``scenarios/`` directory.
This module parses those files and produces fully-initialised
:class:`~swarmos.warehouse.grid.Grid` instances together with
robot start/goal metadata.

The loader is deliberately separated from Grid so that new scenario
formats (e.g. YAML, procedural generation) can be added later without
touching the grid implementation.

Supports two scenario formats:

Phase 1 (single robot, backwards-compatible)::

    {
      "robot": {"start": [x, y], "goal": [x, y]}
    }

Phase 2 (multiple robots)::

    {
      "robots": [
        {"id": "AMR-01", "start": [x, y], "goal": [x, y]},
        ...
      ]
    }
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from swarmos.warehouse.cell import Position
from swarmos.warehouse.grid import Grid


@dataclass(frozen=True)
class RobotSpec:
    """Specification for a single robot within a scenario.

    Attributes:
        robot_id: Unique identifier.
        start:    Starting position.
        goal:     Goal position.
    """

    robot_id: str
    start: Position
    goal: Position


@dataclass(frozen=True)
class ScenarioData:
    """Parsed scenario — all data needed to initialise a simulation run.

    Attributes:
        grid:        The constructed warehouse grid.
        robot_specs: List of robot specifications (id, start, goal).

    Backwards-compatible properties ``robot_start`` and ``robot_goal``
    are provided for Phase 1 single-robot usage.
    """

    grid: Grid
    robot_specs: list[RobotSpec]

    # --- Phase 1 backwards compatibility ---

    @property
    def robot_start(self) -> Position:
        """Start position of the first (or only) robot."""
        return self.robot_specs[0].start

    @property
    def robot_goal(self) -> Position:
        """Goal position of the first (or only) robot."""
        return self.robot_specs[0].goal


def load_scenario(path: str | Path) -> ScenarioData:
    """Load a scenario from a JSON file.

    Supports both Phase 1 (``"robot"``) and Phase 2 (``"robots"``)
    scenario formats.

    Raises:
        FileNotFoundError: If *path* does not exist.
        ValueError:        If the data is structurally invalid.
    """
    file_path = Path(path)
    if not file_path.exists():
        raise FileNotFoundError(f"Scenario file not found: {file_path}")

    with file_path.open() as fh:
        data: dict[str, Any] = json.load(fh)

    return _parse_scenario(data)


def _parse_scenario(data: dict[str, Any]) -> ScenarioData:
    """Parse raw JSON data into a :class:`ScenarioData`."""
    width = _require_int(data, "width")
    height = _require_int(data, "height")

    grid = Grid(width, height)

    # --- obstacles ---
    raw_obstacles = data.get("obstacles", [])
    if not isinstance(raw_obstacles, list):
        raise ValueError("'obstacles' must be a list of [x, y] pairs")
    for item in raw_obstacles:
        pos = _parse_position(item, "obstacle")
        if not grid.in_bounds(pos):
            raise ValueError(f"Obstacle {pos} is out of bounds for {width}×{height} grid")
        grid.add_obstacle(pos)

    # --- robots ---
    robot_specs = _parse_robots(data, grid)

    return ScenarioData(grid=grid, robot_specs=robot_specs)


def _parse_robots(data: dict[str, Any], grid: Grid) -> list[RobotSpec]:
    """Parse robot specifications from either format."""
    # Phase 2 format: "robots" list
    if "robots" in data:
        return _parse_multi_robots(data["robots"], grid)

    # Phase 1 format: single "robot" dict
    if "robot" in data:
        return _parse_single_robot(data["robot"], grid)

    raise ValueError("Scenario must contain either 'robot' or 'robots' key")


def _parse_single_robot(robot_data: Any, grid: Grid) -> list[RobotSpec]:
    """Parse Phase 1 single-robot format."""
    if not isinstance(robot_data, dict):
        raise ValueError("'robot' key must be a dict with 'start' and 'goal'")

    start = _parse_position(robot_data.get("start"), "robot.start")
    goal = _parse_position(robot_data.get("goal"), "robot.goal")

    _validate_robot_position(grid, start, "robot.start")
    _validate_robot_position(grid, goal, "robot.goal")

    return [RobotSpec(robot_id="AMR-01", start=start, goal=goal)]


def _parse_multi_robots(robots_data: Any, grid: Grid) -> list[RobotSpec]:
    """Parse Phase 2 multi-robot format with full validation."""
    if not isinstance(robots_data, list) or len(robots_data) == 0:
        raise ValueError("'robots' must be a non-empty list")

    specs: list[RobotSpec] = []
    seen_ids: set[str] = set()
    seen_starts: dict[str, str] = {}  # position_key -> robot_id

    for i, entry in enumerate(robots_data):
        if not isinstance(entry, dict):
            raise ValueError(f"robots[{i}] must be a dict")

        # --- id ---
        robot_id = entry.get("id")
        if not isinstance(robot_id, str) or not robot_id.strip():
            raise ValueError(f"robots[{i}].id must be a non-empty string")

        if robot_id in seen_ids:
            raise ValueError(f"Duplicate robot ID: {robot_id!r}")
        seen_ids.add(robot_id)

        # --- start ---
        start = _parse_position(entry.get("start"), f"robots[{i}].start")
        _validate_robot_position(grid, start, f"{robot_id}.start")

        start_key = f"{start.x},{start.y}"
        if start_key in seen_starts:
            raise ValueError(
                f"Robots {seen_starts[start_key]!r} and {robot_id!r} "
                f"share the same start position {start}"
            )
        seen_starts[start_key] = robot_id

        # --- goal ---
        goal = _parse_position(entry.get("goal"), f"robots[{i}].goal")
        _validate_robot_position(grid, goal, f"{robot_id}.goal")

        specs.append(RobotSpec(robot_id=robot_id, start=start, goal=goal))

    return specs


# ------------------------------------------------------------------
# Internal helpers
# ------------------------------------------------------------------

def _require_int(data: dict[str, Any], key: str) -> int:
    """Extract a required integer field from *data*."""
    value = data.get(key)
    if not isinstance(value, int):
        raise ValueError(f"'{key}' must be an integer, got {type(value).__name__}")
    return value


def _parse_position(raw: Any, label: str) -> Position:
    """Convert a ``[x, y]`` JSON array into a :class:`Position`."""
    if not isinstance(raw, list) or len(raw) != 2:
        raise ValueError(f"'{label}' must be a [x, y] list, got {raw!r}")
    x, y = raw
    if not isinstance(x, int) or not isinstance(y, int):
        raise ValueError(f"'{label}' coordinates must be integers, got {raw!r}")
    return Position(x, y)


def _validate_robot_position(grid: Grid, pos: Position, label: str) -> None:
    """Validate that a robot position is in-bounds and traversable."""
    if not grid.in_bounds(pos):
        raise ValueError(
            f"{label} position {pos} is out of bounds "
            f"for {grid.width}×{grid.height} grid"
        )
    if not grid.is_traversable(pos):
        raise ValueError(f"{label} position {pos} is on an obstacle")
