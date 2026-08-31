"""Scenario loader — constructs a warehouse grid from configuration data.

Scenarios are stored as JSON files in the ``scenarios/`` directory.
This module parses those files and produces fully-initialised
:class:`~swarmos.warehouse.grid.Grid` instances together with
robot start/goal metadata.

The loader is deliberately separated from Grid so that new scenario
formats (e.g. YAML, procedural generation) can be added later without
touching the grid implementation.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from swarmos.warehouse.cell import Position
from swarmos.warehouse.grid import Grid


@dataclass(frozen=True)
class ScenarioData:
    """Parsed scenario — all data needed to initialise a simulation run.

    Attributes:
        grid:       The constructed warehouse grid.
        robot_start: Starting position for the AMR.
        robot_goal:  Goal position for the AMR.
    """

    grid: Grid
    robot_start: Position
    robot_goal: Position


def load_scenario(path: str | Path) -> ScenarioData:
    """Load a scenario from a JSON file.

    Expected JSON schema::

        {
          "width": int,
          "height": int,
          "obstacles": [[x, y], ...],
          "robot": {
            "start": [x, y],
            "goal":  [x, y]
          }
        }

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
        grid.add_obstacle(pos)

    # --- robot ---
    robot_data = data.get("robot")
    if not isinstance(robot_data, dict):
        raise ValueError("'robot' key must be a dict with 'start' and 'goal'")

    robot_start = _parse_position(robot_data.get("start"), "robot.start")
    robot_goal = _parse_position(robot_data.get("goal"), "robot.goal")

    return ScenarioData(grid=grid, robot_start=robot_start, robot_goal=robot_goal)


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
