"""Tests for the scenario loader — both Phase 1 and Phase 2 formats."""

from __future__ import annotations

import json
import tempfile
from pathlib import Path

import pytest

from swarmos.warehouse.map_loader import load_scenario, ScenarioData


def _write_scenario(data: dict, tmp_path: Path) -> Path:
    """Write a scenario dict to a temp JSON file and return its path."""
    path = tmp_path / "test_scenario.json"
    path.write_text(json.dumps(data))
    return path


class TestPhase1Compat:
    """Verify the Phase 1 single-robot format still works."""

    def test_load_single_robot(self, tmp_path: Path) -> None:
        data = {
            "width": 10, "height": 10,
            "obstacles": [[5, 5]],
            "robot": {"start": [0, 0], "goal": [9, 9]},
        }
        scenario = load_scenario(_write_scenario(data, tmp_path))
        assert len(scenario.robot_specs) == 1
        assert scenario.robot_specs[0].robot_id == "AMR-01"
        assert scenario.robot_start == scenario.robot_specs[0].start
        assert scenario.robot_goal == scenario.robot_specs[0].goal

    def test_backward_compat_properties(self, tmp_path: Path) -> None:
        data = {
            "width": 5, "height": 5,
            "robot": {"start": [1, 1], "goal": [3, 3]},
        }
        scenario = load_scenario(_write_scenario(data, tmp_path))
        from swarmos.warehouse.cell import Position
        assert scenario.robot_start == Position(1, 1)
        assert scenario.robot_goal == Position(3, 3)


class TestMultiRobot:
    """Verify the Phase 2 multi-robot format."""

    def test_load_multiple_robots(self, tmp_path: Path) -> None:
        data = {
            "width": 10, "height": 10,
            "robots": [
                {"id": "R1", "start": [0, 0], "goal": [9, 9]},
                {"id": "R2", "start": [9, 0], "goal": [0, 9]},
                {"id": "R3", "start": [0, 9], "goal": [9, 0]},
            ],
        }
        scenario = load_scenario(_write_scenario(data, tmp_path))
        assert len(scenario.robot_specs) == 3
        ids = [s.robot_id for s in scenario.robot_specs]
        assert ids == ["R1", "R2", "R3"]

    def test_duplicate_ids_rejected(self, tmp_path: Path) -> None:
        data = {
            "width": 10, "height": 10,
            "robots": [
                {"id": "R1", "start": [0, 0], "goal": [9, 9]},
                {"id": "R1", "start": [1, 0], "goal": [8, 9]},
            ],
        }
        with pytest.raises(ValueError, match="Duplicate"):
            load_scenario(_write_scenario(data, tmp_path))

    def test_overlapping_starts_rejected(self, tmp_path: Path) -> None:
        data = {
            "width": 10, "height": 10,
            "robots": [
                {"id": "R1", "start": [0, 0], "goal": [9, 9]},
                {"id": "R2", "start": [0, 0], "goal": [8, 8]},
            ],
        }
        with pytest.raises(ValueError, match="same start position"):
            load_scenario(_write_scenario(data, tmp_path))

    def test_start_out_of_bounds_rejected(self, tmp_path: Path) -> None:
        data = {
            "width": 5, "height": 5,
            "robots": [
                {"id": "R1", "start": [10, 10], "goal": [4, 4]},
            ],
        }
        with pytest.raises(ValueError, match="out of bounds"):
            load_scenario(_write_scenario(data, tmp_path))

    def test_goal_out_of_bounds_rejected(self, tmp_path: Path) -> None:
        data = {
            "width": 5, "height": 5,
            "robots": [
                {"id": "R1", "start": [0, 0], "goal": [10, 10]},
            ],
        }
        with pytest.raises(ValueError, match="out of bounds"):
            load_scenario(_write_scenario(data, tmp_path))

    def test_start_on_obstacle_rejected(self, tmp_path: Path) -> None:
        data = {
            "width": 5, "height": 5,
            "obstacles": [[2, 2]],
            "robots": [
                {"id": "R1", "start": [2, 2], "goal": [4, 4]},
            ],
        }
        with pytest.raises(ValueError, match="obstacle"):
            load_scenario(_write_scenario(data, tmp_path))

    def test_goal_on_obstacle_rejected(self, tmp_path: Path) -> None:
        data = {
            "width": 5, "height": 5,
            "obstacles": [[4, 4]],
            "robots": [
                {"id": "R1", "start": [0, 0], "goal": [4, 4]},
            ],
        }
        with pytest.raises(ValueError, match="obstacle"):
            load_scenario(_write_scenario(data, tmp_path))

    def test_missing_robot_keys_rejected(self, tmp_path: Path) -> None:
        data = {
            "width": 5, "height": 5,
        }
        with pytest.raises(ValueError, match="robot"):
            load_scenario(_write_scenario(data, tmp_path))

    def test_empty_robots_list_rejected(self, tmp_path: Path) -> None:
        data = {
            "width": 5, "height": 5,
            "robots": [],
        }
        with pytest.raises(ValueError, match="non-empty"):
            load_scenario(_write_scenario(data, tmp_path))

    def test_missing_id_rejected(self, tmp_path: Path) -> None:
        data = {
            "width": 5, "height": 5,
            "robots": [
                {"start": [0, 0], "goal": [4, 4]},
            ],
        }
        with pytest.raises(ValueError, match="id"):
            load_scenario(_write_scenario(data, tmp_path))

    def test_file_not_found(self) -> None:
        with pytest.raises(FileNotFoundError):
            load_scenario("/nonexistent/path.json")
