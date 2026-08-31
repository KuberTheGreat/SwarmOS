"""Simulation configuration — engine parameters and rendering settings.

All tuneable constants live here so they are not scattered throughout
the codebase.  Scenario-specific data (grid layout, obstacles, robot
start/goal) lives in scenario JSON files — this module is for *engine*
configuration only.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SimulationConfig:
    """Immutable simulation engine configuration.

    Attributes:
        cell_size:         Pixel size of each grid cell in the renderer.
        tick_rate:         Target frames per second for the simulation loop.
        robot_step_delay:  Number of simulation ticks between each robot
                           movement step.  Higher values make the robot
                           appear to move more slowly (easier to follow
                           visually).
        window_title:      Pygame window title.
    """

    cell_size: int = 40
    tick_rate: int = 60
    robot_step_delay: int = 10
    window_title: str = "SwarmOS — Phase 1"

    @property
    def window_width_for(self) -> None:
        """Marker — window dimensions are computed dynamically from the
        grid size and cell_size.  This property is intentionally not
        stored so the renderer can adapt to any grid."""
        ...
