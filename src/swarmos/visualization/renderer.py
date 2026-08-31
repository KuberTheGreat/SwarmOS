"""Pygame-based renderer for the warehouse simulation.

The renderer is responsible **only** for drawing the current state of
the simulation.  It reads state from the SimulationEngine and presents
it visually — it never mutates simulation state.

Colour palette (designed for clarity, not flash):
    Background grid  — dark charcoal
    Grid lines       — subtle grey
    Obstacles        — warm brown
    Robot            — bright teal
    Goal             — soft gold
    Planned path     — translucent cyan
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import pygame

from swarmos.warehouse.cell import CellType, Position

if TYPE_CHECKING:
    from swarmos.simulation.engine import SimulationEngine

# ------------------------------------------------------------------
# Colour constants (R, G, B)
# ------------------------------------------------------------------
_COL_BACKGROUND  = (30, 30, 36)
_COL_GRID_LINE   = (50, 50, 58)
_COL_OBSTACLE    = (120, 80, 50)
_COL_ROBOT       = (0, 200, 180)
_COL_GOAL        = (230, 190, 60)
_COL_PATH        = (0, 160, 200, 120)  # RGBA — rendered via Surface alpha
_COL_TEXT        = (220, 220, 220)
_COL_ARRIVED     = (80, 200, 100)


class Renderer:
    """Pygame renderer for the SwarmOS simulation.

    Parameters:
        engine: The simulation engine whose state will be drawn.
    """

    def __init__(self, engine: SimulationEngine) -> None:
        self._engine = engine
        self._cell_size = engine.config.cell_size
        self._width = engine.grid.width * self._cell_size
        self._height = engine.grid.height * self._cell_size

        pygame.init()
        self._screen = pygame.display.set_mode((self._width, self._height))
        pygame.display.set_caption(engine.config.window_title)
        self._clock = pygame.time.Clock()
        self._font = pygame.font.SysFont("monospace", 14)

        # Semi-transparent surface for the planned path overlay.
        self._path_surface = pygame.Surface(
            (self._cell_size, self._cell_size), pygame.SRCALPHA
        )
        self._path_surface.fill(_COL_PATH)

    # ------------------------------------------------------------------
    # Coordinate helpers
    # ------------------------------------------------------------------

    def _grid_to_screen(self, pos: Position) -> tuple[int, int]:
        """Convert a grid position to the top-left pixel coordinate."""
        return pos.x * self._cell_size, pos.y * self._cell_size

    def _cell_center(self, pos: Position) -> tuple[int, int]:
        """Return the pixel center of a grid cell."""
        sx, sy = self._grid_to_screen(pos)
        half = self._cell_size // 2
        return sx + half, sy + half

    # ------------------------------------------------------------------
    # Drawing primitives
    # ------------------------------------------------------------------

    def _draw_grid(self) -> None:
        """Draw grid lines."""
        for x in range(0, self._width + 1, self._cell_size):
            pygame.draw.line(self._screen, _COL_GRID_LINE, (x, 0), (x, self._height))
        for y in range(0, self._height + 1, self._cell_size):
            pygame.draw.line(self._screen, _COL_GRID_LINE, (0, y), (self._width, y))

    def _draw_obstacles(self) -> None:
        """Draw obstacle cells as filled rectangles."""
        grid = self._engine.grid
        for y in range(grid.height):
            for x in range(grid.width):
                pos = Position(x, y)
                if grid.get_cell(pos) is CellType.OBSTACLE:
                    rect = pygame.Rect(
                        x * self._cell_size,
                        y * self._cell_size,
                        self._cell_size,
                        self._cell_size,
                    )
                    pygame.draw.rect(self._screen, _COL_OBSTACLE, rect)

    def _draw_path(self) -> None:
        """Draw the robot's planned path as translucent overlays."""
        robot = self._engine.robot
        if robot.path is None:
            return
        for pos in robot.path.waypoints:
            sx, sy = self._grid_to_screen(pos)
            self._screen.blit(self._path_surface, (sx, sy))

    def _draw_goal(self) -> None:
        """Draw the goal as a diamond marker."""
        robot = self._engine.robot
        if robot.goal is None:
            return
        cx, cy = self._cell_center(robot.goal)
        half = self._cell_size // 3
        points = [
            (cx, cy - half),
            (cx + half, cy),
            (cx, cy + half),
            (cx - half, cy),
        ]
        pygame.draw.polygon(self._screen, _COL_GOAL, points)

    def _draw_robot(self) -> None:
        """Draw the robot as a filled circle."""
        robot = self._engine.robot
        cx, cy = self._cell_center(robot.position)
        radius = self._cell_size // 3
        color = _COL_ARRIVED if robot.has_reached_goal else _COL_ROBOT
        pygame.draw.circle(self._screen, color, (cx, cy), radius)

    def _draw_status(self) -> None:
        """Draw a small status line at the top-left."""
        robot = self._engine.robot
        status = f"Tick: {self._engine.tick}  |  State: {robot.state.name}  |  Pos: ({robot.position.x},{robot.position.y})"
        surface = self._font.render(status, True, _COL_TEXT)
        self._screen.blit(surface, (6, 4))

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def render(self) -> None:
        """Draw the current simulation state to the screen."""
        self._screen.fill(_COL_BACKGROUND)
        self._draw_grid()
        self._draw_obstacles()
        self._draw_path()
        self._draw_goal()
        self._draw_robot()
        self._draw_status()
        pygame.display.flip()

    def tick(self) -> None:
        """Wait for the frame-rate clock."""
        self._clock.tick(self._engine.config.tick_rate)

    def handle_events(self) -> bool:
        """Process Pygame events.  Returns ``False`` if the user closed
        the window."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                return False
        return True

    def shutdown(self) -> None:
        """Clean up Pygame resources."""
        pygame.quit()
