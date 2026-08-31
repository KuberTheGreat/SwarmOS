"""Pygame-based renderer for the warehouse simulation.

The renderer is responsible **only** for drawing the current state of
the simulation.  It reads state from the SimulationEngine and presents
it visually — it never mutates simulation state.

Phase 2 additions:
    - Multiple robots with distinct colours
    - Per-robot goal markers and planned paths
    - Robot ID labels
    - Status bar with fleet-level information
    - Metrics overlay

Colour palette (designed for clarity, not flash):
    Background grid  — dark charcoal
    Grid lines       — subtle grey
    Obstacles        — warm brown
    Text             — light grey
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import pygame

from swarmos.warehouse.cell import CellType, Position

if TYPE_CHECKING:
    from swarmos.robot.amr import AMR
    from swarmos.simulation.engine import SimulationEngine

# ------------------------------------------------------------------
# Colour constants (R, G, B)
# ------------------------------------------------------------------
_COL_BACKGROUND  = (30, 30, 36)
_COL_GRID_LINE   = (50, 50, 58)
_COL_OBSTACLE    = (120, 80, 50)
_COL_TEXT         = (220, 220, 220)
_COL_COLLISION    = (255, 60, 60)

# Per-robot colour palette — visually distinct, colourblind-considerate.
_ROBOT_COLOURS: list[tuple[int, int, int]] = [
    (0, 200, 180),    # teal
    (230, 120, 50),   # orange
    (130, 100, 220),  # purple
    (220, 60, 120),   # pink
    (80, 180, 80),    # green
    (60, 140, 230),   # blue
    (200, 200, 60),   # yellow
    (180, 100, 60),   # brown
]

# Path overlay alpha per robot (slightly varied for overlap visibility).
_PATH_ALPHA = 90

# Arrived tint — brighter version of robot colour.
_ARRIVED_BOOST = 60


def _brighten(color: tuple[int, int, int], amount: int = _ARRIVED_BOOST) -> tuple[int, int, int]:
    """Return a brightened version of *color*."""
    return (
        min(255, color[0] + amount),
        min(255, color[1] + amount),
        min(255, color[2] + amount),
    )


def _robot_color(index: int) -> tuple[int, int, int]:
    """Return a deterministic colour for robot at *index*."""
    return _ROBOT_COLOURS[index % len(_ROBOT_COLOURS)]


class Renderer:
    """Pygame renderer for the SwarmOS simulation.

    Parameters:
        engine: The simulation engine whose state will be drawn.
    """

    def __init__(self, engine: SimulationEngine) -> None:
        self._engine = engine
        self._cell_size = engine.config.cell_size
        self._grid_width = engine.grid.width * self._cell_size
        self._grid_height = engine.grid.height * self._cell_size
        # Reserve space for legend at the bottom.
        fleet_size = len(engine.fleet)
        self._legend_height = max(30, 24 * fleet_size + 10)
        self._width = self._grid_width
        self._height = self._grid_height + self._legend_height

        pygame.init()
        self._screen = pygame.display.set_mode((self._width, self._height))
        pygame.display.set_caption(engine.config.window_title)
        self._clock = pygame.time.Clock()
        self._font = pygame.font.SysFont("monospace", 13)
        self._font_small = pygame.font.SysFont("monospace", 10)
        self._font_id = pygame.font.SysFont("monospace", 10, bold=True)

        # Build a colour index for each robot (deterministic order).
        self._robot_colors: dict[str, tuple[int, int, int]] = {}
        for i, robot in enumerate(engine.fleet):
            self._robot_colors[robot.robot_id] = _robot_color(i)

        # Pre-build per-robot path surfaces.
        self._path_surfaces: dict[str, pygame.Surface] = {}
        for robot_id, color in self._robot_colors.items():
            surf = pygame.Surface(
                (self._cell_size, self._cell_size), pygame.SRCALPHA
            )
            surf.fill((*color, _PATH_ALPHA))
            self._path_surfaces[robot_id] = surf

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
        for x in range(0, self._grid_width + 1, self._cell_size):
            pygame.draw.line(self._screen, _COL_GRID_LINE, (x, 0), (x, self._grid_height))
        for y in range(0, self._grid_height + 1, self._cell_size):
            pygame.draw.line(self._screen, _COL_GRID_LINE, (0, y), (self._grid_width, y))

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

    def _draw_paths(self) -> None:
        """Draw all robots' planned paths as translucent overlays."""
        for robot in self._engine.fleet:
            if robot.path is None:
                continue
            surf = self._path_surfaces.get(robot.robot_id)
            if surf is None:
                continue
            for pos in robot.path.waypoints:
                sx, sy = self._grid_to_screen(pos)
                self._screen.blit(surf, (sx, sy))

    def _draw_goals(self) -> None:
        """Draw goal markers for all robots."""
        for robot in self._engine.fleet:
            if robot.goal is None:
                continue
            color = self._robot_colors.get(robot.robot_id, (230, 190, 60))
            cx, cy = self._cell_center(robot.goal)
            half = self._cell_size // 3
            points = [
                (cx, cy - half),
                (cx + half, cy),
                (cx, cy + half),
                (cx - half, cy),
            ]
            pygame.draw.polygon(self._screen, color, points)
            # Draw outline for visibility
            pygame.draw.polygon(self._screen, (255, 255, 255), points, 1)

    def _draw_robots(self) -> None:
        """Draw all robots as filled circles with ID labels."""
        for robot in self._engine.fleet:
            color = self._robot_colors.get(robot.robot_id, (0, 200, 180))
            if robot.has_reached_goal:
                color = _brighten(color)

            cx, cy = self._cell_center(robot.position)
            radius = self._cell_size // 3
            pygame.draw.circle(self._screen, color, (cx, cy), radius)
            # White outline
            pygame.draw.circle(self._screen, (255, 255, 255), (cx, cy), radius, 1)

            # Draw robot ID label
            label = self._font_id.render(robot.robot_id, True, (255, 255, 255))
            label_rect = label.get_rect(center=(cx, cy - radius - 8))
            self._screen.blit(label, label_rect)

    def _draw_status(self) -> None:
        """Draw a status bar in the top-left corner."""
        fleet = self._engine.fleet
        arrived = len(fleet.arrived_robots)
        total = len(fleet)
        collisions = self._engine.metrics.total_collisions
        conflicts = self._engine.metrics.total_path_conflicts
        status = (
            f"Tick: {self._engine.tick}  |  "
            f"Robots: {arrived}/{total} arrived  |  "
            f"Conflicts: {conflicts}  |  "
            f"Collisions: {collisions}"
        )
        surface = self._font.render(status, True, _COL_TEXT)
        # Draw background bar for readability
        bar_rect = pygame.Rect(0, 0, self._grid_width, 20)
        bar_surf = pygame.Surface((self._grid_width, 20), pygame.SRCALPHA)
        bar_surf.fill((0, 0, 0, 160))
        self._screen.blit(bar_surf, (0, 0))
        self._screen.blit(surface, (6, 3))

    def _draw_legend(self) -> None:
        """Draw a legend below the grid showing robot colours and status."""
        y_start = self._grid_height + 4
        x = 10
        for robot in self._engine.fleet:
            color = self._robot_colors.get(robot.robot_id, (200, 200, 200))
            if robot.has_reached_goal:
                color = _brighten(color)

            # Colour swatch
            pygame.draw.circle(self._screen, color, (x + 6, y_start + 8), 5)
            pygame.draw.circle(self._screen, (255, 255, 255), (x + 6, y_start + 8), 5, 1)

            # Label
            state_text = robot.state.name
            label = f"{robot.robot_id}: ({robot.position.x},{robot.position.y}) {state_text}"
            surface = self._font_small.render(label, True, _COL_TEXT)
            self._screen.blit(surface, (x + 16, y_start + 2))

            y_start += 22

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def render(self) -> None:
        """Draw the current simulation state to the screen."""
        self._screen.fill(_COL_BACKGROUND)
        self._draw_grid()
        self._draw_obstacles()
        self._draw_paths()
        self._draw_goals()
        self._draw_robots()
        self._draw_status()
        self._draw_legend()
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
