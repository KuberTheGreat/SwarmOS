"""Command-line interface for SwarmOS.

Provides the ``swarmos`` console command and the ``python -m swarmos``
entry point.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def main(argv: list[str] | None = None) -> None:
    """Parse arguments and launch the simulation."""
    parser = argparse.ArgumentParser(
        prog="swarmos",
        description="SwarmOS — decentralized AMR fleet simulation",
    )
    parser.add_argument(
        "--scenario",
        type=str,
        default=str(Path(__file__).resolve().parents[2] / "scenarios" / "basic_warehouse.json"),
        help="Path to a scenario JSON file (default: scenarios/basic_warehouse.json)",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        default=False,
        help="Run the simulation without the Pygame visualisation",
    )
    parser.add_argument(
        "--cell-size",
        type=int,
        default=40,
        help="Pixel size of each grid cell (default: 40)",
    )
    parser.add_argument(
        "--tick-rate",
        type=int,
        default=60,
        help="Target frames per second (default: 60)",
    )
    parser.add_argument(
        "--step-delay",
        type=int,
        default=10,
        help="Ticks between each robot movement step (default: 10)",
    )

    args = parser.parse_args(argv)

    # Import here to avoid loading Pygame unless needed.
    from swarmos.simulation.config import SimulationConfig
    from swarmos.simulation.engine import SimulationEngine
    from swarmos.robot.amr import AMR
    from swarmos.warehouse.map_loader import load_scenario

    # --- Load scenario ---
    scenario = load_scenario(args.scenario)
    print(f"[SwarmOS] Loaded scenario: {args.scenario}")
    print(f"[SwarmOS] Grid: {scenario.grid.width}×{scenario.grid.height}")
    print(f"[SwarmOS] Start: {scenario.robot_start}  Goal: {scenario.robot_goal}")

    # --- Build simulation ---
    config = SimulationConfig(
        cell_size=args.cell_size,
        tick_rate=args.tick_rate,
        robot_step_delay=args.step_delay,
    )
    robot = AMR(robot_id="amr-1", position=scenario.robot_start)
    engine = SimulationEngine(grid=scenario.grid, robot=robot, config=config)

    # --- Plan path ---
    if not engine.plan_robot(scenario.robot_goal):
        print("[SwarmOS] ERROR: No path found from start to goal!", file=sys.stderr)
        sys.exit(1)

    path = robot.path
    assert path is not None
    print(f"[SwarmOS] Path found: {path.length} waypoints")

    # --- Run ---
    if args.headless:
        _run_headless(engine)
    else:
        _run_visual(engine)


def _run_headless(engine: SimulationEngine) -> None:
    """Run the simulation without visualisation (for benchmarking)."""
    from swarmos.simulation.engine import SimulationEngine  # already imported, for type hints

    print("[SwarmOS] Running headless simulation…")
    while not engine.is_finished:
        engine.update()
    print(f"[SwarmOS] Simulation complete at tick {engine.tick}")
    print(f"[SwarmOS] Robot final position: {engine.robot.position}")


def _run_visual(engine: SimulationEngine) -> None:
    """Run the simulation with the Pygame renderer."""
    from swarmos.visualization.renderer import Renderer

    renderer = Renderer(engine)
    print("[SwarmOS] Visualisation started. Press ESC or close the window to quit.")

    running = True
    while running:
        running = renderer.handle_events()

        if not engine.is_finished:
            engine.update()

        renderer.render()
        renderer.tick()

    renderer.shutdown()
    print(f"[SwarmOS] Simulation ended at tick {engine.tick}")
    print(f"[SwarmOS] Robot state: {engine.robot.state.name}")
    print(f"[SwarmOS] Robot final position: {engine.robot.position}")


if __name__ == "__main__":
    main()
