"""Command-line interface for SwarmOS.

Provides the ``swarmos`` console command and the ``python -m swarmos``
entry point.  Supports both single-robot (Phase 1) and multi-robot
(Phase 2) scenarios, with optional coordination policies (Phase 3+).
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
    parser.add_argument(
        "--policy",
        type=str,
        choices=["none", "stop_and_wait", "distributed"],
        default="none",
        help="Coordination policy: 'none' (Phase 1/2), 'stop_and_wait' (Phase 3), or 'distributed' (Phase 6)",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        default=False,
        help="Print Phase 5 intent exchange and conflict observation logs",
    )

    args = parser.parse_args(argv)

    # Import here to avoid loading Pygame unless needed.
    from swarmos.simulation.config import SimulationConfig
    from swarmos.simulation.engine import SimulationEngine
    from swarmos.robot.amr import AMR
    from swarmos.robot.fleet import Fleet
    from swarmos.warehouse.map_loader import load_scenario

    # --- Load scenario ---
    scenario = load_scenario(args.scenario)
    print(f"[SwarmOS] Loaded scenario: {args.scenario}")
    print(f"[SwarmOS] Grid: {scenario.grid.width}×{scenario.grid.height}")
    print(f"[SwarmOS] Robots: {len(scenario.robot_specs)}")
    for spec in scenario.robot_specs:
        print(f"  {spec.robot_id}: start={spec.start} → goal={spec.goal}")

    # --- Build fleet ---
    fleet = Fleet()
    goals: dict[str, tuple[int, int]] = {}
    for spec in scenario.robot_specs:
        robot = AMR(robot_id=spec.robot_id, position=spec.start)
        fleet.add(robot)
        goals[spec.robot_id] = spec.goal

    # --- Instantiate coordination policy ---
    policy = None
    if args.policy == "stop_and_wait":
        from swarmos.coordination.policy import StopAndWaitPolicy
        policy = StopAndWaitPolicy()
        print(f"[SwarmOS] Policy: {policy.name}")
    elif args.policy == "distributed":
        from swarmos.coordination.distributed_policy import DistributedPolicy
        policy = DistributedPolicy(fleet)
        print(f"[SwarmOS] Policy: {policy.name}")
    else:
        print("[SwarmOS] Policy: NONE (no collision avoidance)")

    # --- Build simulation ---
    config = SimulationConfig(
        cell_size=args.cell_size,
        tick_rate=args.tick_rate,
        robot_step_delay=args.step_delay,
    )
    engine = SimulationEngine(
        grid=scenario.grid,
        robots=fleet,
        config=config,
        policy=policy,
    )

    # --- Plan paths ---
    results = engine.plan_all(goals)
    all_ok = True
    for robot_id, success in results.items():
        robot = fleet.get(robot_id)
        if success and robot.path is not None:
            print(f"[SwarmOS] {robot_id}: path found ({robot.path.length} waypoints)")
        else:
            print(f"[SwarmOS] {robot_id}: NO PATH FOUND", file=sys.stderr)
            all_ok = False

    if not all_ok:
        print("[SwarmOS] ERROR: Not all robots could find paths!", file=sys.stderr)
        sys.exit(1)

    # --- Detect initial path conflicts ---
    engine.detect_initial_conflicts()
    conflicts = engine.metrics.path_conflicts
    if conflicts:
        print(f"[SwarmOS] ⚠ {len(conflicts)} path conflict(s) detected:")
        for c in conflicts[:10]:  # show first 10
            print(f"  {c}")
        if len(conflicts) > 10:
            print(f"  ... and {len(conflicts) - 10} more")
    else:
        print("[SwarmOS] ✓ No path conflicts detected")

    # --- Run ---
    if args.headless:
        _run_headless(engine, verbose=args.verbose)
    else:
        _run_visual(engine, verbose=args.verbose)


def _report_outcome(engine: SimulationEngine) -> None:
    """Print metrics and explicit simulation outcome."""
    print(f"\n{engine.metrics.summary()}")

    # Communication metrics.
    t = engine.transport
    print(f"  Messages sent:        {t.metrics_sent}")
    print(f"  Messages delivered:   {t.metrics_delivered}")

    # Determine and report outcome.
    if engine.fleet.all_arrived:
        print("[SwarmOS] Simulation result: SUCCESS")
    elif engine.deadlocked:
        print("[SwarmOS] Simulation result: DEADLOCK — no progress possible")
    else:
        print("[SwarmOS] Simulation result: TIMEOUT — max ticks reached")

    for robot in engine.fleet:
        print(f"[SwarmOS] {robot.robot_id}: final pos={robot.position}, state={robot.state.name}")


def _log_intents(engine: SimulationEngine) -> None:
    """Print intent exchange and conflict observations for the current tick."""
    from swarmos.communication.intent import IntentAction

    tick = engine.tick
    for robot in engine.fleet:
        if robot.has_reached_goal:
            continue
        peer = robot.peer_state
        if not peer:
            continue

        # Derive own intent label
        from swarmos.robot.state import RobotState
        intent_map = {
            RobotState.MOVING: "MOVE",
            RobotState.WAITING: "WAIT",
            RobotState.IDLE: "IDLE",
            RobotState.ARRIVED: "COMPLETE",
        }
        own_intent = intent_map.get(robot.state, "IDLE")
        own_next = robot.intended_next_position
        print(
            f"  [T={tick}] {robot.robot_id} intent: "
            f"{own_intent} {robot.position} → {own_next}"
        )

        # Show what this robot knows about peers
        for pid, msg in sorted(peer.items()):
            print(
                f"           └─ knows {pid}: "
                f"{msg.intent.name} {msg.position} → {msg.intended_next} "
                f"(t={msg.timestamp})"
            )

        # Observed conflicts
        conflicts = robot.observe_conflicts(tick)
        for c in conflicts:
            print(
                f"           ⚠ observed {c.conflict_type.name} conflict "
                f"with {c.robot_b_id} at {c.position}"
            )

    # --- Phase 6: Negotiation results ---
    from swarmos.coordination.distributed_policy import DistributedPolicy
    if isinstance(engine.policy, DistributedPolicy):
        results = engine.policy.last_results
        for rid, result in sorted(results.items()):
            if result.conflicts:
                decision_label = "PROCEED" if result.decision.name == "MOVE" else "WAIT"
                print(
                    f"  [T={tick}] {rid} negotiation: "
                    f"decision={decision_label}"
                )
                if result.participants:
                    print(
                        f"           participants={result.participants}, "
                        f"winner={result.winner}"
                    )
                print(f"           reason: {result.reason}")


def _run_headless(
    engine: SimulationEngine,
    verbose: bool = False,
) -> None:
    """Run the simulation without visualisation (for benchmarking)."""
    print("[SwarmOS] Running headless simulation…")
    if verbose:
        print("[SwarmOS] Verbose mode: showing intent exchange")

    while not engine.is_finished:
        engine.update()
        if verbose and engine.tick % engine.config.robot_step_delay == 0:
            _log_intents(engine)

    _report_outcome(engine)


def _run_visual(
    engine: SimulationEngine,
    verbose: bool = False,
) -> None:
    """Run the simulation with the Pygame renderer."""
    from swarmos.visualization.renderer import Renderer

    renderer = Renderer(engine)
    print("[SwarmOS] Visualisation started. Press ESC or close the window to quit.")

    running = True
    while running:
        running = renderer.handle_events()

        if not engine.is_finished:
            engine.update()
            if verbose and engine.tick % engine.config.robot_step_delay == 0:
                _log_intents(engine)

        renderer.render()
        renderer.tick()

    renderer.shutdown()
    _report_outcome(engine)


if __name__ == "__main__":
    main()
