# SwarmOS

**Decentralized autonomous mobile robot fleet coordination research framework.**

SwarmOS is a research-oriented simulation and coordination framework for decentralized autonomous mobile robot (AMR) fleets operating in smart warehouse environments. The project explores fully decentralized approaches where each robot acts as an independent agent — planning, communicating, and resolving conflicts without a central fleet controller.

Inspired by the Smart India Hackathon problem statement on *Edge-AI Based Distributed Fleet Coordination for Autonomous Mobile Robots in Smart Warehouses*.

---

## Current Status

### Phase 1 — Single AMR Navigation ✅

A single autonomous mobile robot navigates a 2D warehouse grid from a configurable start position to a goal using A* pathfinding.

### Phase 2 — Multi-AMR Simulation ✅

Multiple independent AMRs navigate simultaneously on the same warehouse grid. Path conflicts and runtime collisions are detected and measured.

**What works:**
- 2D grid-based warehouse with static obstacles
- A* pathfinding with Manhattan distance heuristic
- Multiple independent AMRs (each plans and moves independently)
- Fleet container for managing robot collections
- Path conflict detection (node + edge conflicts, temporal)
- Runtime collision detection
- Simulation metrics (per-robot and aggregate)
- Real-time Pygame visualisation with distinct robot colours, ID labels, path overlays, and legend
- Headless simulation mode for benchmarking
- JSON-based multi-robot scenario configuration
- Backward compatibility with Phase 1 single-robot scenarios
- 105 unit tests

**What is intentionally NOT implemented yet:**
- Conflict **resolution** (detection only)
- P2P communication between robots
- Reservation/negotiation protocols
- Dynamic rerouting
- Task allocation
- Centralized fleet controller (by design — this is a decentralized system)

---

## Architecture

```
swarmos/
├── warehouse/       # Environment model (Grid, Cell, Position)
├── planning/        # Pathfinding algorithms (A*, Path)
├── robot/           # AMR model, state machine, Fleet container
├── coordination/    # Conflict detection, collision detection
├── simulation/      # Engine (headless-capable), config, metrics
├── visualization/   # Pygame renderer (read-only view of state)
└── cli.py           # Entry point and argument parsing
```

**Key design principles:**
- **Simulation ≠ Rendering**: Engine runs without Pygame for headless benchmarking
- **Planner ≠ Robot**: A* is a standalone module; robots consume it
- **Environment ≠ Agent**: Grid answers spatial queries; doesn't decide movement
- **Each robot is an independent agent**: No centralized fleet controller
- **Detection ≠ Resolution**: Conflicts are detected, not automatically resolved
- **Fleet ≠ Controller**: Fleet is a state container, not a decision-maker

See [`docs/architecture.md`](docs/architecture.md) for detailed documentation.

---

## Getting Started

### Prerequisites

- Python 3.11+
- macOS or Linux (Windows: use `.\\.venv\\Scripts\\activate` instead of `source`)

### Setup

```bash
git clone https://github.com/KuberTheGreat/SwarmOS.git
cd SwarmOS
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
```

### Run the Simulation

```bash
# Multi-robot simulation (Pygame window)
python -m swarmos --scenario scenarios/multi_robot.json

# Crossing robots scenario (path conflicts guaranteed)
python -m swarmos --scenario scenarios/crossing_robots.json

# Single robot (Phase 1 backward compat)
python -m swarmos --scenario scenarios/basic_warehouse.json

# Headless (no GUI — for benchmarking)
python -m swarmos --scenario scenarios/multi_robot.json --headless

# Adjust visualisation
python -m swarmos --scenario scenarios/multi_robot.json --cell-size 30 --step-delay 5
```

**Controls:**
- Press `ESC` or close the window to quit

### Run Tests

```bash
pytest
pytest -v          # verbose
pytest --tb=short  # shorter tracebacks
```

---

## Scenario Format

### Multi-Robot (Phase 2)

```json
{
  "width": 20,
  "height": 15,
  "obstacles": [[5, 2], [5, 3]],
  "robots": [
    {"id": "AMR-01", "start": [1, 1], "goal": [18, 13]},
    {"id": "AMR-02", "start": [18, 1], "goal": [1, 13]},
    {"id": "AMR-03", "start": [1, 13], "goal": [18, 1]}
  ]
}
```

### Single-Robot (Phase 1 compat)

```json
{
  "width": 20,
  "height": 15,
  "obstacles": [[5, 2], [5, 3]],
  "robot": {"start": [1, 1], "goal": [18, 13]}
}
```

### Validation

The loader validates:
- Unique robot IDs
- Start/goal positions within grid bounds
- Start/goal positions not on obstacles
- No two robots sharing the same start cell

---

## Conflict Detection

SwarmOS distinguishes between **path conflicts** (static analysis of planned trajectories) and **collisions** (runtime same-cell occupancy).

### Path Conflicts (pre-execution)

| Type | Description |
|------|-------------|
| **NODE** | Two robots planned to occupy the same cell at the same timestep |
| **EDGE** | Two robots planned to swap positions (head-on traversal) at the same timestep |

### Collisions (runtime)

Detected each simulation tick when two robots actually occupy the same cell.

### Example Output

```
[SwarmOS] ⚠ 2 path conflict(s) detected:
  Conflict(EDGE, 'AMR-01' ↔ 'AMR-02', pos=Position(x=10, y=5), t=12)
  Conflict(NODE, 'AMR-01' ↔ 'AMR-03', pos=Position(x=11, y=7), t=16)

═══ Simulation Metrics ═══
  Total ticks:          290
  Robots:               3/3 completed
  Path conflicts:       2
  Runtime collisions:   1
```

---

## Roadmap

| Phase | Description | Status |
|-------|-------------|--------|
| 1 | Single AMR navigation (A* on grid) | ✅ |
| 2 | Multi-AMR simulation + conflict detection | ✅ |
| 3 | Stop-and-wait collision baseline | ⬜ |
| 4 | Peer-to-peer communication | ⬜ |
| 5 | Distributed conflict detection | ⬜ |
| 6 | Reservation / negotiation protocol | ⬜ |
| 7 | Deadlock detection and resolution | ⬜ |
| 8 | Dynamic rerouting | ⬜ |
| 9 | Distributed task allocation | ⬜ |
| 10 | Failure recovery | ⬜ |
| 11 | Fleet telemetry dashboard | ⬜ |
| 12 | Benchmarking framework | ⬜ |
| 13 | Edge deployment | ⬜ |
| 14 | AI/ML enhancements | ⬜ |

---

## License

MIT — see [LICENSE](LICENSE).
