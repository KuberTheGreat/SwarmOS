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

### Phase 3 — Stop-and-Wait Baseline ✅

A deliberately naive stop-and-wait collision avoidance policy. This baseline exists to establish measurable comparison data for future decentralized coordination algorithms.

**Research question:** *Can decentralized coordination reduce total task completion time compared with naive stop-and-wait behavior?*

**What works:**
- 2D grid-based warehouse with static obstacles
- A* pathfinding with Manhattan distance heuristic
- Multiple independent AMRs (each plans and moves independently)
- Fleet container for managing robot collections
- Path conflict detection (node + edge conflicts, temporal)
- Runtime collision detection
- **Stop-and-wait coordination policy** (pluggable via `--policy`)
  - Node conflict resolution (same-cell targets → lower ID moves)
  - Edge conflict detection (head-on swap → both wait)
  - Occupancy conflict detection (cell blocked → wait)
  - Deterministic tie-breaking (lower robot ID wins)
  - Snapshot-based simultaneous updates (iteration-order independent)
  - WAITING robot state with visual indicator
- **Wait and movement metrics tracking** (per-robot and aggregate)
- Simulation metrics (per-robot and aggregate)
- Real-time Pygame visualisation with distinct robot colours, ID labels, path overlays, waiting indicators, and legend
- Headless simulation mode for benchmarking
- JSON-based multi-robot scenario configuration
- **4 baseline benchmark scenarios** (crossing, intersection, chokepoint, head-on)
- Backward compatibility with Phase 1 single-robot scenarios
- 153 unit and integration tests

**What is intentionally NOT implemented (by design):**
- Priority negotiation
- Reservation/communication protocols
- Dynamic rerouting
- Deadlock resolution (beyond simple tie-breaking)
- Task allocation
- Centralized fleet controller (this is a decentralized system)

**Known limitations (by design — this is a baseline):**
- Head-on conflicts in corridors result in deadlock (no rerouting)
- No dynamic path adaptation
- Edge conflicts require both robots to wait (no swap resolution)

---

## Architecture

```
swarmos/
├── warehouse/       # Environment model (Grid, Cell, Position)
├── planning/        # Pathfinding algorithms (A*, Path)
├── robot/           # AMR model, state machine, Fleet container
├── coordination/    # Conflict detection, collision detection, policies
├── simulation/      # Engine (headless-capable), config, metrics
├── visualization/   # Pygame renderer (read-only view of state)
└── cli.py           # Entry point and argument parsing
```

**Key design principles:**
- **Simulation ≠ Rendering**: Engine runs without Pygame for headless benchmarking
- **Planner ≠ Robot**: A* is a standalone module; robots consume it
- **Environment ≠ Agent**: Grid answers spatial queries; doesn't decide movement
- **Each robot is an independent agent**: No centralized fleet controller
- **Detection ≠ Resolution**: Conflicts are detected and resolved via pluggable policies
- **Fleet ≠ Controller**: Fleet is a state container, not a decision-maker
- **Policy ≠ Controller**: Policies are strategies applied to snapshots, not centralized decision-makers

See [`docs/architecture.md`](docs/architecture.md) for detailed documentation.

---

## Getting Started

### Prerequisites

- Python 3.11+
- macOS or Linux (Windows: use `.\\\.venv\\\Scripts\\\activate` instead of `source`)

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
# Multi-robot with stop-and-wait collision avoidance (Phase 3)
python -m swarmos --scenario scenarios/baseline_crossing.json --policy stop_and_wait

# Without policy (Phase 2 behavior — no collision avoidance)
python -m swarmos --scenario scenarios/multi_robot.json

# Baseline scenarios with policy
python -m swarmos --scenario scenarios/baseline_crossing.json --policy stop_and_wait
python -m swarmos --scenario scenarios/baseline_intersection.json --policy stop_and_wait
python -m swarmos --scenario scenarios/baseline_chokepoint.json --policy stop_and_wait
python -m swarmos --scenario scenarios/baseline_headon.json --policy stop_and_wait

# Headless benchmarking with policy
python -m swarmos --scenario scenarios/baseline_crossing.json --headless --policy stop_and_wait --step-delay 1

# Single robot (Phase 1 backward compat)
python -m swarmos --scenario scenarios/basic_warehouse.json

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

### Multi-Robot (Phase 2+)

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

## Conflict Detection & Resolution

SwarmOS distinguishes between **path conflicts** (static analysis of planned trajectories) and **collisions** (runtime same-cell occupancy).

### Path Conflicts (pre-execution)

| Type | Description |
|------|-------------|
| **NODE** | Two robots planned to occupy the same cell at the same timestep |
| **EDGE** | Two robots planned to swap positions (head-on traversal) at the same timestep |

### Collisions (runtime)

Detected each simulation tick when two robots actually occupy the same cell.

### Coordination Policy (Phase 3)

The `--policy stop_and_wait` flag enables the stop-and-wait baseline:
- **Node conflicts**: Lower robot ID moves, higher waits
- **Edge conflicts**: Both robots wait (swap is unsafe without rerouting)
- **Occupancy conflicts**: Robot waits if target cell is occupied by a non-moving robot
- **Deterministic**: Same scenario always produces identical results
- **No centralized controller**: Policy is a strategy applied to snapshots

### Example Output

```
═══ Simulation Metrics ═══
  Policy:               STOP_AND_WAIT
  Total ticks:          11
  Robots:               3/3 completed
  Total movements:      27
  Total wait ticks:     3
  Path conflicts:       2
  Runtime collisions:   0
      AMR-01: path=10, steps=9, waits=0, arrived at tick 9
      AMR-02: path=10, steps=9, waits=1, arrived at tick 10
      AMR-03: path=10, steps=9, waits=2, arrived at tick 11
```

---

## Baseline Scenarios (Phase 3)

| Scenario | Robots | Description | Expected Outcome |
|----------|--------|-------------|-----------------|
| `baseline_crossing.json` | 3 | Paths cross at center of open grid | All arrive, some waits |
| `baseline_intersection.json` | 4 | 4-way intersection through corridors | Deadlock (expected) |
| `baseline_chokepoint.json` | 4 | 1-cell-wide corridor, bidirectional | Deadlock (expected) |
| `baseline_headon.json` | 2 | Head-on in 1-cell corridor | Deadlock (expected) |

Deadlocks are **expected** and **by design** — they demonstrate the limitation of naive stop-and-wait without rerouting. Future phases will resolve these through decentralized coordination.

---

## Roadmap

| Phase | Description | Status |
|-------|-------------|--------|
| 1 | Single AMR navigation (A* on grid) | ✅ |
| 2 | Multi-AMR simulation + conflict detection | ✅ |
| 3 | Stop-and-wait collision baseline | ✅ |
| 4 | Peer-to-peer communication | ✅ |
| 5 | Distributed conflict detection | ✅ |
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

