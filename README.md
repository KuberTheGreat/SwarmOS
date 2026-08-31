# SwarmOS

**Decentralized autonomous mobile robot fleet coordination research framework.**

SwarmOS is a research-oriented simulation and coordination framework for decentralized autonomous mobile robot (AMR) fleets operating in smart warehouse environments. The project explores fully decentralized approaches where each robot acts as an independent agent — planning, communicating, and resolving conflicts without a central fleet controller.

Inspired by the Smart India Hackathon problem statement on *Edge-AI Based Distributed Fleet Coordination for Autonomous Mobile Robots in Smart Warehouses*.

---

## Current Status

### Phase 1 — Single AMR Navigation ✅

A single autonomous mobile robot navigates a 2D warehouse grid from a configurable start position to a goal using A* pathfinding, with real-time Pygame visualisation.

**What works:**
- 2D grid-based warehouse representation with static obstacles
- A* pathfinding with Manhattan distance heuristic
- Single AMR navigating a planned path
- Real-time Pygame visualisation (grid, obstacles, path, robot, goal)
- Headless simulation mode for benchmarking
- JSON-based scenario configuration
- Comprehensive unit tests

---

## Architecture

```
swarmos/
├── warehouse/       # Environment model (Grid, Cell, Position)
├── planning/        # Pathfinding algorithms (A*, Path)
├── robot/           # AMR model and state machine
├── simulation/      # Engine (headless-capable) and configuration
├── visualization/   # Pygame renderer (read-only view of state)
└── cli.py           # Entry point and argument parsing
```

**Key design principles:**
- **Simulation ≠ Rendering**: The engine runs without Pygame for headless benchmarking
- **Planner ≠ Robot**: A* is a standalone module; robots consume it
- **Environment ≠ Agent**: The Grid answers spatial queries; it doesn't decide movement
- **Each robot is an independent agent**: Designed for future multi-robot decentralised coordination

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
# Launch with default scenario (Pygame window)
python -m swarmos

# Or use the installed console command
swarmos

# Run headless (no GUI — for benchmarking)
python -m swarmos --headless

# Custom scenario
python -m swarmos --scenario scenarios/basic_warehouse.json

# Adjust visualisation
python -m swarmos --cell-size 30 --tick-rate 30 --step-delay 5
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

Scenarios are JSON files in `scenarios/`:

```json
{
  "width": 20,
  "height": 15,
  "obstacles": [[5, 2], [5, 3], [5, 4]],
  "robot": {
    "start": [1, 1],
    "goal": [18, 13]
  }
}
```

---

## Roadmap

| Phase | Description | Status |
|-------|-------------|--------|
| 1 | Single AMR navigation (A* on grid) | ✅ |
| 2 | Multi-AMR simulation | ⬜ |
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
