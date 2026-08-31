# SwarmOS — Architecture

## Overview

SwarmOS is structured around a strict separation of concerns that enables:
- **Headless simulation** for automated benchmarking
- **Independent robot agents** for decentralised multi-robot coordination
- **Modular planners** that can be swapped or extended
- **Clean testing** without GUI dependencies
- **Conflict detection** separate from conflict resolution

---

## Module Responsibilities

```mermaid
graph TD
    CLI[cli.py] --> ML[map_loader]
    CLI --> Engine[SimulationEngine]
    CLI --> Renderer[Renderer]

    ML --> Grid[Grid]
    ML --> ScenarioData
    ML --> RobotSpec

    Engine --> Grid
    Engine --> Fleet
    Engine --> Config[SimulationConfig]
    Engine --> Metrics[SimulationMetrics]
    Engine --> Detector[Conflict Detector]

    Fleet --> AMR1[AMR-01]
    Fleet --> AMR2[AMR-02]
    Fleet --> AMR3[AMR-03]

    AMR1 --> Planner[A* Planner]
    AMR2 --> Planner
    AMR3 --> Planner

    Planner --> PathObj[Path]
    Planner --> Grid

    AMR1 --> State[RobotState]
    AMR2 --> State
    AMR3 --> State

    Detector --> ConflictModel["Conflict / Collision"]
    Detector --> Fleet

    Renderer --> Engine

    style Renderer fill:#2d5a27,color:#fff
    style Engine fill:#1a3a5c,color:#fff
    style Planner fill:#5c3a1a,color:#fff
    style Grid fill:#3a1a5c,color:#fff
    style Fleet fill:#5c1a3a,color:#fff
    style Detector fill:#5c5c1a,color:#fff
    style Metrics fill:#1a5c5c,color:#fff
```

### `warehouse/` — Environment Model

| Module | Responsibility |
|--------|---------------|
| `cell.py` | `Position` (immutable coordinate) and `CellType` (extensible enum) |
| `grid.py` | 2D grid: bounds checking, traversability, neighbour queries |
| `map_loader.py` | Parses scenario JSON → `Grid` + list of `RobotSpec` (supports both Phase 1 and Phase 2 formats) |

**Design rule**: The warehouse answers environmental questions ("Is this cell traversable?"). It never decides *where* a robot should go.

### `planning/` — Pathfinding

| Module | Responsibility |
|--------|---------------|
| `astar.py` | A* search with Manhattan heuristic. Input: Grid + start + goal. Output: Path or None. |
| `path.py` | Ordered waypoint sequence with cursor-based traversal |

**Design rule**: The planner depends only on the Grid interface. It has no knowledge of robots, rendering, or simulation time.

### `robot/` — Agent Model

| Module | Responsibility |
|--------|---------------|
| `amr.py` | AMR class: owns its position, state, path. Uses the planner. |
| `state.py` | `RobotState` enum (IDLE → MOVING → ARRIVED) |
| `fleet.py` | Fleet: ordered collection of AMRs with unique IDs. State container, not a controller. |

**Design rule**: Each AMR is an independent agent. The Fleet holds them but doesn't direct them. This prepares for the decentralised architecture where each robot owns its own decision-making.

### `coordination/` — Conflict Detection *(Phase 2)*

| Module | Responsibility |
|--------|---------------|
| `conflict.py` | Data models: `Conflict` (path overlap), `Collision` (runtime overlap), `ConflictType` enum |
| `detector.py` | Static path conflict detection (node + edge) and runtime collision detection |

**Design rule**: Detection ≠ Resolution. This module *reports* problems. Future phases will add resolution protocols in separate modules.

### `simulation/` — Engine

| Module | Responsibility |
|--------|---------------|
| `engine.py` | Discrete-tick simulation loop. Manages grid + fleet + time + metrics. |
| `config.py` | Immutable configuration (cell size, tick rate, step delay) |
| `metrics.py` | Per-robot and aggregate metrics (steps, arrivals, collisions, conflicts) |

**Design rule**: The engine is headless-capable. It never imports Pygame. It orchestrates *time*, not *decisions*.

### `visualization/` — Rendering

| Module | Responsibility |
|--------|---------------|
| `renderer.py` | Pygame renderer. Draws all robots with distinct colours, paths, goals, IDs, and legend. |

**Design rule**: The renderer is a pure *observer*. It can be removed without affecting simulation correctness.

---

## Multi-AMR Data Flow

```mermaid
sequenceDiagram
    participant CLI
    participant Loader as MapLoader
    participant Fleet
    participant Engine
    participant AMR1 as AMR-01
    participant AMR2 as AMR-02
    participant Planner as A*
    participant Detector
    participant Renderer

    CLI->>Loader: load_scenario("multi_robot.json")
    Loader-->>CLI: ScenarioData (grid, robot_specs[])

    CLI->>Fleet: Fleet()
    loop For each RobotSpec
        CLI->>Fleet: add(AMR(id, start))
    end

    CLI->>Engine: SimulationEngine(grid, fleet, config)
    CLI->>Engine: plan_all(goals)
    Engine->>AMR1: plan(grid, goal)
    AMR1->>Planner: find_path(grid, start, goal)
    Planner-->>AMR1: Path
    Engine->>AMR2: plan(grid, goal)
    AMR2->>Planner: find_path(grid, start, goal)
    Planner-->>AMR2: Path

    CLI->>Engine: detect_initial_conflicts()
    Engine->>Detector: detect_path_conflicts(fleet)
    Detector-->>Engine: list[Conflict]

    CLI->>Renderer: Renderer(engine)

    loop Every Frame
        CLI->>Renderer: handle_events()
        CLI->>Engine: update()
        Engine->>AMR1: step()
        Engine->>AMR2: step()
        Engine->>Detector: detect_collisions(fleet, tick)
        Detector-->>Engine: list[Collision]
        CLI->>Renderer: render()
        CLI->>Renderer: tick()
    end
```

---

## Simulation Loop

```
INITIALIZE CLI + parse args
         │
         ▼
    LOAD SCENARIO (JSON → Grid + RobotSpecs)
         │
         ▼
    CREATE FLEET (add AMR for each spec)
         │
         ▼
    CREATE ENGINE (grid, fleet, config)
         │
         ▼
    PLAN ALL PATHS (each AMR calls A* independently)
         │
         ▼
    DETECT INITIAL PATH CONFLICTS (static analysis)
         │
         ▼
  ┌─── RUN LOOP ◄──────────────────────────┐
  │      │                                   │
  │      ▼                                   │
  │  HANDLE EVENTS (quit?)                   │
  │      │                                   │
  │      ▼                                   │
  │  UPDATE ENGINE                           │
  │    ├── step ALL robots (insertion order)  │
  │    ├── detect collisions                 │
  │    └── record metrics                    │
  │      │                                   │
  │      ▼                                   │
  │  RENDER (draw all robots, paths, legend) │
  │      │                                   │
  │      ▼                                   │
  │  ALL ARRIVED? ─── No ───────────────────┘
  │      │
  │     Yes
  │      │
  └──── PRINT METRICS + END
```

---

## Conflict Detection Model

### Key Distinction

| Concept | When | How |
|---------|------|-----|
| **Path Conflict** | Before/during execution | Static analysis of planned trajectories |
| **Collision** | During execution | Runtime position comparison |

### Conflict Types

```mermaid
flowchart LR
    subgraph "NODE Conflict"
        A1["AMR-01 at (5,5) t=3"]
        A2["AMR-02 at (5,5) t=3"]
        A1 -.->|"same cell, same time"| A2
    end

    subgraph "EDGE Conflict"
        B1["AMR-01: (3,0)→(4,0) t=2→3"]
        B2["AMR-02: (4,0)→(3,0) t=2→3"]
        B1 -.->|"swap positions"| B2
    end
```

### Temporal Model

Paths are treated as timed trajectories where the waypoint index = discrete timestep:

```
AMR-01: t=0:(1,1) → t=1:(2,1) → t=2:(3,1) → t=3:(4,1) → ...
AMR-02: t=0:(7,1) → t=1:(6,1) → t=2:(5,1) → t=3:(4,1) → ...
                                                    ↑
                                            NODE conflict at t=3
```

After a robot's path ends, it stays at its final position (clamped).

---

## A* Planning Flow

```mermaid
flowchart TD
    Start[find_path called] --> Validate{Validate start & goal}
    Validate -->|Invalid| Error[Raise ValueError]
    Validate -->|start == goal| Trivial[Return Path of length 1]
    Validate -->|Valid| Init[Initialize open set with start]

    Init --> Loop{Open set empty?}
    Loop -->|Yes| NoPath[Return None]
    Loop -->|No| Pop[Pop lowest f-cost node]

    Pop --> GoalCheck{Is it the goal?}
    GoalCheck -->|Yes| Reconstruct[Reconstruct path via came_from]
    GoalCheck -->|No| Expand[Get traversable neighbours]

    Expand --> UpdateCosts[Update g, h, f costs]
    UpdateCosts --> Push[Push improved neighbours to open set]
    Push --> Loop
```

---

## Future Extension Points

| Future Feature | Extension Point |
|---------------|----------------|
| Conflict resolution | Add `coordination/resolver.py` — responds to detected Conflicts |
| New cell types | Add variants to `CellType` enum |
| Re-planning | AMR calls `plan()` again with updated grid state |
| Local world model | AMR stores its own `Grid` copy, updated via sensors |
| Peer communication | Add `CommunicationChannel` injected into each AMR |
| Battery/velocity | Add fields to AMR |
| New planners | Create `planning/dijkstra.py`, same `find_path()` interface |
| Benchmarking | Use `--headless` mode, compare metrics across strategies |
| Stop-and-wait baseline | Add `coordination/stop_and_wait.py` — pause on conflict |
| Dashboard | Read-only observer, same as Renderer pattern |

### Future Robot Architecture (Decentralised)

```
Robot N
├── local state (position, battery, task)
├── local planner (A* or better)
├── local world model (own copy of grid + peer positions)
├── peer communication (P2P messaging)
└── local decision making (conflict resolution, task negotiation)
```

No central `FleetController`. The simulation engine orchestrates *time* (ticks), not *decisions*.

---

## Testing Strategy

- **105 tests** across 7 test files
- Tests depend only on domain modules (`warehouse`, `planning`, `robot`, `coordination`, `simulation`)
- Tests never import Pygame
- Tests verify **behaviour**, not implementation details
- All pathfinding edge cases are covered
- Conflict detection validated with temporal awareness
- Phase 1 backward compatibility verified in scenario tests
