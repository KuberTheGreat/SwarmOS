# SwarmOS — Architecture

## Overview

SwarmOS is structured around a strict separation of concerns that enables:
- **Headless simulation** for automated benchmarking
- **Independent robot agents** for decentralised multi-robot coordination
- **Modular planners** that can be swapped or extended
- **Clean testing** without GUI dependencies

---

## Module Responsibilities

```mermaid
graph TD
    CLI[cli.py] --> ML[map_loader]
    CLI --> Engine[SimulationEngine]
    CLI --> Renderer[Renderer]

    ML --> Grid[Grid]
    ML --> ScenarioData

    Engine --> Grid
    Engine --> AMR
    Engine --> Config[SimulationConfig]

    AMR --> Planner[A* Planner]
    AMR --> PathObj[Path]
    AMR --> State[RobotState]

    Planner --> Grid
    Planner --> PathObj

    Renderer --> Engine

    style Renderer fill:#2d5a27,color:#fff
    style Engine fill:#1a3a5c,color:#fff
    style Planner fill:#5c3a1a,color:#fff
    style Grid fill:#3a1a5c,color:#fff
    style AMR fill:#5c1a3a,color:#fff
```

### `warehouse/` — Environment Model

| Module | Responsibility |
|--------|---------------|
| `cell.py` | `Position` (immutable coordinate) and `CellType` (extensible enum) |
| `grid.py` | 2D grid: bounds checking, traversability, neighbour queries |
| `map_loader.py` | Parses scenario JSON → `Grid` + robot start/goal |

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

**Design rule**: Each AMR is an independent agent. It *uses* a planner but doesn't *contain* planning logic. This prepares for the future decentralised architecture where each robot has its own local world model and decision loop.

### `simulation/` — Engine

| Module | Responsibility |
|--------|---------------|
| `engine.py` | Discrete-tick simulation loop. Manages grid + robots + time. |
| `config.py` | Immutable configuration (cell size, tick rate, step delay) |

**Design rule**: The engine is headless-capable. It never imports Pygame. This is critical for running thousands of benchmark simulations without a display.

### `visualization/` — Rendering

| Module | Responsibility |
|--------|---------------|
| `renderer.py` | Pygame renderer. Reads engine state, draws it. Never mutates simulation state. |

**Design rule**: The renderer is a pure *observer*. It can be removed entirely without affecting simulation correctness.

---

## Data Flow

```mermaid
sequenceDiagram
    participant CLI
    participant Loader as MapLoader
    participant Engine
    participant AMR
    participant Planner as A*
    participant Renderer

    CLI->>Loader: load_scenario("basic_warehouse.json")
    Loader-->>CLI: ScenarioData (grid, start, goal)

    CLI->>AMR: AMR(id, start_position)
    CLI->>Engine: SimulationEngine(grid, robot, config)

    CLI->>Engine: plan_robot(goal)
    Engine->>AMR: plan(grid, goal)
    AMR->>Planner: find_path(grid, start, goal)
    Planner-->>AMR: Path (waypoints)

    CLI->>Renderer: Renderer(engine)

    loop Every Frame
        CLI->>Renderer: handle_events()
        CLI->>Engine: update()
        Engine->>AMR: step()
        AMR->>AMR: advance along path
        CLI->>Renderer: render()
        Renderer->>Engine: read state
        CLI->>Renderer: tick()
    end
```

---

## Simulation Loop

```
INITIALIZE CLI + parse args
         │
         ▼
    LOAD SCENARIO (JSON → Grid + start/goal)
         │
         ▼
    CREATE AMR (id, start position)
         │
         ▼
    CREATE ENGINE (grid, robot, config)
         │
         ▼
    PLAN PATH (AMR calls A* → Path)
         │
         ▼
  ┌─── RUN LOOP ◄──────────────────┐
  │      │                          │
  │      ▼                          │
  │  HANDLE EVENTS (quit?)          │
  │      │                          │
  │      ▼                          │
  │  UPDATE ENGINE (tick + step)    │
  │      │                          │
  │      ▼                          │
  │  RENDER (draw state)            │
  │      │                          │
  │      ▼                          │
  │  GOAL REACHED? ─── No ─────────┘
  │      │
  │     Yes
  │      │
  └──── END
```

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

The Phase 1 architecture is designed so these additions are natural, not refactors:

| Future Feature | Extension Point |
|---------------|----------------|
| Multiple robots | Engine holds `list[AMR]`; update loop iterates all |
| New cell types | Add variants to `CellType` enum |
| Re-planning | AMR calls `plan()` again with updated grid state |
| Local world model | AMR stores its own `Grid` copy, updated via sensors |
| Peer communication | Add `CommunicationChannel` injected into each AMR |
| Conflict resolution | AMR checks reservations before `step()` |
| Battery/velocity | Add fields to AMR (slots already extensible) |
| New planners | Create `planning/dijkstra.py`, same interface |
| Benchmarking | Use `--headless` mode, run many scenarios |
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

- Tests depend only on domain modules (`warehouse`, `planning`, `robot`)
- Tests never import Pygame
- Tests verify **behaviour**, not implementation details
- All pathfinding edge cases are covered: unreachable goals, invalid positions, obstacles, trivial paths
