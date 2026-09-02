# SwarmOS — Architecture

## Overview

SwarmOS is structured around a strict separation of concerns that enables:
- **Headless simulation** for automated benchmarking
- **Independent robot agents** for decentralised multi-robot coordination
- **Modular planners** that can be swapped or extended
- **Clean testing** without GUI dependencies
- **Conflict detection** separate from conflict resolution
- **Pluggable coordination policies** for movement decision strategies

---

## Module Responsibilities

```mermaid
graph TD
    CLI[cli.py] --> ML[map_loader]
    CLI --> Engine[SimulationEngine]
    CLI --> Renderer[Renderer]
    CLI --> PolicySelect["--policy flag"]

    ML --> Grid[Grid]
    ML --> ScenarioData
    ML --> RobotSpec

    Engine --> Grid
    Engine --> Fleet
    Engine --> Config[SimulationConfig]
    Engine --> Metrics[SimulationMetrics]
    Engine --> Detector[Conflict Detector]
    Engine --> Policy[CoordinationPolicy]

    Policy --> SAW[StopAndWaitPolicy]
    Policy --> Future["Future: DecentralizedPolicy"]

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
    style Policy fill:#5c3a5c,color:#fff
    style SAW fill:#5c3a5c,color:#fff
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
| `amr.py` | AMR class: owns its position, state, path. Uses the planner. Phase 3: `wait()`, `resume()`, `intended_next_position`. |
| `state.py` | `RobotState` enum (IDLE → MOVING → ARRIVED; Phase 3: WAITING) |
| `fleet.py` | Fleet: ordered collection of AMRs with unique IDs. State container, not a controller. |

**Design rule**: Each AMR is an independent agent. The Fleet holds them but doesn't direct them. This prepares for the decentralised architecture where each robot owns its own decision-making.

### `coordination/` — Conflict Detection & Coordination Policies

| Module | Responsibility |
|--------|---------------|
| `conflict.py` | Data models: `Conflict` (path overlap), `Collision` (runtime overlap), `ConflictType` enum |
| `detector.py` | Static path conflict detection (node + edge) and runtime collision detection |
| `decision.py` | `Decision` enum (MOVE/WAIT) and `RobotSnapshot` dataclass *(Phase 3)* |
| `policy.py` | `CoordinationPolicy` ABC and `StopAndWaitPolicy` concrete implementation *(Phase 3)* |

**Design rule**: Detection ≠ Resolution. Policies are pluggable strategies applied to snapshots — NOT centralized controllers. Each robot's decision depends only on the observable shared state.

### `simulation/` — Engine

| Module | Responsibility |
|--------|---------------|
| `engine.py` | Discrete-tick simulation loop. Manages grid + fleet + time + metrics. Phase 3: snapshot-based policy-aware updates. |
| `config.py` | Immutable configuration (cell size, tick rate, step delay) |
| `metrics.py` | Per-robot and aggregate metrics (steps, arrivals, collisions, conflicts, wait ticks, movements) |

**Design rule**: The engine is headless-capable. It never imports Pygame. It orchestrates *time*, not *decisions*.

### `visualization/` — Rendering

| Module | Responsibility |
|--------|---------------|
| `renderer.py` | Pygame renderer. Draws all robots with distinct colours, paths, goals, IDs, waiting indicators, and legend. |

**Design rule**: The renderer is a pure *observer*. It can be removed without affecting simulation correctness.

---

## Coordination Policy *(Phase 3)*

### Policy Interface

```mermaid
classDiagram
    class CoordinationPolicy {
        <<abstract>>
        +name: str
        +decide(snapshots: list~RobotSnapshot~) dict~str, Decision~
    }

    class StopAndWaitPolicy {
        +name: str = "STOP_AND_WAIT"
        +decide(snapshots) dict~str, Decision~
    }

    class Decision {
        <<enumeration>>
        MOVE
        WAIT
    }

    class RobotSnapshot {
        <<frozen dataclass>>
        +robot_id: str
        +position: Position
        +intended_next: Position
        +state: RobotState
    }

    CoordinationPolicy <|-- StopAndWaitPolicy
    CoordinationPolicy ..> RobotSnapshot : "input"
    CoordinationPolicy ..> Decision : "output"
```

### Stop-and-Wait Decision Flow

```mermaid
flowchart TD
    Start["For each active robot"] --> Check1{"Intended next\n== position?"}
    Check1 -->|Yes| Move1[MOVE — staying put]
    Check1 -->|No| Check2{"Node conflict?\nMultiple robots → same cell"}

    Check2 -->|Yes| Check3{"Lowest ID?"}
    Check3 -->|Yes| Move2[MOVE — wins tie]
    Check3 -->|No| Wait1[WAIT — loses tie]

    Check2 -->|No| Check4{"Edge conflict?\nSwap positions"}
    Check4 -->|Yes| Wait2[WAIT — both wait]

    Check4 -->|No| Check5{"Occupancy conflict?\nTarget cell occupied"}
    Check5 -->|Yes| Check6{"Occupant leaving?"}
    Check6 -->|Yes| Move3[MOVE — cell being vacated]
    Check6 -->|No| Wait3[WAIT — cell blocked]
    Check5 -->|No| Move4[MOVE — no conflict]
```

### Snapshot-Based Simultaneous Updates

The engine ensures that all robots are evaluated against the **same** world state:

```
TICK N:
  1. SNAPSHOT ── Read each robot's position + intended_next + state
  2. DECIDE  ── Pass all snapshots to policy.decide()
  3. APPLY   ── For each robot:
                  WAIT → robot.wait() → record_wait()
                  MOVE → robot.resume() + robot.step() → record_movement()
  4. DETECT  ── Run collision detection on final positions
  5. RECORD  ── Update metrics (arrivals, collisions)
```

**Critical property**: Robot iteration order does NOT affect the result. Decisions are made from the frozen snapshot, not from live state.

### Deterministic Tie-Breaking

When two or more robots conflict, the winner is determined by **lexicographic string comparison** of robot IDs:

```
"AMR-01" < "AMR-02" < "AMR-03" < ... < "AMR-10"
```

- Lower ID wins (gets MOVE)
- Higher ID yields (gets WAIT)
- Applies to node conflicts only — edge conflicts require both to wait

### Known Limitations (by design)

These are intentional limitations of the naive baseline:

| Limitation | Why |
|-----------|-----|
| Head-on deadlock | No rerouting — robots can't go around each other |
| Edge conflict deadlock | Both wait forever if paths can't diverge |
| No priority negotiation | By design — future phases will add this |
| No communication | By design — future phases will add P2P |

---

## Multi-AMR Data Flow

```mermaid
sequenceDiagram
    participant CLI
    participant Loader as MapLoader
    participant Fleet
    participant Engine
    participant Policy as CoordinationPolicy
    participant AMR1 as AMR-01
    participant AMR2 as AMR-02
    participant Planner as A*
    participant Detector
    participant Renderer

    CLI->>Loader: load_scenario("scenario.json")
    Loader-->>CLI: ScenarioData (grid, robot_specs[])

    CLI->>Fleet: Fleet()
    loop For each RobotSpec
        CLI->>Fleet: add(AMR(id, start))
    end

    CLI->>Policy: StopAndWaitPolicy() [if --policy stop_and_wait]
    CLI->>Engine: SimulationEngine(grid, fleet, config, policy)
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

        Note over Engine,Policy: Snapshot-based update (Phase 3)
        Engine->>Engine: Build RobotSnapshot[]
        Engine->>Policy: decide(snapshots)
        Policy-->>Engine: dict[robot_id → Decision]
        Engine->>AMR1: wait() or resume()+step()
        Engine->>AMR2: wait() or resume()+step()

        Engine->>Detector: detect_collisions(fleet, tick)
        Detector-->>Engine: list[Collision]
        CLI->>Renderer: render()
        CLI->>Renderer: tick()
    end
```

---

## Simulation Loop

```
INITIALIZE CLI + parse args + select policy
         │
         ▼
    LOAD SCENARIO (JSON → Grid + RobotSpecs)
         │
         ▼
    CREATE FLEET (add AMR for each spec)
         │
         ▼
    CREATE POLICY (if --policy specified)
         │
         ▼
    CREATE ENGINE (grid, fleet, config, policy)
         │
         ▼
    PLAN ALL PATHS (each AMR calls A* independently)
         │
         ▼
    DETECT INITIAL PATH CONFLICTS (static analysis)
         │
         ▼
  ┌─── RUN LOOP ◄──────────────────────────────┐
  │      │                                       │
  │      ▼                                       │
  │  HANDLE EVENTS (quit?)                       │
  │      │                                       │
  │      ▼                                       │
  │  UPDATE ENGINE                               │
  │    ├── [With policy]                         │
  │    │   ├── snapshot all robots               │
  │    │   ├── policy.decide(snapshots)           │
  │    │   ├── apply MOVE/WAIT per robot          │
  │    │   └── record wait/movement metrics       │
  │    ├── [Without policy]                      │
  │    │   └── step ALL robots (insertion order)  │
  │    ├── detect collisions                     │
  │    └── record metrics                        │
  │      │                                       │
  │      ▼                                       │
  │  RENDER (draw robots, paths, waiting, legend) │
  │      │                                       │
  │      ▼                                       │
  │  ALL ARRIVED? ─── No ────────────────────────┘
  │      │
  │     Yes
  │      │
  └──── PRINT METRICS + END
```

---

## Robot State Machine

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> MOVING : plan(goal) succeeds
    IDLE --> ARRIVED : plan(goal) where start==goal
    MOVING --> ARRIVED : path complete
    MOVING --> WAITING : wait() [policy conflict]
    WAITING --> MOVING : resume() [conflict cleared]
    WAITING --> WAITING : wait() [conflict persists]
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
| Decentralised coordination | Add `coordination/decentralized.py` implementing `CoordinationPolicy` |
| New cell types | Add variants to `CellType` enum |
| Re-planning / rerouting | AMR calls `plan()` again with updated grid state |
| Local world model | AMR stores its own `Grid` copy, updated via sensors |
| Peer communication | Add `CommunicationChannel` injected into each AMR |
| Battery/velocity | Add fields to AMR |
| New planners | Create `planning/dijkstra.py`, same `find_path()` interface |
| Benchmarking | Use `--headless` mode, compare metrics across policies |
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

- **153 tests** across 9 test files
- Tests depend only on domain modules (`warehouse`, `planning`, `robot`, `coordination`, `simulation`)
- Tests never import Pygame
- Tests verify **behaviour**, not implementation details
- All pathfinding edge cases are covered
- Conflict detection validated with temporal awareness
- **Policy unit tests**: node/edge/occupancy conflicts, tie-breaking, iteration-order independence
- **Integration tests**: end-to-end scenarios, state transitions, metrics, determinism, backward compat
- Phase 1 backward compatibility verified in scenario tests

