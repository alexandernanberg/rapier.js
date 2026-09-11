# rts-sim

Deterministic lockstep simulation core for an AoE2-style RTS. This is the
scaffold every later system gets written against — movement, combat, pathing,
economy — not a game yet.

It deliberately has **no dependency on rapier, three.js, React or the DOM**. A
rigid-body solver is the wrong tool for RTS unit movement (see "Why no
physics" below), and the sim must be able to run headless, in a worker, or on a
server.

## Why lockstep

Clients exchange _orders_, never state, and each one runs a bit-identical
simulation. Two consequences make this worth adopting even before there is
multiplayer:

- **Replays and saves are the same feature.** The match is a pure function of
  `(config, order log)`, so a few kilobytes reproduce it exactly.
- **Desync detection is a regression test.** `verifyReplay` re-runs a recorded
  log and reports the first tick whose state hash disagrees. That turns a
  refactor — or an ECS migration — from a silent-bug hunt into pass/fail.

## The rules

Anything under `src/sim/` and `src/core/` is simulation code and must obey
these. Breaking one does not fail loudly; it desyncs a match an hour in.

1. **No `Math.random()`.** Draw from `world.rng` (seeded xoshiro128\*\*), which
   is part of the world state and therefore part of the checksum.
2. **No `Math.sin`, `cos`, `tan`, `asin`, `acos`, `atan`, `atan2`, `exp`,
   `log`, `pow`, `hypot` or `cbrt`.** These are engine-dependent
   approximations. Use `core/math.ts`, which is pure arithmetic.
   `+ - * /`, `Math.sqrt`, `abs`, `floor`, `ceil`, `round`, `sign`, `min`,
   `max` and `imul` _are_ exactly specified and safe.
3. **No wall-clock.** No `Date.now`, no `performance.now`, no variable
   timestep. `dt` is a constant in `SimConfig`.
4. **No structural changes inside a system.** Spawning, despawning and
   adding/removing components go through `world.cmd`, applied at one fixed
   point in the tick. This is what stops "one unit in fifty doesn't take
   damage" bugs, and it keeps the semantics ours rather than the ECS's.
5. **Never index a store with an entity handle.** bitECS packs a generation
   counter into the handle; use `idOf(world, eid)`. Getting this wrong reads
   another unit's data _deterministically_, so the checksum will not catch it.
6. **Register new state in `COMPONENT_SPECS`.** Hashing and snapshotting are
   driven from that list. `components.test.ts` fails the typecheck if it drifts
   from `Stores`. State that cannot live in a component — the pathfinder's
   pending queue, the terrain — needs its own `hashInto` and a call from
   `hashWorld`. Anything stateful in neither place is invisible to desync
   detection.
7. **Budget work by count, never by time.** "As many paths as fit in 3 ms"
   makes the simulation a function of how fast the machine is, which desyncs on
   the first slow frame. A fixed K per tick means a busy moment costs latency
   instead of correctness.
8. **Break every tie explicitly.** A comparator that leaves equal-cost items
   in container order — A*'s open set is the obvious one — lets two clients
   pick differently. Fall through to something unique, like a tile index.

## Shape of a tick

```
1. take this tick's orders, sorted by (player, seq)   <- network order cannot matter
2. flush commands                                     <- orders take effect this tick
3. run SYSTEMS in declared order                      <- systems queue, never mutate structure
4. flush commands                                     <- system effects land at one known point
```

`SYSTEMS` is a literal list, so the order is reviewable:

```
pathRequest -> pathService -> movement -> separation -> death
```

Requests are queued before they are serviced, so a unit ordered this tick can
be pathed this tick if the budget allows. Separation runs after movement so it
corrects the positions movement just wrote.

Reordering any of this changes results and invalidates every recorded replay.
Bump `SIM_VERSION` when you do.

## Cost

Measured at 2000 units on a 128x128 map with obstacles, all ordered to one
destination:

|                          | per tick    |
| ------------------------ | ----------- |
| whole tick, steady state | **0.84 ms** |
| `hashWorld()`            | 1.87 ms     |

The first tick of that order builds **one** flow field and runs **zero**
searches, serving all 2000 units at once. Before flow fields the same scene cost
21 ms a tick and still had a pathfinding queue that never drained.

### Why the threshold is 4

A corner-to-corner A* search serves one unit; a flow field serves every unit
heading to that tile:

| map     | one A* search | one flow field | field expansions |
| ------- | ------------- | -------------- | ---------------- |
| 64x64   | 0.74 ms       | 1.12 ms        | 3 994            |
| 128x128 | 2.07 ms       | 4.39 ms        | 16 180           |
| 256x256 | 7.45 ms       | 19.15 ms       | 65 128           |

A field costs about 2.1 searches, so it pays for itself at three units sharing a
destination. `flowFieldThreshold` defaults to 4 — just above the measured
break-even, so a pair of scouts still gets cheap individual searches.

### What is still a wall

A 256x256 field costs 19 ms, which is 38% of a 50 ms tick for a single build.
Two ways out when maps get that big, neither built yet:

- Spread one Dijkstra across several ticks (resumable, budgeted by expansions
  rather than by whole fields).
- Hierarchical clusters, so a field covers a portal graph rather than every
  tile.

Scattered destinations also still cost one search each, bounded by `pathBudget`.
That is the case HPA\* would fix.

The checksum is not free either — it walks every entity and component in sorted
order. Hash every tick in tests, where naming the exact tick of a divergence is
the point; in a real match compare every 20-30 ticks.

## Layout

| Path                       | Purpose                                           |
| -------------------------- | ------------------------------------------------- |
| `core/rand.ts`             | Seeded PRNG; state is hashable                    |
| `core/math.ts`             | Deterministic trig and vector helpers             |
| `core/hash.ts`             | 64-bit FNV-1a over typed arrays                   |
| `sim/components.ts`        | Component stores and the spec that drives hashing |
| `sim/command_buffer.ts`    | Deferred structural changes                       |
| `sim/orders.ts`            | Player orders and canonical per-tick ordering     |
| `sim/world.ts`             | World construction, command application           |
| `sim/systems.ts`           | Systems and the explicit schedule                 |
| `sim/tick.ts`              | The fixed timestep                                |
| `sim/snapshot.ts`          | Canonical hash, readable dump, world diff         |
| `sim/grid/tile_map.ts`     | Terrain: integer weights, world/tile conversion   |
| `sim/grid/astar.ts`        | A* for one unit's route                           |
| `sim/grid/heap.ts`         | Min-heap with the ordering both planners need     |
| `sim/grid/spatial_hash.ts` | Uniform-grid neighbour queries                    |
| `sim/path/flow_field.ts`   | Cost-to-goal for a whole map, plus its cache      |
| `sim/path/pathfinder.ts`   | Route requests and the routing policy             |
| `replay.ts`                | Recording and verifying an order log              |

## Usage

```ts
import {createSimWorld, Recorder, OrderType, run, step, hashWorld, idOf} from "rts-sim";
import {getAllEntities} from "bitecs";

const world = createSimWorld({
    seed: 0xc0ffee,
    mapWidth: 64,
    mapHeight: 64,
    // Terrain is declared, not mutated, so the config alone reproduces the map
    // and a replay log needs to carry nothing else about it.
    obstacles: [{x: 30, y: 0, w: 2, h: 50, weight: 0}],
});
const recorder = new Recorder();

// Player 0 spawns a militia at (10, 10). The order executes `orderDelay`
// ticks later, so the unit does not exist yet.
recorder.issue(world, 0, OrderType.Spawn, 1, 10, 10);
run(world, world.config.orderDelay + 1);

// Orders address entities by handle, so send it somewhere once it exists.
const [militia] = getAllEntities(world);
recorder.issue(world, 0, OrderType.Move, militia, 50, 12);

// Routing round the wall is about 100 world units, and a militia covers 0.17
// per tick. `readme.test.ts` runs this, so it cannot drift.
run(world, 800);

const {Position} = world.stores;
console.log(Position.x[idOf(world, militia)], hashWorld(world));

// `recorder.orders` plus the config is the whole replay.
```

When a replay disagrees, `hashWorld` names the tick and `diffWorlds` names the
entity and field:

```ts
const diffs = diffWorlds(mine, theirs);
// [{entityId: 3, component: "Position", field: "x", left: 12.5, right: 12.500000001}]
```

## Choice of ECS

bitECS 0.4, because components are user-managed: we allocate the typed arrays,
bitECS only tracks which entity has which component. That is what makes
`hashWorld` a read of arrays we own, and what will make the eventual worker
handoff a copy rather than a reconstruction.

The portability layer is the command buffer and the snapshot, not a facade over
the ECS API — abstracting over an ECS ends in writing a worse one. Replacing
bitECS means replacing the query and entity-lifecycle layer while the stores and
every system's inner loop stay put.

## Movement, and why there is no physics

Unit movement is three layers, none of which is a solver:

1. **Route** — three tiers, cheapest first. A cached flow field is free; one is
   built when `flowFieldThreshold` units share a destination; anything else
   gets a single A* search, bounded by `pathBudget`. Ties in either planner's
   open set break on `(cost, tieBreak, tileIndex)`; without that last term two
   clients pick different equal-cost routes and desync, which is why both go
   through the same `TieBrokenHeap`.
2. **Follow** — steer at the next waypoint, retiring any already reached; or,
   on a flow field, read the field at whatever tile the unit currently stands
   on. The field case has no route length to truncate and nothing to re-plan,
   and a unit shoved aside by separation recovers for free. A unit with no route
   yet walks straight at its order position rather than freezing, which reads as
   responsiveness instead of input lag.
3. **Separate** — symmetric circle push-apart over the spatial grid, each unit
   resolving half of each overlap.

AoE2 has no rigid-body dynamics, and neither does this. Projectiles will be
analytic ballistic arcs and terrain is a heightmap sample. A constraint solver
would be slower, harder to tune, non-deterministic across platforms, and would
fight the pathfinder for control of unit positions.

A physics engine is still the right tool for _cosmetic_ effects — collapsing
buildings, debris, ragdolls — run client-side in the render layer, seeded
per-client, never feeding a single bit back into the sim.

## Not yet built

Formations (40 units ordered to one point currently jostle around it), HPA\* for
scattered long routes, resumable field builds for large maps, combat, vision and
fog, the network transport, and the render layer. Each is written against the
harness here.
