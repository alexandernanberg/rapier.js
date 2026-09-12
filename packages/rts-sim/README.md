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
   in container order — A\*'s open set is the obvious one — lets two clients
   pick differently. Fall through to something unique, like a tile index. Both
   planners share `TieBrokenHeap` so there is one implementation to get right.
9. **Never trust pathfinding to keep units out of walls.** Separation pushes
   units with no idea where terrain is. Every write to `Position` goes through
   `moveClamped`.

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

|                          |             |
| ------------------------ | ----------- |
| whole tick, steady state | **1.10 ms** |
| `hashWorld()`            | 1.80 ms     |

### Segment build is flat in map size

One segment integrates a 3x3 block of sectors, so its cost depends on
`sectorSize` and nothing else:

| map     | one segment | a whole-map field would be |
| ------- | ----------- | -------------------------- |
| 64x64   | 0.028 ms    | 1.12 ms                    |
| 128x128 | 0.034 ms    | 4.39 ms                    |
| 256x256 | 0.041 ms    | 19.15 ms                   |
| 512x512 | 0.044 ms    | ~77 ms                     |

That is the entire reason the portal graph exists. A whole-map field grows with
the map; a segment does not.

### Path quality

Excess distance over a straight line, on open ground, measured end to end:

| goal from (4, 32) | whole-map, 8 directions | segments + LOS + slid portals |
| ----------------- | ----------------------- | ----------------------------- |
| (60, 32)          | 0.0%                    | 0.4%                          |
| (60, 36)          | 2.7%                    | 0.7%                          |
| (60, 44)          | 6.5%                    | 7.3%                          |
| (60, 56)          | 8.2%                    | 1.3%                          |
| (60, 60)          | 8.0%                    | 1.1%                          |
| **mean**          | **5.1%**                | **2.2%**                      |

Line-of-sight cells get the exact direction to their target, which is most of
open ground. The residual is portal-constrained crossings: a segment can only
aim at somewhere on its window's edge, so a route that wants to cut a corner
the window does not contain still bends. (60, 44) is a case where it bends
worse than the old whole-map field did — the honest shape of the trade.

### Terrain changes are incremental

A whole-graph rebuild is not affordable when every completed building invalidates
the graph, so node indices are stable — each sector owns a fixed arena,
subdivided by which of its four boundaries a node belongs to — and a change
relinks only the sectors whose node sets moved.

Cost of a 3x3 building going up, against rebuilding from scratch:

| map     | full rebuild | incremental | speedup |
| ------- | ------------ | ----------- | ------- |
| 64x64   | 3.5 ms       | 0.43 ms     | 8x      |
| 128x128 | 14.8 ms      | 1.46 ms     | 10x     |
| 256x256 | 90 ms        | 2.77 ms     | 33x     |
| 512x512 | 272 ms       | 3.12 ms     | 87x     |

The incremental figure is nearly flat in map size where the full rebuild grows
fourfold per doubling. It is not perfectly flat because the confined searches
index arrays sized to the whole map, so a larger map means colder memory.

`TileMap` tracks the bounding box of changed tiles for this, and `PortalGraph`
is its only consumer — everything else just compares `revision`. Separate
changes within one update merge into one box, which over-approximates the dirty
set but never misses it.

## Layout

| Path                       | Purpose                                               |
| -------------------------- | ----------------------------------------------------- |
| `core/rand.ts`             | Seeded PRNG; state is hashable                        |
| `core/math.ts`             | Deterministic trig and vector helpers                 |
| `core/hash.ts`             | 64-bit FNV-1a over typed arrays                       |
| `sim/components.ts`        | Component stores and the spec that drives hashing     |
| `sim/command_buffer.ts`    | Deferred structural changes                           |
| `sim/orders.ts`            | Player orders and canonical per-tick ordering         |
| `sim/world.ts`             | World construction, command application               |
| `sim/systems.ts`           | Systems and the explicit schedule                     |
| `sim/tick.ts`              | The fixed timestep                                    |
| `sim/snapshot.ts`          | Canonical hash, readable dump, world diff             |
| `sim/grid/tile_map.ts`     | Terrain: integer weights, world/tile conversion       |
| `sim/grid/sectors.ts`      | Sector partitioning; bounds every cost above the grid |
| `sim/grid/portal_graph.ts` | HPA\* portal graph and the abstract route             |
| `sim/grid/astar.ts`        | A\* over the raw grid; the cross-check for the graph  |
| `sim/grid/heap.ts`         | Min-heap with the ordering every planner needs        |
| `sim/grid/spatial_hash.ts` | Uniform-grid neighbour queries                        |
| `sim/path/flow_segment.ts` | Windowed flow field with a line-of-sight pass         |
| `sim/path/pathfinder.ts`   | Segment requests and their budget                     |
| `replay.ts`                | Recording and verifying an order log                  |

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

Four layers, none of which is a solver. The design follows the one Age of
Empires IV describes — portal graph, segmented flow, steering — because its
requirements are the same ones: hundreds of units, a grid, and terrain that
changes while they walk.

1. **Abstract route** — A\* over a portal graph, after HPA\*. Sectors are
   divided, portals detected on each shared edge, and portal-to-portal cost
   found by a search confined to one sector. A route is tens of nodes rather
   than thousands of tiles, and it tells a segment which couple of sectors are
   worth integrating at all.
2. **Flow segment** — a field over the 3x3 block of sectors around the one a
   unit stands in, integrated from the last route cell before the route leaves
   that window. Cells with a clear line to the target get the exact direction
   to it; the rest fall back to eight-direction Dijkstra, which is fine because
   they are the cells hugging an obstacle. Keyed by (sector, goal), so every
   unit in a sector heading the same way reads one segment and later orders
   across the same ground reuse it.
3. **Separate** — symmetric circle push-apart over the spatial grid, each unit
   resolving half of each overlap.
4. **Terrain collision** — positions are clamped out of walls on write, axis by
   axis so a blocked diagonal still slides. Pathfinding cannot cover this and is
   not meant to: separation pushes units with no idea where the walls are.

AoE2 has no rigid-body dynamics, and neither does this. Projectiles will be
analytic ballistic arcs and terrain is a heightmap sample. A constraint solver
would be slower, harder to tune, non-deterministic across platforms, and would
fight the pathfinder for control of unit positions.

A physics engine is still the right tool for _cosmetic_ effects — collapsing
buildings, debris, ragdolls — run client-side in the render layer, seeded
per-client, never feeding a single bit back into the sim.

### Reading

- [Pathing in Age of Empires IV: Flow Fields and Steering Behaviors](https://media.gdcvault.com/GDC+2022/Speaker+Slides/Pathing+In+Age_Cheng_Frank+2022-03-29+00.16.38.pdf)
  (Frank Cheng, GDC 2022) — the architecture this follows.
- [Crowd Pathfinding and Steering Using Flow Field Tiles](https://www.gameaipro.com/GameAIPro/GameAIPro_Chapter23_Crowd_Pathfinding_and_Steering_Using_Flow_Field_Tiles.pdf)
  (Elijah Emerson, Game AI Pro) — flow field tiles, which the above builds on.
- [Near Optimal Hierarchical Path-Finding](https://webdocs.cs.ualberta.ca/~mmueller/ps/hpastar.pdf)
  (Botea, Muller, Schaeffer) — the portal graph.

Not yet taken from that reading: an eikonal/fast-marching integration with an
8-bit gradient for the shadowed cells, the BFS-and-shadow-lines form of the LOS
pass (faster than the per-cell raycast here), extended flow for mixed unit
sizes, and formations via a virtual leader.

## Not yet built

In rough order of how much they matter:

1. **Formations** — 40 units ordered at one point jostle around it, because
   they cannot all stand there. A virtual leader following the route with units
   holding spots around it, falling back to the flow when they have no line of
   sight to their spot, is the shape that works.
2. **Radius-aware terrain collision** — only unit centres are tested, so a
   radius can overlap a wall by a fraction of a tile. The real fix is obstacle
   steering, not a bigger clamp.
3. Combat, vision and fog, the network transport, and the render layer.
