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
   from `Stores`.

## Shape of a tick

```
1. take this tick's orders, sorted by (player, seq)   <- network order cannot matter
2. flush commands                                     <- orders take effect this tick
3. run SYSTEMS in declared order                      <- systems queue, never mutate structure
4. flush commands                                     <- system effects land at one known point
```

Reordering these phases changes results and invalidates every recorded replay.
Bump `SIM_VERSION` when you do.

## Cost

Measured at 2000 moving units on this machine:

| | per tick |
| --- | --- |
| `step()` | 0.14 ms (0.3% of a 50 ms budget) |
| `hashWorld()` | 1.33 ms |

The checksum is roughly 9x the cost of the tick it verifies, because it walks
every entity and component in sorted order. Hash every tick in tests, where
naming the exact tick of a divergence is the point; in a real match compare
every 20-30 ticks.

## Layout

| Path                    | Purpose                                           |
| ----------------------- | ------------------------------------------------- |
| `core/rand.ts`          | Seeded PRNG; state is hashable                    |
| `core/math.ts`          | Deterministic trig and vector helpers             |
| `core/hash.ts`          | 64-bit FNV-1a over typed arrays                   |
| `sim/components.ts`     | Component stores and the spec that drives hashing |
| `sim/command_buffer.ts` | Deferred structural changes                       |
| `sim/orders.ts`         | Player orders and canonical per-tick ordering     |
| `sim/world.ts`          | World construction, command application           |
| `sim/systems.ts`        | Systems and the explicit schedule                 |
| `sim/tick.ts`           | The fixed timestep                                |
| `sim/snapshot.ts`       | Canonical hash, readable dump, world diff         |
| `replay.ts`             | Recording and verifying an order log              |

## Usage

```ts
import {createSimWorld, Recorder, OrderType, step, hashWorld} from "rts-sim";

const world = createSimWorld({seed: 0xc0ffee});
const recorder = new Recorder();

recorder.issue(world, 0, OrderType.Spawn, 1, 10, 10); // player 0 spawns a militia
step(world);

console.log(hashWorld(world));
```

When a replay disagrees, `hashWorld` names the tick and `diffWorlds` names the
entity and field:

```ts
const diffs = diffWorlds(mine, theirs);
// [{entityId: 3, component: "Position", field: "x", left: 12.5, right: 12.500000001}]
```

## Why no physics

AoE2 has no rigid-body dynamics, and neither should this. Unit separation is a
circle push-apart on a tile grid, projectiles are analytic ballistic arcs, and
terrain is a heightmap sample. A constraint solver would be slower, harder to
tune, non-deterministic across platforms, and would fight the pathfinder for
control of unit positions.

A physics engine is still the right tool for _cosmetic_ effects — collapsing
buildings, debris, ragdolls — run client-side in the render layer, seeded
per-client, never feeding a single bit back into the sim.

## Choice of ECS

bitECS 0.4, because components are user-managed: we allocate the typed arrays,
bitECS only tracks which entity has which component. That is what makes
`hashWorld` a read of arrays we own, and what will make the eventual worker
handoff a copy rather than a reconstruction.

The portability layer is the command buffer and the snapshot, not a facade over
the ECS API — abstracting over an ECS ends in writing a worse one. Replacing
bitECS means replacing the query and entity-lifecycle layer while the stores and
every system's inner loop stay put.

## Not yet built

Spatial grid, pathfinding (HPA\* + flow fields), combat, vision and fog, the
network transport, and the render layer. Each is written against the harness
here.
