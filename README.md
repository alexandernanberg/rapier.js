<p align="center">
  <img src="https://www.rapier.rs/img/rapier_logo_color_textpath_dark.svg" alt="crates.io">
</p>
<p align="center">
    <a href="https://discord.gg/vt9DJSW">
        <img src="https://img.shields.io/discord/507548572338880513.svg?logo=discord&colorB=7289DA">
    </a>
    <a href="https://opensource.org/licenses/Apache-2.0">
        <img src="https://img.shields.io/badge/License-Apache%202.0-blue.svg">
    </a>
</p>
<p align = "center">
    <strong>
        <a href="https://rapier.rs">Website</a> | <a href="https://rapier.rs/docs/">Documentation</a>
    </strong>
</p>

---

<p align = "center">
<b>2D and 3D physics engines</b>
<i>for the JavaScript programming language.</i>
</p>

---

## Fork Differences

This is a fork of [@dimforge/rapier.js](https://github.com/dimforge/rapier.js) with performance improvements and modernized tooling.

- Rapier 0.35 with glam math library
- pnpm monorepo with tsdown bundler
- Contiguous transform buffer (body reads with zero WASM crossings)
- Zero-allocation getters and scene queries (optional target parameter)
- Batch transform setters (`setTransform`, `setNextKinematicTransform`)
- Full `IntegrationParameters` surface (warm-starting, contact softness, contact clustering/recycling, friction model)
- Contact modification hooks (`PhysicsHooks.modifySolverContacts`) and the full multibody joint API (anchors, limits, motors)
- Built-in benchmarks
- Simplified package variants (2 per dimension, SIMD by default)

### Benchmarks

3D, 5000 bodies / 1000 casts (`world.step()` on a 3000-body pyramid), mean
times (Apple M1 Max, Node v24.16.0):

| Benchmark                                 | Fork    | Official | Speedup |
| ------------------------------------------ | ------- | -------- | ------- |
| world.step() [active]                     | 2.262ms | 5.102ms  | 2.3x    |
| world.step() [sleeping]                   | 755ns   | 2.5µs    | 3.3x    |
| create 5000 bodies+colliders               | 3.312ms | 15.831ms | 4.8x    |
| spawn+despawn 500 bodies (1000 resident)   | 419.0µs | 1.649ms  | 3.9x    |
| castRay x1000                              | 487.6µs | 1.212ms  | 2.5x    |
| castRayAndGetNormal x1000                  | 899.2µs | 1.735ms  | 1.9x    |
| intersectionsWithRay x1000                 | 825.9µs | 2.148ms  | 2.6x    |
| projectPoint x1000                         | 1.581ms | 2.709ms  | 1.7x    |
| intersectionsWithPoint x1000               | 104.8µs | 308.0µs  | 2.9x    |
| body.translation() [alloc]                 | 31.9µs  | 321.2µs  | 10.1x   |
| body.translation() [reuse]                 | 26.9µs  | 313.6µs  | 11.7x   |
| body.rotation() [alloc]                    | 29.4µs  | 344.8µs  | 11.7x   |
| body.rotation() [reuse]                    | 30.0µs  | 346.1µs  | 11.5x   |
| body.linvel() [alloc]                      | 31.5µs  | 320.0µs  | 10.2x   |
| body.linvel() [reuse]                      | 27.8µs  | 317.2µs  | 11.4x   |
| collider.translation() [alloc]             | 35.2µs  | 315.3µs  | 9.0x    |
| collider.translation() [reuse]             | 32.7µs  | 317.7µs  | 9.7x    |
| body.setTransform()                        | 109.8µs | 154.1µs  | 1.4x    |
| body.setNextKinematicTransform()           | 100.2µs | 152.5µs  | 1.5x    |
| body.setLinvel()                           | 68.3µs  | 970.4µs  | 14.2x   |
| body.applyImpulse()                        | 74.1µs  | 976.4µs  | 13.2x   |
| body.addForce()                            | 69.6µs  | 970.7µs  | 14.0x   |
| body.applyImpulseAtPoint()                 | 105.7µs | 1.904ms  | 18.0x   |

Official = `@dimforge/rapier3d-compat` v0.20.0. Reuse = optional zero-allocation
`target` parameter — official added the same parameter to these getters in
this release, so both variants are now real measurements for both packages;
the fork's remaining edge comes from reading a JS-side buffer synced once per
`world.step()` instead of crossing into WASM on every call. Run `pnpm bench` /
`pnpm bench --official` to benchmark on your machine.

### What Makes It Faster

**Contiguous transform buffer (zero WASM crossings for body reads)**

Body transforms are synced into a contiguous `Float32Array` backed by WASM linear memory during `world.step()`. Reading `translation()`, `rotation()`, `linvel()`, and `angvel()` reads directly from this buffer with no WASM boundary crossing.

```typescript
// Reads from shared Float32Array — no WASM call
const pos = body.translation();

// Zero-allocation variant (reuses existing object)
const _pos = {x: 0, y: 0, z: 0};
body.translation(_pos);
```

Supported: `translation()`, `rotation()`, `linvel()`, `angvel()`, `nextTranslation()`, `nextRotation()`, `localCom()`, `worldCom()`

**Zero-allocation scene queries**

Scene queries and the remaining vector getters take the same optional `target`, passed as the last argument (after the filter arguments). The target is returned as-is, so a hot loop can reuse one result object:

```typescript
const _hit = new RAPIER.RayColliderIntersection(undefined!, 0, {x: 0, y: 0, z: 0});

// Writes into _hit instead of allocating a hit and a normal vector
const hit = world.castRayAndGetNormal(
    ray,
    100,
    true,
    undefined,
    undefined,
    undefined,
    undefined,
    undefined,
    _hit,
);
```

Supported on `castRayAndGetNormal()`, `projectPoint()`, `projectPointAndGetFeature()` and `castShape()` (on `World` and `BroadPhase`), on the `Collider` and `Shape` query methods, on the mass-property and force getters of `RigidBody`, on joint anchors and frames, and on the vehicle controller's wheel getters. A query that misses returns `null` and leaves the target untouched.

**Optimized ray casting**

Ray origin/direction passed as primitives directly to WASM, avoiding temporary `RawVector` allocations.

---

## Installation

```bash
# 2D physics
npm install @alexandernanberg/rapier2d

# 3D physics
npm install @alexandernanberg/rapier3d
```

## Usage

```typescript
import RAPIER from "@alexandernanberg/rapier2d";

await RAPIER.init();

const gravity = {x: 0.0, y: -9.81};
const world = new RAPIER.World(gravity);

// Create a dynamic rigid body
const bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(0.0, 10.0);
const body = world.createRigidBody(bodyDesc);

// Create a collider attached to the body
const colliderDesc = RAPIER.ColliderDesc.ball(0.5);
world.createCollider(colliderDesc, body);

// Run the simulation
world.step();
console.log(body.translation()); // { x: 0, y: ~9.99 }
```

## Package Variants

Each package ships 2 variants via subpath exports. Both are built with WASM
SIMD (`simd128`) enabled:

| Import Path                         | WASM Loading         |
| ----------------------------------- | -------------------- |
| `@alexandernanberg/rapier2d`        | `fetch()` at runtime |
| `@alexandernanberg/rapier2d/compat` | Embedded base64      |

**When to use which:**

- **Default**: Best for web apps (smaller bundle, parallel loading)
- **Compat**: For environments without `fetch()` (SSR, workers, tests)

Both variants require [simd128 support](https://caniuse.com/?search=simd), which is
available in Chrome 91+, Firefox 89+, Safari 16.4+ and Node 16.4+. Since Rapier 0.35
SIMD is always compiled in, so a non-SIMD build would run the same code paths
scalar — measurably slower (~1.5x on contact-heavy 3D scenes) rather than smaller.

## Building from Source

### Prerequisites

- Node.js 24+
- pnpm (`npm install -g pnpm`)
- Rust toolchain (`rustup`)
- wasm-pack (`cargo install wasm-pack`)

### Build Commands

```bash
pnpm install            # Install dependencies
pnpm build              # Full build (WASM + TypeScript)
pnpm build:wasm         # WASM only (2D + 3D)
pnpm build:ts           # TypeScript only
pnpm build:2d           # 2D package only
pnpm build:3d           # 3D package only
```

### Running Testbeds

```bash
pnpm dev:testbed2d      # http://localhost:5173
pnpm dev:testbed3d      # http://localhost:5173
```

### Benchmarks

```bash
pnpm bench                    # Run and compare against baseline
pnpm bench --save-baseline    # Save current results as new baseline
pnpm bench --no-compare       # Run without baseline comparison
pnpm bench --no-memory        # Skip the allocation/GC measurements
pnpm bench:2d                 # Full 2D benchmark
pnpm bench --quick            # Quick mode (fewer iterations)
```

**Sizes:** 5000 bodies and 1000 casts per iteration (1000 and 250 under
`--quick`), so that one iteration is hundreds of microseconds of real work rather
than a handful of operations wrapped in loop and timer overhead.

`world.step()` is measured twice. A settled pyramid puts essentially every island
to sleep — 1 of 3001 bodies stays awake — and stepping it costs about a thousand
times less than stepping the same scene while it is active, so the two are
reported separately rather than as one number that depends on how long the scene
was left to settle.

**Allocations:** after the timing benchmarks, the suite measures how much JS heap
each read path allocates per operation, and how much GC that causes per million
operations. Timing and allocation are measured separately: allocation needs a
quiet heap and forced collections, which the timing harness deliberately avoids.

Each measurement window is sized to stay inside the young generation and repeated
seven times, discarding the windows where a collection ran and taking the
smallest of the rest — background allocation only ever adds to a window, and a
collection only ever subtracts. The scene and the query set are seeded, since how
much a query allocates depends on how often it hits.

**Baseline comparison:**

- Results are compared against `packages/benchmarks/baseline.json`
- Timing thresholds: >15% = warning, >30% = regression
- Allocation thresholds: >25% = warning, >50% = regression, ignoring changes
  under 128 bytes/op (below that it is measurement noise)
- Exit code 1 on regression (useful for CI)

## License

Apache 2.0
