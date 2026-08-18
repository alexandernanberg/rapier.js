# CLAUDE.md - AI Assistant Context for rapier.js

## Project Overview

This is a **fork** of rapier.js - TypeScript bindings for the Rapier physics engine (Rust → WASM).

## Prerequisites

- Node.js 24+
- pnpm (`npm install -g pnpm`)
- Rust toolchain (`rustup`)
- wasm-pack (`cargo install wasm-pack`)

- **Package scope**: `@alexandernanberg/rapier-{2d,3d}`
- **Monorepo**: pnpm workspaces
- **Stack**: Rust + wasm-bindgen → WASM, TypeScript bindings

### Performance Goals

1. Minimize JS↔WASM boundary crossings
2. Avoid temporary object allocations in hot paths
3. Prefer getters over methods for simple computed values

## Repository Structure

```
crates/
  rapier-wasm-2d/     # WASM crate for 2D (uses shared src/)
  rapier-wasm-3d/     # WASM crate for 3D (uses shared src/)

packages/
  rapier-2d/          # TypeScript bindings for 2D physics
  rapier-3d/          # TypeScript bindings for 3D physics
  testbed2d/          # 2D demo application
  testbed3d/          # 3D demo application
```

## Package Variants

Each package (`rapier-2d`, `rapier-3d`) ships 2 variants:

| Import Path             | WASM Loading         |
| ----------------------- | -------------------- |
| `@.../rapier-2d`        | `fetch()` at runtime |
| `@.../rapier-2d/compat` | Embedded base64      |

Both variants are built with WASM SIMD (`simd128`) enabled.

**Usage**:

- Default: Best for web apps (smaller bundle, parallel loading)
- Compat: For environments without `fetch()` (SSR, workers, tests)

## Build Commands

```bash
pnpm build              # Full build (WASM + TypeScript)
pnpm build:wasm         # WASM for 2D + 3D
pnpm build:ts           # TypeScript packages only
pnpm build:2d           # 2D only (WASM + TS)
pnpm build:3d           # 3D only (WASM + TS)
pnpm typecheck          # Type check all packages
pnpm fmt                # Format code with oxfmt
pnpm dev:testbed2d      # Run 2D demo
pnpm dev:testbed3d      # Run 3D demo
```

**`pnpm build:wasm` must run at least once before `pnpm build:ts` or `pnpm test`
on a fresh clone.** `wasm/release/` has its `.js` and `.wasm` committed, but
`.d.ts` files are gitignored, so `tsdown` fails with `"InitInput" is not exported
by "wasm/release/rapier_wasm_*.js"` until wasm-bindgen regenerates them. This
needs the Rust toolchain and `wasm-pack`; tests import the built packages, so
they can't run without it either.

## Benchmarks

Run performance benchmarks to measure physics engine performance:

```bash
pnpm bench              # Full 3D benchmark
pnpm bench:2d           # Full 2D benchmark
pnpm bench:quick        # Quick mode (fewer iterations)
```

**Benchmark categories:**

- **Simulation**: `world.step()` on a 3000-body pyramid, measured both active
  (sleeping disabled) and settled — a settled scene sleeps and steps ~1000x
  faster, so the two are separate benchmarks
- **Lifecycle**: Body creation/destruction throughput
- **Queries**: Ray casting and point projection, 1000 casts against 5000 bodies
- **Getters**: Property access with/without allocation, over 5000 bodies
- **Allocations**: bytes of JS heap per operation, and the GC count/pause time
  that causes per million operations, for each read path with and without its
  `target` form (`--no-memory` skips this pass)

Sizes drop to 1000 bodies / 250 casts under `--quick` (what CI runs). Benchmark
names embed their size, so a `--quick` run is never compared against a full-run
baseline.

Results are saved to `packages/benchmarks/results/` as timestamped JSON files.

The allocation pass (`src/memory.ts`) is separate from mitata: it forces a
collection, sizes each measurement window to stay inside the young generation,
and takes the smallest of seven GC-free windows. Two things it must keep doing —
results have to escape into a sink (otherwise V8's escape analysis deletes the
allocation being measured), and the heap has to be sampled before awaiting the
GC observer's flush (the observer only receives entries on a later timer turn).

## Critical Memory Management Patterns

### Rule 1: Always `init()` Before API Use

```typescript
import RAPIER from "@alexandernanberg/rapier2d";

await RAPIER.init(); // REQUIRED before any API calls
const world = new RAPIER.World({x: 0, y: -9.81});
```

### Rule 2: Free Raw Objects After `intoRaw()` Calls

When passing data to WASM, temporary raw objects must be freed:

```typescript
// CORRECT - free raw objects after use
let rawOrig = VectorOps.intoRaw(ray.origin);
let rawDir = VectorOps.intoRaw(ray.dir);

let result = this.raw.castRay(rawOrig, rawDir, maxToi);

rawOrig.free(); // REQUIRED
rawDir.free(); // REQUIRED

return result;
```

```typescript
// WRONG - memory leak
let rawOrig = VectorOps.intoRaw(ray.origin);
let rawDir = VectorOps.intoRaw(ray.dir);
return this.raw.castRay(rawOrig, rawDir, maxToi);
// rawOrig and rawDir are never freed!
```

### Rule 3: `fromRaw()` Auto-Frees, `intoRaw()` Does Not

- `fromRaw(raw)`: Consumes and frees the raw object automatically
- `intoRaw()`: Returns raw object that YOU must free

### Rule 4: Free World/Controller Resources

Classes with `raw` property need explicit cleanup:

```typescript
class KinematicCharacterController {
    public free() {
        if (!!this.raw) {
            this.raw.free();
        }
        this.raw = undefined;
    }
}
```

## Zero-Allocation Getters

For hot paths, use the optional `target` parameter to avoid allocations:

```typescript
// Allocating (creates new object each call)
const pos = body.translation();

// Zero-allocation (reuses existing object)
const _pos = {x: 0, y: 0, z: 0};
body.translation(_pos); // writes into _pos
```

Supported methods:

- **RigidBody**: `translation()`, `rotation()`, `linvel()`, `angvel()`,
  `nextTranslation()`, `nextRotation()`, `localCom()`, `worldCom()`,
  `velocityAtPoint()`, `effectiveInvMass()`, `userForce()`, and (3D only)
  `userTorque()`, `principalInertia()`, `invPrincipalInertia()`,
  `principalInertiaLocalFrame()`, `effectiveAngularInertia()`,
  `effectiveWorldInvInertia()`
- **Collider**: `translation()`, `rotation()`, `halfExtents()`,
  `heightfieldScale()`, `projectPoint()`, `castShape()`, `castCollider()`,
  `contactShape()`, `contactCollider()`, `castRayAndGetNormal()`
- **Shape**: `projectPoint()`, `castShape()`, `contactShape()`,
  `castRayAndGetNormal()`
- **World / BroadPhase**: `castRayAndGetNormal()`, `projectPoint()`,
  `projectPointAndGetFeature()`, `castShape()`
- **ImpulseJoint**: `anchor1()`, `anchor2()`, and (3D only) `frameX1()`,
  `frameX2()`
- **DynamicRayCastVehicleController** (3D): the wheel vector getters

Queries take `target` as their last argument, after the filter arguments. When a
query misses, it returns `null` and leaves the target untouched.

## 2D vs 3D Differences

| Concept  | 2D                                         | 3D                                 |
| -------- | ------------------------------------------ | ---------------------------------- |
| Rotation | `number` (radians)                         | `Quaternion` `{x,y,z,w}`           |
| Vector   | `{x, y}`                                   | `{x, y, z}`                        |
| Shapes   | Ball, Cuboid, Capsule, ConvexPolygon, etc. | + ConvexPolyhedron, Cylinder, Cone |

## Common Pitfalls

1. **Memory leaks**: Missing `.free()` after `intoRaw()` calls
2. **Uninitialized WASM**: Calling API before `await init()`
3. **Wrong rotation type**: Using quaternion in 2D or number in 3D
4. **Stale handles**: Using `RigidBodyHandle` after body removed from world

## Key Type Patterns

### Handles vs Objects

```typescript
// Handle = lightweight reference (number)
type RigidBodyHandle = number;
type ColliderHandle = number;

// Get actual object from set using handle
const body = world.getRigidBody(handle);
```

### Descriptor Pattern

```typescript
// Use descriptors to configure before creation
const bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 5);

const body = world.createRigidBody(bodyDesc);
```

## File Navigation Hints

| Looking for...       | Check                                                         |
| -------------------- | ------------------------------------------------------------- |
| Shape definitions    | `packages/rapier-{2d,3d}/src/geometry/shape.ts`               |
| Rigid body API       | `packages/rapier-{2d,3d}/src/dynamics/rigid_body.ts`          |
| World/simulation     | `packages/rapier-{2d,3d}/src/pipeline/world.ts`               |
| Collision detection  | `packages/rapier-{2d,3d}/src/geometry/narrow_phase.ts`        |
| Ray/shape casting    | `packages/rapier-{2d,3d}/src/geometry/broad_phase.ts`         |
| Character controller | `packages/rapier-{2d,3d}/src/control/character_controller.ts` |
| WASM init logic      | `packages/rapier-{2d,3d}/src/init.ts`, `init-compat.ts`       |
| Math utilities       | `packages/rapier-{2d,3d}/src/math.ts`                         |

## Changesets

When making meaningful changes to `rapier-2d` or `rapier-3d` (bug fixes, new features, perf improvements), create a changeset:

```bash
pnpm changeset
```

Use `patch` for fixes, `minor` for features/breaking changes (we're pre-1.0). Skip changesets for CI, docs, testbed-only, or formatting changes.

## Pre-commit Checks

Always run these before committing:

```bash
pnpm fmt                # Format TypeScript/JS with oxfmt
pnpm lint               # Lint with oxlint
cargo fmt               # Format Rust code
```

## Testing

Testbeds serve as integration tests:

```bash
pnpm dev:testbed2d   # http://localhost:5173
pnpm dev:testbed3d   # http://localhost:5173
```

Demo files in `packages/testbed{2d,3d}/src/demos/` show usage patterns.
