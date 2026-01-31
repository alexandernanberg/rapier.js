# CLAUDE.md - AI Assistant Context for rapier.js

## Project Overview

This is a **fork** of rapier.js - TypeScript bindings for the Rapier physics engine (Rust → WASM).

## Prerequisites

- Node.js 24+
- pnpm (`npm install -g pnpm`)
- Rust toolchain (`rustup`)
- wasm-pack (`cargo install wasm-pack`)

-   **Package scope**: `@alexandernanberg/rapier-{2d,3d}`
-   **Monorepo**: pnpm workspaces
-   **Stack**: Rust + wasm-bindgen → WASM, TypeScript bindings

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

Each package (`rapier-2d`, `rapier-3d`) ships 4 variants:

| Import Path                  | WASM Loading         | SIMD |
| ---------------------------- | -------------------- | ---- |
| `@.../rapier-2d`             | `fetch()` at runtime | No   |
| `@.../rapier-2d/simd`        | `fetch()` at runtime | Yes  |
| `@.../rapier-2d/compat`      | Embedded base64      | No   |
| `@.../rapier-2d/compat-simd` | Embedded base64      | Yes  |

**Usage**:

-   Default/SIMD: Best for web apps (smaller bundle, parallel loading)
-   Compat variants: For environments without `fetch()` (SSR, workers, tests)

## Build Commands

```bash
pnpm build              # Full build (WASM + TypeScript)
pnpm build:wasm         # All WASM variants
pnpm build:ts           # TypeScript packages only
pnpm build:2d           # 2D only (WASM + TS)
pnpm build:3d           # 3D only (WASM + TS)
pnpm typecheck          # Type check all packages
pnpm fmt                # Format code with Prettier
pnpm dev:testbed2d      # Run 2D demo
pnpm dev:testbed3d      # Run 3D demo
```

## Benchmarks

Run performance benchmarks to measure physics engine performance:

```bash
pnpm bench              # Full 3D benchmark
pnpm bench:2d           # Full 2D benchmark
pnpm bench:quick        # Quick mode (fewer iterations)
```

**Benchmark categories:**
- **Simulation**: `world.step()` performance with stacked bodies
- **Lifecycle**: Body creation/destruction throughput
- **Queries**: Ray casting and point projection performance
- **Getters**: Property access with/without allocation

Results are saved to `packages/benchmarks/results/` as timestamped JSON files.

## Critical Memory Management Patterns

### Rule 1: Always `init()` Before API Use

```typescript
import RAPIER from "@alexandernanberg/rapier-2d";

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

-   `fromRaw(raw)`: Consumes and frees the raw object automatically
-   `intoRaw()`: Returns raw object that YOU must free

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
const _pos = { x: 0, y: 0, z: 0 };
body.translation(_pos);  // writes into _pos
```

Supported methods: `translation()`, `rotation()`, `linvel()`, `angvel()`,
`nextTranslation()`, `nextRotation()`, `localCom()`, `worldCom()` on RigidBody,
and `translation()`, `rotation()` on Collider.

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

## Testing

Testbeds serve as integration tests:

```bash
pnpm dev:testbed2d   # http://localhost:5173
pnpm dev:testbed3d   # http://localhost:5173
```

Demo files in `packages/testbed{2d,3d}/src/demos/` show usage patterns.
