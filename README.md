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

- Rapier 0.32 with glam math library
- pnpm monorepo with tsdown bundler
- Contiguous transform buffer (body reads with zero WASM crossings)
- Zero-allocation getters (optional target parameter)
- Batch transform setters (`setTransform`, `setNextKinematicTransform`)
- Built-in benchmarks
- Simplified package variants (4 per dimension)

### Benchmarks

3D, 500 bodies (Ubuntu, Node v24.13.0):

| Benchmark                        | Mean    | Min     | Max     |
| -------------------------------- | ------- | ------- | ------- |
| world.step() [500 bodies]        | 851.7µs | 38.7µs  | 1.594ms |
| create 1000 bodies+colliders     | 5.480ms | 4.565ms | 9.371ms |
| spawn+despawn 100 bodies         | 780.1µs | 484.8µs | 1.369ms |
| castRay (500 bodies)             | 3.8µs   | 2.5µs   | 24.0µs  |
| castRayAndGetNormal              | 4.1µs   | 3.0µs   | 23.6µs  |
| intersectionsWithRay             | 6.6µs   | 3.7µs   | 186.1µs |
| projectPoint                     | 6.6µs   | 3.1µs   | 255.8µs |
| intersectionsWithPoint           | 2.9µs   | 2.3µs   | 34.7µs  |
| body.translation() [alloc]       | 19.5µs  | 6.0µs   | 611.9µs |
| body.translation() [reuse]       | 11.0µs  | 5.2µs   | 40.4µs  |
| body.rotation() [alloc]          | 12.8µs  | 5.4µs   | 218.1µs |
| body.rotation() [reuse]          | 12.9µs  | 5.5µs   | 70.3µs  |
| body.linvel() [alloc]            | 16.7µs  | 5.7µs   | 247.9µs |
| body.linvel() [reuse]            | 10.9µs  | 5.0µs   | 46.6µs  |
| collider.translation() [alloc]   | 62.3µs  | 54.2µs  | 320.8µs |
| collider.translation() [reuse]   | 59.4µs  | 53.7µs  | 223.7µs |
| body.setTransform()              | 24.7µs  | 23.0µs  | 116.2µs |
| body.setNextKinematicTransform() | 23.3µs  | 22.5µs  | 39.4µs  |

Run `pnpm bench` to benchmark on your machine.

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

Each package ships 4 variants via subpath exports:

| Import Path                              | WASM Loading         | SIMD |
| ---------------------------------------- | -------------------- | ---- |
| `@alexandernanberg/rapier2d`             | `fetch()` at runtime | No   |
| `@alexandernanberg/rapier2d/simd`        | `fetch()` at runtime | Yes  |
| `@alexandernanberg/rapier2d/compat`      | Embedded base64      | No   |
| `@alexandernanberg/rapier2d/compat-simd` | Embedded base64      | Yes  |

**When to use which:**

- **Default/SIMD**: Best for web apps (smaller bundle, parallel loading)
- **Compat variants**: For environments without `fetch()` (SSR, workers, tests)
- **SIMD variants**: Better performance, requires [simd128 support](https://caniuse.com/?search=simd)

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
pnpm build:wasm         # WASM only (all variants)
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
pnpm bench:2d                 # Full 2D benchmark
pnpm bench --quick            # Quick mode (fewer iterations)
```

**Baseline comparison:**

- Results are compared against `packages/benchmarks/baseline.json`
- Thresholds: >15% = warning, >30% = regression
- Exit code 1 on regression (useful for CI)

## License

Apache 2.0
