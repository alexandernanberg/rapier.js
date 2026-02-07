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

3D, 3000 bodies, mean times (Apple M1 Max, Node v22.21.1):

| Benchmark                        | Fork    | Official | Speedup |
| -------------------------------- | ------- | -------- | ------- |
| world.step()                     | 1.188ms | 1.150ms  | —       |
| create 1000 bodies+colliders     | 3.215ms | 3.401ms  | 1.1x    |
| spawn+despawn 100 bodies         | 362.7µs | 378.5µs  | —       |
| castRay x100                     | 69.5µs  | 122.9µs  | 1.8x    |
| castRayAndGetNormal x100         | 97.9µs  | 152.1µs  | 1.6x    |
| intersectionsWithRay x100        | 112.1µs | 172.9µs  | 1.5x    |
| projectPoint x100                | 122.0µs | 168.9µs  | 1.4x    |
| intersectionsWithPoint x100      | 31.2µs  | 29.0µs   | —       |
| body.translation()               | 6.3µs   | 209.9µs  | 33x     |
| body.translation() [reuse]       | 5.6µs   | n/a      | 37x     |
| body.rotation()                  | 5.2µs   | 223.0µs  | 43x     |
| body.rotation() [reuse]          | 5.2µs   | n/a      | 43x     |
| body.linvel()                    | 5.5µs   | 207.4µs  | 38x     |
| body.linvel() [reuse]            | 4.6µs   | n/a      | 45x     |
| collider.translation()           | 60.9µs  | 208.8µs  | 3.4x    |
| collider.translation() [reuse]   | 60.8µs  | n/a      | 3.4x    |
| body.setTransform()              | 22.2µs  | 31.3µs   | 1.4x    |
| body.setNextKinematicTransform() | 20.7µs  | 33.2µs   | 1.6x    |

Official = `@dimforge/rapier3d-compat` v0.19.3. Reuse = zero-allocation target parameter (fork only), speedup compared against official alloc. Getter times are for 1000 bodies. Run `pnpm bench` / `pnpm bench --official` to benchmark on your machine.

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
