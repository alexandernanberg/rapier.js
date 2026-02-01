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

### Why This Fork?

1. **Latest Rapier**: Rapier 0.32 (official npm is on 0.29)
2. **Zero-allocation getters**: Optional `target` parameter avoids object creation in hot paths
3. **Optimized WASM boundary**: Ray casting passes primitives directly instead of temporary WASM objects
4. **Modern build**: pnpm monorepo, tsdown bundler, smaller bundles

### Benchmarks vs Official

Comparison against `@dimforge/rapier3d-compat@0.19.3` (3D, 1000 bodies):

| Benchmark            | Fork     | Official | Improvement |
| -------------------- | -------- | -------- | ----------- |
| world.step()         | 1.37ms   | 1.40ms   | ~same       |
| create 1000 bodies   | 3.8ms    | 4.1ms    | **7%**      |
| castRay              | 2.6µs    | 4.0µs    | **35%**     |
| body.translation()   | 73µs     | 210µs    | **2.9x**    |
| body.rotation()      | 70µs     | 223µs    | **3.2x**    |

### What Makes It Faster

**Zero-allocation getters (3x faster)**

The official package allocates a new object every call:

```typescript
// Official: allocates {x, y, z} every call
for (const body of bodies) {
  const pos = body.translation(); // new object
}
```

This fork accepts an optional target to reuse:

```typescript
// Fork: zero allocations
const pos = {x: 0, y: 0, z: 0};
for (const body of bodies) {
  body.translation(pos); // writes into existing object
}
```

Supported: `translation()`, `rotation()`, `linvel()`, `angvel()`, `nextTranslation()`, `nextRotation()`, `localCom()`, `worldCom()`

**Optimized ray casting (35% faster)**

Ray origin/direction passed as primitives directly to WASM, avoiding temporary `RawVector` allocations.

Run `pnpm bench --official` to compare on your machine.

---

## Installation

```bash
# 2D physics
npm install @alexandernanberg/rapier-2d

# 3D physics
npm install @alexandernanberg/rapier-3d
```

## Usage

```typescript
import RAPIER from "@alexandernanberg/rapier-2d";

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

| Import Path                               | WASM Loading         | SIMD |
| ----------------------------------------- | -------------------- | ---- |
| `@alexandernanberg/rapier-2d`             | `fetch()` at runtime | No   |
| `@alexandernanberg/rapier-2d/simd`        | `fetch()` at runtime | Yes  |
| `@alexandernanberg/rapier-2d/compat`      | Embedded base64      | No   |
| `@alexandernanberg/rapier-2d/compat-simd` | Embedded base64      | Yes  |

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
