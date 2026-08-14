---
"@alexandernanberg/rapier2d": minor
"@alexandernanberg/rapier3d": minor
---

feat: SIMD by default, drop the non-SIMD variants

Both packages now ship 2 entry points instead of 4, and both are built with WASM
SIMD (`simd128`) enabled:

| Import Path                         | WASM Loading         |
| ----------------------------------- | -------------------- |
| `@alexandernanberg/rapier2d`        | `fetch()` at runtime |
| `@alexandernanberg/rapier2d/compat` | Embedded base64      |

**Breaking:** the `/simd` and `/compat-simd` entry points were removed. Migrate by
dropping the suffix — `@alexandernanberg/rapier3d/compat-simd` becomes
`@alexandernanberg/rapier3d/compat`, and `/simd` becomes the package root. The
non-SIMD builds are gone; the default entry points now _are_ the SIMD builds.

Since Rapier 0.35, SIMD is always compiled in (backed by `wide`, which falls back
to scalar where unsupported), so a non-SIMD build was no longer smaller or
differently-featured — it ran the same code paths scalar. Measured on a 3000-body
3D pyramid with every body awake, the SIMD build steps **1.53x faster** (min
11.00ms vs 16.81ms per step; median 13.70ms vs 18.35ms). Keeping a scalar build as
the default meant most users silently got the slow path.

`simd128` is supported in Chrome 91+, Firefox 89+, Safari 16.4+ and Node 16.4+. If
you need to target something older, pin to a previous release.

The CI check that asserts SIMD opcodes are present now guards the default build
rather than a side variant.
