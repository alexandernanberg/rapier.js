---
"@alexandernanberg/rapier2d": patch
"@alexandernanberg/rapier3d": patch
---

perf: remove per-call WASM allocations from body/collider creation, scene queries and transform getters

Three changes to the JS↔WASM boundary, all aimed at the same cost: every
`RawVector`/`RawRotation`/query-result object costs a WASM allocation, a JS
wrapper object and a `FinalizationRegistry` register/unregister pair (~300ns
each on Node 22).

- **Body and collider creation** passed their poses, velocities and mass
  properties as `RawVector`/`RawRotation` handles — 7 temporaries per rigid-body
  and 6 per collider. They are now passed component-wise as scalars, the same
  pattern already used by the force/velocity setters.
- **Scene queries** (`castRay`, `castRayAndGetNormal`, `intersectionsWithRay`,
  `projectPoint`, `projectPointAndGetFeature`, `intersectionsWithPoint`)
  allocated a raw result object per hit (plus a `RawVector` for each normal or
  projected point), and the point queries allocated a `RawVector` for their
  input. Results are now written into a small scratch buffer that JS reads
  directly, and points are passed as scalars.
- **Transform getters** (`translation()`, `rotation()`, `linvel()`, `angvel()`
  on `RigidBody`, `translation()`/`rotation()` on `Collider`) read from a
  `Float32Array` view into WASM memory, which is detached whenever WASM memory
  grows. Until now that view was only re-created by the next `World.step()`, so
  every read in between fell back to a per-call WASM round-trip — for example
  after spawning bodies or running a query mid-frame. The view is now
  re-attached on the spot, and only genuinely stale data (a body or collider was
  created or moved directly) still goes through WASM.

Measured with `pnpm bench` (3D, Node 22): creating 1000 bodies+colliders 6.10ms
→ 1.28ms, spawn+despawn 100 bodies 0.57ms → 0.14ms, `projectPoint` ×100 0.22ms
→ 0.11ms, `castRayAndGetNormal` ×100 0.14ms → 0.10ms, and transform getters
~0.05ms → ~0.007ms per 1000 reads. Simulation stepping itself is unchanged.

Also fixes `World.propagateModifiedBodyPositionsToColliders()` in 2D not
invalidating the collider transform buffer, which could return stale collider
positions until the next `World.step()`.
