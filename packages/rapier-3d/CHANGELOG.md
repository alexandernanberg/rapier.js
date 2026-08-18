# @alexandernanberg/rapier3d

## 0.2.0

### Minor Changes

- [#27](https://github.com/alexandernanberg/rapier.js/pull/27) [`582f40d`](https://github.com/alexandernanberg/rapier.js/commit/582f40d89f8a2e730afcd604054be33d6edb0403) Thanks [@alexandernanberg](https://github.com/alexandernanberg)! - Add contact modification hooks, fill in the multibody joint API, add the missing
  3D `ColliderDesc.halfspace`, and fix the PID controller's gain setters.
  
  `PhysicsHooks.modifySolverContacts(context)` is now supported, for colliders
  carrying the (newly exposed) `ActiveHooks.MODIFY_SOLVER_CONTACTS` flag. The
  `ContactModificationContext` it receives can read the pair, its normal and each
  solver contact, and can override the manifold's friction and restitution, move
  or drop individual contacts, and set a per-contact tangent velocity (conveyor
  belts). The context points straight at the manifold being built rather than
  allocating a wasm object per call, so it is only valid for the duration of the
  hook — outside of it every getter reads zero and every setter is a no-op. The
  32-bit `userData` a hook writes survives to the next steps and is readable with
  `TempContactManifold.userData()`. `filterContactPair` and
  `filterIntersectionPair` are now optional, so a hooks object may implement only
  what it needs.
  
  `MultibodyJoint` gained the accessors that were commented out on both sides of
  the boundary: `body1()`/`body2()`, `type()`, `anchor1()`/`anchor2()` (and
  `frameX1()`/`frameX2()` in 3D, all with the usual optional `target`), plus
  limits (`limitsEnabled()`, `limitsMin()`, `limitsMax()`, `setLimits()`) and
  motors (`configureMotorModel()`, `setMotorMaxForce()`, `configureMotorVelocity()`,
  `configureMotorPosition()`, `configureMotor()`) on unit joints, and per-axis
  motors on 3D spherical multibody joints. `MultibodyJointSet.createJoint()` now
  takes the `RigidBodySet` as its first argument, matching `ImpulseJointSet`, so
  the joints can resolve their bodies.
  
  `ColliderDesc.halfspace(normal)` was missing from the 3D package even though the
  `HalfSpace` shape and its raw constructor were both there; 2D already had it.
  
  `PidController.setKi()` and `setKd()` both wrote the proportional gain, so the
  integral and derivative gains could not be changed after construction. `setKd()`
  also wrote every axis' gain into the x component on the Rust side.

- [#20](https://github.com/alexandernanberg/rapier.js/pull/20) [`08dcdeb`](https://github.com/alexandernanberg/rapier.js/commit/08dcdebaff3da2e9518a02a1fe9d0bf73ec9c58d) Thanks [@alexandernanberg](https://github.com/alexandernanberg)! - Add compound shapes and convex decomposition, ported from upstream: the new `Compound`
  shape, `ColliderDesc.compound()`, and `ColliderDesc.convexDecomposition()` (VHACD, tunable
  through `VHACDParameters`). `Shape.fromRawShape()` reconstructs any shape — including the
  sub-shapes of a compound — from its raw handle, and `Shape.fromRaw()` now goes through it.
  
  Also fixes the vertex/index buffers exported for convex polyhedra: `Collider.shape` returned
  vertices from the shape's point set but indices from its triangulation, which don't always
  agree, so the resulting mesh could not be fed back into `ColliderDesc.convexMesh()`. Both are
  now derived from the same recomputed convex hull.

- [#19](https://github.com/alexandernanberg/rapier.js/pull/19) [`56a7c1c`](https://github.com/alexandernanberg/rapier.js/commit/56a7c1ce45b6e3743817959de30be7f672fbfa9f) Thanks [@alexandernanberg](https://github.com/alexandernanberg)! - feat: SIMD by default, drop the non-SIMD variants
  
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

- [#26](https://github.com/alexandernanberg/rapier.js/pull/26) [`0feb42c`](https://github.com/alexandernanberg/rapier.js/commit/0feb42c3ae816d339bdb0787024aa3c3d50a98b6) Thanks [@alexandernanberg](https://github.com/alexandernanberg)! - feat: zero-allocation targets on scene queries, and the full solver tuning surface
  
  - **Optional `target` on the remaining allocating getters and queries.** Until
    now only the transform getters (`translation()`, `rotation()`, `linvel()`, …)
    could write into a caller-owned object; every other query allocated a fresh
    result — plus a vector per point or normal inside it — on each call. `target`
    is now accepted by `castRayAndGetNormal()`, `projectPoint()`,
    `projectPointAndGetFeature()` and `castShape()` on `World` and `BroadPhase`,
    by `projectPoint()`, `castShape()`, `castCollider()`, `contactShape()`,
    `contactCollider()`, `castRayAndGetNormal()`, `halfExtents()` and
    `heightfieldScale()` on `Collider`, by the equivalent `Shape` queries, by
    `velocityAtPoint()`, `effectiveInvMass()`, `userForce()` and the 3D mass-
    property getters on `RigidBody`, by `anchor1()`/`anchor2()` (and 3D
    `frameX1()`/`frameX2()`) on `ImpulseJoint`, and by the wheel vector getters on
    `DynamicRayCastVehicleController`. It is the last argument, after the filter
    arguments; the target is returned as-is, its nested vectors are reused rather
    than replaced, and a query that misses returns `null` and leaves the target
    untouched.
  
  - **The rest of `IntegrationParameters`.** Only 9 of the ~20 solver knobs Rapier
    exposes were bound. Added: `contactNaturalFrequency`, `contactDampingRatio`,
    `staticContactNaturalFrequency`, `staticContactDampingRatio`,
    `warmstartCoefficient`, `warmstartJoints`, `minCcdDt`,
    `normalizedMaxCorrectiveVelocity`, `normalizedMaxLinearVelocity`,
    `numInternalStabilizationIterations`, `contactClustering`, `contactRecycling`,
    `normalizedContactRecycleDistance`, `frictionInBiasPass`, and (3D only)
    `frictionModel` with the new `FrictionModel` enum (`Simplified`, `Coulomb`).
    Contact softness was previously write-only through
    `contact_natural_frequency`; that setter is kept as an alias of
    `contactNaturalFrequency`, which is now also readable.

- [#19](https://github.com/alexandernanberg/rapier.js/pull/19) [`56a7c1c`](https://github.com/alexandernanberg/rapier.js/commit/56a7c1ce45b6e3743817959de30be7f672fbfa9f) Thanks [@alexandernanberg](https://github.com/alexandernanberg)! - feat: upgrade to Rapier 0.35
  
  Upgrades the underlying physics engine from Rapier 0.32 to 0.35, spanning three
  upstream breaking releases. Most of the upgrade is internal, but a few things
  change for consumers.
  
  **API changes**
  
  - `IntegrationParameters.minIslandSize` / `World.minIslandSize` were removed.
    Awake bodies are now solved as a single active set, so the parameter no longer
    exists upstream.
  - On a contact manifold, solver contacts now store a per-body anchor instead of a
    single world-space point, which are exposed as the new `solverContactAnchor1(i)`
    and `solverContactAnchor2(i)`. The anchors are expressed in the body's
    center-of-mass-centered local frame, or in world-space when that side has no
    solver body (no rigid-body, or world-attached by dominance — fixed bodies
    included). `solverContactPoint(i)` is still available and still returns a
    world-space point, now midway between both surfaces; its return type is now
    `Vector | null`, which is what it already returned for an out-of-bounds index.
  - `solverContactFriction(i)` and `solverContactRestitution(i)` were replaced by
    `friction()` and `restitution()` on the manifold. Both coefficients are now
    stored per-manifold rather than per solver-contact, so they are identical for
    every contact of a manifold.
  
  **Behaviour changes**
  
  - Contact defaults changed upstream: `normalizedPredictionDistance` is now `0.02`
    (was `0.002`) and `normalizedAllowedLinearError` is now `0.005` (was `0.001`),
    which greatly reduces tunneling through thin walls.
  - Sleeping was rewritten around persistent islands: an island now sleeps and
    wakes strictly as a unit, so a body is never frozen while something it touches
    still moves. Sleep eligibility is judged on the actual per-step pose
    displacement instead of velocities, and bodies now become eligible after `0.5`s
    instead of `2.0`s.
  - CCD was rewritten around sweep-based time of impact. Fast dynamic bodies now
    always run CCD against fixed colliders; `setCcdEnabled`/`enableCcd` now upgrade
    a body to a "bullet" that additionally sweeps kinematic and dynamic bodies. Set
    `World.maxCcdSubsteps` to `0` to disable CCD entirely.
  - Body velocities are now capped per substep (linear speed at 400 units/s by
    default, rotation at ~45° per step).
  - `narrowPhase.contactPair()` no longer invokes its callback for two colliders
    attached to the same rigid-body, since the broad phase no longer pairs them.
  - World snapshots taken with an earlier version cannot be restored: the island
    manager and rigid-body activation serialization formats changed upstream.

- [#20](https://github.com/alexandernanberg/rapier.js/pull/20) [`08dcdeb`](https://github.com/alexandernanberg/rapier.js/commit/08dcdebaff3da2e9518a02a1fe9d0bf73ec9c58d) Thanks [@alexandernanberg](https://github.com/alexandernanberg)! - Remove per-call WASM allocations from shape-cast hits, shape contacts, contact manifolds,
  contact-force events, character-controller collisions and PID-controller corrections. These
  now write into a scratch buffer in a single boundary crossing instead of returning temporary
  raw objects. `TempContactManifold` also regains `solverContactPoint()`, which resolves solver
  contacts back to world-space; `World.contactPair` passes the rigid-body set through for it.

- [#20](https://github.com/alexandernanberg/rapier.js/pull/20) [`08dcdeb`](https://github.com/alexandernanberg/rapier.js/commit/08dcdebaff3da2e9518a02a1fe9d0bf73ec9c58d) Thanks [@alexandernanberg](https://github.com/alexandernanberg)! - Port upstream joint additions: `ImpulseJoint.setFrameX1/setFrameX2/setLocalFrame1/setLocalFrame2`,
  `UnitImpulseJoint.setMotorMaxForce`, per-axis motor configuration on `SphericalImpulseJoint`
  (with the new `JointAxis` enum), and `JointData.revoluteWithAxes` for hinges whose local axis
  differs on each body.

### Patch Changes

- [#22](https://github.com/alexandernanberg/rapier.js/pull/22) [`70a51e9`](https://github.com/alexandernanberg/rapier.js/commit/70a51e9ea147de7c975232ed177b7f39655105f2) Thanks [@alexandernanberg](https://github.com/alexandernanberg)! - Fix physics hooks being ignored when stepping without an event queue, and
  `KinematicCharacterController.up()` returning a raw WASM handle.
  
  `World.step(undefined, hooks)` previously dropped the hooks: the pipeline only
  forwarded them on the `stepWithEvents` branch, and the plain `step` binding
  hardcoded a no-op hook implementation on the Rust side, so `filterContactPair`
  and `filterIntersectionPair` were never called unless an `EventQueue` was also
  passed. Hooks are now routed through a new `stepWithHooks` binding; the
  hookless path is unchanged so it does not pay for marshalling them.
  
  `up()` returned `this.raw.up()` without unwrapping it, so callers got a
  `RawVector` handle instead of a plain vector — and since nothing freed it, every
  call leaked.

- [#21](https://github.com/alexandernanberg/rapier.js/pull/21) [`bdd0458`](https://github.com/alexandernanberg/rapier.js/commit/bdd045831be23db5f56c69dc40bb0c9a0ba7e4cd) Thanks [@alexandernanberg](https://github.com/alexandernanberg)! - Refresh the shared transform buffers incrementally instead of rewriting them on every step.
  
  `World.step()` used to walk the entire rigid-body and collider arenas afterwards to refill the buffers the transform getters read from. That walk is proportional to the number of entities in the world, not to how many of them moved, so it dominated the step time of any scene that had settled: with 3000 sleeping bodies it accounted for essentially the whole step.
  
  The sync now only rewrites the slots that can have changed — the bodies the island manager reports as active, the bodies that were active during the previous step (so a body that just fell asleep still gets its final pose written), the colliders attached to those bodies, and anything created or mutated from JS since the last step. Scenes where most entities keep moving fall back to the sequential pass, so they are unaffected.
  
  Measured on the benchmark suite (3D, 3000-body pyramid): `world.step()` goes from ~88µs to ~2µs once the pyramid settles. The equivalent 2D scene goes from ~100µs to ~5µs per step. A scene where nothing ever sleeps is unchanged.
  
  `refreshTransformBuffer` also stops allocating a new `Float32Array` view on every step when the buffer has neither moved nor been resized.

- [#19](https://github.com/alexandernanberg/rapier.js/pull/19) [`56a7c1c`](https://github.com/alexandernanberg/rapier.js/commit/56a7c1ce45b6e3743817959de30be7f672fbfa9f) Thanks [@alexandernanberg](https://github.com/alexandernanberg)! - fix: remove unsound `unsafe impl Send`/`Sync` on the physics hooks
  
  `RawPhysicsHooks` holds JS values (`js_sys::Object`/`Function`), which are not
  `Send`/`Sync`. Rapier's `PhysicsHooks` trait required `Send + Sync`, so the
  bindings asserted both with a hand-written `unsafe impl`, justified only by the
  observation that wasm is single-threaded.
  
  Rapier 0.35 added the `unsync-callbacks` feature, which drops the `Sync` bound
  from `PhysicsHooks` and `EventHandler` (through `utils::MaybeSync`) for exactly
  this case — thread-affine callbacks such as a JS closure. Enabling it lets the
  hooks be accepted as-is, so both `unsafe impl`s are gone with no change in
  behaviour.

- [#24](https://github.com/alexandernanberg/rapier.js/pull/24) [`e34717f`](https://github.com/alexandernanberg/rapier.js/commit/e34717fdd5983c341bc8c5ddc74facd4945090dd) Thanks [@alexandernanberg](https://github.com/alexandernanberg)! - fix: `World.restoreSnapshot` in 2D reports decode failures instead of hiding them
  
  `SerializationPipeline.deserializeAll` is `Option<RawDeserializedWorld>` on the
  Rust side — it returns nothing when the snapshot cannot be decoded, which is the
  documented outcome of restoring a snapshot taken by a different version of the
  engine. 3D propagated that as `World | null`, but 2D forced it away with two
  non-null assertions:
  
  ```ts
  return World.fromRaw(this.raw.deserializeAll(data)!)!;
  ```
  
  So `World.restoreSnapshot(badSnapshot)` in 2D was typed `World` while actually
  evaluating to `null`, and the failure only surfaced later as a confusing
  `TypeError` on the first property access.
  
  **Breaking (2D only, types):** `World.restoreSnapshot` and
  `SerializationPipeline.deserializeAll` now return `World | null`, matching 3D.
  Callers who know their snapshot is good can assert with `!`; everyone else
  should branch on the result.
  
  Both dimensions gain a test covering the failure path.

- [#23](https://github.com/alexandernanberg/rapier.js/pull/23) [`d6b099f`](https://github.com/alexandernanberg/rapier.js/commit/d6b099f61341512f1e542e7149e645e4a5e40cd3) Thanks [@alexandernanberg](https://github.com/alexandernanberg)! - fix: reject malformed mesh arrays instead of trapping, and stop copying getter
  results across the JS/WASM boundary
  
  **Malformed mesh input**
  
  Building a mesh shape from a vertex or index array whose length was not a whole
  number of elements — a ragged buffer out of a mesh loader, a typed array sliced
  by hand — used to trap the WASM module. That surfaced in JS as a bare
  `RuntimeError: unreachable` with no indication of what was wrong, because the
  bindings install no panic hook.
  
  The affected builders now return `null`/`undefined` for such input, which is what
  they already did for meshes they simply could not build:
  `TriMesh`, `ConvexPolyhedron`/`ConvexPolygon` (both the hull and the indexed
  form), `ColliderDesc.convexDecomposition` and its `WithParams` variant.
  `Polyline` cannot report an error, so it now drops a trailing partial vertex or
  segment instead of trapping, matching what the voxel builders next to it already
  did.
  
  **Getter results no longer cross the boundary**
  
  Getters that hand back a vector or a bundle of components used to write into a
  `Float32Array` passed in from JS. Every one of those writes is a call out to JS —
  one per component with `set_index` (19 of them for a single character collision),
  or one per call plus a temporary view with `copy_from`.
  
  They now write into a small fixed buffer inside WASM's own memory, which JS reads
  through a persistent `Float32Array` view. The getter call itself is the only
  boundary crossing left. This is the same arrangement the transform buffers and the
  broad-phase query results already used.
  
  Measured on 1000 bodies with the transform buffer forced stale, so every read
  takes the WASM path:
  
  | getter                    | before | after |
  | ------------------------- | ------ | ----- |
  | `body.rotation(target)`   | 89 ns  | 34 ns |
  | `body.linvel(target)`     | 98 ns  | 29 ns |
  | `body.nextTranslation(t)` | 79 ns  | 22 ns |
  | `collider.translation(t)` | 76 ns  | 32 ns |
  
  Reads served from the transform buffer were already allocation- and
  crossing-free, and are unchanged.
  
  **Breaking, for direct users of the raw API only**
  
  If you call the raw getters yourself (`world.bodies.raw.rbTranslation(...)` and
  friends) rather than going through `RigidBody`/`Collider`, they no longer take a
  `Float32Array` argument, and the result is read from the shared buffer instead.
  The new `scratch()` export returns the view to read it from:
  
  ```ts
  import {scratch} from "@alexandernanberg/rapier3d";
  
  world.bodies.raw.rbTranslation(body.handle);
  const s = scratch();
  const translation = {x: s[0], y: s[1], z: s[2]};
  ```
  
  Read the components out before the next call into WASM: the buffer is shared, and
  the next getter overwrites it.

- [#24](https://github.com/alexandernanberg/rapier.js/pull/24) [`e34717f`](https://github.com/alexandernanberg/rapier.js/commit/e34717fdd5983c341bc8c5ddc74facd4945090dd) Thanks [@alexandernanberg](https://github.com/alexandernanberg)! - chore: update dependencies
  
  Refreshes the crates the WASM bindings are built from — notably `wasm-bindgen`
  0.2.108 → 0.2.127, `rapier` 0.35.0 → 0.35.1 (with `parry` 0.30.2 and `nalgebra`
  0.35.0) — and moves `bincode` from 1.3 to 2.0. `bincode` 2 replaced the free
  `serialize`/`deserialize` functions with a `serde` integration module that takes
  an explicit configuration; the world serializer now passes
  `bincode::config::legacy()`, which reproduces bincode 1's encoding, so snapshot
  bytes are unchanged.

- [#25](https://github.com/alexandernanberg/rapier.js/pull/25) [`152999f`](https://github.com/alexandernanberg/rapier.js/commit/152999f41942d1a06a2fb0b2e2bfea14d391e90f) Thanks [@alexandernanberg](https://github.com/alexandernanberg)! - fix: report bad shape and joint input as errors instead of trapping the WASM
  module
  
  A WASM panic cannot be caught: it aborts the module and reaches JS as a bare
  `RuntimeError: unreachable`, which says nothing about what went wrong and leaves
  the instance unusable. Several builders reached one on input that a caller can
  plausibly produce.
  
  **Shape builders**
  
  These now report invalid input rather than trapping. The shape wrappers throw an
  `Error` naming the constraint that was violated; the raw builders behind them
  return `undefined`.
  
  - `Polyline` and `TriMesh`, `ConvexPolyhedron`/`ConvexPolygon` (indexed form),
    `RoundConvexPolyhedron`, and `ColliderDesc.convexDecomposition` — a segment,
    triangle or face index pointing past the last vertex. Only `TriMesh` validated
    its indices; every other builder indexed the vertex buffer directly.
  - `Polyline` — a ragged vertex or index array. It used to silently drop the
    trailing partial element, since it had no way to report the error; now it
    rejects the input like the other mesh builders.
  - `Heightfield` — a height buffer that doesn't hold exactly
    `(nrows + 1) * (ncols + 1)` entries (2D: at least two entries), or a degenerate
    grid.
  - `Compound` — a triangle mesh or polyline sub-shape. Those are composite shapes
    and cannot be nested in a compound, exactly like the nested `Compound` the
    constructor already rejected. The constructor now rejects all three.
  
  **Multibody joints**
  
  `MultibodyJointSet.createJoint` now throws when the joint would leave the
  multibody in an invalid configuration — `parent2` already has a parent joint, or
  both bodies already belong to the same multibody, which would close a loop.
  Rapier rejects the insert in both cases; the rejection used to come back as a
  sentinel handle that JS immediately used as a real one, panicking on the very
  next accessor.
  
  **Panic hook**
  
  Any panic that does get through now logs its message and source location to
  `console.error` before the module aborts, instead of vanishing behind
  `RuntimeError: unreachable`.

- [#17](https://github.com/alexandernanberg/rapier.js/pull/17) [`6704526`](https://github.com/alexandernanberg/rapier.js/commit/67045267a9e620d3fd3db82348ee0a9f87e57b77) Thanks [@alexandernanberg](https://github.com/alexandernanberg)! - perf: remove per-call WASM allocations from body/collider creation, scene queries and transform getters
  
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

## 0.1.2

### Patch Changes

- [`fa0dbe5`](https://github.com/alexandernanberg/rapier.js/commit/fa0dbe52a277ba37d7376f9926a3b3eaedb5b63e) Thanks [@alexandernanberg](https://github.com/alexandernanberg)! - perf: pass vectors as scalars to force/velocity setters to remove per-call allocations

  `setLinvel`, `setAngvel`, `addForce`, `applyImpulse`, `addTorque`,
  `applyTorqueImpulse`, `addForceAtPoint`, and `applyImpulseAtPoint` previously
  marshalled their vector arguments through `VectorOps.intoRaw()`, which allocates
  a `RawVector` in WASM memory and crosses the JS↔WASM boundary three times (alloc,
  set, free) per call. They now pass the vector components as scalar arguments
  directly — the same zero-allocation pattern already used by `setTranslation`.

  This makes these per-frame setters 14–25× faster (e.g. `setLinvel` ~217ns → ~15ns,
  `applyImpulseAtPoint` ~404ns → ~16ns for 1000 bodies) and removes all GC pressure
  from applying forces/impulses each step — a common hot path for character
  controllers, vehicles, thrusters and projectiles.

- [#12](https://github.com/alexandernanberg/rapier.js/pull/12) [`261b414`](https://github.com/alexandernanberg/rapier.js/commit/261b414600dedc14aea19749cbd05c809a209069) Thanks [@alexandernanberg](https://github.com/alexandernanberg)! - perf: read collider world transforms from a shared buffer

  `Collider.translation()` and `Collider.rotation()` previously crossed the
  JS↔WASM boundary on every call. Collider world transforms are now mirrored into
  a contiguous buffer (filled inside the Rust `step()`, mirroring the rigid-body
  transform buffer) and read directly from JS as plain array accesses, eliminating
  per-call boundary crossings in render loops (~7× faster for reading many
  colliders per frame). Relative getters (`translationWrtParent` /
  `rotationWrtParent`) are unaffected.

  The buffer is a `Float32Array` view into WASM memory, so reads go through a
  `liveBuffer()` guard (mirroring `RigidBody.liveBuffer()`): the view is dropped
  and reads fall back to the WASM path whenever it is invalidated (a collider was
  created or mutated) or detached by WASM memory growth (`memory.grow()` leaves a
  zero-length view). `World.step()` rebuilds the view each step.

- [#11](https://github.com/alexandernanberg/rapier.js/pull/11) [`da2cfab`](https://github.com/alexandernanberg/rapier.js/commit/da2cfab202787916a982962aa0abeea9b845567b) Thanks [@alexandernanberg](https://github.com/alexandernanberg)! - Fix stale/detached transform-buffer reads after WASM memory growth

  `RigidBody` position/velocity getters and setters read through a `Float32Array`
  view that points directly into WASM linear memory. When WASM memory grew between
  two `World.step()` calls (e.g. creating colliders or joints, or scene queries and
  setters that allocate via `intoRaw()`), the underlying `ArrayBuffer` was detached
  and the cached view became unusable — reads silently returned `NaN` and writes
  were silently dropped. Only `createRigidBody` invalidated the view, so the other
  growth paths were missed.

  Getters/setters now detect a detached view (length `0`) and fall back to reading
  directly from WASM until the next `World.step()` rebuilds it. Shared-memory
  (threads/SIMD) builds are unaffected since growing shared memory does not detach.

## 0.1.1

### Patch Changes

- [#7](https://github.com/alexandernanberg/rapier.js/pull/7) [`b9661f7`](https://github.com/alexandernanberg/rapier.js/commit/b9661f73c410e805a8ed13ebfc1074d4edaabef8) Thanks [@alexandernanberg](https://github.com/alexandernanberg)! - Fix bugs, memory leak, and code improvements

  - Fix `removeMultibodyJoint` checking wrong guard condition (`this.impulseJoints` → `this.multibodyJoints`)
  - Fix memory leak in `setHalfExtents` (missing `rawPoint.free()`)
  - Fix `ActiveCollisionTypes.ALL` missing `FIXED_FIXED` (had duplicate `KINEMATIC_KINEMATIC`)
  - Fix `lockRotations`/`lockTranslations` calling deprecated methods instead of current ones
  - Fix `ColliderSet.unmap` parameter type (`ImpulseJointHandle` → `ColliderHandle`)
  - Use shared module-level scratch buffer for `Collider` instead of per-instance allocation

- [`a3c89e9`](https://github.com/alexandernanberg/rapier.js/commit/a3c89e9f8c2a15348026d82c0dd747e84bcf95b1) Thanks [@alexandernanberg](https://github.com/alexandernanberg)! - Fix wasm-bindgen borrow tracking leak in transform buffer

  - Replace `transformBufferView(&self)` which leaked its shared borrow counter, causing `&mut self` calls after `world.step()` to fail with "recursive use of an object detected"
  - Construct `Float32Array` view on JS side from `WebAssembly.Memory` + packed ptr/len, bypassing wasm-bindgen borrow tracking entirely

## 0.1.0

### Minor Changes

- [#6](https://github.com/alexandernanberg/rapier.js/pull/6) [`278c7cc`](https://github.com/alexandernanberg/rapier.js/commit/278c7cc9663b44848f7d584857646539c052d890) Thanks [@alexandernanberg](https://github.com/alexandernanberg)! - Add ECS-inspired contiguous transform buffer for ~92% faster body getters

  Replace per-body WASM calls with a bulk Float64Array transform buffer that syncs all body transforms in a single WASM boundary crossing. This dramatically reduces overhead for `translation()`, `rotation()`, `linvel()`, `angvel()`, and related getters — especially when using the zero-allocation `target` parameter.
