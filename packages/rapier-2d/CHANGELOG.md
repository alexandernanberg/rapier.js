# @alexandernanberg/rapier2d

## 0.3.0

### Minor Changes

- [#29](https://github.com/alexandernanberg/rapier.js/pull/29) [`fabcdf6`](https://github.com/alexandernanberg/rapier.js/commit/fabcdf65b403720102596f7d8813f4396dd694c3) Thanks [@alexandernanberg](https://github.com/alexandernanberg)! - Fix a batch of correctness bugs found in an audit of the bindings, most of them
  in the paths that read pose and velocity straight out of the shared transform
  buffer.
  
  - `linvel()`/`angvel()` now reflect `applyImpulse`, `applyTorqueImpulse`,
    `applyImpulseAtPoint`, `sleep()`, `setBodyType()` and the PID/vehicle
    controllers immediately, instead of returning the pre-mutation value until
    the next `step()`. Every WASM-side mutation now writes the body's slot
    through, so the JS-side write-throughs in the setters are gone (which also
    stops a rejected non-normalized quaternion from being written into the
    buffer). `setTransform` with a rejected rotation still honours `wakeUp`.
  - Spherical joints reported `JointType.Generic` (and were wrapped as a generic
    joint) because the type-detection mask listed the angular rather than the
    linear axes.
  - After `World.restoreSnapshot`, a parentless collider's `parent()` returned
    the body at arena index 0. `Coarena.get` now also treats a missing handle as
    "no entity" so an empty `Option` from WASM can never alias to index 0.
  - `DynamicRayCastVehicleController.wheelGroundObject()` returned a collider
    for an airborne wheel for the same reason; it now returns `null`, and the
    misnamed `setIndexForwardAxis` setter is joined by a proper
    `indexForwardAxis` setter (the old one is deprecated).
  - 2D `RigidBody.restrictTranslations` passed `enableX` for both axes.
  - The auto-drained `EventQueue` never cleared contact-force events, so a
    collider with `ActiveEvents.CONTACT_FORCE_EVENTS` grew the queue forever
    unless `drainContactForceEvents` was called every step.
  - A user callback that throws inside `drainCollisionEvents` /
    `drainContactForceEvents` now surfaces the exception instead of being
    swallowed. The remaining events are still delivered first, the queue stays
    usable afterwards, and a contact-force event is freed even when the callback
    throws.
  - `JointData.intoRaw()` leaked the axis vector of generic joints, and a joint
    whose axis cannot be normalized now throws a readable error instead of an
    opaque wasm-bindgen assertion. A `HalfSpace` with a zero normal is rejected
    the same way instead of producing a NaN plane, and `setUp` on the character
    controller ignores a zero vector.
  - `World.restoreSnapshot` freed neither its `SerializationPipeline` nor the
    deserialized-world shell; `NarrowPhase.contactPair` now frees its raw
    manifold/pair even when the callback throws.
  - In 3D, `Collider.vertices()` returned `undefined` for polylines, and round
    cones had no `radius()`/`halfHeight()`; `RigidBody.collider(i)` with an
    out-of-range index no longer traps inside WASM.
  - The character controller resets its computed collisions and grounded state
    when its collider has been removed, instead of reporting the previous call's
    results.
  - Bitflag arguments (`ActiveEvents`, `ActiveHooks`, `QueryFilterFlags`,
    `TriMeshFlags`, ...) are now truncated to their known bits rather than
    silently dropping _all_ flags when an unknown bit is set.
  - `init()` no longer triggers wasm-bindgen's deprecation warning on every call.
  - 2D: `ColliderDesc.polyline` accepts `null` indices like 3D, `TriMesh.flags`
    is optional and the dead `ColliderDesc.rotationsEnabled` field is gone.
  
  Performance:
  
  - The remaining getters that handed a wasm-bindgen object across the boundary
    per call now write into the shared scratch buffer instead, so their `target`
    form is genuinely allocation-free and each is a single WASM call:
    `RigidBody.velocityAtPoint/effectiveInvMass/userForce/userTorque/
  principalInertia/invPrincipalInertia/principalInertiaLocalFrame/
  effectiveWorldInvInertia/effectiveAngularInertia`, `Collider.halfExtents/
  heightfieldScale`, the joint `anchor1/anchor2/frameX1/frameX2` getters, the
    vehicle wheel vector getters and `KinematicCharacterController.up()` (which
    now also takes a `target`).
  - `Collider.castRayAndGetNormal/castRay/intersectsRay/containsPoint/
  projectPoint` and the `Shape` equivalents pass their inputs as scalars and
    read their results out of the scratch buffer: a collider ray cast went from
    two input allocations, a result object and four getter crossings to one
    call. Feature ids ride along as exact `u32` bit patterns.
  - The pending-refresh list of the transform buffer is deduplicated, so
    mutating the same body many times between steps (a force applied every
    frame, say) no longer pushes the sync into a full pass.
  - The release profile sets `panic = "abort"`.
  
  Typing changes that fall out of this: `Collider.halfExtents()` and
  `heightfieldScale()` are typed `Vector | null` (they always returned `null`
  for other shapes at runtime), `Shape.castRayAndGetNormal()` is typed
  `RayIntersection | null`, `RigidBody.collider(i)` is typed `Collider | null`
  (it returns `null` for an out-of-range index instead of trapping),
  `Collider.projectPoint()` is no longer nullable, and
  `RayIntersection.fromRaw`/`PointProjection.fromRaw` are replaced by
  `fromBuffer`.

- [#37](https://github.com/alexandernanberg/rapier.js/pull/37) [`51c9279`](https://github.com/alexandernanberg/rapier.js/commit/51c9279df41e62afabe947289a53e714adb880d9) Thanks [@alexandernanberg](https://github.com/alexandernanberg)! - Fix a third batch of bugs found auditing the bindings, and take more
  allocations and boundary crossings off hot paths.
  
  Bugs:
  
  - `World.forEachActiveRigidBody` no longer hands the callback `null` for a body
    an earlier iteration of the same walk removed (the active set is a snapshot
    taken before the walk starts).
  - `World.propagateModifiedBodyPositionsToColliders` writes the moved
    colliders' new world poses into the transform buffer instead of invalidating
    it, so every collider read in the rest of the frame stays on the buffer
    rather than crossing into WASM.
  - `setRotation` with a quaternion that cannot be normalized still honours
    `wakeUp`, like `setTransform` already did.
  - `ContactForceEvent.started()` is exposed: `true` on the first step a pair's
    force crosses its threshold, `false` while it stays above it.
  - `ColliderDesc.convexDecomposition` with `resolution`, `planeDownsampling` or
    `convexHullDownsampling` set to zero no longer traps the module.
  - Creating an impulse or multibody joint between a body and itself throws
    instead of building a degenerate joint graph.
  - `DynamicRayCastVehicleController.indexUpAxis`/`indexForwardAxis` reject an
    index outside `0..3` instead of silently turning the chassis velocity `NaN`.
  - `PidController` treats a zero or non-finite target rotation as "no
    correction" instead of steering the body toward the identity.
  - `Collider.setRotationWrtParent` (3D) stores the quaternion it was given
    instead of a lossy axis-angle round trip that could flip its sign.
  - `World.free()` can be called twice; `World.takeSnapshot()` throws instead of
    returning `undefined` when serialization fails.
  - `World.contactPairsWith`/`intersectionPairsWith` stop enumerating once the
    callback has thrown, and `NarrowPhase.contactPairsWith`/
    `intersectionPairsWith` end early when the callback returns `false`, like the
    other enumerations.
  - The raw shape of a collider is freed even when its creation throws, and a
    `Compound` whose sub-shape is rejected part-way frees the ones built before
    it. `BroadPhase.castShape` reads its result before freeing the query shape.
  
  Performance:
  
  - `setNextKinematicTranslation`/`setNextKinematicRotation`/
    `setNextKinematicTransform` no longer count toward the incremental
    transform-sync budget: driving more than a few dozen kinematic bodies per
    frame used to push every step into a full body _and_ collider re-sync.
  - `RigidBody.setAdditionalMassProperties`, `Collider.setMassProperties`, the
    `ImpulseJoint` anchor/frame setters and
    `KinematicCharacterController.setUp` pass components instead of allocating
    WASM vector/rotation temporaries per call (which also could not be freed if
    the call threw).
  - `KinematicCharacterController.computedCollision` is one WASM call instead of
    three, and the controller no longer allocates a raw collision object.
  - `PhysicsHooks.filterContactPair`/`filterIntersectionPair` receive the pair's
    handles through the shared scratch buffer instead of four boxed JS numbers
    per call, which removes about eight boundary crossings per candidate pair
    per step.

- [#39](https://github.com/alexandernanberg/rapier.js/pull/39) [`b86a272`](https://github.com/alexandernanberg/rapier.js/commit/b86a272e17a742e6d877092004e1ff31c9ce449d) Thanks [@alexandernanberg](https://github.com/alexandernanberg)! - Fix a fourth batch of bugs found auditing the bindings, and take the remaining
  per-event allocations off the event queue.
  
  Bugs:
  
  - Stepping with a hooks object that lacks a hook the previous hooks object had
    no longer keeps running the previous object's hook: the wrappers are dropped
    together with the object they close over.
  - `CoefficientCombineRule` gains `ClampedSum` and `GeometricMean`, which
    rapier supports and which used to be silently turned into `Max` on the way
    in (and could not be named on the way out).
  - Callbacks handed directly to `BroadPhase.intersectionsWithPoint`,
    `intersectionsWithShape`, `collidersWithAabbIntersectingAabb` and to
    `NarrowPhase.contactPairsWith` / `intersectionPairsWith` (rather than through
    the `World` wrappers) now stop the walk and propagate when they throw, instead
    of the error being swallowed at the WASM boundary. The Rust enumerations also
    stop on a failed callback rather than calling it again for every remaining
    hit.
  - `Collider.radius()`, `roundRadius()`, `halfHeight()`, `vertices()`,
    `heightfieldHeights()` and (3D) `heightfieldNRows()` / `heightfieldNCols()`
    are typed and documented as returning `null` for a collider of another shape,
    which is what they did; they used to claim `number` / `Float32Array`.
  - A rope joint that was given a velocity motor keeps reporting `JointType.Rope`
    rather than `Spring`.
  - `ImpulseJointSet.createJoint`, `MultibodyJointSet.createJoint` and
    `SerializationPipeline.serializeAll` free their temporary raw objects even
    when the WASM call throws.
  - The documented defaults of `IntegrationParameters` match rapier 0.35
    (`contactDampingRatio` 10, `warmstartJoints` false,
    `normalizedMaxCorrectiveVelocity` 3, `normalizedMaxLinearVelocity` 400,
    `numInternalStabilizationIterations` 1, `normalizedContactRecycleDistance`
    0.05).
  - 2D: `PidAxesMask.LinZ`, which the controller ignored, is removed.
  
  Performance:
  
  - The event queue is now the event handler rapier writes into directly, in the
    buffer layout JS reads, instead of going through an `mpsc` channel that
    allocated and copied every event twice.
  - A step given an event queue but no hooks no longer marshals a hooks object
    and three absent functions across the boundary, nor makes rapier consult a
    hooks object that answers "no hook" for every flagged pair.
  - The WASM-resident drain buffers keep their typed-array views when the buffer
    neither moved nor resized, so a steady-state event drain or active-body walk
    allocates nothing.
  - (3D) `DynamicRayCastVehicleController.addWheel` and the wheel vector setters
    pass their vectors component-wise instead of allocating raw vectors.
  - Building a compound shape moves its parts instead of cloning them.

- [#33](https://github.com/alexandernanberg/rapier.js/pull/33) [`641da71`](https://github.com/alexandernanberg/rapier.js/commit/641da71b0fd66e05ff2fca92e8a111eb53ac37d0) Thanks [@alexandernanberg](https://github.com/alexandernanberg)! - Fix another batch of bugs found auditing the bindings, and take several
  allocation and boundary-crossing costs off hot paths.
  
  Bugs:
  
  - `init()` shares one in-flight initialization between concurrent callers.
    Two overlapping `await init()` calls used to each fetch and instantiate the
    module, and the second one to finish swapped the WASM exports out from under
    every object the first had created.
  - `World.forEachActiveRigidBody` no longer stops early when the callback grows
    WASM memory (any `intoRaw`, query or created entity can). The handle buffer
    view was read once up front, and a detached view reads as empty.
  - `collider.shape` reflects `setRadius`, `setHalfExtents`, `setHalfHeight` and
    `setRoundRadius` instead of the shape the collider was created with.
  - `translation()`/`rotation()`/`linvel()`/`angvel()` on a removed body or
    collider throw like every other accessor does, instead of reading the stale
    slot — or, once the arena index was recycled, another entity's transform.
  - A body pushed by the character controller (`applyImpulsesToDynamicBodies`)
    reports its new velocity right after `computeColliderMovement`, not after the
    next `step()`.
  - Creating a collider on a removed body throws a JS error; casting or
    contacting against a removed collider returns `null`; updating a vehicle
    whose chassis was removed is a no-op. All three used to trap the module.
  - A quaternion that drifted off unit length is normalized wherever it is
    accepted (descriptors and setters alike). Descriptors used to apply it as-is,
    skewing the pose, while the setters silently ignored it.
  - 2D `RigidBody.setRotation` overwrote the buffered angle with the unwrapped
    input, so `rotation()` disagreed with itself before and after the next step.
  - `JointType.Rope` and `JointType.Spring` are reported for rope and spring
    joints instead of `Generic`; the `contact_natural_frequency` alias can be
    read back.
  
  Performance:
  
  - Creating a body or collider no longer forces every other body's or
    collider's transform reads onto the WASM path until the next `step()`: the
    new slot is written at creation and the view re-pointed.
  - `Collider.setTranslation`/`setRotation` (and the `WrtParent` variants) write
    the new pose through the buffer instead of invalidating it for the whole set.
  - Forces, torques, damping, gravity scale, CCD, dominance, mass-property and
    solver-iteration setters no longer count toward the incremental-sync budget,
    which an `addForce` per body per frame used to exhaust into a full re-sync
    every step.
  - `ContactModificationContext.setNormal`, `setSolverContactPoint1/2` and
    `setSolverContactTangentVelocity` pass components instead of allocating a
    WASM vector per call; the `SdpMatrix3` `target` path, the event-drain
    strides, the compat `init()` re-decode and the debug-render colour
    conversion no longer allocate or cross the boundary needlessly.

- [#35](https://github.com/alexandernanberg/rapier.js/pull/35) [`4eb2964`](https://github.com/alexandernanberg/rapier.js/commit/4eb2964f71d28a75b19ad3860723c84e93616cee) Thanks [@alexandernanberg](https://github.com/alexandernanberg)! - Fix a third batch of bugs found auditing the bindings, and take the remaining
  per-call allocations off the shape-query and controller paths.
  
  Bugs:
  
  - A handle kept past its entity's removal no longer resolves to whatever
    entity recycled its arena index: `getRigidBody`, `getCollider`, `contains`
    and the joint lookups compare the full handle (index and generation), and
    removing an already removed entity is a no-op instead of detaching the live
    one that took its slot.
  - Every accessor on a removed `RigidBody`, `Collider` or joint throws a JS
    error, not just the buffered transform reads. The Rust `expect` they used to
    hit is a WASM trap under `panic = "abort"`, and one that leaves the set's
    borrow flag stuck so the world can neither be stepped nor freed.
  - Exceptions thrown from query predicates, hit callbacks and physics hooks are
    reported instead of dropped at the WASM boundary. A throwing predicate used
    to match every collider, a throwing hit callback kept iterating, a throwing
    `filterContactPair` filtered the pair out, and removing a collider from
    inside a query callback (rejected by wasm-bindgen for aliasing) silently did
    nothing. The query stops (or the hook answers as an absent hook would for the
    rest of the step) and the error is re-thrown once the WASM call has returned
    and released its borrows and temporaries.
  - `RigidBody.setTransform(_, _, true)` wakes a sleeping body even when only
    one of translation and rotation changed. Rapier wakes from inside its "this
    component changed" branch, and the wake-up rode on the other component.
  - Creating a joint on a removed body throws at the creation site instead of
    trapping the module from inside the next `step()`.
  - The character controller publishes the post-impulse velocity of every body
    rapier pushed, not only the one behind the reported hit; rapier resolves the
    push with a contact query over the character's neighbourhood and the body it
    picks need not be the hit one.
  - `NarrowPhase.contactPair` is re-entrant: a nested call from inside the
    callback used to overwrite and free the outer manifold.
  - An unknown `massPropsMode` on a `ColliderDesc`, or ragged voxel data,
    throws instead of tripping an assert or silently dropping coordinates. Shape
    query rotations, joint frames and compound sub-shape rotations are normalized
    like every other quaternion input, so a drifted quaternion no longer scales
    the query shape by its squared length. `setHalfHeight` keeps a capsule's
    axis. 2D `JointType` gained the `Generic` variant the Rust side already
    reports.
  
  Performance:
  
  - `Collider.castShape`/`castCollider`/`intersectsShape`/`contactShape`/
    `contactCollider`, every `Shape` query, `World.castShape`,
    `intersectionWithShape`, `intersectionsWithShape` and
    `collidersWithAabbIntersectingAabb` pass poses and velocities as components
    and read their result out of the scratch buffer, the way the ray queries
    already do. A `Collider.castShape` used to cost six WASM allocations (each
    with a `FinalizationRegistry` entry) and nine boundary crossings; it now costs
    the shape's allocation and one crossing. The character controller's
    translation delta, the PID controller's targets and `setHalfExtents` are
    passed the same way.
  - The "most bodies moved, rewrite every slot" shortcut of the transform sync
    was decided on the body count alone, so a few awake bodies among thousands of
    standalone colliders (tile maps, static level geometry) rewrote every
    collider slot every step. The collider sync decides on the collider count
    and keeps the moved-body list whenever it needs it.
  - Collider setters that cannot move the collider (shape, material, groups,
    events, mass, flags, voxel edits) and `RigidBody.wakeUp` no longer count
    toward the incremental-sync budget, which used to force a full pass every
    step for scenes editing many colliders per frame.
  - `World.castRay` accepts a `target`.

- [#32](https://github.com/alexandernanberg/rapier.js/pull/32) [`4be10f9`](https://github.com/alexandernanberg/rapier.js/commit/4be10f9fbaf7cf2bd50ce5a4efe189b7700d7c92) Thanks [@alexandernanberg](https://github.com/alexandernanberg)! - Move the remaining bulk read paths onto WASM-resident buffers, so a drain or a
  debug render costs one boundary crossing instead of one per item.
  
  - **Debug rendering.** `RawDebugRenderPipeline` no longer allocates a JS
    `Float32Array` from Rust and copies the lines into it on every frame. The
    vertex and color buffers stay in WASM memory and JS reads them through a view.
    `World.debugRender` takes an optional `target: DebugRenderBuffers` that it
    copies into and returns, so a caller that keeps one around allocates nothing
    per frame once it has grown to fit; without a target the behaviour is
    unchanged and a fresh pair is returned. `DebugRenderPipeline.vertices` and
    `.colors` are now views straight into WASM memory — no copy at all — and are
    valid only until the next call into WASM.
  
  - **Event draining.** `EventQueue.drainCollisionEvents` and
    `drainContactForceEvents` now move every pending event into a buffer with one
    call and walk it from JS. Contact force events no longer allocate a WASM
    object per event that JS has to read through four more calls and then free, so
    `TempContactForceEvent` has lost its `raw` field and its `free()` method.
    Handlers that throw still see every event delivered, with the first error
    re-thrown once the walk finishes.
  
  - **Active body iteration.** `IslandManager.forEachActiveRigidBodyHandle` (and
    `World.forEachActiveRigidBody` on top of it) publishes the handles into a
    buffer instead of calling into JS once per active body. The closure now also
    runs after WASM has released its borrow of the island manager rather than
    during it.
  
  Handles travel through these buffers as their arena index and generation in
  separate `u32` slots, so they come back bit-exact.

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
