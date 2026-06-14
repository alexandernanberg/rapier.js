# @alexandernanberg/rapier2d

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
