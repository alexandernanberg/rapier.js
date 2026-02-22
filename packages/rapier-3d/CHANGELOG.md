# @alexandernanberg/rapier3d

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
