---
"@alexandernanberg/rapier2d": patch
"@alexandernanberg/rapier3d": patch
---

Fix a WASM memory leak and a stale transform-buffer view after free

- `JointData.intoRaw()` (3D) leaked a `RawVector` for every `Generic` impulse
  joint created or deserialized: the `Generic` case allocated `rawAx` via
  `intoRaw()` but never freed it (unlike the `Prismatic` and `Revolute` cases).
  It is now freed after use.
- `RigidBodySet.free()` replaced the shared transform-buffer reference with a
  brand-new object, leaving any lingering `RigidBody` instances pointing at the
  old reference whose `Float32Array` still viewed the freed `transform_data`.
  A getter on such a body would read freed linear memory and return garbage
  instead of failing. The buffer view is now nulled in-place so those bodies
  fall back to WASM and fail loudly.
