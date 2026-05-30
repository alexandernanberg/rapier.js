---
"@alexandernanberg/rapier2d": patch
"@alexandernanberg/rapier3d": patch
---

perf: read collider world transforms from a shared buffer

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
