---
"@alexandernanberg/rapier2d": minor
"@alexandernanberg/rapier3d": minor
---

Move the remaining bulk read paths onto WASM-resident buffers, so a drain or a
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
