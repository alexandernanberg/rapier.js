---
"@alexandernanberg/rapier2d": patch
"@alexandernanberg/rapier3d": patch
---

Fix stale/detached transform-buffer reads after WASM memory growth

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
