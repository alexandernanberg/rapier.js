---
"@alexandernanberg/rapier2d": minor
"@alexandernanberg/rapier3d": minor
---

Read contact manifolds through a WASM-resident buffer instead of boxed raw
pointers.

`narrowPhase.contactPair()` / `world.contactPair()` used to allocate two WASM
objects per pair (the pair and each manifold, each freed again after the
callback) and cross the JS↔WASM boundary once per field read. One call now
writes every manifold of the pair — normals, contact points, solver contacts,
friction and restitution — into a buffer owned by the narrow phase, and the
`TempContactManifold` handed to the callback is a cursor that reads straight out
of a typed-array view onto it. The whole walk is a single boundary crossing and
allocates nothing when the vector getters are given a `target`.

The buffer is a snapshot, so a manifold can no longer dangle: the raw pointers
were invalidated by the next `step()`, which the buffer is not. Nested
`contactPair` calls from inside the callback each get their own buffer, as
before.

`TempContactManifold` no longer has a `raw` field, a `bodies` field or a
`free()` method (there is nothing to free), and its getters return `null` /
`0` for an out-of-range contact index as they always did.
