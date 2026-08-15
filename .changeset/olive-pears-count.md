---
"@alexandernanberg/rapier2d": patch
"@alexandernanberg/rapier3d": patch
---

fix: reject malformed mesh arrays instead of trapping, and stop copying getter
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
