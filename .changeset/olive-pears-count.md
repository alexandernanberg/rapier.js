---
"@alexandernanberg/rapier2d": patch
"@alexandernanberg/rapier3d": patch
---

fix: reject malformed mesh arrays instead of trapping, and cut boundary crossings
for contact/hit payloads

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

**Fewer JS calls per payload**

`RawShapeContact::getComponents`, `RawShapeCastHit::getComponents` and
`RawCharacterCollision::getComponents` wrote their payload one element at a time,
and every one of those writes is its own call out to JS — 13 per shape-cast hit
or contact, 19 per character collision in 3D. Each now stages its components and
hands them over in a single `Float32Array::copy_from`.

Note for anyone reaching past the bindings into the raw API: `copy_from` asserts
that the JS buffer's length matches the payload exactly, so the scratch buffers
these methods write into are now sized per dimension rather than for the largest
of the two.
