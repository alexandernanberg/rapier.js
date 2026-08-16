---
"@alexandernanberg/rapier2d": patch
"@alexandernanberg/rapier3d": patch
---

fix: report bad shape and joint input as errors instead of trapping the WASM
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
