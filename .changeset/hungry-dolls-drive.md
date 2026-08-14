---
"@alexandernanberg/rapier2d": minor
"@alexandernanberg/rapier3d": minor
---

Add compound shapes and convex decomposition, ported from upstream: the new `Compound`
shape, `ColliderDesc.compound()`, and `ColliderDesc.convexDecomposition()` (VHACD, tunable
through `VHACDParameters`). `Shape.fromRawShape()` reconstructs any shape — including the
sub-shapes of a compound — from its raw handle, and `Shape.fromRaw()` now goes through it.

Also fixes the vertex/index buffers exported for convex polyhedra: `Collider.shape` returned
vertices from the shape's point set but indices from its triangulation, which don't always
agree, so the resulting mesh could not be fed back into `ColliderDesc.convexMesh()`. Both are
now derived from the same recomputed convex hull.
