---
"@alexandernanberg/rapier2d": minor
"@alexandernanberg/rapier3d": minor
---

Require a target object on every getter and scene query

The allocating form of every read path is gone. Getters and queries that used to
return a freshly allocated `Vector`, `Rotation`, matrix or result object now
require a caller-owned target and write into it, so the zero-allocation path is
the only path rather than an opt-in.

```typescript
// Before
const pos = body.translation();
const hit = world.castRayAndGetNormal(ray, 100, true);

// After
const _pos = {x: 0, y: 0, z: 0};
const _hit = new RAPIER.RayColliderIntersection();

body.translation(_pos);
world.castRayAndGetNormal(ray, 100, true, _hit);
```

Migration notes:

- **Query argument order changed.** `target` sits after the required query
  inputs and _before_ the optional filter arguments — TypeScript does not allow
  a required parameter to follow an optional one, so it cannot be last. Simple
  getters take it as their only argument.
- **Result types are zero-arg constructible** and pre-fill their nested vectors:
  `new RAPIER.RayColliderIntersection()`. They no longer accept constructor
  arguments.
- **`castRay` and `intersectionsWithRay` now take targets too**
  (`RayColliderHit` and `RayColliderIntersection`). `intersectionsWithRay`
  reuses the one target for every hit, so a callback that retains a result must
  copy it.
- **`Ray.pointAt`, `KinematicCharacterController.up` and `computedCollision`**
  take targets as well; `computedCollision`'s parameter is now required.
- A query that misses still returns `null` and leaves the target untouched.

`VectorOps.zeros()` and `RotationOps.identity()` remain for deliberately
allocating a target.
