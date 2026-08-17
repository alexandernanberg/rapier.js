---
"@alexandernanberg/rapier2d": minor
"@alexandernanberg/rapier3d": minor
---

feat: zero-allocation targets on scene queries, and the full solver tuning surface

- **Optional `target` on the remaining allocating getters and queries.** Until
  now only the transform getters (`translation()`, `rotation()`, `linvel()`, …)
  could write into a caller-owned object; every other query allocated a fresh
  result — plus a vector per point or normal inside it — on each call. `target`
  is now accepted by `castRayAndGetNormal()`, `projectPoint()`,
  `projectPointAndGetFeature()` and `castShape()` on `World` and `BroadPhase`,
  by `projectPoint()`, `castShape()`, `castCollider()`, `contactShape()`,
  `contactCollider()`, `castRayAndGetNormal()`, `halfExtents()` and
  `heightfieldScale()` on `Collider`, by the equivalent `Shape` queries, by
  `velocityAtPoint()`, `effectiveInvMass()`, `userForce()` and the 3D mass-
  property getters on `RigidBody`, by `anchor1()`/`anchor2()` (and 3D
  `frameX1()`/`frameX2()`) on `ImpulseJoint`, and by the wheel vector getters on
  `DynamicRayCastVehicleController`. It is the last argument, after the filter
  arguments; the target is returned as-is, its nested vectors are reused rather
  than replaced, and a query that misses returns `null` and leaves the target
  untouched.

- **The rest of `IntegrationParameters`.** Only 9 of the ~20 solver knobs Rapier
  exposes were bound. Added: `contactNaturalFrequency`, `contactDampingRatio`,
  `staticContactNaturalFrequency`, `staticContactDampingRatio`,
  `warmstartCoefficient`, `warmstartJoints`, `minCcdDt`,
  `normalizedMaxCorrectiveVelocity`, `normalizedMaxLinearVelocity`,
  `numInternalStabilizationIterations`, `contactClustering`, `contactRecycling`,
  `normalizedContactRecycleDistance`, `frictionInBiasPass`, and (3D only)
  `frictionModel` with the new `FrictionModel` enum (`Simplified`, `Coulomb`).
  Contact softness was previously write-only through
  `contact_natural_frequency`; that setter is kept as an alias of
  `contactNaturalFrequency`, which is now also readable.
