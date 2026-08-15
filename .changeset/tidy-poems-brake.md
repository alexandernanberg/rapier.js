---
"@alexandernanberg/rapier2d": minor
"@alexandernanberg/rapier3d": minor
---

feat: upgrade to Rapier 0.35

Upgrades the underlying physics engine from Rapier 0.32 to 0.35, spanning three
upstream breaking releases. Most of the upgrade is internal, but a few things
change for consumers.

**API changes**

- `IntegrationParameters.minIslandSize` / `World.minIslandSize` were removed.
  Awake bodies are now solved as a single active set, so the parameter no longer
  exists upstream.
- On a contact manifold, solver contacts now store a per-body anchor instead of a
  single world-space point, which are exposed as the new `solverContactAnchor1(i)`
  and `solverContactAnchor2(i)`. The anchors are expressed in the body's
  center-of-mass-centered local frame, or in world-space when that side has no
  solver body (no rigid-body, or world-attached by dominance — fixed bodies
  included). `solverContactPoint(i)` is still available and still returns a
  world-space point, now midway between both surfaces; its return type is now
  `Vector | null`, which is what it already returned for an out-of-bounds index.
- `solverContactFriction(i)` and `solverContactRestitution(i)` were replaced by
  `friction()` and `restitution()` on the manifold. Both coefficients are now
  stored per-manifold rather than per solver-contact, so they are identical for
  every contact of a manifold.

**Behaviour changes**

- Contact defaults changed upstream: `normalizedPredictionDistance` is now `0.02`
  (was `0.002`) and `normalizedAllowedLinearError` is now `0.005` (was `0.001`),
  which greatly reduces tunneling through thin walls.
- Sleeping was rewritten around persistent islands: an island now sleeps and
  wakes strictly as a unit, so a body is never frozen while something it touches
  still moves. Sleep eligibility is judged on the actual per-step pose
  displacement instead of velocities, and bodies now become eligible after `0.5`s
  instead of `2.0`s.
- CCD was rewritten around sweep-based time of impact. Fast dynamic bodies now
  always run CCD against fixed colliders; `setCcdEnabled`/`enableCcd` now upgrade
  a body to a "bullet" that additionally sweeps kinematic and dynamic bodies. Set
  `World.maxCcdSubsteps` to `0` to disable CCD entirely.
- Body velocities are now capped per substep (linear speed at 400 units/s by
  default, rotation at ~45° per step).
- `narrowPhase.contactPair()` no longer invokes its callback for two colliders
  attached to the same rigid-body, since the broad phase no longer pairs them.
- World snapshots taken with an earlier version cannot be restored: the island
  manager and rigid-body activation serialization formats changed upstream.
