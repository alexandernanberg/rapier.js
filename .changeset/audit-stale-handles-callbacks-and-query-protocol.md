---
"@alexandernanberg/rapier2d": minor
"@alexandernanberg/rapier3d": minor
---

Fix a third batch of bugs found auditing the bindings, and take the remaining
per-call allocations off the shape-query and controller paths.

Bugs:

- A handle kept past its entity's removal no longer resolves to whatever
  entity recycled its arena index: `getRigidBody`, `getCollider`, `contains`
  and the joint lookups compare the full handle (index and generation), and
  removing an already removed entity is a no-op instead of detaching the live
  one that took its slot.
- Every accessor on a removed `RigidBody`, `Collider` or joint throws a JS
  error, not just the buffered transform reads. The Rust `expect` they used to
  hit is a WASM trap under `panic = "abort"`, and one that leaves the set's
  borrow flag stuck so the world can neither be stepped nor freed.
- Exceptions thrown from query predicates, hit callbacks and physics hooks are
  reported instead of dropped at the WASM boundary. A throwing predicate used
  to match every collider, a throwing hit callback kept iterating, a throwing
  `filterContactPair` filtered the pair out, and removing a collider from
  inside a query callback (rejected by wasm-bindgen for aliasing) silently did
  nothing. The query stops (or the hook answers as an absent hook would for the
  rest of the step) and the error is re-thrown once the WASM call has returned
  and released its borrows and temporaries.
- `RigidBody.setTransform(_, _, true)` wakes a sleeping body even when only
  one of translation and rotation changed. Rapier wakes from inside its "this
  component changed" branch, and the wake-up rode on the other component.
- Creating a joint on a removed body throws at the creation site instead of
  trapping the module from inside the next `step()`.
- The character controller publishes the post-impulse velocity of every body
  rapier pushed, not only the one behind the reported hit; rapier resolves the
  push with a contact query over the character's neighbourhood and the body it
  picks need not be the hit one.
- `NarrowPhase.contactPair` is re-entrant: a nested call from inside the
  callback used to overwrite and free the outer manifold.
- An unknown `massPropsMode` on a `ColliderDesc`, or ragged voxel data,
  throws instead of tripping an assert or silently dropping coordinates. Shape
  query rotations, joint frames and compound sub-shape rotations are normalized
  like every other quaternion input, so a drifted quaternion no longer scales
  the query shape by its squared length. `setHalfHeight` keeps a capsule's
  axis. 2D `JointType` gained the `Generic` variant the Rust side already
  reports.

Performance:

- `Collider.castShape`/`castCollider`/`intersectsShape`/`contactShape`/
  `contactCollider`, every `Shape` query, `World.castShape`,
  `intersectionWithShape`, `intersectionsWithShape` and
  `collidersWithAabbIntersectingAabb` pass poses and velocities as components
  and read their result out of the scratch buffer, the way the ray queries
  already do. A `Collider.castShape` used to cost six WASM allocations (each
  with a `FinalizationRegistry` entry) and nine boundary crossings; it now costs
  the shape's allocation and one crossing. The character controller's
  translation delta, the PID controller's targets and `setHalfExtents` are
  passed the same way.
- The "most bodies moved, rewrite every slot" shortcut of the transform sync
  was decided on the body count alone, so a few awake bodies among thousands of
  standalone colliders (tile maps, static level geometry) rewrote every
  collider slot every step. The collider sync decides on the collider count
  and keeps the moved-body list whenever it needs it.
- Collider setters that cannot move the collider (shape, material, groups,
  events, mass, flags, voxel edits) and `RigidBody.wakeUp` no longer count
  toward the incremental-sync budget, which used to force a full pass every
  step for scenes editing many colliders per frame.
- `World.castRay` accepts a `target`.
