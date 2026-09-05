---
"@alexandernanberg/rapier2d": minor
"@alexandernanberg/rapier3d": minor
---

Fix a third batch of bugs found auditing the bindings, and take more
allocations and boundary crossings off hot paths.

Bugs:

- `World.forEachActiveRigidBody` no longer hands the callback `null` for a body
  an earlier iteration of the same walk removed (the active set is a snapshot
  taken before the walk starts).
- `World.propagateModifiedBodyPositionsToColliders` writes the moved
  colliders' new world poses into the transform buffer instead of invalidating
  it, so every collider read in the rest of the frame stays on the buffer
  rather than crossing into WASM.
- `setRotation` with a quaternion that cannot be normalized still honours
  `wakeUp`, like `setTransform` already did.
- `ContactForceEvent.started()` is exposed: `true` on the first step a pair's
  force crosses its threshold, `false` while it stays above it.
- `ColliderDesc.convexDecomposition` with `resolution`, `planeDownsampling` or
  `convexHullDownsampling` set to zero no longer traps the module.
- Creating an impulse or multibody joint between a body and itself throws
  instead of building a degenerate joint graph.
- `DynamicRayCastVehicleController.indexUpAxis`/`indexForwardAxis` reject an
  index outside `0..3` instead of silently turning the chassis velocity `NaN`.
- `PidController` treats a zero or non-finite target rotation as "no
  correction" instead of steering the body toward the identity.
- `Collider.setRotationWrtParent` (3D) stores the quaternion it was given
  instead of a lossy axis-angle round trip that could flip its sign.
- `World.free()` can be called twice; `World.takeSnapshot()` throws instead of
  returning `undefined` when serialization fails.
- `World.contactPairsWith`/`intersectionPairsWith` stop enumerating once the
  callback has thrown, and `NarrowPhase.contactPairsWith`/
  `intersectionPairsWith` end early when the callback returns `false`, like the
  other enumerations.
- The raw shape of a collider is freed even when its creation throws, and a
  `Compound` whose sub-shape is rejected part-way frees the ones built before
  it. `BroadPhase.castShape` reads its result before freeing the query shape.

Performance:

- `setNextKinematicTranslation`/`setNextKinematicRotation`/
  `setNextKinematicTransform` no longer count toward the incremental
  transform-sync budget: driving more than a few dozen kinematic bodies per
  frame used to push every step into a full body *and* collider re-sync.
- `RigidBody.setAdditionalMassProperties`, `Collider.setMassProperties`, the
  `ImpulseJoint` anchor/frame setters and
  `KinematicCharacterController.setUp` pass components instead of allocating
  WASM vector/rotation temporaries per call (which also could not be freed if
  the call threw).
- `KinematicCharacterController.computedCollision` is one WASM call instead of
  three, and the controller no longer allocates a raw collision object.
