---
"@alexandernanberg/rapier2d": minor
"@alexandernanberg/rapier3d": minor
---

Fix another batch of bugs found auditing the bindings, and take several
allocation and boundary-crossing costs off hot paths.

Bugs:

- `init()` shares one in-flight initialization between concurrent callers.
  Two overlapping `await init()` calls used to each fetch and instantiate the
  module, and the second one to finish swapped the WASM exports out from under
  every object the first had created.
- `World.forEachActiveRigidBody` no longer stops early when the callback grows
  WASM memory (any `intoRaw`, query or created entity can). The handle buffer
  view was read once up front, and a detached view reads as empty.
- `collider.shape` reflects `setRadius`, `setHalfExtents`, `setHalfHeight` and
  `setRoundRadius` instead of the shape the collider was created with.
- `translation()`/`rotation()`/`linvel()`/`angvel()` on a removed body or
  collider throw like every other accessor does, instead of reading the stale
  slot — or, once the arena index was recycled, another entity's transform.
- A body pushed by the character controller (`applyImpulsesToDynamicBodies`)
  reports its new velocity right after `computeColliderMovement`, not after the
  next `step()`.
- Creating a collider on a removed body throws a JS error; casting or
  contacting against a removed collider returns `null`; updating a vehicle
  whose chassis was removed is a no-op. All three used to trap the module.
- A quaternion that drifted off unit length is normalized wherever it is
  accepted (descriptors and setters alike). Descriptors used to apply it as-is,
  skewing the pose, while the setters silently ignored it.
- 2D `RigidBody.setRotation` overwrote the buffered angle with the unwrapped
  input, so `rotation()` disagreed with itself before and after the next step.
- `JointType.Rope` and `JointType.Spring` are reported for rope and spring
  joints instead of `Generic`; the `contact_natural_frequency` alias can be
  read back.

Performance:

- Creating a body or collider no longer forces every other body's or
  collider's transform reads onto the WASM path until the next `step()`: the
  new slot is written at creation and the view re-pointed.
- `Collider.setTranslation`/`setRotation` (and the `WrtParent` variants) write
  the new pose through the buffer instead of invalidating it for the whole set.
- Forces, torques, damping, gravity scale, CCD, dominance, mass-property and
  solver-iteration setters no longer count toward the incremental-sync budget,
  which an `addForce` per body per frame used to exhaust into a full re-sync
  every step.
- `ContactModificationContext.setNormal`, `setSolverContactPoint1/2` and
  `setSolverContactTangentVelocity` pass components instead of allocating a
  WASM vector per call; the `SdpMatrix3` `target` path, the event-drain
  strides, the compat `init()` re-decode and the debug-render colour
  conversion no longer allocate or cross the boundary needlessly.
