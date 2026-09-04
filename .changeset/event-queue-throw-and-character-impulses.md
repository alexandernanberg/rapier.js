---
"@alexandernanberg/rapier2d": patch
"@alexandernanberg/rapier3d": patch
---

Fix three follow-up bugs from the bindings audit.

- A drain callback that throws no longer costs the rest of the step's events.
  `drainCollisionEvents` and `drainContactForceEvents` deliver every queued
  event and then surface the first error, rather than consuming and discarding
  everything after the throw.
- Those two also no longer leave the `EventQueue` unusable. The error used to be
  thrown from inside the WASM frame, which skipped the borrow guard's drop and
  left the queue permanently marked as borrowed — the next `drainCollisionEvents`,
  `clear()` or `free()` then aborted the module with "recursive use of an object
  detected". The error is returned to the JS glue instead, so the frame unwinds
  normally.
- `KinematicCharacterController.computeColliderMovement` now publishes the
  buffered velocities of the bodies it pushed when `applyImpulsesToDynamicBodies`
  is set. `linvel()` on a body shoved by the character returned the pre-impulse
  value until the next `step()`.
- `RigidBody.collider(i)` throws on an out-of-range index instead of returning
  `null` from a method typed `Collider`.
