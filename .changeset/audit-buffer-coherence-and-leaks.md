---
"@alexandernanberg/rapier2d": patch
"@alexandernanberg/rapier3d": patch
---

Fix a batch of correctness bugs found in an audit of the bindings, most of them
in the paths that read pose and velocity straight out of the shared transform
buffer.

- `linvel()`/`angvel()` now reflect `applyImpulse`, `applyTorqueImpulse`,
  `applyImpulseAtPoint`, `sleep()`, `setBodyType()` and the PID/vehicle
  controllers immediately, instead of returning the pre-mutation value until
  the next `step()`. Every WASM-side mutation now writes the body's slot
  through, so the JS-side write-throughs in the setters are gone (which also
  stops a rejected non-normalized quaternion from being written into the
  buffer). `setTransform` with a rejected rotation still honours `wakeUp`.
- Spherical joints reported `JointType.Generic` (and were wrapped as a generic
  joint) because the type-detection mask listed the angular rather than the
  linear axes.
- After `World.restoreSnapshot`, a parentless collider's `parent()` returned
  the body at arena index 0. `Coarena.get` now also treats a missing handle as
  "no entity" so an empty `Option` from WASM can never alias to index 0.
- `DynamicRayCastVehicleController.wheelGroundObject()` returned a collider
  for an airborne wheel for the same reason; it now returns `null`, and the
  misnamed `setIndexForwardAxis` setter is joined by a proper
  `indexForwardAxis` setter (the old one is deprecated).
- 2D `RigidBody.restrictTranslations` passed `enableX` for both axes.
- The auto-drained `EventQueue` never cleared contact-force events, so a
  collider with `ActiveEvents.CONTACT_FORCE_EVENTS` grew the queue forever
  unless `drainContactForceEvents` was called every step.
- A user callback that throws inside `drainCollisionEvents` /
  `drainContactForceEvents` now surfaces the exception (after the queue has
  been drained) instead of being swallowed.
- `JointData.intoRaw()` leaked the axis vector of generic joints, and a joint
  whose axis cannot be normalized now throws a readable error instead of an
  opaque wasm-bindgen assertion. A `HalfSpace` with a zero normal is rejected
  the same way instead of producing a NaN plane, and `setUp` on the character
  controller ignores a zero vector.
- `World.restoreSnapshot` freed neither its `SerializationPipeline` nor the
  deserialized-world shell; `NarrowPhase.contactPair` now frees its raw
  manifold/pair even when the callback throws.
- In 3D, `Collider.vertices()` returned `undefined` for polylines, and round
  cones had no `radius()`/`halfHeight()`; `RigidBody.collider(i)` with an
  out-of-range index no longer traps inside WASM.
- The character controller resets its computed collisions and grounded state
  when its collider has been removed, instead of reporting the previous call's
  results.
- Bitflag arguments (`ActiveEvents`, `ActiveHooks`, `QueryFilterFlags`,
  `TriMeshFlags`, ...) are now truncated to their known bits rather than
  silently dropping _all_ flags when an unknown bit is set.
- `init()` no longer triggers wasm-bindgen's deprecation warning on every call.
- 2D: `ColliderDesc.polyline` accepts `null` indices like 3D, `TriMesh.flags`
  is optional and the dead `ColliderDesc.rotationsEnabled` field is gone.

Performance: the pending-refresh list of the transform buffer is now
deduplicated, so mutating the same body many times between steps (a force
applied every frame, say) no longer pushes the sync into a full pass. The
release profile sets `panic = "abort"`.
