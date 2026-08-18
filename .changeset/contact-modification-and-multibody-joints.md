---
"@alexandernanberg/rapier2d": minor
"@alexandernanberg/rapier3d": minor
---

Add contact modification hooks, fill in the multibody joint API, add the missing
3D `ColliderDesc.halfspace`, and fix the PID controller's gain setters.

`PhysicsHooks.modifySolverContacts(context)` is now supported, for colliders
carrying the (newly exposed) `ActiveHooks.MODIFY_SOLVER_CONTACTS` flag. The
`ContactModificationContext` it receives can read the pair, its normal and each
solver contact, and can override the manifold's friction and restitution, move
or drop individual contacts, and set a per-contact tangent velocity (conveyor
belts). The context points straight at the manifold being built rather than
allocating a wasm object per call, so it is only valid for the duration of the
hook — outside of it every getter reads zero and every setter is a no-op. The
32-bit `userData` a hook writes survives to the next steps and is readable with
`TempContactManifold.userData()`. `filterContactPair` and
`filterIntersectionPair` are now optional, so a hooks object may implement only
what it needs.

`MultibodyJoint` gained the accessors that were commented out on both sides of
the boundary: `body1()`/`body2()`, `type()`, `anchor1()`/`anchor2()` (and
`frameX1()`/`frameX2()` in 3D, all with the usual optional `target`), plus
limits (`limitsEnabled()`, `limitsMin()`, `limitsMax()`, `setLimits()`) and
motors (`configureMotorModel()`, `setMotorMaxForce()`, `configureMotorVelocity()`,
`configureMotorPosition()`, `configureMotor()`) on unit joints, and per-axis
motors on 3D spherical multibody joints. `MultibodyJointSet.createJoint()` now
takes the `RigidBodySet` as its first argument, matching `ImpulseJointSet`, so
the joints can resolve their bodies.

`ColliderDesc.halfspace(normal)` was missing from the 3D package even though the
`HalfSpace` shape and its raw constructor were both there; 2D already had it.

`PidController.setKi()` and `setKd()` both wrote the proportional gain, so the
integral and derivative gains could not be changed after construction. `setKd()`
also wrote every axis' gain into the x component on the Rust side.
