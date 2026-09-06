---
"@alexandernanberg/rapier2d": minor
"@alexandernanberg/rapier3d": minor
---

Fix a fourth batch of bugs found auditing the bindings, and take the remaining
per-event allocations off the event queue.

Bugs:

- Stepping with a hooks object that lacks a hook the previous hooks object had
  no longer keeps running the previous object's hook: the wrappers are dropped
  together with the object they close over.
- `CoefficientCombineRule` gains `ClampedSum` and `GeometricMean`, which
  rapier supports and which used to be silently turned into `Max` on the way
  in (and could not be named on the way out).
- Callbacks handed directly to `BroadPhase.intersectionsWithPoint`,
  `intersectionsWithShape`, `collidersWithAabbIntersectingAabb` and to
  `NarrowPhase.contactPairsWith` / `intersectionPairsWith` (rather than through
  the `World` wrappers) now stop the walk and propagate when they throw, instead
  of the error being swallowed at the WASM boundary. The Rust enumerations also
  stop on a failed callback rather than calling it again for every remaining
  hit.
- `Collider.radius()`, `roundRadius()`, `halfHeight()`, `vertices()`,
  `heightfieldHeights()` and (3D) `heightfieldNRows()` / `heightfieldNCols()`
  are typed and documented as returning `null` for a collider of another shape,
  which is what they did; they used to claim `number` / `Float32Array`.
- A rope joint that was given a velocity motor keeps reporting `JointType.Rope`
  rather than `Spring`.
- `ImpulseJointSet.createJoint`, `MultibodyJointSet.createJoint` and
  `SerializationPipeline.serializeAll` free their temporary raw objects even
  when the WASM call throws.
- The documented defaults of `IntegrationParameters` match rapier 0.35
  (`contactDampingRatio` 10, `warmstartJoints` false,
  `normalizedMaxCorrectiveVelocity` 3, `normalizedMaxLinearVelocity` 400,
  `numInternalStabilizationIterations` 1, `normalizedContactRecycleDistance`
  0.05).
- 2D: `PidAxesMask.LinZ`, which the controller ignored, is removed.

Performance:

- The event queue is now the event handler rapier writes into directly, in the
  buffer layout JS reads, instead of going through an `mpsc` channel that
  allocated and copied every event twice.
- A step given an event queue but no hooks no longer marshals a hooks object
  and three absent functions across the boundary, nor makes rapier consult a
  hooks object that answers "no hook" for every flagged pair.
- The WASM-resident drain buffers keep their typed-array views when the buffer
  neither moved nor resized, so a steady-state event drain or active-body walk
  allocates nothing.
- (3D) `DynamicRayCastVehicleController.addWheel` and the wheel vector setters
  pass their vectors component-wise instead of allocating raw vectors.
- Building a compound shape moves its parts instead of cloning them.
