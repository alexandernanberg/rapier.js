---
"@alexandernanberg/rapier2d": minor
"@alexandernanberg/rapier3d": minor
---

Port upstream joint additions: `ImpulseJoint.setFrameX1/setFrameX2/setLocalFrame1/setLocalFrame2`,
`UnitImpulseJoint.setMotorMaxForce`, per-axis motor configuration on `SphericalImpulseJoint`
(with the new `JointAxis` enum), and `JointData.revoluteWithAxes` for hinges whose local axis
differs on each body.
