---
"@alexandernanberg/rapier3d": minor
---

Configure generic joints one axis at a time. `GenericImpulseJoint` and
`SphericalImpulseJoint` now share a `MultiAxisImpulseJoint` base exposing
`setLimits`, `limitsEnabled`, `limitsMin`, `limitsMax` and the motor
configuration for any `JointAxis`, so a generic joint's limits and motors can be
set after creation (a joint-simulated vehicle suspension, for instance).
`SphericalImpulseJoint` keeps the motor methods it already had, and gains the
limit ones.
