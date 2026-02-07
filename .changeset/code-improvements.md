---
"@alexandernanberg/rapier2d": patch
"@alexandernanberg/rapier3d": patch
---

Fix bugs, memory leak, and code improvements

- Fix `removeMultibodyJoint` checking wrong guard condition (`this.impulseJoints` → `this.multibodyJoints`)
- Fix memory leak in `setHalfExtents` (missing `rawPoint.free()`)
- Fix `ActiveCollisionTypes.ALL` missing `FIXED_FIXED` (had duplicate `KINEMATIC_KINEMATIC`)
- Fix `lockRotations`/`lockTranslations` calling deprecated methods instead of current ones
- Fix `ColliderSet.unmap` parameter type (`ImpulseJointHandle` → `ColliderHandle`)
- Use shared module-level scratch buffer for `Collider` instead of per-instance allocation
