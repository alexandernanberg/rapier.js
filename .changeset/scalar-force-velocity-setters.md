---
"@alexandernanberg/rapier2d": patch
"@alexandernanberg/rapier3d": patch
---

perf: pass vectors as scalars to force/velocity setters to remove per-call allocations

`setLinvel`, `setAngvel`, `addForce`, `applyImpulse`, `addTorque`,
`applyTorqueImpulse`, `addForceAtPoint`, and `applyImpulseAtPoint` previously
marshalled their vector arguments through `VectorOps.intoRaw()`, which allocates
a `RawVector` in WASM memory and crosses the JS↔WASM boundary three times (alloc,
set, free) per call. They now pass the vector components as scalar arguments
directly — the same zero-allocation pattern already used by `setTranslation`.

This makes these per-frame setters 14–25× faster (e.g. `setLinvel` ~217ns → ~15ns,
`applyImpulseAtPoint` ~404ns → ~16ns for 1000 bodies) and removes all GC pressure
from applying forces/impulses each step — a common hot path for character
controllers, vehicles, thrusters and projectiles.
