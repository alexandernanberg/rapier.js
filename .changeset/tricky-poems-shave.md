---
"@alexandernanberg/rapier2d": minor
"@alexandernanberg/rapier3d": minor
---

Remove per-call WASM allocations from shape-cast hits, shape contacts, contact manifolds,
contact-force events, character-controller collisions and PID-controller corrections. These
now write into a scratch buffer in a single boundary crossing instead of returning temporary
raw objects. `TempContactManifold` also regains `solverContactPoint()`, which resolves solver
contacts back to world-space; `World.contactPair` passes the rigid-body set through for it.
