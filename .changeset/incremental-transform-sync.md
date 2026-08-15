---
"@alexandernanberg/rapier2d": patch
"@alexandernanberg/rapier3d": patch
---

Refresh the shared transform buffers incrementally instead of rewriting them on every step.

`World.step()` used to walk the entire rigid-body and collider arenas afterwards to refill the buffers the transform getters read from. That walk is proportional to the number of entities in the world, not to how many of them moved, so it dominated the step time of any scene that had settled: with 3000 sleeping bodies it accounted for essentially the whole step.

The sync now only rewrites the slots that can have changed — the bodies the island manager reports as active, the bodies that were active during the previous step (so a body that just fell asleep still gets its final pose written), the colliders attached to those bodies, and anything created or mutated from JS since the last step. Scenes where most entities keep moving fall back to the sequential pass, so they are unaffected.

Measured on the benchmark suite (3D, 3000-body pyramid): `world.step()` goes from ~88µs to ~2µs once the pyramid settles. The equivalent 2D scene goes from ~100µs to ~5µs per step. A scene where nothing ever sleeps is unchanged.

`refreshTransformBuffer` also stops allocating a new `Float32Array` view on every step when the buffer has neither moved nor been resized.
