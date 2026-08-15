---
"@alexandernanberg/rapier2d": patch
"@alexandernanberg/rapier3d": patch
---

Fix physics hooks being ignored when stepping without an event queue, and
`KinematicCharacterController.up()` returning a raw WASM handle.

`World.step(undefined, hooks)` previously dropped the hooks: the pipeline only
forwarded them on the `stepWithEvents` branch, and the plain `step` binding
hardcoded a no-op hook implementation on the Rust side, so `filterContactPair`
and `filterIntersectionPair` were never called unless an `EventQueue` was also
passed. Hooks are now routed through a new `stepWithHooks` binding; the
hookless path is unchanged so it does not pay for marshalling them.

`up()` returned `this.raw.up()` without unwrapping it, so callers got a
`RawVector` handle instead of a plain vector — and since nothing freed it, every
call leaked.
