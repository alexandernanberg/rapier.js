---
"@alexandernanberg/rapier2d": patch
"@alexandernanberg/rapier3d": patch
---

fix: remove unsound `unsafe impl Send`/`Sync` on the physics hooks

`RawPhysicsHooks` holds JS values (`js_sys::Object`/`Function`), which are not
`Send`/`Sync`. Rapier's `PhysicsHooks` trait required `Send + Sync`, so the
bindings asserted both with a hand-written `unsafe impl`, justified only by the
observation that wasm is single-threaded.

Rapier 0.35 added the `unsync-callbacks` feature, which drops the `Sync` bound
from `PhysicsHooks` and `EventHandler` (through `utils::MaybeSync`) for exactly
this case — thread-affine callbacks such as a JS closure. Enabling it lets the
hooks be accepted as-is, so both `unsafe impl`s are gone with no change in
behaviour.
