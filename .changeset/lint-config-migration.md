---
"@alexandernanberg/rapier2d": patch
"@alexandernanberg/rapier3d": patch
---

Tidy up the bindings while adopting a shared lint config. `PhysicsHooks` now
declares its three hooks as function properties instead of methods, which is
stricter about parameter variance but accepts the same object literals and
classes. `Shape.fromRaw` reports an unrecognised shape by name rather than by
raw discriminant, and the compat `init()` is a plain function returning a
promise instead of an `async` one — it still has to be awaited.
