---
"@alexandernanberg/rapier2d": patch
"@alexandernanberg/rapier3d": patch
---

chore: update dependencies

Refreshes the crates the WASM bindings are built from — notably `wasm-bindgen`
0.2.108 → 0.2.127, `rapier` 0.35.0 → 0.35.1 (with `parry` 0.30.2 and `nalgebra`
0.35.0) — and moves `bincode` from 1.3 to 2.0. `bincode` 2 replaced the free
`serialize`/`deserialize` functions with a `serde` integration module that takes
an explicit configuration; the world serializer now passes
`bincode::config::legacy()`, which reproduces bincode 1's encoding, so snapshot
bytes are unchanged.
