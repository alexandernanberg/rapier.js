---
"@alexandernanberg/rapier2d": patch
"@alexandernanberg/rapier3d": patch
---

Fix wasm-bindgen borrow tracking leak in transform buffer

- Replace `transformBufferView(&self)` which leaked its shared borrow counter, causing `&mut self` calls after `world.step()` to fail with "recursive use of an object detected"
- Construct `Float32Array` view on JS side from `WebAssembly.Memory` + packed ptr/len, bypassing wasm-bindgen borrow tracking entirely
