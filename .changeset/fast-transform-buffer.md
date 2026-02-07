---
"@alexandernanberg/rapier2d": minor
"@alexandernanberg/rapier3d": minor
---

Add ECS-inspired contiguous transform buffer for ~92% faster body getters

Replace per-body WASM calls with a bulk Float64Array transform buffer that syncs all body transforms in a single WASM boundary crossing. This dramatically reduces overhead for `translation()`, `rotation()`, `linvel()`, `angvel()`, and related getters — especially when using the zero-allocation `target` parameter.
