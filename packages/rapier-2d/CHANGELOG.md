# @alexandernanberg/rapier2d

## 0.1.0

### Minor Changes

- [#6](https://github.com/alexandernanberg/rapier.js/pull/6) [`278c7cc`](https://github.com/alexandernanberg/rapier.js/commit/278c7cc9663b44848f7d584857646539c052d890) Thanks [@alexandernanberg](https://github.com/alexandernanberg)! - Add ECS-inspired contiguous transform buffer for ~92% faster body getters

  Replace per-body WASM calls with a bulk Float64Array transform buffer that syncs all body transforms in a single WASM boundary crossing. This dramatically reduces overhead for `translation()`, `rotation()`, `linvel()`, `angvel()`, and related getters — especially when using the zero-allocation `target` parameter.
