//! # Rapier
//! Fast and deterministic WASM physics engine.

#![allow(non_snake_case)] // JS uses camelCase, so we will follow its convention for the generated bindings.
                          // #![deny(missing_docs)]

#[cfg(feature = "dim2")]
extern crate rapier2d as rapier;
#[cfg(feature = "dim3")]
extern crate rapier3d as rapier;
#[macro_use]
extern crate serde;

#[wasm_bindgen::prelude::wasm_bindgen]
extern "C" {
    #[wasm_bindgen::prelude::wasm_bindgen(js_namespace = console, js_name = error)]
    fn console_error(message: &str);
}

/// Logs the panic message and its source location before the module aborts.
///
/// `wasm32-unknown-unknown` cannot unwind, so a panic anywhere in here reaches JS
/// as a bare `RuntimeError: unreachable` — no message, no location, nothing to act
/// on. The bindings validate their inputs so that a bad argument comes back as
/// `null` instead of panicking, but a panic that does get through should at least
/// say what it was.
///
/// This runs on module instantiation, before any binding can be called.
#[wasm_bindgen::prelude::wasm_bindgen(start)]
fn install_panic_hook() {
    std::panic::set_hook(Box::new(|info| console_error(&info.to_string())));
}

#[wasm_bindgen::prelude::wasm_bindgen]
pub fn version() -> String {
    env!("CARGO_PKG_VERSION").to_string()
}

#[wasm_bindgen::prelude::wasm_bindgen]
pub fn reserve_memory(extra_bytes_count: u32) {
    let mut unused: Vec<u8> = vec![];
    unused.reserve(extra_bytes_count as usize);
    std::hint::black_box(&unused);
}

/// Returns the WASM linear memory object.
///
/// Used by the TypeScript bindings to create Float32Array views from
/// raw pointers without going through wasm-bindgen borrow tracking.
#[wasm_bindgen::prelude::wasm_bindgen]
pub fn wasmMemory() -> wasm_bindgen::JsValue {
    wasm_bindgen::memory()
}

pub mod control;
pub mod dynamics;
pub mod geometry;
pub mod math;
pub mod pipeline;
pub(crate) mod scratch;
pub(crate) mod transform_buffer;
pub mod utils;
