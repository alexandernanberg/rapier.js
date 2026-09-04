import base64 from "base64-js";
import {initSync} from "../wasm/release/rapier_wasm_3d";
// @ts-ignore - WASM file imported as base64 string by bundler
import wasmBase64 from "../wasm/release/rapier_wasm_3d_bg.wasm";

let initialized = false;

/**
 * Initializes RAPIER with embedded WASM (no separate file needed).
 * Has to be called and awaited before using any library methods.
 */
export async function init() {
    // `initSync` is a no-op once the module exists, but the base64 decode of
    // the embedded binary runs before it gets the chance to say so.
    if (initialized) return;
    const wasmBytes = base64.toByteArray(wasmBase64 as unknown as string);
    initSync({module: wasmBytes});
    initialized = true;
}
