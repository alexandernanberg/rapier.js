import base64 from "base64-js";
import {initSync} from "../wasm/release/rapier_wasm_3d";
// @ts-ignore - WASM file imported as base64 string by bundler
import wasmBase64 from "../wasm/release/rapier_wasm_3d_bg.wasm";

/**
 * Initializes RAPIER with embedded WASM (no separate file needed).
 * Has to be called and awaited before using any library methods.
 */
export async function init() {
    const wasmBytes = base64.toByteArray(wasmBase64 as unknown as string);
    initSync(wasmBytes.buffer);
}
