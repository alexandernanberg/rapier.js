import wasmInit, { InitInput } from "../wasm/release/rapier_wasm_2d";

/**
 * Initializes RAPIER using fetch (requires WASM file to be served).
 * Has to be called and awaited before using any library methods.
 */
export async function init(input?: InitInput) {
    if (!input) {
        input = new URL("rapier_wasm_2d_bg.wasm", import.meta.url);
    }
    await wasmInit(input);
}
