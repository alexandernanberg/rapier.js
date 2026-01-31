import wasmInit, {InitInput} from "../wasm/release-simd/rapier_wasm_3d";

/**
 * Initializes RAPIER SIMD using fetch (requires WASM file to be served).
 * Has to be called and awaited before using any library methods.
 */
export async function init(input?: InitInput) {
    if (!input) {
        input = new URL("rapier_wasm_3d_simd_bg.wasm", import.meta.url);
    }
    await wasmInit(input);
}
