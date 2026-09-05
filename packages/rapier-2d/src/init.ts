import wasmInit, {InitInput} from "../wasm/release/rapier_wasm_2d";

let pending: Promise<void> | undefined;

/**
 * Initializes RAPIER using fetch (requires WASM file to be served).
 * Has to be called and awaited before using any library methods.
 *
 * Concurrent and repeated calls share one initialization: the generated loader
 * only short-circuits once the module is *already* instantiated, so two callers
 * awaiting `init()` at the same time (two components mounting, say) would each
 * fetch and instantiate their own module, and the second one to finish would
 * swap the exports out from under every object the first had created. Only the
 * first call's `input` is used.
 */
export function init(input?: InitInput): Promise<void> {
    if (!pending) {
        const source = input ?? new URL("rapier_wasm_2d_bg.wasm", import.meta.url);
        pending = wasmInit({module_or_path: source}).then(
            () => undefined,
            (error) => {
                // Let a later call retry after a failed fetch.
                pending = undefined;
                throw error;
            },
        );
    }
    return pending;
}
