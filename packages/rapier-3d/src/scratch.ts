import {scratchBufferInfo, wasmMemory} from "./raw";
import {unpackBufferInfo} from "./transform_buffer";

/**
 * @internal View onto the WASM-side scratch buffer that getters write into.
 *
 * Getters used to take a `Float32Array` and write through it, which puts every
 * component across the JS/WASM boundary. They now write into WASM's own memory
 * and JS reads the components out of this view, so the getter call is the only
 * crossing.
 *
 * The buffer has a fixed capacity allocated once, so its address never changes.
 * The view still has to be rebuilt whenever WASM memory growth detaches it,
 * which shows up as a `byteLength` of `0` — the same handling as the transform
 * buffers and the broad-phase query results.
 */
let _view: Float32Array | null = null;
let _ptr = 0;
let _len = 0;
let _memory: WebAssembly.Memory | null = null;

/**
 * Returns a usable view of the scratch buffer, (re-)creating it if this is the
 * first call or if WASM memory growth detached the previous one.
 *
 * @internal
 */
export function scratch(): Float32Array {
    const view = _view;
    // A view detached by memory growth has a zero byteLength.
    if (view !== null && view.byteLength !== 0) return view;

    if (_ptr === 0) {
        const info = unpackBufferInfo(scratchBufferInfo());
        _ptr = info.ptr;
        _len = info.len;
        _memory = wasmMemory() as unknown as WebAssembly.Memory;
    }

    return (_view = new Float32Array(_memory!.buffer, _ptr, _len));
}
