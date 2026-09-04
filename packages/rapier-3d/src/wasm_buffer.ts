import {wasmMemory} from "./raw";

/**
 * @internal A view onto a WASM-resident `f32` buffer that a call published as a
 * packed `ptr | len << 32` pair.
 *
 * Rust fills the buffer and hands back its address; JS reads the payload out of a
 * typed-array view rather than having Rust push each value across the boundary.
 * Two things make such a view unusable, and both are handled here:
 *
 * - **Detachment**: any allocating WASM call can grow the linear memory, which
 *   detaches every view onto it (a detached view reports a `byteLength` of `0`
 *   and reads yield `undefined`). The buffer itself does not move, so the view is
 *   simply rebuilt from the same pointer.
 * - **Reallocation**: the producing call may have grown the buffer, which moves
 *   it. {@link WasmBuffer.reset} re-points the view, so it must be called with
 *   the freshly returned info every time.
 *
 * This is the growable counterpart to the fixed-capacity buffers in
 * `scratch.ts` and `transform_buffer.ts`.
 */
const EMPTY_F32 = new Float32Array(0);
const EMPTY_U32 = new Uint32Array(0);

// Decodes the packed `ptr | len << 32` pair in place, without allocating.
const _infoF64 = new Float64Array(1);
const _infoU32 = new Uint32Array(_infoF64.buffer);

/**
 * Reassembles a handle from the two `u32` halves a buffer carries it as.
 *
 * A handle is an `f64` holding `arenaIndex | generation << 32`, which no single
 * `f32` slot could hold. Same little-endian aliasing as `coarena.ts`.
 *
 * @internal
 */
export function handleFromParts(index: number, generation: number): number {
    _infoU32[0] = index;
    _infoU32[1] = generation;
    return _infoF64[0];
}

/** @internal */
export class WasmBuffer {
    private _f32: Float32Array = EMPTY_F32;
    private _u32: Uint32Array = EMPTY_U32;
    private _ptr = 0;
    private _len = 0;
    private _memory: WebAssembly.Memory | null = null;

    /** Number of `f32` slots the last {@link reset} published. */
    public get length(): number {
        return this._len;
    }

    /**
     * Points this view at the buffer described by `info`, the packed pair the
     * producing WASM call returned.
     */
    public reset(info: number) {
        _infoF64[0] = info;
        this._ptr = _infoU32[0];
        this._len = _infoU32[1];
        this._memory ??= wasmMemory() as unknown as WebAssembly.Memory;
        // The buffer may have been moved by the call that filled it, so both views
        // are dropped and rebuilt lazily against the new address.
        this._f32 = EMPTY_F32;
        this._u32 = EMPTY_U32;
    }

    /** Forgets the buffer, for when the WASM object owning it has been freed. */
    public release() {
        this._ptr = 0;
        this._len = 0;
        this._f32 = EMPTY_F32;
        this._u32 = EMPTY_U32;
    }

    /** The buffer as floats, re-attached if WASM memory growth detached it. */
    public f32(): Float32Array {
        const view = this._f32;
        // A view detached by memory growth has a zero byteLength — so does a view
        // of an empty buffer, which is why the length is checked first.
        if (this._len === 0) return EMPTY_F32;
        if (view.byteLength !== 0) return view;
        return (this._f32 = new Float32Array(this._memory!.buffer, this._ptr, this._len));
    }

    /** The same slots viewed as `u32`, for payloads carrying integers. */
    public u32(): Uint32Array {
        const view = this._u32;
        if (this._len === 0) return EMPTY_U32;
        if (view.byteLength !== 0) return view;
        return (this._u32 = new Uint32Array(this._memory!.buffer, this._ptr, this._len));
    }
}
