/**
 * @internal Shared view onto a contiguous transform buffer living in WASM memory.
 *
 * `World.step()` fills these buffers on the Rust side and then refreshes the JS
 * view, so transform getters can read plain `Float32Array` slots instead of
 * crossing the WASM boundary.
 *
 * Two things can make the view unusable:
 *
 * - **Detachment**: any allocating WASM call can grow the linear memory, which
 *   detaches every `Float32Array` created from it (reads then yield `undefined`
 *   and the view reports a `byteLength` of `0`). The buffer contents themselves
 *   survive the growth, so the view can simply be rebuilt from the same pointer.
 * - **Invalidation**: creating or directly mutating a body/collider makes the
 *   buffer contents stale (and may move the buffer). Reads must then go through
 *   WASM until the next `World.step()` refills it.
 *
 * `ptr === 0` marks the invalidated state; a detached-but-valid view is
 * transparently re-attached by {@link liveTransformBuffer}.
 */
export interface TransformBufferRef {
    /** The current view, or `null` if it still has to be (re-)created. */
    buffer: Float32Array | null;
    /** Byte offset of the buffer in WASM memory, or `0` if the contents are stale. */
    ptr: number;
    /** Number of `f32` elements in the buffer. */
    len: number;
    /** The WASM memory `buffer` is a view into. */
    memory: WebAssembly.Memory | null;
}

// Scratch buffers for unpacking transformBufferInfo f64 → ptr + len
const _infoBuf = new Float64Array(1);
const _infoView = new Uint32Array(_infoBuf.buffer);

/**
 * Unpacks the `ptr | len << 32` pair returned by the WASM `*BufferInfo()`
 * getters. Only called when a view has to be (re-)created, so the returned
 * object is not on any hot path.
 *
 * @internal
 */
export function unpackBufferInfo(info: number): {ptr: number; len: number} {
    _infoBuf[0] = info;
    return {ptr: _infoView[0], len: _infoView[1]};
}

/** @internal */
export function createTransformBufferRef(): TransformBufferRef {
    return {buffer: null, ptr: 0, len: 0, memory: null};
}

/**
 * Marks the buffer contents as stale, forcing reads through WASM until the next
 * `World.step()`.
 *
 * @internal
 */
export function invalidateTransformBuffer(ref: TransformBufferRef) {
    ref.buffer = null;
    ref.ptr = 0;
}

/**
 * Points `ref` at the buffer described by `info` (the `ptr | len << 32` pair
 * returned by `transformBufferInfo()`), and re-creates the view.
 *
 * @internal
 */
export function refreshTransformBuffer(
    ref: TransformBufferRef,
    info: number,
    memory: WebAssembly.Memory,
) {
    const {ptr, len} = unpackBufferInfo(info);
    ref.ptr = ptr;
    ref.len = len;
    ref.memory = memory;
    ref.buffer = new Float32Array(memory.buffer, ptr, len);
}

/**
 * Returns a usable view of the buffer, re-attaching it if WASM memory growth
 * detached it, or `null` if the contents are stale and reads must go through
 * WASM.
 *
 * @internal
 */
export function liveTransformBuffer(ref: TransformBufferRef): Float32Array | null {
    const view = ref.buffer;
    // A detached view (after memory growth) has a zero byteLength.
    if (view !== null && view.byteLength !== 0) return view;
    if (ref.ptr === 0) return null;
    return (ref.buffer = new Float32Array(ref.memory!.buffer, ref.ptr, ref.len));
}
