/**
 * Number of f32 values per body in the transform buffer.
 * Layout: translation(2) + rotation(1) + linvel(2) + angvel(1) = 6
 */
export const BODY_TRANSFORM_STRIDE = 6;

/**
 * @internal Shared container for the transform buffer, passed by reference
 * to RigidBody instances so they can read updated data without circular imports.
 */
export interface TransformBufferRef {
    buffer: Float32Array | null;
}

const _fconv = new Float64Array(1);
const _uconv = new Uint32Array(_fconv.buffer);

/**
 * @internal Extracts the arena index (lower 32 bits) from a handle (f64).
 * This is the same bit-manipulation used by Coarena.
 */
export function handleToIndex(handle: number): number {
    _fconv[0] = handle;
    return _uconv[0];
}
