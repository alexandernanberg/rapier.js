//! A small WASM-resident buffer that getters write their results into, and that
//! JS reads directly through a `Float32Array` view.
//!
//! Handing a value back through a `js_sys::Float32Array` parameter means the data
//! crosses the boundary: `set_index` is one call out to JS per component (19 of
//! them for a character collision), and `copy_from` is one call plus a temporary
//! JS view of the source, and it asserts that the target's length matches the
//! payload exactly. Writing into WASM's own memory costs neither — the getter
//! call is the only crossing, and JS picks the components straight out of a view
//! it already holds.
//!
//! Same shape as the query-result buffer on `RawBroadPhase` and the transform
//! buffers: fixed capacity, allocated once, so its address never changes and the
//! JS view only ever has to be rebuilt when memory growth detaches it.
//!
//! Every writer here fills the buffer and returns immediately, and every reader
//! copies out before calling into WASM again, so the single shared buffer is
//! never live across two calls.

use rapier::math::{Rotation, Vector};
use std::cell::Cell;
use wasm_bindgen::prelude::*;

/// Long enough for the largest payload: a character collision writes
/// `1 + 6 * DIM` floats plus the two `u32` halves of the hit collider's handle,
/// which is 21 in 3D.
const SCRATCH_LEN: usize = 21;

thread_local! {
    static SCRATCH: [Cell<f32>; SCRATCH_LEN] = const { [const { Cell::new(0.0) }; SCRATCH_LEN] };
}

/// Returns the scratch buffer pointer and length packed into a single `f64`.
/// Low 32 bits = byte offset in WASM memory, high 32 bits = f32 element count.
///
/// Packed the same way as the transform and query-result buffers, so the JS side
/// decodes all three with one helper.
#[wasm_bindgen]
pub fn scratchBufferInfo() -> f64 {
    SCRATCH.with(|scratch| {
        let ptr = scratch.as_ptr() as u32;
        f64::from_bits(ptr as u64 | ((SCRATCH_LEN as u64) << 32))
    })
}

/// Writes `values` into the start of the scratch buffer.
///
/// Values past [`SCRATCH_LEN`] are dropped rather than panicking; the constant is
/// sized for the largest payload, so that cannot happen for any caller here.
#[inline]
pub(crate) fn write(values: &[f32]) {
    SCRATCH.with(|scratch| {
        for (slot, value) in scratch.iter().zip(values) {
            slot.set(*value);
        }
    });
}

/// Writes a single vector's components into the start of the scratch buffer.
#[inline]
pub(crate) fn write_vector(v: Vector) {
    #[cfg(feature = "dim2")]
    write(&[v.x, v.y]);
    #[cfg(feature = "dim3")]
    write(&[v.x, v.y, v.z]);
}

/// Writes a rotation into the start of the scratch buffer: the angle in 2D, the
/// quaternion components in 3D.
#[inline]
pub(crate) fn write_rotation(r: Rotation) {
    #[cfg(feature = "dim2")]
    write(&[r.angle()]);
    #[cfg(feature = "dim3")]
    write(&[r.x, r.y, r.z, r.w]);
}

/// Reinterprets a `u32` as the `f32` slot that carries it.
///
/// Feature ids and enum discriminants ride along in the scratch buffer as raw
/// bit patterns rather than as converted floats, so a feature id above 2^24
/// (a big heightfield cell index, say) survives the trip exactly. JS reads them
/// back through a `Uint32Array` view onto the same memory.
#[inline]
pub(crate) fn u32_bits(value: u32) -> f32 {
    f32::from_bits(value)
}

/// The bit pattern written for a feature id that does not exist.
pub(crate) const NO_FEATURE_ID: u32 = u32::MAX;
