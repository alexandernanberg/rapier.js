use crate::utils::{self, FlatHandle};
use rapier::geometry::{ColliderHandle, ShapeCastHit};
use wasm_bindgen::prelude::*;

/// Writes `hit` into `buffer` as `[time_of_impact, witness1, witness2, normal1, normal2]`.
fn write_hit(hit: &ShapeCastHit, buffer: &js_sys::Float32Array) {
    buffer.set_index(0, hit.time_of_impact);

    #[cfg(feature = "dim2")]
    {
        let components = [hit.witness1, hit.witness2, hit.normal1, hit.normal2];
        for (i, u) in components.iter().enumerate() {
            buffer.set_index(1 + i as u32 * 2, u.x);
            buffer.set_index(2 + i as u32 * 2, u.y);
        }
    }

    #[cfg(feature = "dim3")]
    {
        let components = [hit.witness1, hit.witness2, hit.normal1, hit.normal2];
        for (i, u) in components.iter().enumerate() {
            buffer.set_index(1 + i as u32 * 3, u.x);
            buffer.set_index(2 + i as u32 * 3, u.y);
            buffer.set_index(3 + i as u32 * 3, u.z);
        }
    }
}

#[wasm_bindgen]
pub struct RawShapeCastHit {
    pub(crate) hit: ShapeCastHit,
}

#[wasm_bindgen]
impl RawShapeCastHit {
    /// Writes this hit into the given buffer, in a single call.
    ///
    /// Layout: `[time_of_impact, witness1, witness2, normal1, normal2]`.
    pub fn getComponents(&self, buffer: &js_sys::Float32Array) {
        write_hit(&self.hit, buffer);
    }
}

#[wasm_bindgen]
pub struct RawColliderShapeCastHit {
    pub(crate) handle: ColliderHandle,
    pub(crate) hit: ShapeCastHit,
}

#[wasm_bindgen]
impl RawColliderShapeCastHit {
    pub fn colliderHandle(&self) -> FlatHandle {
        utils::flat_handle(self.handle.0)
    }

    /// Writes this hit into the given buffer, in a single call.
    ///
    /// Layout: `[time_of_impact, witness1, witness2, normal1, normal2]`.
    pub fn getComponents(&self, buffer: &js_sys::Float32Array) {
        write_hit(&self.hit, buffer);
    }
}
