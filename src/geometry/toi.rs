use crate::scratch;
use crate::utils::{self, FlatHandle};
use rapier::geometry::{ColliderHandle, ShapeCastHit};
use wasm_bindgen::prelude::*;

/// Writes `hit` into the scratch buffer as
/// `[time_of_impact, witness1, witness2, normal1, normal2]`.
fn write_hit(hit: &ShapeCastHit) {
    let w1 = hit.witness1;
    let w2 = hit.witness2;
    let n1 = hit.normal1;
    let n2 = hit.normal2;

    #[cfg(feature = "dim2")]
    let components = [
        hit.time_of_impact,
        w1.x,
        w1.y,
        w2.x,
        w2.y,
        n1.x,
        n1.y,
        n2.x,
        n2.y,
    ];

    #[cfg(feature = "dim3")]
    let components = [
        hit.time_of_impact,
        w1.x,
        w1.y,
        w1.z,
        w2.x,
        w2.y,
        w2.z,
        n1.x,
        n1.y,
        n1.z,
        n2.x,
        n2.y,
        n2.z,
    ];

    scratch::write(&components);
}

#[wasm_bindgen]
pub struct RawShapeCastHit {
    pub(crate) hit: ShapeCastHit,
}

#[wasm_bindgen]
impl RawShapeCastHit {
    /// Writes this hit into the shared scratch buffer, in a single call.
    ///
    /// Layout: `[time_of_impact, witness1, witness2, normal1, normal2]`.
    pub fn getComponents(&self) {
        write_hit(&self.hit);
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

    /// Writes this hit into the shared scratch buffer, in a single call.
    ///
    /// Layout: `[time_of_impact, witness1, witness2, normal1, normal2]`.
    pub fn getComponents(&self) {
        write_hit(&self.hit);
    }
}
