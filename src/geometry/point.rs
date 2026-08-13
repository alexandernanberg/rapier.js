use crate::math::RawVector;
use rapier::geometry::PointProjection;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct RawPointProjection(pub(crate) PointProjection);

#[wasm_bindgen]
impl RawPointProjection {
    pub fn point(&self) -> RawVector {
        self.0.point.into()
    }

    pub fn isInside(&self) -> bool {
        self.0.is_inside
    }
}
