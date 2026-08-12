use crate::geometry::feature::IntoTypeValue;
use crate::geometry::RawFeatureType;
use crate::math::RawVector;
use rapier::geometry::RayIntersection;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct RawRayIntersection(pub(crate) RayIntersection);

#[wasm_bindgen]
impl RawRayIntersection {
    pub fn normal(&self) -> RawVector {
        self.0.normal.into()
    }

    pub fn time_of_impact(&self) -> f32 {
        self.0.time_of_impact
    }

    pub fn featureType(&self) -> RawFeatureType {
        self.0.feature.into_type()
    }

    pub fn featureId(&self) -> Option<u32> {
        self.0.feature.into_value()
    }
}
