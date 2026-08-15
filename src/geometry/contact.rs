use crate::scratch;
use rapier::parry::query;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct RawShapeContact {
    pub(crate) contact: query::Contact,
}

#[wasm_bindgen]
impl RawShapeContact {
    /// Writes this contact into the shared scratch buffer, in a single call.
    ///
    /// Layout: `[distance, point1, point2, normal1, normal2]`.
    pub fn getComponents(&self) {
        let p1 = self.contact.point1;
        let p2 = self.contact.point2;
        let n1 = self.contact.normal1;
        let n2 = self.contact.normal2;

        #[cfg(feature = "dim2")]
        let components = [
            self.contact.dist,
            p1.x,
            p1.y,
            p2.x,
            p2.y,
            n1.x,
            n1.y,
            n2.x,
            n2.y,
        ];

        #[cfg(feature = "dim3")]
        let components = [
            self.contact.dist,
            p1.x,
            p1.y,
            p1.z,
            p2.x,
            p2.y,
            p2.z,
            n1.x,
            n1.y,
            n1.z,
            n2.x,
            n2.y,
            n2.z,
        ];

        scratch::write(&components);
    }
}
