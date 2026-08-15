use rapier::parry::query;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct RawShapeContact {
    pub(crate) contact: query::Contact,
}

#[wasm_bindgen]
impl RawShapeContact {
    /// Writes this contact into the given buffer, in a single call.
    ///
    /// Layout: `[distance, point1, point2, normal1, normal2]`.
    ///
    /// The components are staged in a stack array and handed over in a single
    /// `copy_from`: every `set_index` would otherwise be its own call out to JS.
    pub fn getComponents(&self, buffer: &js_sys::Float32Array) {
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

        buffer.copy_from(&components);
    }
}
