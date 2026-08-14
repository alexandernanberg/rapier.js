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
    pub fn getComponents(&self, buffer: &js_sys::Float32Array) {
        buffer.set_index(0, self.contact.dist);

        #[cfg(feature = "dim2")]
        {
            let components = [
                self.contact.point1,
                self.contact.point2,
                self.contact.normal1,
                self.contact.normal2,
            ];
            for (i, u) in components.iter().enumerate() {
                buffer.set_index(1 + i as u32 * 2, u.x);
                buffer.set_index(2 + i as u32 * 2, u.y);
            }
        }

        #[cfg(feature = "dim3")]
        {
            let components = [
                self.contact.point1,
                self.contact.point2,
                self.contact.normal1,
                self.contact.normal2,
            ];
            for (i, u) in components.iter().enumerate() {
                buffer.set_index(1 + i as u32 * 3, u.x);
                buffer.set_index(2 + i as u32 * 3, u.y);
                buffer.set_index(3 + i as u32 * 3, u.z);
            }
        }
    }
}
