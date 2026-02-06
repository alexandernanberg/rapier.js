use crate::dynamics::RawRigidBodySet;
use js_sys::Float32Array;
use wasm_bindgen::prelude::*;

/// Stride per body in 3D: translation(3) + rotation(4) + linvel(3) + angvel(3) = 13
#[cfg(feature = "dim3")]
pub const BODY_STRIDE: usize = 13;

/// Stride per body in 2D: translation(2) + rotation(1) + linvel(2) + angvel(1) = 6
#[cfg(feature = "dim2")]
pub const BODY_STRIDE: usize = 6;

/// A contiguous buffer that mirrors rigid body transforms in WASM linear memory.
///
/// After calling `sync()`, JavaScript can read body properties directly from
/// the returned Float32Array view without any additional WASM boundary crossings.
#[wasm_bindgen]
pub struct RawBodyTransformBuffer {
    data: Vec<f32>,
}

#[wasm_bindgen]
impl RawBodyTransformBuffer {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Self {
            data: Vec::new(),
        }
    }

    /// Returns the number of floats per body in the buffer.
    pub fn stride(&self) -> usize {
        BODY_STRIDE
    }

    /// Syncs all rigid body transforms into the contiguous buffer and returns
    /// a Float32Array view directly into WASM linear memory.
    ///
    /// The returned view is only valid until the next WASM memory growth.
    /// Callers should use it immediately and not retain references across allocations.
    pub fn sync(&mut self, bodies: &RawRigidBodySet) -> Float32Array {
        let mut max_index: usize = 0;
        for (handle, _) in bodies.0.iter() {
            let (index, _) = handle.0.into_raw_parts();
            max_index = max_index.max(index as usize);
        }

        let required_len = if bodies.0.len() > 0 {
            (max_index + 1) * BODY_STRIDE
        } else {
            0
        };

        if self.data.len() < required_len {
            self.data.resize(required_len, 0.0);
        }

        for (handle, body) in bodies.0.iter() {
            let (index, _) = handle.0.into_raw_parts();
            let offset = index as usize * BODY_STRIDE;

            let pos = body.position();
            let lv = body.linvel();

            #[cfg(feature = "dim3")]
            {
                let t = pos.translation;
                let r = pos.rotation;
                let av = body.angvel();
                self.data[offset] = t.x;
                self.data[offset + 1] = t.y;
                self.data[offset + 2] = t.z;
                self.data[offset + 3] = r.x;
                self.data[offset + 4] = r.y;
                self.data[offset + 5] = r.z;
                self.data[offset + 6] = r.w;
                self.data[offset + 7] = lv.x;
                self.data[offset + 8] = lv.y;
                self.data[offset + 9] = lv.z;
                self.data[offset + 10] = av.x;
                self.data[offset + 11] = av.y;
                self.data[offset + 12] = av.z;
            }

            #[cfg(feature = "dim2")]
            {
                let t = pos.translation;
                let r = pos.rotation.angle();
                let av = body.angvel();
                self.data[offset] = t.x;
                self.data[offset + 1] = t.y;
                self.data[offset + 2] = r;
                self.data[offset + 3] = lv.x;
                self.data[offset + 4] = lv.y;
                self.data[offset + 5] = av;
            }
        }

        unsafe { Float32Array::view(&self.data) }
    }
}
