use crate::utils;
use rapier::dynamics::IslandManager;
use wasm_bindgen::prelude::*;

/// The island manager, plus the buffer its active-body handles are published
/// into. Field `1` is that buffer; see [`RawIslandManager::activeBodyHandles`].
#[wasm_bindgen]
pub struct RawIslandManager(pub(crate) IslandManager, Vec<f32>);

impl RawIslandManager {
    /// Wraps an island manager restored from a snapshot.
    pub(crate) fn from_inner(islands: IslandManager) -> Self {
        RawIslandManager(islands, Vec::new())
    }
}

#[wasm_bindgen]
impl RawIslandManager {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        RawIslandManager::from_inner(IslandManager::new())
    }

    /// Writes the integer handle of every active rigid-body into this manager's
    /// buffer and returns that buffer's pointer and length packed into a single
    /// `f64`: low 32 bits the byte offset in WASM memory, high 32 bits the `f32`
    /// element count.
    ///
    /// After a short time of inactivity, a rigid-body is automatically deactivated
    /// ("asleep") by the physics engine in order to save computational power. A
    /// sleeping rigid-body never moves unless it is moved manually by the user.
    ///
    /// Each handle takes two slots — its arena index and its generation, as raw
    /// `u32` bit patterns, because the `f64` a handle packs them into would not
    /// survive an `f32`. JS reads them back through a `Uint32Array` view.
    ///
    /// Calling a JS function per handle instead cost one boundary crossing (and one
    /// boxed `f64`) for every active body, every frame. It also ran the callback
    /// while WASM still held a borrow of this manager; the borrow is released here
    /// before JS walks the buffer.
    ///
    /// The buffer may be reallocated by this call, so the returned pointer is only
    /// valid until the next one.
    pub fn activeBodyHandles(&mut self) -> f64 {
        self.1.clear();

        for handle in self.0.active_bodies() {
            let bits = utils::flat_handle(handle.0).to_bits();
            self.1.push(f32::from_bits(bits as u32));
            self.1.push(f32::from_bits((bits >> 32) as u32));
        }

        let ptr = self.1.as_ptr() as u32;
        let len = self.1.len() as u32;
        f64::from_bits(ptr as u64 | ((len as u64) << 32))
    }
}
