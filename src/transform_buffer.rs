//! Bookkeeping for the contiguous transform buffers that JS reads directly.
//!
//! Both the rigid-body set and the collider set publish their transforms into a
//! `Vec<f32>` indexed by `arena_index * STRIDE`, which the TypeScript bindings
//! map as a `Float32Array` view (see `packages/rapier-*/src/transform_buffer.ts`).
//!
//! Refilling *every* slot after every step means walking the whole body and
//! collider arenas, which is pure overhead once a scene settles: the solver has
//! nothing left to integrate, yet the sync still touches every `RigidBody` and
//! `Collider` (several cache lines each) just to rewrite values that did not
//! change. The helpers here let a sync touch only the slots that can actually
//! have changed — the bodies the island manager reports as active, plus the ones
//! mutated from JS since the previous step.

use rapier::dynamics::RigidBodyHandle;
use rapier::geometry::ColliderHandle;

/// A handle that addresses a slot in a transform buffer.
pub(crate) trait ArenaHandle: Copy {
    /// The arena index the transform buffer is indexed by.
    fn arena_index(self) -> u32;
}

impl ArenaHandle for RigidBodyHandle {
    #[inline]
    fn arena_index(self) -> u32 {
        self.0.into_raw_parts().0
    }
}

impl ArenaHandle for ColliderHandle {
    #[inline]
    fn arena_index(self) -> u32 {
        self.0.into_raw_parts().0
    }
}

/// A dense set of arena indices, used to deduplicate a per-step refresh list.
///
/// Callers are expected to remove everything they inserted once they are done
/// with a pass, so the words stay zeroed between steps and clearing costs
/// `O(inserted)` rather than `O(index space)`.
#[derive(Default)]
pub(crate) struct IndexSet {
    words: Vec<u64>,
}

impl IndexSet {
    /// Inserts `index`, returning `true` if it was not already in the set.
    #[inline]
    pub(crate) fn insert(&mut self, index: u32) -> bool {
        let word = index as usize / 64;
        let bit = 1u64 << (index % 64);

        if self.words.len() <= word {
            self.words.resize(word + 1, 0);
        }

        let inserted = self.words[word] & bit == 0;
        self.words[word] |= bit;
        inserted
    }

    /// Removes `index` from the set.
    #[inline]
    pub(crate) fn remove(&mut self, index: u32) {
        let word = index as usize / 64;
        if let Some(w) = self.words.get_mut(word) {
            *w &= !(1u64 << (index % 64));
        }
    }
}

/// A transform buffer plus the list of slots that went stale outside of the
/// simulation (entities created or mutated from JS since the last sync).
pub(crate) struct TransformBuffer<H> {
    /// The buffer itself. `STRIDE` floats per arena index; holes left by unused
    /// indices are zero-filled and never read.
    pub(crate) data: Vec<f32>,
    /// Handles the next sync has to refresh even though the simulation may not
    /// have touched them.
    pending: Vec<H>,
    /// Dedup set for `pending`, keyed by arena index. Without it, mutating the
    /// same few entities many times between steps (a force applied to a body
    /// every frame, say) would count each call toward the full-sync threshold.
    pending_set: IndexSet,
    /// Rewrite every slot on the next sync. Set for a fresh or deserialized set,
    /// and whenever `pending` grew past the point where a full pass is cheaper.
    needs_full_sync: bool,
}

impl<H> Default for TransformBuffer<H> {
    fn default() -> Self {
        Self {
            data: Vec::new(),
            // Nothing has been published yet, so the first sync has to fill the
            // whole buffer (this is also the deserialization entry point, where
            // the set already holds entities).
            pending: Vec::new(),
            pending_set: IndexSet::default(),
            needs_full_sync: true,
        }
    }
}

impl<H: ArenaHandle> TransformBuffer<H> {
    /// Returns the buffer pointer and length packed into a single `f64`.
    /// Low 32 bits = byte offset in WASM memory, high 32 bits = f32 element count.
    pub(crate) fn info(&self) -> f64 {
        let ptr = self.data.as_ptr() as u32;
        let len = self.data.len() as u32;
        f64::from_bits(ptr as u64 | ((len as u64) << 32))
    }

    /// Flags `handle`'s slot as stale so the next sync refreshes it.
    ///
    /// `set_len` is the number of entities in the set: once enough distinct
    /// slots are pending, a full pass is cheaper than a scattered one, so the
    /// list is dropped in favour of `needs_full_sync`. That bound also keeps the
    /// list from growing without limit when JS mutates entities for a long time
    /// without stepping the simulation.
    #[inline]
    pub(crate) fn mark_pending(&mut self, handle: H, set_len: usize) {
        if self.needs_full_sync {
            return;
        }

        if self.pending.len() >= (set_len / 2).max(64) {
            self.clear_pending();
            self.needs_full_sync = true;
            return;
        }

        if self.pending_set.insert(handle.arena_index()) {
            self.pending.push(handle);
        }
    }

    /// Releases a removed entity's arena index from the dedup set, so whatever
    /// entity recycles the index before the next sync can be marked in turn.
    ///
    /// Without this, a body removed and re-created between two steps would be
    /// swallowed by the stale mark of its predecessor — and if the newcomer is
    /// fixed, the island manager never reports it either, so its slot would
    /// never be written. The stale handle itself stays in `pending`; the sync
    /// drops it when it finds no entity behind it.
    #[inline]
    pub(crate) fn forget(&mut self, handle: H) {
        self.pending_set.remove(handle.arena_index());
    }

    /// Drops the pending list. Only valid for a caller that is about to rewrite
    /// every slot anyway.
    #[inline]
    pub(crate) fn clear_pending(&mut self) {
        for handle in self.pending.drain(..) {
            self.pending_set.remove(handle.arena_index());
        }
    }

    /// Takes the pending list, leaving an empty one behind. The caller must give
    /// the allocation back with [`Self::restore_pending`].
    #[inline]
    pub(crate) fn take_pending(&mut self) -> Vec<H> {
        core::mem::take(&mut self.pending)
    }

    /// Gives the (now drained) `pending` allocation back after a sync.
    #[inline]
    pub(crate) fn restore_pending(&mut self, mut pending: Vec<H>) {
        for handle in pending.drain(..) {
            self.pending_set.remove(handle.arena_index());
        }
        self.pending = pending;
    }

    /// Whether the next sync must rewrite every slot; clears the flag.
    #[inline]
    pub(crate) fn take_needs_full_sync(&mut self) -> bool {
        core::mem::replace(&mut self.needs_full_sync, false)
    }

    /// Returns the `STRIDE` floats belonging to `index` if that slot already
    /// exists, without ever growing (and therefore moving) the buffer.
    ///
    /// This is the write-through path for mutations made between steps: JS may
    /// hold a live view onto the buffer at that point, so the data must not be
    /// reallocated. A slot that does not exist yet belongs to an entity created
    /// since the last sync; its `pending` mark gets it written on the next one.
    #[inline]
    pub(crate) fn existing_slot<const STRIDE: usize>(
        &mut self,
        index: u32,
    ) -> Option<&mut [f32; STRIDE]> {
        let offset = index as usize * STRIDE;
        self.data.get_mut(offset..)?.first_chunk_mut::<STRIDE>()
    }

    /// Returns the `STRIDE` floats belonging to `index`, growing the buffer if
    /// this is the first time that index is written.
    ///
    /// Returning a fixed-size array keeps the per-component writes free of bounds
    /// checks.
    #[inline]
    pub(crate) fn slot<const STRIDE: usize>(&mut self, index: u32) -> &mut [f32; STRIDE] {
        let offset = index as usize * STRIDE;

        if self.data.len() < offset + STRIDE {
            self.data.resize(offset + STRIDE, 0.0);
        }

        self.data[offset..]
            .first_chunk_mut::<STRIDE>()
            .expect("transform buffer was just grown to fit this slot")
    }
}
