use crate::dynamics::RawRigidBodySet;
use crate::scratch;
use crate::utils::{self, pack_buffer_info, FlatHandle};
use rapier::geometry::{ContactManifold, NarrowPhase};
use rapier::math::{Vector, DIM};
use wasm_bindgen::prelude::*;

/// `f32` slots the fixed part of one contact manifold occupies in the pair
/// buffer: the world-space normal, both local normals, the user data, both
/// subshape indices, friction, restitution, and the two counts that size the
/// variable part that follows.
const MANIFOLD_STRIDE: usize = 3 * DIM + 7;

/// `f32` slots one contact point occupies: both local points, the distance, both
/// feature ids, the normal impulse and the tangent impulse (`DIM - 1`
/// components).
const CONTACT_STRIDE: usize = 3 * DIM + 3;

/// `f32` slots one solver contact occupies: both anchors, the world-space point,
/// the tangent velocity and the distance.
const SOLVER_CONTACT_STRIDE: usize = 4 * DIM + 1;

/// The stride of a manifold's fixed part in the contact-pair buffer, so the JS
/// side can walk it without hard-coding the layout.
#[wasm_bindgen]
pub fn contactManifoldStride() -> u32 {
    MANIFOLD_STRIDE as u32
}

/// The stride of one contact point in the contact-pair buffer.
#[wasm_bindgen]
pub fn contactPointStride() -> u32 {
    CONTACT_STRIDE as u32
}

/// The stride of one solver contact in the contact-pair buffer.
#[wasm_bindgen]
pub fn solverContactStride() -> u32 {
    SOLVER_CONTACT_STRIDE as u32
}

/// Appends a vector's components to `out`.
#[inline]
fn push_vector(out: &mut Vec<f32>, v: Vector) {
    #[cfg(feature = "dim2")]
    out.extend_from_slice(&[v.x, v.y]);
    #[cfg(feature = "dim3")]
    out.extend_from_slice(&[v.x, v.y, v.z]);
}

/// Appends a `u32` to `out` as a raw bit pattern, for JS to read back through a
/// `Uint32Array` view (see `scratch::u32_bits`).
#[inline]
fn push_u32(out: &mut Vec<f32>, value: u32) {
    out.push(scratch::u32_bits(value));
}

/// Appends one manifold to `out`, fixed part first and then its contacts and
/// solver contacts.
fn push_manifold(out: &mut Vec<f32>, manifold: &ContactManifold, bodies: &RawRigidBodySet) {
    let data = &manifold.data;
    push_vector(out, data.normal);
    push_vector(out, manifold.local_n1);
    push_vector(out, manifold.local_n2);
    push_u32(out, data.user_data);
    push_u32(out, manifold.subshape1);
    push_u32(out, manifold.subshape2);
    out.push(data.friction);
    out.push(data.restitution);
    push_u32(out, manifold.points.len() as u32);
    push_u32(out, data.solver_contacts.len() as u32);

    for contact in &manifold.points {
        push_vector(out, contact.local_p1);
        push_vector(out, contact.local_p2);
        out.push(contact.dist);
        push_u32(out, contact.fid1.0);
        push_u32(out, contact.fid2.0);
        out.push(contact.data.impulse);
        #[cfg(feature = "dim2")]
        out.push(contact.data.tangent_impulse.x);
        #[cfg(feature = "dim3")]
        out.extend_from_slice(&[
            contact.data.tangent_impulse.x,
            contact.data.tangent_impulse.y,
        ]);
    }

    for contact in data.solver_contacts.iter() {
        push_vector(out, contact.anchor1);
        push_vector(out, contact.anchor2);
        // Solver contacts store one body-local anchor per surface (the two differ
        // by the current separation along the normal), so resolving them back to
        // world-space needs the bodies they are anchored to. The point the solver
        // acts on is midway between both surfaces.
        let (p1, p2) = data.solver_contact_world_points(contact, &bodies.bodies);
        push_vector(out, (p1 + p2) / 2.0);
        push_vector(out, contact.tangent_velocity);
        out.push(contact.dist);
    }
}

#[wasm_bindgen]
pub struct RawNarrowPhase {
    pub(crate) narrow_phase: NarrowPhase,
    /// The buffers `contact_pair` publishes manifolds through, one per nesting
    /// depth of the JS walk (see [`RawNarrowPhase::contact_pair`]).
    pair_buffers: Vec<Vec<f32>>,
}

/// Whether an enumeration should go on after the callback answered `result`:
/// an explicit `false` stops it, as does a failed call; anything else continues.
#[inline]
fn continue_after(result: Result<JsValue, JsValue>) -> bool {
    match result {
        Ok(value) => value.as_bool() != Some(false),
        Err(_) => false,
    }
}

impl RawNarrowPhase {
    pub(crate) fn from_narrow_phase(narrow_phase: NarrowPhase) -> Self {
        RawNarrowPhase {
            narrow_phase,
            pair_buffers: Vec::new(),
        }
    }
}

impl Default for RawNarrowPhase {
    fn default() -> Self {
        Self::new()
    }
}

#[wasm_bindgen]
impl RawNarrowPhase {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Self::from_narrow_phase(NarrowPhase::new())
    }

    /// Calls `f` with the handle of every collider in contact with `handle1`.
    ///
    /// Like the broad-phase enumerations, a callback that returns `false` (which
    /// is also what the JS guard answers once the user's callback has thrown)
    /// or that fails ends the walk: the remaining pairs would only be handed to
    /// a callback that no longer wants them.
    pub fn contact_pairs_with(&self, handle1: FlatHandle, f: js_sys::Function) {
        let this = JsValue::null();
        let handle1 = utils::collider_handle(handle1);
        for pair in self.narrow_phase.contact_pairs_with(handle1) {
            let handle2 = if pair.collider1 == handle1 {
                utils::flat_handle(pair.collider2.0)
            } else {
                utils::flat_handle(pair.collider1.0)
            };

            if !continue_after(f.call1(&this, &JsValue::from(handle2))) {
                break;
            }
        }
    }

    /// Publishes every contact manifold between `handle1` and `handle2` into a
    /// WASM-resident buffer and returns its pointer and length packed into a
    /// single `f64` (see `utils::pack_buffer_info`). Two colliders without a
    /// contact pair publish a header with zero manifolds.
    ///
    /// Layout: `[flipped, numManifolds]` (both as raw `u32` bit patterns), then
    /// each manifold as `MANIFOLD_STRIDE` fixed slots followed by
    /// `numContacts * CONTACT_STRIDE` contact slots and
    /// `numSolverContacts * SOLVER_CONTACT_STRIDE` solver-contact slots, so a
    /// manifold's size is only known from its own header and JS walks the buffer
    /// sequentially. `flipped` is `1` when the pair stores `handle1` as its
    /// second collider, in which case every `1`/`2` field applies to the other
    /// collider than its name says. Vectors are written component-wise; the user
    /// data, subshape indices, feature ids and counts are `u32` bit patterns.
    ///
    /// Handing the pair and each manifold to JS as boxed raw pointers instead
    /// meant two WASM objects allocated and freed per pair, one boundary crossing
    /// per field read, and pointers that dangled as soon as the world stepped.
    /// The buffer is a snapshot, so it stays valid whatever JS does next, and the
    /// whole read is one crossing.
    ///
    /// `depth` selects which buffer to fill: a `contactPair` callback that calls
    /// `contactPair` again must not overwrite the manifolds the outer callback is
    /// still reading, so JS passes its nesting depth and each level gets its own
    /// buffer. The buffer is reused (and may be reallocated) by the next call at
    /// the same depth, so the returned pointer is only valid until then.
    pub fn contact_pair(
        &mut self,
        handle1: FlatHandle,
        handle2: FlatHandle,
        bodies: &RawRigidBodySet,
        depth: usize,
    ) -> f64 {
        let collider1 = utils::collider_handle(handle1);
        let collider2 = utils::collider_handle(handle2);

        // Deep nesting is a user bug rather than a real workload, and each level
        // costs a buffer that lives as long as the narrow-phase. Beyond this the
        // levels share one buffer, which stays memory-safe (JS reads a snapshot,
        // never a pointer) but may hand a nested walk's manifolds to the outer one.
        let depth = depth.min(MAX_PAIR_BUFFERS - 1);
        while self.pair_buffers.len() <= depth {
            self.pair_buffers.push(Vec::new());
        }
        let out = &mut self.pair_buffers[depth];
        out.clear();

        match self.narrow_phase.contact_pair(collider1, collider2) {
            Some(pair) => {
                push_u32(out, (pair.collider1 != collider1) as u32);
                push_u32(out, pair.manifolds.len() as u32);
                for manifold in &pair.manifolds {
                    push_manifold(out, manifold, bodies);
                }
            }
            None => {
                push_u32(out, 0);
                push_u32(out, 0);
            }
        }

        // Publish the whole allocation, not just the filled part. JS sizes its
        // walk from the header, so the published length only has to change when
        // the buffer grows, and the typed-array view over it survives from one
        // pair to the next instead of being rebuilt whenever the contact count
        // differs. The zero-fill covers the spare capacity only, a few hundred
        // bytes at most.
        let capacity = out.capacity();
        out.resize(capacity, 0.0);
        pack_buffer_info(out)
    }

    pub fn intersection_pairs_with(&self, handle1: FlatHandle, f: js_sys::Function) {
        let this = JsValue::null();
        let handle1 = utils::collider_handle(handle1);
        for (h1, h2, inter) in self.narrow_phase.intersection_pairs_with(handle1) {
            if inter {
                let handle2 = if h1 == handle1 {
                    utils::flat_handle(h2.0)
                } else {
                    utils::flat_handle(h1.0)
                };

                if !continue_after(f.call1(&this, &JsValue::from(handle2))) {
                    break;
                }
            }
        }
    }

    pub fn intersection_pair(&self, handle1: FlatHandle, handle2: FlatHandle) -> bool {
        let handle1 = utils::collider_handle(handle1);
        let handle2 = utils::collider_handle(handle2);
        self.narrow_phase.intersection_pair(handle1, handle2) == Some(true)
    }
}

/// How many nesting depths of `contact_pair` get their own buffer.
const MAX_PAIR_BUFFERS: usize = 8;
