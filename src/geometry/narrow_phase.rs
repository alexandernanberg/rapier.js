use crate::dynamics::RawRigidBodySet;
use crate::scratch;
use crate::utils::{self, FlatHandle};
use rapier::geometry::{ContactManifold, ContactPair, NarrowPhase};
use rapier::math::Real;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct RawNarrowPhase(pub(crate) NarrowPhase);

#[wasm_bindgen]
impl RawNarrowPhase {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        RawNarrowPhase(NarrowPhase::new())
    }

    pub fn contact_pairs_with(&self, handle1: FlatHandle, f: js_sys::Function) {
        let this = JsValue::null();
        let handle1 = utils::collider_handle(handle1);
        for pair in self.0.contact_pairs_with(handle1) {
            let handle2 = if pair.collider1 == handle1 {
                utils::flat_handle(pair.collider2.0)
            } else {
                utils::flat_handle(pair.collider1.0)
            };

            let _ = f.call1(&this, &JsValue::from(handle2));
        }
    }

    pub fn contact_pair(&self, handle1: FlatHandle, handle2: FlatHandle) -> Option<RawContactPair> {
        let handle1 = utils::collider_handle(handle1);
        let handle2 = utils::collider_handle(handle2);
        self.0
            .contact_pair(handle1, handle2)
            .map(|p| RawContactPair(p as *const ContactPair))
    }

    pub fn intersection_pairs_with(&self, handle1: FlatHandle, f: js_sys::Function) {
        let this = JsValue::null();
        let handle1 = utils::collider_handle(handle1);
        for (h1, h2, inter) in self.0.intersection_pairs_with(handle1) {
            if inter {
                let handle2 = if h1 == handle1 {
                    utils::flat_handle(h2.0)
                } else {
                    utils::flat_handle(h1.0)
                };

                let _ = f.call1(&this, &JsValue::from(handle2));
            }
        }
    }

    pub fn intersection_pair(&self, handle1: FlatHandle, handle2: FlatHandle) -> bool {
        let handle1 = utils::collider_handle(handle1);
        let handle2 = utils::collider_handle(handle2);
        self.0.intersection_pair(handle1, handle2) == Some(true)
    }
}

#[wasm_bindgen]
pub struct RawContactPair(*const ContactPair);
#[wasm_bindgen]
pub struct RawContactManifold(*const ContactManifold);

// SAFETY: the use of a raw pointer is very unsafe.
//         We need this because wasm-bindgen doesn't support
//         lifetimes. So for the moment, we have to make sure
//         that our TypeScript wrapper properly free the pair
//         before the user has a chance to invalidate this pointer.
#[wasm_bindgen]
impl RawContactPair {
    pub fn collider1(&self) -> FlatHandle {
        unsafe { utils::flat_handle((*self.0).collider1.0) }
    }

    pub fn collider2(&self) -> FlatHandle {
        unsafe { utils::flat_handle((*self.0).collider2.0) }
    }

    pub fn numContactManifolds(&self) -> usize {
        unsafe { (*self.0).manifolds.len() }
    }
    pub fn contactManifold(&self, i: usize) -> Option<RawContactManifold> {
        unsafe {
            (&(*self.0).manifolds)
                .get(i)
                .map(|m| RawContactManifold(m as *const ContactManifold))
        }
    }
}

#[wasm_bindgen]
impl RawContactManifold {
    pub fn normal(&self) {
        unsafe { scratch::write_vector((*self.0).data.normal) }
    }

    /// The user-defined data attached to this manifold, as set from a
    /// contact-modification hook. Preserved across steps.
    pub fn user_data(&self) -> u32 {
        unsafe { (*self.0).data.user_data }
    }

    pub fn local_n1(&self) {
        unsafe { scratch::write_vector((*self.0).local_n1) }
    }

    pub fn local_n2(&self) {
        unsafe { scratch::write_vector((*self.0).local_n2) }
    }

    pub fn subshape1(&self) -> u32 {
        unsafe { (*self.0).subshape1 }
    }

    pub fn subshape2(&self) -> u32 {
        unsafe { (*self.0).subshape2 }
    }

    pub fn num_contacts(&self) -> usize {
        unsafe { (*self.0).points.len() }
    }

    pub fn contact_local_p1(&self, i: usize) -> bool {
        unsafe {
            (&(*self.0).points).get(i).is_some_and(|c| {
                scratch::write_vector(c.local_p1);
                true
            })
        }
    }

    pub fn contact_local_p2(&self, i: usize) -> bool {
        unsafe {
            (&(*self.0).points).get(i).is_some_and(|c| {
                scratch::write_vector(c.local_p2);
                true
            })
        }
    }

    pub fn contact_dist(&self, i: usize) -> Real {
        unsafe { (&(*self.0).points).get(i).map(|c| c.dist).unwrap_or(0.0) }
    }

    pub fn contact_fid1(&self, i: usize) -> u32 {
        unsafe { (&(*self.0).points).get(i).map(|c| c.fid1.0).unwrap_or(0) }
    }

    pub fn contact_fid2(&self, i: usize) -> u32 {
        unsafe { (&(*self.0).points).get(i).map(|c| c.fid2.0).unwrap_or(0) }
    }

    pub fn contact_impulse(&self, i: usize) -> Real {
        unsafe {
            (&(*self.0).points)
                .get(i)
                .map(|c| c.data.impulse)
                .unwrap_or(0.0)
        }
    }

    #[cfg(feature = "dim2")]
    pub fn contact_tangent_impulse(&self, i: usize) -> Real {
        unsafe {
            (&(*self.0).points)
                .get(i)
                .map(|c| c.data.tangent_impulse.x)
                .unwrap_or(0.0)
        }
    }

    #[cfg(feature = "dim3")]
    pub fn contact_tangent_impulse_x(&self, i: usize) -> Real {
        unsafe {
            (&(*self.0).points)
                .get(i)
                .map(|c| c.data.tangent_impulse.x)
                .unwrap_or(0.0)
        }
    }

    #[cfg(feature = "dim3")]
    pub fn contact_tangent_impulse_y(&self, i: usize) -> Real {
        unsafe {
            (&(*self.0).points)
                .get(i)
                .map(|c| c.data.tangent_impulse.y)
                .unwrap_or(0.0)
        }
    }

    pub fn num_solver_contacts(&self) -> usize {
        unsafe { (*self.0).data.solver_contacts.len() }
    }

    /// The contact point on the first body's surface.
    ///
    /// Since rapier 0.35 this is expressed in that body's center-of-mass-centered local
    /// frame, or in world-space when the first side has no solver body (no rigid-body, or
    /// world-attached by dominance — fixed bodies included).
    pub fn solver_contact_anchor1(&self, i: usize) -> bool {
        unsafe {
            (&(*self.0).data).solver_contacts.get(i).is_some_and(|c| {
                scratch::write_vector(c.anchor1);
                true
            })
        }
    }

    /// The contact point on the second body's surface, expressed like
    /// [`Self::solver_contact_anchor1`].
    pub fn solver_contact_anchor2(&self, i: usize) -> bool {
        unsafe {
            (&(*self.0).data).solver_contacts.get(i).is_some_and(|c| {
                scratch::write_vector(c.anchor2);
                true
            })
        }
    }

    /// The world-space contact point the solver acts on, midway between both surfaces.
    ///
    /// Solver contacts store one body-local anchor per surface (the two differ by the
    /// current separation along the normal), so resolving them back to world-space needs
    /// the bodies they are anchored to.
    pub fn solver_contact_point(&self, bodies: &RawRigidBodySet, i: usize) -> bool {
        unsafe {
            let data = &(*self.0).data;
            data.solver_contacts.get(i).is_some_and(|c| {
                let (p1, p2) = data.solver_contact_world_points(c, &bodies.bodies);
                scratch::write_vector((p1 + p2) / 2.0);
                true
            })
        }
    }

    pub fn solver_contact_dist(&self, i: usize) -> Real {
        unsafe {
            (&(*self.0).data)
                .solver_contacts
                .get(i)
                .map(|c| c.dist)
                .unwrap_or(0.0)
        }
    }

    /// The effective friction coefficient of this manifold's contacts.
    ///
    /// Since rapier 0.35 friction is stored per-manifold instead of per solver-contact,
    /// so it is identical for every contact of this manifold.
    pub fn friction(&self) -> Real {
        unsafe { (*self.0).data.friction }
    }

    /// The effective restitution coefficient of this manifold's contacts.
    ///
    /// Since rapier 0.35 restitution is stored per-manifold instead of per solver-contact,
    /// so it is identical for every contact of this manifold.
    pub fn restitution(&self) -> Real {
        unsafe { (*self.0).data.restitution }
    }

    pub fn solver_contact_tangent_velocity(&self, i: usize) -> bool {
        unsafe {
            (&(*self.0).data).solver_contacts.get(i).is_some_and(|c| {
                scratch::write_vector(c.tangent_velocity);
                true
            })
        }
    }
}
