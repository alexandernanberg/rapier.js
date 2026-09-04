use crate::math::RawVector;
use crate::scratch;
use crate::utils::{self, FlatHandle};
use rapier::geometry::{SolverContacts, SolverFlags};
use rapier::math::{Real, Vector};
use rapier::pipeline::{ContactModificationContext, PairFilterContext, PhysicsHooks};
use std::cell::Cell;
use wasm_bindgen::prelude::*;

pub struct RawPhysicsHooks {
    pub this: js_sys::Object,
    pub filter_contact_pair: Option<js_sys::Function>,
    pub filter_intersection_pair: Option<js_sys::Function>,
    pub modify_solver_contacts: Option<js_sys::Function>,
}

// NOTE: `RawPhysicsHooks` holds JS values, which are not `Send`/`Sync` (see
//       https://github.com/rustwasm/wasm-bindgen/pull/955). We used to paper over
//       that with `unsafe impl Send`/`Sync`, justified only by wasm being
//       single-threaded. Since rapier 0.35 the `unsync-callbacks` feature drops the
//       `Sync` bound from `PhysicsHooks` (via `utils::MaybeSync`), so the hooks are
//       accepted as-is and no unsafe assertion is needed.

/// The pointers into the [`ContactModificationContext`] that is currently being
/// handed to JS, or `None` outside of a `modify_solver_contacts` call.
///
/// The context borrows the narrow-phase's manifold, so it cannot outlive the hook
/// call. Rather than allocating a wasm-bindgen object per modified manifold (and
/// relying on JS to free it), the pointers live here for the duration of the call
/// and [`RawContactModificationContext`] — a handle JS constructs once and keeps —
/// reads them. Every accessor goes through [`with_context`], so a context object
/// that JS held on to past the hook reads nothing instead of a dangling pointer.
#[derive(Copy, Clone)]
struct ActiveContext {
    collider1: FlatHandle,
    collider2: FlatHandle,
    rigid_body1: Option<FlatHandle>,
    rigid_body2: Option<FlatHandle>,
    solver_contacts: *mut SolverContacts,
    normal: *mut Vector,
    friction: *mut Real,
    restitution: *mut Real,
    user_data: *mut u32,
}

impl ActiveContext {
    /// # Safety
    /// Only valid while the hook call this context was made current for is running.
    #[inline]
    unsafe fn contacts(&self) -> &SolverContacts {
        unsafe { &*self.solver_contacts }
    }

    /// # Safety
    /// Only valid while the hook call this context was made current for is running.
    #[inline]
    #[allow(clippy::mut_from_ref)]
    unsafe fn contacts_mut(&self) -> &mut SolverContacts {
        unsafe { &mut *self.solver_contacts }
    }
}

thread_local! {
    static ACTIVE_CONTEXT: Cell<Option<ActiveContext>> = const { Cell::new(None) };
}

/// Runs `f` on the context of the hook call currently in progress, or returns
/// `default` if called outside of a contact-modification hook.
#[inline]
fn with_context<T>(default: T, f: impl FnOnce(&ActiveContext) -> T) -> T {
    match ACTIVE_CONTEXT.get() {
        Some(context) => f(&context),
        None => default,
    }
}

impl PhysicsHooks for RawPhysicsHooks {
    fn filter_contact_pair(&self, ctxt: &PairFilterContext) -> Option<SolverFlags> {
        // A collider can carry the hook flag while the hooks object implements only
        // the other callbacks. Leaving the pair alone beats filtering it out.
        let Some(filter) = self.filter_contact_pair.as_ref() else {
            return Some(SolverFlags::default());
        };

        let rb1 = ctxt
            .rigid_body1
            .map(|rb| JsValue::from(utils::flat_handle(rb.0)))
            .unwrap_or(JsValue::NULL);
        let rb2 = ctxt
            .rigid_body2
            .map(|rb| JsValue::from(utils::flat_handle(rb.0)))
            .unwrap_or(JsValue::NULL);

        let collider1 = JsValue::from(utils::flat_handle(ctxt.collider1.0));
        let collider2 = JsValue::from(utils::flat_handle(ctxt.collider2.0));

        let result = filter
            .call4(&self.this, &collider1, &collider2, &rb1, &rb2)
            .ok()?;
        let flags = result.as_f64()?;
        // TODO: not sure exactly why we have to do `flags as u32` instead
        //       of `flags.to_bits() as u32`.
        Some(SolverFlags::from_bits_truncate(flags as u32))
    }

    fn filter_intersection_pair(&self, ctxt: &PairFilterContext) -> bool {
        let Some(filter) = self.filter_intersection_pair.as_ref() else {
            return true;
        };

        let rb1 = ctxt
            .rigid_body1
            .map(|rb| JsValue::from(utils::flat_handle(rb.0)))
            .unwrap_or(JsValue::NULL);
        let rb2 = ctxt
            .rigid_body2
            .map(|rb| JsValue::from(utils::flat_handle(rb.0)))
            .unwrap_or(JsValue::NULL);

        let collider1 = JsValue::from(utils::flat_handle(ctxt.collider1.0));
        let collider2 = JsValue::from(utils::flat_handle(ctxt.collider2.0));

        filter
            .call4(&self.this, &collider1, &collider2, &rb1, &rb2)
            .ok()
            .and_then(|res| res.as_bool())
            .unwrap_or(false)
    }

    fn modify_solver_contacts(&self, ctxt: &mut ContactModificationContext) {
        let Some(modify) = self.modify_solver_contacts.as_ref() else {
            return;
        };

        let context = ActiveContext {
            collider1: utils::flat_handle(ctxt.collider1.0),
            collider2: utils::flat_handle(ctxt.collider2.0),
            rigid_body1: ctxt.rigid_body1.map(|rb| utils::flat_handle(rb.0)),
            rigid_body2: ctxt.rigid_body2.map(|rb| utils::flat_handle(rb.0)),
            solver_contacts: &mut *ctxt.solver_contacts,
            normal: &mut *ctxt.normal,
            friction: &mut *ctxt.friction,
            restitution: &mut *ctxt.restitution,
            user_data: &mut *ctxt.user_data,
        };

        // Restore rather than clear: the hook can step no world of its own, but
        // keeping this a save/restore means a nested call could never truncate the
        // outer context.
        let previous = ACTIVE_CONTEXT.replace(Some(context));
        let _ = modify.call0(&self.this);
        ACTIVE_CONTEXT.set(previous);
    }
}

/// A handle to the contact-modification context of the hook call in progress.
///
/// JS constructs one of these once and reuses it: all the accessors read the
/// context that [`RawPhysicsHooks::modify_solver_contacts`] made current, so the
/// hook needs no per-call allocation. Outside of a hook call every getter reads
/// zero and every setter is a no-op.
#[wasm_bindgen]
pub struct RawContactModificationContext;

#[wasm_bindgen]
impl RawContactModificationContext {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        RawContactModificationContext
    }

    /// Is a contact-modification hook call currently in progress?
    pub fn isActive(&self) -> bool {
        ACTIVE_CONTEXT.get().is_some()
    }

    pub fn collider1(&self) -> Option<FlatHandle> {
        with_context(None, |c| Some(c.collider1))
    }

    pub fn collider2(&self) -> Option<FlatHandle> {
        with_context(None, |c| Some(c.collider2))
    }

    pub fn rigidBody1(&self) -> Option<FlatHandle> {
        with_context(None, |c| c.rigid_body1)
    }

    pub fn rigidBody2(&self) -> Option<FlatHandle> {
        with_context(None, |c| c.rigid_body2)
    }

    /// Writes the contact normal into the scratch buffer.
    pub fn normal(&self) -> bool {
        with_context(false, |c| {
            scratch::write_vector(unsafe { *c.normal });
            true
        })
    }

    /// Sets the contact normal. It is expected to be a unit vector pointing from
    /// the first collider towards the second one.
    pub fn setNormal(&self, normal: &RawVector) {
        with_context((), |c| unsafe {
            *c.normal = normal.0;
        })
    }

    /// The friction coefficient applied to every contact of this manifold.
    pub fn friction(&self) -> Real {
        with_context(0.0, |c| unsafe { *c.friction })
    }

    pub fn setFriction(&self, friction: Real) {
        with_context((), |c| unsafe {
            *c.friction = friction;
        })
    }

    /// The restitution coefficient applied to every contact of this manifold.
    pub fn restitution(&self) -> Real {
        with_context(0.0, |c| unsafe { *c.restitution })
    }

    pub fn setRestitution(&self, restitution: Real) {
        with_context((), |c| unsafe {
            *c.restitution = restitution;
        })
    }

    /// The user-defined data attached to the manifold. It is preserved across steps.
    pub fn userData(&self) -> u32 {
        with_context(0, |c| unsafe { *c.user_data })
    }

    pub fn setUserData(&self, userData: u32) {
        with_context((), |c| unsafe {
            *c.user_data = userData;
        })
    }

    pub fn numSolverContacts(&self) -> usize {
        with_context(0, |c| unsafe { c.contacts().len() })
    }

    /// Removes the `i`-th solver contact, so the solver ignores it entirely.
    ///
    /// This swaps the last contact into `i`, so contact indices shift: iterate
    /// backwards when removing more than one.
    pub fn removeSolverContact(&self, i: usize) {
        with_context((), |c| unsafe {
            let contacts = c.contacts_mut();
            if i < contacts.len() {
                contacts.swap_remove(i);
            }
        })
    }

    /// Removes every solver contact, disabling the contact response for this
    /// manifold while still reporting the collision.
    pub fn clearSolverContacts(&self) {
        with_context((), |c| unsafe { c.contacts_mut().clear() })
    }

    /// Writes the world-space contact point on the first collider into the scratch buffer.
    pub fn solverContactPoint1(&self, i: usize) -> bool {
        with_context(false, |c| unsafe {
            c.contacts().get(i).is_some_and(|contact| {
                scratch::write_vector(contact.anchor1);
                true
            })
        })
    }

    /// Writes the world-space contact point on the second collider into the scratch buffer.
    pub fn solverContactPoint2(&self, i: usize) -> bool {
        with_context(false, |c| unsafe {
            c.contacts().get(i).is_some_and(|contact| {
                scratch::write_vector(contact.anchor2);
                true
            })
        })
    }

    pub fn setSolverContactPoint1(&self, i: usize, point: &RawVector) {
        with_context((), |c| unsafe {
            if let Some(contact) = c.contacts_mut().get_mut(i) {
                contact.anchor1 = point.0;
            }
        })
    }

    pub fn setSolverContactPoint2(&self, i: usize, point: &RawVector) {
        with_context((), |c| unsafe {
            if let Some(contact) = c.contacts_mut().get_mut(i) {
                contact.anchor2 = point.0;
            }
        })
    }

    /// The separation of the `i`-th solver contact: negative means penetration.
    pub fn solverContactDist(&self, i: usize) -> Real {
        with_context(0.0, |c| unsafe {
            c.contacts().get(i).map_or(0.0, |contact| contact.dist)
        })
    }

    pub fn setSolverContactDist(&self, i: usize, dist: Real) {
        with_context((), |c| unsafe {
            if let Some(contact) = c.contacts_mut().get_mut(i) {
                contact.dist = dist;
            }
        })
    }

    /// Writes the tangent (surface) velocity of the `i`-th solver contact into the
    /// scratch buffer.
    pub fn solverContactTangentVelocity(&self, i: usize) -> bool {
        with_context(false, |c| unsafe {
            c.contacts().get(i).is_some_and(|contact| {
                scratch::write_vector(contact.tangent_velocity);
                true
            })
        })
    }

    /// Sets the tangent (surface) velocity of the `i`-th solver contact, which is
    /// what makes a collider behave like a conveyor belt.
    pub fn setSolverContactTangentVelocity(&self, i: usize, velocity: &RawVector) {
        with_context((), |c| unsafe {
            if let Some(contact) = c.contacts_mut().get_mut(i) {
                contact.tangent_velocity = velocity.0;
            }
        })
    }
}

impl Default for RawContactModificationContext {
    fn default() -> Self {
        Self::new()
    }
}
