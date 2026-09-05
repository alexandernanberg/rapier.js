use crate::dynamics::{RawGenericJoint, RawRigidBodySet};
use crate::utils::{self, FlatHandle};
use rapier::dynamics::{MultibodyJoint, MultibodyJointSet};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct RawMultibodyJointSet(pub(crate) MultibodyJointSet);

impl RawMultibodyJointSet {
    pub(crate) fn map<T>(&self, handle: FlatHandle, f: impl FnOnce(&MultibodyJoint) -> T) -> T {
        let (body, link_id) = self
            .0
            .get(utils::multibody_joint_handle(handle))
            .expect("Invalid Joint reference. It may have been removed from the physics World.");
        f(body.link(link_id).unwrap().joint())
    }

    pub(crate) fn map_mut<T>(
        &mut self,
        handle: FlatHandle,
        f: impl FnOnce(&mut MultibodyJoint) -> T,
    ) -> T {
        let (body, link_id) = self
            .0
            .get_mut(utils::multibody_joint_handle(handle))
            .expect("Invalid Joint reference. It may have been removed from the physics World.");
        f(&mut body.link_mut(link_id).unwrap().joint)
    }
}

#[wasm_bindgen]
impl RawMultibodyJointSet {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        RawMultibodyJointSet(MultibodyJointSet::new())
    }

    /// Inserts a multibody joint, or returns `None` if it would leave the multibody
    /// in an invalid configuration: `parent2` already has a parent joint, or both
    /// bodies already belong to the same multibody (which would close a loop).
    ///
    /// A failed insert used to come back as `FlatHandle::MAX`, which JS took for a
    /// real handle — the very next accessor looked it up and hit the "Invalid Joint
    /// reference" `expect` in [`Self::map`], i.e. a WASM trap for what is an
    /// ordinary rejected-topology error.
    pub fn createJoint(
        &mut self,
        bodies: &RawRigidBodySet,
        params: &RawGenericJoint,
        parent1: FlatHandle,
        parent2: FlatHandle,
        wakeUp: bool,
    ) -> Option<FlatHandle> {
        let parent1 = utils::body_handle(parent1);
        let parent2 = utils::body_handle(parent2);
        // Same as the impulse joint set: a joint attached to a body that is no
        // longer there would trap the module from inside the next `step()`.
        if !bodies.bodies.contains(parent1) || !bodies.bodies.contains(parent2) {
            return None;
        }

        self.0
            .insert(parent1, parent2, params.0.clone(), wakeUp)
            .map(|h| utils::flat_handle(h.0))
    }

    pub fn remove(&mut self, handle: FlatHandle, wakeUp: bool) {
        let handle = utils::multibody_joint_handle(handle);
        self.0.remove(handle, wakeUp);
    }

    pub fn contains(&self, handle: FlatHandle) -> bool {
        self.0.get(utils::multibody_joint_handle(handle)).is_some()
    }

    /// Applies the given JavaScript function to the integer handle of each joint managed by this physics world.
    ///
    /// # Parameters
    /// - `f(handle)`: the function to apply to the integer handle of each joint managed by this set. Called as `f(collider)`.
    pub fn forEachJointHandle(&self, f: &js_sys::Function) {
        let this = JsValue::null();
        for (handle, _, _, _) in self.0.iter() {
            let _ = f.call1(&this, &JsValue::from(utils::flat_handle(handle.0)));
        }
    }

    /// Applies the given JavaScript function to the integer handle of each joint attached to the given rigid-body.
    ///
    /// # Parameters
    /// - `f(handle)`: the function to apply to the integer handle of each joint attached to the rigid-body. Called as `f(collider)`.
    pub fn forEachJointAttachedToRigidBody(&self, body: FlatHandle, f: &js_sys::Function) {
        let this = JsValue::null();
        for (_, _, handle) in self.0.attached_joints(utils::body_handle(body)) {
            let _ = f.call1(&this, &JsValue::from(utils::flat_handle(handle.0)));
        }
    }
}
