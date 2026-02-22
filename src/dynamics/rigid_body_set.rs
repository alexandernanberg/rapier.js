use crate::dynamics::{RawImpulseJointSet, RawIslandManager, RawMultibodyJointSet};
use crate::geometry::RawColliderSet;
use crate::math::{RawRotation, RawVector};
use crate::utils::{self, FlatHandle};
use rapier::dynamics::{MassProperties, RigidBody, RigidBodyBuilder, RigidBodySet, RigidBodyType};
use rapier::math::Pose;
use wasm_bindgen::prelude::*;

/// Number of f32 values per body in the transform buffer.
#[cfg(feature = "dim3")]
const BODY_STRIDE: usize = 13; // translation(3) + rotation(4) + linvel(3) + angvel(3)
#[cfg(feature = "dim2")]
const BODY_STRIDE: usize = 6; // translation(2) + rotation(1) + linvel(2) + angvel(1)

#[wasm_bindgen]
pub enum RawRigidBodyType {
    Dynamic,
    Fixed,
    KinematicPositionBased,
    KinematicVelocityBased,
}

impl Into<RigidBodyType> for RawRigidBodyType {
    fn into(self) -> RigidBodyType {
        match self {
            RawRigidBodyType::Dynamic => RigidBodyType::Dynamic,
            RawRigidBodyType::Fixed => RigidBodyType::Fixed,
            RawRigidBodyType::KinematicPositionBased => RigidBodyType::KinematicPositionBased,
            RawRigidBodyType::KinematicVelocityBased => RigidBodyType::KinematicVelocityBased,
        }
    }
}

impl Into<RawRigidBodyType> for RigidBodyType {
    fn into(self) -> RawRigidBodyType {
        match self {
            RigidBodyType::Dynamic => RawRigidBodyType::Dynamic,
            RigidBodyType::Fixed => RawRigidBodyType::Fixed,
            RigidBodyType::KinematicPositionBased => RawRigidBodyType::KinematicPositionBased,
            RigidBodyType::KinematicVelocityBased => RawRigidBodyType::KinematicVelocityBased,
        }
    }
}

#[wasm_bindgen]
pub struct RawRigidBodySet {
    pub(crate) bodies: RigidBodySet,
    pub(crate) transform_data: Vec<f32>,
}

impl RawRigidBodySet {
    pub(crate) fn map<T>(&self, handle: FlatHandle, f: impl FnOnce(&RigidBody) -> T) -> T {
        let body = self.bodies.get(utils::body_handle(handle)).expect(
            "Invalid RigidBody reference. It may have been removed from the physics World.",
        );
        f(body)
    }

    pub(crate) fn map_mut<T>(
        &mut self,
        handle: FlatHandle,
        f: impl FnOnce(&mut RigidBody) -> T,
    ) -> T {
        let body = self.bodies.get_mut(utils::body_handle(handle)).expect(
            "Invalid RigidBody reference. It may have been removed from the physics World.",
        );
        f(body)
    }

    /// Syncs all rigid body transforms into the contiguous buffer.
    ///
    /// Called internally from the physics pipeline step for cache locality.
    /// Not exposed via wasm-bindgen to avoid borrow tracking issues.
    pub(crate) fn sync_transform_data(&mut self) {
        let mut max_index: usize = 0;
        for (handle, _) in self.bodies.iter() {
            let (index, _) = handle.0.into_raw_parts();
            max_index = max_index.max(index as usize);
        }

        let required_len = if self.bodies.len() > 0 {
            (max_index + 1) * BODY_STRIDE
        } else {
            0
        };

        if self.transform_data.len() < required_len {
            self.transform_data.resize(required_len, 0.0);
        }

        for (handle, body) in self.bodies.iter() {
            let (index, _) = handle.0.into_raw_parts();
            let offset = index as usize * BODY_STRIDE;

            let pos = body.position();
            let lv = body.linvel();

            #[cfg(feature = "dim3")]
            {
                let t = pos.translation;
                let r = pos.rotation;
                let av = body.angvel();
                self.transform_data[offset] = t.x;
                self.transform_data[offset + 1] = t.y;
                self.transform_data[offset + 2] = t.z;
                self.transform_data[offset + 3] = r.x;
                self.transform_data[offset + 4] = r.y;
                self.transform_data[offset + 5] = r.z;
                self.transform_data[offset + 6] = r.w;
                self.transform_data[offset + 7] = lv.x;
                self.transform_data[offset + 8] = lv.y;
                self.transform_data[offset + 9] = lv.z;
                self.transform_data[offset + 10] = av.x;
                self.transform_data[offset + 11] = av.y;
                self.transform_data[offset + 12] = av.z;
            }

            #[cfg(feature = "dim2")]
            {
                let t = pos.translation;
                let r = pos.rotation.angle();
                let av = body.angvel();
                self.transform_data[offset] = t.x;
                self.transform_data[offset + 1] = t.y;
                self.transform_data[offset + 2] = r;
                self.transform_data[offset + 3] = lv.x;
                self.transform_data[offset + 4] = lv.y;
                self.transform_data[offset + 5] = av;
            }
        }
    }
}

#[wasm_bindgen]
impl RawRigidBodySet {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        RawRigidBodySet {
            bodies: RigidBodySet::new(),
            transform_data: Vec::new(),
        }
    }

    /// Returns the transform buffer pointer and length packed into a single f64.
    /// Low 32 bits = byte offset in WASM memory, high 32 bits = f32 element count.
    pub fn transformBufferInfo(&self) -> f64 {
        let ptr = self.transform_data.as_ptr() as u32;
        let len = self.transform_data.len() as u32;
        f64::from_bits(ptr as u64 | ((len as u64) << 32))
    }

    /// Returns the number of floats per body in the buffer.
    pub fn transformBufferStride(&self) -> usize {
        BODY_STRIDE
    }

    #[cfg(feature = "dim3")]
    pub fn createRigidBody(
        &mut self,
        enabled: bool,
        translation: &RawVector,
        rotation: &RawRotation,
        gravityScale: f32,
        mass: f32,
        massOnly: bool,
        centerOfMass: &RawVector,
        linvel: &RawVector,
        angvel: &RawVector,
        principalAngularInertia: &RawVector,
        angularInertiaFrame: &RawRotation,
        translationEnabledX: bool,
        translationEnabledY: bool,
        translationEnabledZ: bool,
        rotationEnabledX: bool,
        rotationEnabledY: bool,
        rotationEnabledZ: bool,
        linearDamping: f32,
        angularDamping: f32,
        rb_type: RawRigidBodyType,
        canSleep: bool,
        sleeping: bool,
        softCcdPrediction: f32,
        ccdEnabled: bool,
        dominanceGroup: i8,
        additional_solver_iterations: usize,
    ) -> FlatHandle {
        let pos = Pose::from_parts(translation.0, rotation.0);

        let mut rigid_body = RigidBodyBuilder::new(rb_type.into())
            .enabled(enabled)
            .pose(pos)
            .gravity_scale(gravityScale)
            .enabled_translations(
                translationEnabledX,
                translationEnabledY,
                translationEnabledZ,
            )
            .enabled_rotations(rotationEnabledX, rotationEnabledY, rotationEnabledZ)
            .linvel(linvel.0)
            .angvel(angvel.0)
            .linear_damping(linearDamping)
            .angular_damping(angularDamping)
            .can_sleep(canSleep)
            .sleeping(sleeping)
            .ccd_enabled(ccdEnabled)
            .dominance_group(dominanceGroup)
            .additional_solver_iterations(additional_solver_iterations)
            .soft_ccd_prediction(softCcdPrediction);

        rigid_body = if massOnly {
            rigid_body.additional_mass(mass)
        } else {
            let props = MassProperties::with_principal_inertia_frame(
                centerOfMass.0.into(),
                mass,
                principalAngularInertia.0,
                angularInertiaFrame.0,
            );
            rigid_body.additional_mass_properties(props)
        };

        utils::flat_handle(self.bodies.insert(rigid_body.build()).0)
    }

    #[cfg(feature = "dim2")]
    pub fn createRigidBody(
        &mut self,
        enabled: bool,
        translation: &RawVector,
        rotation: &RawRotation,
        gravityScale: f32,
        mass: f32,
        massOnly: bool,
        centerOfMass: &RawVector,
        linvel: &RawVector,
        angvel: f32,
        principalAngularInertia: f32,
        translationEnabledX: bool,
        translationEnabledY: bool,
        rotationsEnabled: bool,
        linearDamping: f32,
        angularDamping: f32,
        rb_type: RawRigidBodyType,
        canSleep: bool,
        sleeping: bool,
        softCcdPrediciton: f32,
        ccdEnabled: bool,
        dominanceGroup: i8,
        additional_solver_iterations: usize,
    ) -> FlatHandle {
        let pos = Pose::from_parts(translation.0, rotation.0);
        let mut rigid_body = RigidBodyBuilder::new(rb_type.into())
            .enabled(enabled)
            .pose(pos)
            .gravity_scale(gravityScale)
            .enabled_translations(translationEnabledX, translationEnabledY)
            .linvel(linvel.0)
            .angvel(angvel)
            .linear_damping(linearDamping)
            .angular_damping(angularDamping)
            .can_sleep(canSleep)
            .sleeping(sleeping)
            .ccd_enabled(ccdEnabled)
            .dominance_group(dominanceGroup)
            .additional_solver_iterations(additional_solver_iterations)
            .soft_ccd_prediction(softCcdPrediciton);

        rigid_body = if massOnly {
            rigid_body.additional_mass(mass)
        } else {
            let props = MassProperties::new(centerOfMass.0.into(), mass, principalAngularInertia);
            rigid_body.additional_mass_properties(props)
        };

        if !rotationsEnabled {
            rigid_body = rigid_body.lock_rotations();
        }

        utils::flat_handle(self.bodies.insert(rigid_body.build()).0)
    }

    pub fn remove(
        &mut self,
        handle: FlatHandle,
        islands: &mut RawIslandManager,
        colliders: &mut RawColliderSet,
        joints: &mut RawImpulseJointSet,
        articulations: &mut RawMultibodyJointSet,
    ) {
        let handle = utils::body_handle(handle);
        self.bodies.remove(
            handle,
            &mut islands.0,
            &mut colliders.0,
            &mut joints.0,
            &mut articulations.0,
            true,
        );
    }

    /// The number of rigid-bodies on this set.
    pub fn len(&self) -> usize {
        self.bodies.len()
    }

    /// Checks if a rigid-body with the given integer handle exists.
    pub fn contains(&self, handle: FlatHandle) -> bool {
        self.bodies.get(utils::body_handle(handle)).is_some()
    }

    /// Applies the given JavaScript function to the integer handle of each rigid-body managed by this set.
    ///
    /// # Parameters
    /// - `f(handle)`: the function to apply to the integer handle of each rigid-body managed by this set. Called as `f(collider)`.
    pub fn forEachRigidBodyHandle(&self, f: &js_sys::Function) {
        let this = JsValue::null();
        for (handle, _) in self.bodies.iter() {
            let _ = f.call1(&this, &JsValue::from(utils::flat_handle(handle.0)));
        }
    }

    pub fn propagateModifiedBodyPositionsToColliders(&mut self, colliders: &mut RawColliderSet) {
        self.bodies
            .propagate_modified_body_positions_to_colliders(&mut colliders.0);
    }
}
