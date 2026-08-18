use crate::dynamics::{RawImpulseJointSet, RawJointAxis, RawJointType, RawMotorModel};
use crate::math::{RawRotation, RawVector};
use crate::utils::{self, FlatHandle};
use rapier::dynamics::JointAxis;
use rapier::math::Pose;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
impl RawImpulseJointSet {
    /// The type of this joint.
    pub fn jointType(&self, handle: FlatHandle) -> RawJointType {
        self.map(handle, |j| j.data.locked_axes.into())
    }

    /// The unique integer identifier of the first rigid-body this joint it attached to.
    pub fn jointBodyHandle1(&self, handle: FlatHandle) -> FlatHandle {
        self.map(handle, |j| utils::flat_handle(j.body1().0))
    }

    /// The unique integer identifier of the second rigid-body this joint is attached to.
    pub fn jointBodyHandle2(&self, handle: FlatHandle) -> FlatHandle {
        self.map(handle, |j| utils::flat_handle(j.body2().0))
    }

    /// The angular part of the joint’s local frame relative to the first rigid-body it is attached to.
    pub fn jointFrameX1(&self, handle: FlatHandle) -> RawRotation {
        self.map(handle, |j| j.data.local_frame1.rotation.into())
    }

    /// The angular part of the joint’s local frame relative to the second rigid-body it is attached to.
    pub fn jointFrameX2(&self, handle: FlatHandle) -> RawRotation {
        self.map(handle, |j| j.data.local_frame2.rotation.into())
    }

    /// The position of the first anchor of this joint.
    ///
    /// The first anchor gives the position of the points application point on the
    /// local frame of the first rigid-body it is attached to.
    pub fn jointAnchor1(&self, handle: FlatHandle) -> RawVector {
        self.map(handle, |j| j.data.local_frame1.translation.into())
    }

    /// The position of the second anchor of this joint.
    ///
    /// The second anchor gives the position of the points application point on the
    /// local frame of the second rigid-body it is attached to.
    pub fn jointAnchor2(&self, handle: FlatHandle) -> RawVector {
        self.map(handle, |j| j.data.local_frame2.translation.into())
    }

    /// Sets the position of the first local anchor
    pub fn jointSetAnchor1(&mut self, handle: FlatHandle, newPos: &RawVector) {
        self.map_mut(handle, |j| {
            j.data.set_local_anchor1(newPos.0.into());
        });
    }

    /// Sets the position of the second local anchor
    pub fn jointSetAnchor2(&mut self, handle: FlatHandle, newPos: &RawVector) {
        self.map_mut(handle, |j| {
            j.data.set_local_anchor2(newPos.0.into());
        })
    }

    /// Sets the angular part of the joint's local frame relative to the first rigid-body.
    pub fn jointSetFrameX1(&mut self, handle: FlatHandle, newRot: &RawRotation) {
        self.map_mut(handle, |j| {
            j.data.local_frame1.rotation = newRot.0;
        });
    }

    /// Sets the angular part of the joint's local frame relative to the second rigid-body.
    pub fn jointSetFrameX2(&mut self, handle: FlatHandle, newRot: &RawRotation) {
        self.map_mut(handle, |j| {
            j.data.local_frame2.rotation = newRot.0;
        });
    }

    /// Sets the full local frame (anchor + rotation) for the first rigid-body attachment.
    pub fn jointSetLocalFrame1(
        &mut self,
        handle: FlatHandle,
        anchor: &RawVector,
        rot: &RawRotation,
    ) {
        self.map_mut(handle, |j| {
            j.data.set_local_frame1(Pose::from_parts(anchor.0, rot.0));
        });
    }

    /// Sets the full local frame (anchor + rotation) for the second rigid-body attachment.
    pub fn jointSetLocalFrame2(
        &mut self,
        handle: FlatHandle,
        anchor: &RawVector,
        rot: &RawRotation,
    ) {
        self.map_mut(handle, |j| {
            j.data.set_local_frame2(Pose::from_parts(anchor.0, rot.0));
        });
    }

    /// Are contacts between the rigid-bodies attached by this joint enabled?
    pub fn jointContactsEnabled(&self, handle: FlatHandle) -> bool {
        self.map(handle, |j| j.data.contacts_enabled)
    }

    /// Sets whether contacts are enabled between the rigid-bodies attached by this joint.
    pub fn jointSetContactsEnabled(&mut self, handle: FlatHandle, enabled: bool) {
        self.map_mut(handle, |j| {
            j.data.contacts_enabled = enabled;
        });
    }

    /// Are the limits for this joint enabled?
    pub fn jointLimitsEnabled(&self, handle: FlatHandle, axis: RawJointAxis) -> bool {
        self.map(handle, |j| {
            j.data.limit_axes.contains(JointAxis::from(axis).into())
        })
    }

    /// Return the lower limit along the given joint axis.
    pub fn jointLimitsMin(&self, handle: FlatHandle, axis: RawJointAxis) -> f32 {
        self.map(handle, |j| j.data.limits[axis as usize].min)
    }

    /// If this is a prismatic joint, returns its upper limit.
    pub fn jointLimitsMax(&self, handle: FlatHandle, axis: RawJointAxis) -> f32 {
        self.map(handle, |j| j.data.limits[axis as usize].max)
    }

    /// Enables and sets the joint limits
    pub fn jointSetLimits(&mut self, handle: FlatHandle, axis: RawJointAxis, min: f32, max: f32) {
        self.map_mut(handle, |j| {
            j.data.set_limits(axis.into(), [min, max]);
        });
    }

    pub fn jointConfigureMotorModel(
        &mut self,
        handle: FlatHandle,
        axis: RawJointAxis,
        model: RawMotorModel,
    ) {
        self.map_mut(handle, |j| {
            j.data.motors[axis as usize].model = model.into()
        })
    }

    /// Sets the maximum force (or torque, for angular axes) the motor of the
    /// given axis can deliver.
    pub fn jointSetMotorMaxForce(&mut self, handle: FlatHandle, axis: RawJointAxis, maxForce: f32) {
        self.map_mut(handle, |j| {
            j.data.set_motor_max_force(axis.into(), maxForce);
        })
    }

    pub fn jointConfigureMotorVelocity(
        &mut self,
        handle: FlatHandle,
        axis: RawJointAxis,
        targetVel: f32,
        factor: f32,
    ) {
        self.jointConfigureMotor(handle, axis, 0.0, targetVel, 0.0, factor)
    }

    pub fn jointConfigureMotorPosition(
        &mut self,
        handle: FlatHandle,
        axis: RawJointAxis,
        targetPos: f32,
        stiffness: f32,
        damping: f32,
    ) {
        self.jointConfigureMotor(handle, axis, targetPos, 0.0, stiffness, damping)
    }

    pub fn jointConfigureMotor(
        &mut self,
        handle: FlatHandle,
        axis: RawJointAxis,
        targetPos: f32,
        targetVel: f32,
        stiffness: f32,
        damping: f32,
    ) {
        self.map_mut(handle, |j| {
            j.data
                .set_motor(axis.into(), targetPos, targetVel, stiffness, damping);
        })
    }
}
