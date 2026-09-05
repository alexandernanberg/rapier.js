use crate::dynamics::{RawImpulseJointSet, RawJointAxis, RawJointType, RawMotorModel};
use crate::scratch;
use crate::utils::{self, FlatHandle};
use rapier::dynamics::JointAxis;
#[cfg(feature = "dim2")]
use rapier::math::Rotation;
use rapier::math::{Pose, Vector};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
impl RawImpulseJointSet {
    /// The type of this joint.
    pub fn jointType(&self, handle: FlatHandle) -> RawJointType {
        self.map(handle, |j| RawJointType::from_generic(&j.data))
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
    pub fn jointFrameX1(&self, handle: FlatHandle) {
        self.map(handle, |j| {
            scratch::write_rotation(j.data.local_frame1.rotation)
        })
    }

    /// The angular part of the joint’s local frame relative to the second rigid-body it is attached to.
    pub fn jointFrameX2(&self, handle: FlatHandle) {
        self.map(handle, |j| {
            scratch::write_rotation(j.data.local_frame2.rotation)
        })
    }

    /// The position of the first anchor of this joint.
    ///
    /// The first anchor gives the position of the points application point on the
    /// local frame of the first rigid-body it is attached to.
    pub fn jointAnchor1(&self, handle: FlatHandle) {
        self.map(handle, |j| {
            scratch::write_vector(j.data.local_frame1.translation)
        })
    }

    /// The position of the second anchor of this joint.
    ///
    /// The second anchor gives the position of the points application point on the
    /// local frame of the second rigid-body it is attached to.
    pub fn jointAnchor2(&self, handle: FlatHandle) {
        self.map(handle, |j| {
            scratch::write_vector(j.data.local_frame2.translation)
        })
    }

    // The frame setters take their vectors and rotations component-wise: a
    // `RawVector`/`RawRotation` temporary costs a WASM allocation plus a
    // `FinalizationRegistry` registration each, for a value that is read once.
    // In 3D a drifted quaternion is normalized like every other rotation input;
    // one with no direction to recover leaves the rotation as it was.

    /// Sets the position of the first local anchor
    #[cfg(feature = "dim3")]
    pub fn jointSetAnchor1(&mut self, handle: FlatHandle, x: f32, y: f32, z: f32) {
        self.map_mut(handle, |j| {
            j.data.set_local_anchor1(Vector::new(x, y, z).into());
        });
    }

    /// Sets the position of the first local anchor
    #[cfg(feature = "dim2")]
    pub fn jointSetAnchor1(&mut self, handle: FlatHandle, x: f32, y: f32) {
        self.map_mut(handle, |j| {
            j.data.set_local_anchor1(Vector::new(x, y).into());
        });
    }

    /// Sets the position of the second local anchor
    #[cfg(feature = "dim3")]
    pub fn jointSetAnchor2(&mut self, handle: FlatHandle, x: f32, y: f32, z: f32) {
        self.map_mut(handle, |j| {
            j.data.set_local_anchor2(Vector::new(x, y, z).into());
        })
    }

    /// Sets the position of the second local anchor
    #[cfg(feature = "dim2")]
    pub fn jointSetAnchor2(&mut self, handle: FlatHandle, x: f32, y: f32) {
        self.map_mut(handle, |j| {
            j.data.set_local_anchor2(Vector::new(x, y).into());
        })
    }

    /// Sets the angular part of the joint's local frame relative to the first rigid-body.
    #[cfg(feature = "dim3")]
    pub fn jointSetFrameX1(&mut self, handle: FlatHandle, x: f32, y: f32, z: f32, w: f32) {
        if let Some(q) = utils::unit_rotation(x, y, z, w) {
            self.map_mut(handle, |j| {
                j.data.local_frame1.rotation = q;
            });
        }
    }

    /// Sets the angular part of the joint's local frame relative to the first rigid-body.
    #[cfg(feature = "dim2")]
    pub fn jointSetFrameX1(&mut self, handle: FlatHandle, angle: f32) {
        self.map_mut(handle, |j| {
            j.data.local_frame1.rotation = Rotation::new(angle);
        });
    }

    /// Sets the angular part of the joint's local frame relative to the second rigid-body.
    #[cfg(feature = "dim3")]
    pub fn jointSetFrameX2(&mut self, handle: FlatHandle, x: f32, y: f32, z: f32, w: f32) {
        if let Some(q) = utils::unit_rotation(x, y, z, w) {
            self.map_mut(handle, |j| {
                j.data.local_frame2.rotation = q;
            });
        }
    }

    /// Sets the angular part of the joint's local frame relative to the second rigid-body.
    #[cfg(feature = "dim2")]
    pub fn jointSetFrameX2(&mut self, handle: FlatHandle, angle: f32) {
        self.map_mut(handle, |j| {
            j.data.local_frame2.rotation = Rotation::new(angle);
        });
    }

    /// Sets the full local frame (anchor + rotation) for the first rigid-body attachment.
    #[cfg(feature = "dim3")]
    pub fn jointSetLocalFrame1(
        &mut self,
        handle: FlatHandle,
        anchor_x: f32,
        anchor_y: f32,
        anchor_z: f32,
        rot_x: f32,
        rot_y: f32,
        rot_z: f32,
        rot_w: f32,
    ) {
        let rot = utils::unit_rotation(rot_x, rot_y, rot_z, rot_w);
        self.map_mut(handle, |j| {
            // A rejected rotation keeps the current one; the anchor still moves.
            let rot = rot.unwrap_or(j.data.local_frame1.rotation);
            j.data.set_local_frame1(Pose::from_parts(
                Vector::new(anchor_x, anchor_y, anchor_z),
                rot,
            ));
        });
    }

    /// Sets the full local frame (anchor + rotation) for the first rigid-body attachment.
    #[cfg(feature = "dim2")]
    pub fn jointSetLocalFrame1(
        &mut self,
        handle: FlatHandle,
        anchor_x: f32,
        anchor_y: f32,
        angle: f32,
    ) {
        self.map_mut(handle, |j| {
            j.data.set_local_frame1(Pose::from_parts(
                Vector::new(anchor_x, anchor_y),
                Rotation::new(angle),
            ));
        });
    }

    /// Sets the full local frame (anchor + rotation) for the second rigid-body attachment.
    #[cfg(feature = "dim3")]
    pub fn jointSetLocalFrame2(
        &mut self,
        handle: FlatHandle,
        anchor_x: f32,
        anchor_y: f32,
        anchor_z: f32,
        rot_x: f32,
        rot_y: f32,
        rot_z: f32,
        rot_w: f32,
    ) {
        let rot = utils::unit_rotation(rot_x, rot_y, rot_z, rot_w);
        self.map_mut(handle, |j| {
            let rot = rot.unwrap_or(j.data.local_frame2.rotation);
            j.data.set_local_frame2(Pose::from_parts(
                Vector::new(anchor_x, anchor_y, anchor_z),
                rot,
            ));
        });
    }

    /// Sets the full local frame (anchor + rotation) for the second rigid-body attachment.
    #[cfg(feature = "dim2")]
    pub fn jointSetLocalFrame2(
        &mut self,
        handle: FlatHandle,
        anchor_x: f32,
        anchor_y: f32,
        angle: f32,
    ) {
        self.map_mut(handle, |j| {
            j.data.set_local_frame2(Pose::from_parts(
                Vector::new(anchor_x, anchor_y),
                Rotation::new(angle),
            ));
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
