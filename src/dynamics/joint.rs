use crate::math::{RawRotation, RawVector};
use rapier::dynamics::{
    FixedJointBuilder, GenericJoint, JointAxesMask, JointAxis, MotorModel, PrismaticJointBuilder,
    RevoluteJointBuilder, RopeJointBuilder, SpringJointBuilder,
};
#[cfg(feature = "dim3")]
use rapier::dynamics::{GenericJointBuilder, SphericalJointBuilder};
use rapier::math::Pose;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
#[cfg(feature = "dim2")]
pub enum RawJointType {
    Revolute,
    Fixed,
    Prismatic,
    Rope,
    Spring,
    Generic,
}

#[wasm_bindgen]
#[cfg(feature = "dim3")]
pub enum RawJointType {
    Revolute,
    Fixed,
    Prismatic,
    Rope,
    Spring,
    Spherical,
    Generic,
}

impl RawJointType {
    /// Classifies a joint from its whole configuration.
    ///
    /// Rope and spring joints lock no axis at all — they couple the linear axes
    /// and constrain the distance through a limit (rope) or a motor (spring) —
    /// so the locked-axes mapping below can only ever call them `Generic`.
    pub(crate) fn from_generic(joint: &GenericJoint) -> Self {
        if joint.locked_axes.is_empty() && joint.coupled_axes == JointAxesMask::LIN_AXES {
            let motor = &joint.motors[JointAxis::LinX as usize];
            let has_motor = joint.motor_axes.contains(JointAxesMask::LIN_X);
            let has_limit = joint.limit_axes.contains(JointAxesMask::LIN_X);
            // A spring is a position motor with a stiffness (or damping); a rope
            // on which a velocity motor was configured has a motor too, but a
            // stiffness-less one, and stays a rope.
            if has_motor && (motor.stiffness != 0.0 || motor.damping != 0.0 || !has_limit) {
                return RawJointType::Spring;
            }
            if has_limit {
                return RawJointType::Rope;
            }
        }
        joint.locked_axes.into()
    }
}

/// The type of this joint.
#[cfg(feature = "dim2")]
impl From<JointAxesMask> for RawJointType {
    fn from(ty: JointAxesMask) -> RawJointType {
        let rev_axes = JointAxesMask::LOCKED_REVOLUTE_AXES;
        let pri_axes = JointAxesMask::LOCKED_PRISMATIC_AXES;
        let fix_axes = JointAxesMask::LOCKED_FIXED_AXES;

        if ty == rev_axes {
            RawJointType::Revolute
        } else if ty == pri_axes {
            RawJointType::Prismatic
        } else if ty == fix_axes {
            RawJointType::Fixed
        } else {
            RawJointType::Generic
        }
    }
}

/// The type of this joint.
#[cfg(feature = "dim3")]
impl From<JointAxesMask> for RawJointType {
    fn from(ty: JointAxesMask) -> RawJointType {
        // Rapier's own constants: the hand-built masks had drifted (spherical was
        // written as the *angular* axes, so spherical joints reported as Generic).
        let rev_axes = JointAxesMask::LOCKED_REVOLUTE_AXES;
        let pri_axes = JointAxesMask::LOCKED_PRISMATIC_AXES;
        let sph_axes = JointAxesMask::LOCKED_SPHERICAL_AXES;
        let fix_axes = JointAxesMask::LOCKED_FIXED_AXES;

        if ty == rev_axes {
            RawJointType::Revolute
        } else if ty == pri_axes {
            RawJointType::Prismatic
        } else if ty == sph_axes {
            RawJointType::Spherical
        } else if ty == fix_axes {
            RawJointType::Fixed
        } else {
            RawJointType::Generic
        }
    }
}

#[wasm_bindgen]
pub enum RawMotorModel {
    AccelerationBased,
    ForceBased,
}

impl From<RawMotorModel> for MotorModel {
    fn from(model: RawMotorModel) -> MotorModel {
        match model {
            RawMotorModel::AccelerationBased => MotorModel::AccelerationBased,
            RawMotorModel::ForceBased => MotorModel::ForceBased,
        }
    }
}

#[cfg(feature = "dim2")]
#[wasm_bindgen]
#[derive(Copy, Clone)]
pub enum RawJointAxis {
    LinX,
    LinY,
    AngX,
}

#[cfg(feature = "dim3")]
#[wasm_bindgen]
#[derive(Copy, Clone)]
pub enum RawJointAxis {
    LinX,
    LinY,
    LinZ,
    AngX,
    AngY,
    AngZ,
}

impl From<RawJointAxis> for JointAxis {
    fn from(axis: RawJointAxis) -> JointAxis {
        match axis {
            RawJointAxis::LinX => JointAxis::LinX,
            RawJointAxis::LinY => JointAxis::LinY,
            #[cfg(feature = "dim3")]
            RawJointAxis::LinZ => JointAxis::LinZ,
            RawJointAxis::AngX => JointAxis::AngX,
            #[cfg(feature = "dim3")]
            RawJointAxis::AngY => JointAxis::AngY,
            #[cfg(feature = "dim3")]
            RawJointAxis::AngZ => JointAxis::AngZ,
        }
    }
}

#[wasm_bindgen]
pub struct RawGenericJoint(pub(crate) GenericJoint);

#[wasm_bindgen]
impl RawGenericJoint {
    /// Creates a new joint descriptor that builds generic joints.
    ///
    /// Generic joints allow arbitrary axes of freedom to be selected
    /// for the joint from the available 6 degrees of freedom.
    #[cfg(feature = "dim3")]
    pub fn generic(
        anchor1: &RawVector,
        anchor2: &RawVector,
        axis: &RawVector,
        lockedAxes: u8,
    ) -> Option<RawGenericJoint> {
        let axesMask: JointAxesMask = JointAxesMask::from_bits(lockedAxes)?;
        let axis = axis.0.try_normalize()?;
        let joint: GenericJoint = GenericJointBuilder::new(axesMask)
            .local_anchor1(anchor1.0.into())
            .local_anchor2(anchor2.0.into())
            .local_axis1(axis)
            .local_axis2(axis)
            .into();
        Some(Self(joint))
    }

    pub fn spring(
        rest_length: f32,
        stiffness: f32,
        damping: f32,
        anchor1: &RawVector,
        anchor2: &RawVector,
    ) -> Self {
        Self(
            SpringJointBuilder::new(rest_length, stiffness, damping)
                .local_anchor1(anchor1.0.into())
                .local_anchor2(anchor2.0.into())
                .into(),
        )
    }

    pub fn rope(length: f32, anchor1: &RawVector, anchor2: &RawVector) -> Self {
        Self(
            RopeJointBuilder::new(length)
                .local_anchor1(anchor1.0.into())
                .local_anchor2(anchor2.0.into())
                .into(),
        )
    }

    /// Create a new joint descriptor that builds spherical joints.
    ///
    /// A spherical joints allows three relative rotational degrees of freedom
    /// by preventing any relative translation between the anchors of the
    /// two attached rigid-bodies.
    #[cfg(feature = "dim3")]
    pub fn spherical(anchor1: &RawVector, anchor2: &RawVector) -> Self {
        Self(
            SphericalJointBuilder::new()
                .local_anchor1(anchor1.0.into())
                .local_anchor2(anchor2.0.into())
                .into(),
        )
    }

    /// Creates a new joint descriptor that builds a Prismatic joint.
    ///
    /// A prismatic joint removes all the degrees of freedom between the
    /// affected bodies, except for the translation along one axis.
    ///
    /// Returns `None` if any of the provided axes cannot be normalized.
    #[cfg(feature = "dim2")]
    pub fn prismatic(
        anchor1: &RawVector,
        anchor2: &RawVector,
        axis: &RawVector,
        limitsEnabled: bool,
        limitsMin: f32,
        limitsMax: f32,
    ) -> Option<RawGenericJoint> {
        let axis = axis.0.try_normalize()?;
        let mut joint = PrismaticJointBuilder::new(axis)
            .local_anchor1(anchor1.0.into())
            .local_anchor2(anchor2.0.into());

        if limitsEnabled {
            joint = joint.limits([limitsMin, limitsMax]);
        }

        Some(Self(joint.into()))
    }

    /// Creates a new joint descriptor that builds a Prismatic joint.
    ///
    /// A prismatic joint removes all the degrees of freedom between the
    /// affected bodies, except for the translation along one axis.
    ///
    /// Returns `None` if any of the provided axes cannot be normalized.
    #[cfg(feature = "dim3")]
    pub fn prismatic(
        anchor1: &RawVector,
        anchor2: &RawVector,
        axis: &RawVector,
        limitsEnabled: bool,
        limitsMin: f32,
        limitsMax: f32,
    ) -> Option<RawGenericJoint> {
        let axis = axis.0.try_normalize()?;
        let mut joint = PrismaticJointBuilder::new(axis)
            .local_anchor1(anchor1.0.into())
            .local_anchor2(anchor2.0.into());

        if limitsEnabled {
            joint = joint.limits([limitsMin, limitsMax]);
        }

        Some(Self(joint.into()))
    }

    /// Creates a new joint descriptor that builds a Fixed joint.
    ///
    /// A fixed joint removes all the degrees of freedom between the affected bodies.
    pub fn fixed(
        anchor1: &RawVector,
        axes1: &RawRotation,
        anchor2: &RawVector,
        axes2: &RawRotation,
    ) -> RawGenericJoint {
        let pos1 = Pose::from_parts(anchor1.0, axes1.0);
        let pos2 = Pose::from_parts(anchor2.0, axes2.0);
        Self(
            FixedJointBuilder::new()
                .local_frame1(pos1)
                .local_frame2(pos2)
                .into(),
        )
    }

    /// Create a new joint descriptor that builds Revolute joints.
    ///
    /// A revolute joint removes all degrees of freedom between the affected
    /// bodies except for the rotation.
    #[cfg(feature = "dim2")]
    pub fn revolute(anchor1: &RawVector, anchor2: &RawVector) -> Option<RawGenericJoint> {
        Some(Self(
            RevoluteJointBuilder::new()
                .local_anchor1(anchor1.0.into())
                .local_anchor2(anchor2.0.into())
                .into(),
        ))
    }

    /// Create a new joint descriptor that builds Revolute joints.
    ///
    /// A revolute joint removes all degrees of freedom between the affected
    /// bodies except for the rotation along one axis.
    #[cfg(feature = "dim3")]
    pub fn revolute(
        anchor1: &RawVector,
        anchor2: &RawVector,
        axis: &RawVector,
    ) -> Option<RawGenericJoint> {
        let axis = axis.0.try_normalize()?;
        Some(Self(
            RevoluteJointBuilder::new(axis)
                .local_anchor1(anchor1.0.into())
                .local_anchor2(anchor2.0.into())
                .into(),
        ))
    }

    /// Create a new joint descriptor that builds Revolute joints with
    /// independent local axes for each attached rigid-body.
    ///
    /// This is equivalent to a revolute joint, except that the hinge axis is
    /// given in the local-space of each body instead of assuming both local
    /// axes are identical.
    ///
    /// Returns `None` if any of the provided axes cannot be normalized.
    #[cfg(feature = "dim3")]
    pub fn revoluteWithAxes(
        anchor1: &RawVector,
        anchor2: &RawVector,
        axis1: &RawVector,
        axis2: &RawVector,
    ) -> Option<RawGenericJoint> {
        let axis1 = axis1.0.try_normalize()?;
        let axis2 = axis2.0.try_normalize()?;
        let joint: GenericJoint = GenericJointBuilder::new(JointAxesMask::LOCKED_REVOLUTE_AXES)
            .local_anchor1(anchor1.0.into())
            .local_anchor2(anchor2.0.into())
            .local_axis1(axis1)
            .local_axis2(axis2)
            .into();
        Some(Self(joint))
    }
}
