use crate::dynamics::RawRigidBodySet;
use crate::scratch;
use crate::utils::{self, FlatHandle};
use rapier::control::PidController;
use rapier::dynamics::AxesMask;
use rapier::math::{Rotation, Vector};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct RawPidController {
    controller: PidController,
}

#[wasm_bindgen]
impl RawPidController {
    #[wasm_bindgen(constructor)]
    pub fn new(kp: f32, ki: f32, kd: f32, axes_mask: u8) -> Self {
        let controller = PidController::new(kp, ki, kd, AxesMask::from_bits_truncate(axes_mask));
        Self { controller }
    }

    pub fn set_kp(&mut self, kp: f32, axes: u8) {
        let axes = AxesMask::from_bits_truncate(axes);
        if axes.contains(AxesMask::LIN_X) {
            self.controller.pd.lin_kp.x = kp;
        }
        if axes.contains(AxesMask::LIN_Y) {
            self.controller.pd.lin_kp.y = kp;
        }
        #[cfg(feature = "dim3")]
        if axes.contains(AxesMask::LIN_Z) {
            self.controller.pd.lin_kp.z = kp;
        }
        #[cfg(feature = "dim3")]
        if axes.contains(AxesMask::ANG_X) {
            self.controller.pd.ang_kp.x = kp;
        }
        #[cfg(feature = "dim3")]
        if axes.contains(AxesMask::ANG_Y) {
            self.controller.pd.ang_kp.y = kp;
        }
        if axes.contains(AxesMask::ANG_Z) {
            #[cfg(feature = "dim2")]
            {
                self.controller.pd.ang_kp = kp;
            }
            #[cfg(feature = "dim3")]
            {
                self.controller.pd.ang_kp.z = kp;
            }
        }
    }

    pub fn set_ki(&mut self, ki: f32, axes: u8) {
        let axes = AxesMask::from_bits_truncate(axes);
        if axes.contains(AxesMask::LIN_X) {
            self.controller.lin_ki.x = ki;
        }
        if axes.contains(AxesMask::LIN_Y) {
            self.controller.lin_ki.y = ki;
        }
        #[cfg(feature = "dim3")]
        if axes.contains(AxesMask::LIN_Z) {
            self.controller.lin_ki.z = ki;
        }
        #[cfg(feature = "dim3")]
        if axes.contains(AxesMask::ANG_X) {
            self.controller.ang_ki.x = ki;
        }
        #[cfg(feature = "dim3")]
        if axes.contains(AxesMask::ANG_Y) {
            self.controller.ang_ki.y = ki;
        }
        if axes.contains(AxesMask::ANG_Z) {
            #[cfg(feature = "dim2")]
            {
                self.controller.ang_ki = ki;
            }
            #[cfg(feature = "dim3")]
            {
                self.controller.ang_ki.z = ki;
            }
        }
    }

    pub fn set_kd(&mut self, kd: f32, axes: u8) {
        let axes = AxesMask::from_bits_truncate(axes);
        if axes.contains(AxesMask::LIN_X) {
            self.controller.pd.lin_kd.x = kd;
        }
        if axes.contains(AxesMask::LIN_Y) {
            self.controller.pd.lin_kd.y = kd;
        }
        #[cfg(feature = "dim3")]
        if axes.contains(AxesMask::LIN_Z) {
            self.controller.pd.lin_kd.z = kd;
        }
        #[cfg(feature = "dim3")]
        if axes.contains(AxesMask::ANG_X) {
            self.controller.pd.ang_kd.x = kd;
        }
        #[cfg(feature = "dim3")]
        if axes.contains(AxesMask::ANG_Y) {
            self.controller.pd.ang_kd.y = kd;
        }
        if axes.contains(AxesMask::ANG_Z) {
            #[cfg(feature = "dim2")]
            {
                self.controller.pd.ang_kd = kd;
            }
            #[cfg(feature = "dim3")]
            {
                self.controller.pd.ang_kd.z = kd;
            }
        }
    }

    pub fn set_axes_mask(&mut self, axes_mask: u8) {
        self.controller.pd.axes = AxesMask::from_bits_truncate(axes_mask);
    }

    pub fn reset_integrals(&mut self) {
        self.controller.reset_integrals();
    }

    /// Applies the linear correction to the body's velocity.
    ///
    /// The targets are passed component-wise so the JS side allocates no
    /// `RawVector` per call (this runs once per controlled body per frame).
    #[cfg(feature = "dim2")]
    pub fn apply_linear_correction(
        &mut self,
        dt: f32,
        bodies: &mut RawRigidBodySet,
        rb_handle: FlatHandle,
        target_x: f32,
        target_y: f32,
        target_linvel_x: f32,
        target_linvel_y: f32,
    ) {
        self.do_apply_linear_correction(
            dt,
            bodies,
            rb_handle,
            Vector::new(target_x, target_y),
            Vector::new(target_linvel_x, target_linvel_y),
        )
    }

    /// Applies the linear correction to the body's velocity.
    ///
    /// The targets are passed component-wise so the JS side allocates no
    /// `RawVector` per call (this runs once per controlled body per frame).
    #[cfg(feature = "dim3")]
    pub fn apply_linear_correction(
        &mut self,
        dt: f32,
        bodies: &mut RawRigidBodySet,
        rb_handle: FlatHandle,
        target_x: f32,
        target_y: f32,
        target_z: f32,
        target_linvel_x: f32,
        target_linvel_y: f32,
        target_linvel_z: f32,
    ) {
        self.do_apply_linear_correction(
            dt,
            bodies,
            rb_handle,
            Vector::new(target_x, target_y, target_z),
            Vector::new(target_linvel_x, target_linvel_y, target_linvel_z),
        )
    }

    #[cfg(feature = "dim2")]
    pub fn apply_angular_correction(
        &mut self,
        dt: f32,
        bodies: &mut RawRigidBodySet,
        rb_handle: FlatHandle,
        target_rotation: f32,
        target_angvel: f32,
    ) {
        let rb_handle = crate::utils::body_handle(rb_handle);
        bodies.mark_pending(rb_handle);
        let Some(rb) = bodies.bodies.get_mut(rb_handle) else {
            return;
        };

        let correction = self.controller.angular_rigid_body_correction(
            dt,
            rb,
            Rotation::new(target_rotation),
            target_angvel,
        );
        rb.set_angvel(rb.angvel() + correction, true);
        bodies.write_through(rb_handle);
    }

    /// Applies the angular correction to the body's velocity. The target
    /// rotation is normalized like every other quaternion input.
    #[cfg(feature = "dim3")]
    pub fn apply_angular_correction(
        &mut self,
        dt: f32,
        bodies: &mut RawRigidBodySet,
        rb_handle: FlatHandle,
        target_rotation_x: f32,
        target_rotation_y: f32,
        target_rotation_z: f32,
        target_rotation_w: f32,
        target_angvel_x: f32,
        target_angvel_y: f32,
        target_angvel_z: f32,
    ) {
        let rb_handle = crate::utils::body_handle(rb_handle);
        bodies.mark_pending(rb_handle);
        let Some(rb) = bodies.bodies.get_mut(rb_handle) else {
            return;
        };

        let target_rotation = utils::unit_rotation(
            target_rotation_x,
            target_rotation_y,
            target_rotation_z,
            target_rotation_w,
        )
        .unwrap_or(Rotation::IDENTITY);
        let correction = self.controller.angular_rigid_body_correction(
            dt,
            rb,
            target_rotation,
            Vector::new(target_angvel_x, target_angvel_y, target_angvel_z),
        );
        rb.set_angvel(rb.angvel() + correction, true);
        bodies.write_through(rb_handle);
    }

    /// Writes the linear correction into the scratch buffer.
    #[cfg(feature = "dim2")]
    pub fn linear_correction(
        &mut self,
        dt: f32,
        bodies: &RawRigidBodySet,
        rb_handle: FlatHandle,
        target_x: f32,
        target_y: f32,
        target_linvel_x: f32,
        target_linvel_y: f32,
    ) {
        self.do_linear_correction(
            dt,
            bodies,
            rb_handle,
            Vector::new(target_x, target_y),
            Vector::new(target_linvel_x, target_linvel_y),
        )
    }

    /// Writes the linear correction into the scratch buffer.
    #[cfg(feature = "dim3")]
    pub fn linear_correction(
        &mut self,
        dt: f32,
        bodies: &RawRigidBodySet,
        rb_handle: FlatHandle,
        target_x: f32,
        target_y: f32,
        target_z: f32,
        target_linvel_x: f32,
        target_linvel_y: f32,
        target_linvel_z: f32,
    ) {
        self.do_linear_correction(
            dt,
            bodies,
            rb_handle,
            Vector::new(target_x, target_y, target_z),
            Vector::new(target_linvel_x, target_linvel_y, target_linvel_z),
        )
    }

    #[cfg(feature = "dim2")]
    pub fn angular_correction(
        &mut self,
        dt: f32,
        bodies: &RawRigidBodySet,
        rb_handle: FlatHandle,
        target_rotation: f32,
        target_angvel: f32,
    ) -> f32 {
        let rb_handle = crate::utils::body_handle(rb_handle);
        let Some(rb) = bodies.bodies.get(rb_handle) else {
            return 0.0;
        };

        self.controller.angular_rigid_body_correction(
            dt,
            rb,
            Rotation::new(target_rotation),
            target_angvel,
        )
    }

    /// Writes the angular correction into the scratch buffer. The target
    /// rotation is normalized like every other quaternion input.
    #[cfg(feature = "dim3")]
    pub fn angular_correction(
        &mut self,
        dt: f32,
        bodies: &RawRigidBodySet,
        rb_handle: FlatHandle,
        target_rotation_x: f32,
        target_rotation_y: f32,
        target_rotation_z: f32,
        target_rotation_w: f32,
        target_angvel_x: f32,
        target_angvel_y: f32,
        target_angvel_z: f32,
    ) {
        let rb_handle = crate::utils::body_handle(rb_handle);
        let target_rotation = utils::unit_rotation(
            target_rotation_x,
            target_rotation_y,
            target_rotation_z,
            target_rotation_w,
        )
        .unwrap_or(Rotation::IDENTITY);
        let target_angvel = Vector::new(target_angvel_x, target_angvel_y, target_angvel_z);
        let correction = bodies
            .bodies
            .get(rb_handle)
            .map(|rb| {
                self.controller.angular_rigid_body_correction(
                    dt,
                    rb,
                    target_rotation,
                    target_angvel,
                )
            })
            .unwrap_or(Vector::ZERO);

        scratch::write_vector(correction);
    }
}

impl RawPidController {
    fn do_apply_linear_correction(
        &mut self,
        dt: f32,
        bodies: &mut RawRigidBodySet,
        rb_handle: FlatHandle,
        target_translation: Vector,
        target_linvel: Vector,
    ) {
        let rb_handle = utils::body_handle(rb_handle);
        bodies.mark_pending(rb_handle);
        let Some(rb) = bodies.bodies.get_mut(rb_handle) else {
            return;
        };

        let correction = self.controller.linear_rigid_body_correction(
            dt,
            rb,
            target_translation.into(),
            target_linvel,
        );
        rb.set_linvel(rb.linvel() + correction, true);
        bodies.write_through(rb_handle);
    }

    fn do_linear_correction(
        &mut self,
        dt: f32,
        bodies: &RawRigidBodySet,
        rb_handle: FlatHandle,
        target_translation: Vector,
        target_linvel: Vector,
    ) {
        let rb_handle = crate::utils::body_handle(rb_handle);
        let correction = bodies
            .bodies
            .get(rb_handle)
            .map(|rb| {
                self.controller.linear_rigid_body_correction(
                    dt,
                    rb,
                    target_translation.into(),
                    target_linvel,
                )
            })
            .unwrap_or(Vector::ZERO);

        scratch::write_vector(correction);
    }
}
