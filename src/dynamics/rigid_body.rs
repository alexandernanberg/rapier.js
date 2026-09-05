use crate::dynamics::{RawRigidBodySet, RawRigidBodyType};
use crate::geometry::RawColliderSet;
use crate::scratch;
use crate::utils::{self, FlatHandle};
use rapier::dynamics::MassProperties;
use rapier::math::{Rotation, Vector};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
impl RawRigidBodySet {
    /// The world-space translation of this rigid-body, written to the scratch buffer.
    #[cfg(feature = "dim2")]
    pub fn rbTranslation(&self, handle: FlatHandle) {
        self.map(handle, |rb| {
            let t = rb.position().translation;
            scratch::write(&[t.x, t.y]);
        });
    }

    /// The world-space translation of this rigid-body, written to the scratch buffer.
    #[cfg(feature = "dim3")]
    pub fn rbTranslation(&self, handle: FlatHandle) {
        self.map(handle, |rb| {
            let t = rb.position().translation;
            scratch::write(&[t.x, t.y, t.z]);
        });
    }

    /// The world-space orientation of this rigid-body, written to the scratch buffer.
    #[cfg(feature = "dim2")]
    pub fn rbRotation(&self, handle: FlatHandle) {
        self.map(handle, |rb| {
            scratch::write(&[rb.position().rotation.angle()]);
        });
    }

    /// The world-space orientation of this rigid-body, written to the scratch buffer.
    #[cfg(feature = "dim3")]
    pub fn rbRotation(&self, handle: FlatHandle) {
        self.map(handle, |rb| {
            let r = rb.position().rotation;
            scratch::write(&[r.x, r.y, r.z, r.w]);
        });
    }

    /// Put the given rigid-body to sleep.
    pub fn rbSleep(&mut self, handle: FlatHandle) {
        self.map_mut(handle, |rb| rb.sleep());
    }

    /// Is this rigid-body sleeping?
    pub fn rbIsSleeping(&self, handle: FlatHandle) -> bool {
        self.map(handle, |rb| rb.is_sleeping())
    }

    /// Is the velocity of this rigid-body not zero?
    pub fn rbIsMoving(&self, handle: FlatHandle) -> bool {
        self.map(handle, |rb| rb.is_moving())
    }

    /// The world-space next translation of this rigid-body, written to the scratch buffer.
    ///
    /// If this rigid-body is kinematic this value is set by the `setNextKinematicTranslation`
    /// method and is used for estimating the kinematic body velocity at the next timestep.
    /// For non-kinematic bodies, this value is currently unspecified.
    #[cfg(feature = "dim2")]
    pub fn rbNextTranslation(&self, handle: FlatHandle) {
        self.map(handle, |rb| {
            let t = rb.next_position().translation;
            scratch::write(&[t.x, t.y]);
        });
    }

    /// The world-space next translation of this rigid-body, written to the scratch buffer.
    ///
    /// If this rigid-body is kinematic this value is set by the `setNextKinematicTranslation`
    /// method and is used for estimating the kinematic body velocity at the next timestep.
    /// For non-kinematic bodies, this value is currently unspecified.
    #[cfg(feature = "dim3")]
    pub fn rbNextTranslation(&self, handle: FlatHandle) {
        self.map(handle, |rb| {
            let t = rb.next_position().translation;
            scratch::write(&[t.x, t.y, t.z]);
        });
    }

    /// The world-space next orientation of this rigid-body, written to the scratch buffer.
    ///
    /// If this rigid-body is kinematic this value is set by the `setNextKinematicRotation`
    /// method and is used for estimating the kinematic body velocity at the next timestep.
    /// For non-kinematic bodies, this value is currently unspecified.
    #[cfg(feature = "dim2")]
    pub fn rbNextRotation(&self, handle: FlatHandle) {
        self.map(handle, |rb| {
            scratch::write(&[rb.next_position().rotation.angle()]);
        });
    }

    /// The world-space next orientation of this rigid-body, written to the scratch buffer.
    ///
    /// If this rigid-body is kinematic this value is set by the `setNextKinematicRotation`
    /// method and is used for estimating the kinematic body velocity at the next timestep.
    /// For non-kinematic bodies, this value is currently unspecified.
    #[cfg(feature = "dim3")]
    pub fn rbNextRotation(&self, handle: FlatHandle) {
        self.map(handle, |rb| {
            let r = rb.next_position().rotation;
            scratch::write(&[r.x, r.y, r.z, r.w]);
        });
    }

    /// Sets the translation of this rigid-body.
    ///
    /// # Parameters
    /// - `x`: the world-space position of the rigid-body along the `x` axis.
    /// - `y`: the world-space position of the rigid-body along the `y` axis.
    /// - `z`: the world-space position of the rigid-body along the `z` axis.
    /// - `wakeUp`: forces the rigid-body to wake-up so it is properly affected by forces if it
    /// wasn't moving before modifying its position.
    #[cfg(feature = "dim3")]
    pub fn rbSetTranslation(&mut self, handle: FlatHandle, x: f32, y: f32, z: f32, wakeUp: bool) {
        self.map_mut(handle, |rb| {
            rb.set_translation(Vector::new(x, y, z), wakeUp);
        })
    }

    /// Sets the translation of this rigid-body.
    ///
    /// # Parameters
    /// - `x`: the world-space position of the rigid-body along the `x` axis.
    /// - `y`: the world-space position of the rigid-body along the `y` axis.
    /// - `wakeUp`: forces the rigid-body to wake-up so it is properly affected by forces if it
    /// wasn't moving before modifying its position.
    #[cfg(feature = "dim2")]
    pub fn rbSetTranslation(&mut self, handle: FlatHandle, x: f32, y: f32, wakeUp: bool) {
        self.map_mut(handle, |rb| {
            rb.set_translation(Vector::new(x, y), wakeUp);
        })
    }

    /// Sets the rotation quaternion of this rigid-body.
    ///
    /// This does nothing if a zero quaternion is provided.
    ///
    /// # Parameters
    /// - `x`: the first vector component of the quaternion.
    /// - `y`: the second vector component of the quaternion.
    /// - `z`: the third vector component of the quaternion.
    /// - `w`: the scalar component of the quaternion.
    /// - `wakeUp`: forces the rigid-body to wake-up so it is properly affected by forces if it
    /// wasn't moving before modifying its position.
    #[cfg(feature = "dim3")]
    pub fn rbSetRotation(
        &mut self,
        handle: FlatHandle,
        x: f32,
        y: f32,
        z: f32,
        w: f32,
        wakeUp: bool,
    ) {
        if let Some(q) = utils::unit_rotation(x, y, z, w) {
            self.map_mut(handle, |rb| rb.set_rotation(q, wakeUp))
        } else if wakeUp {
            // A rejected (zero) quaternion leaves the rotation alone, but the
            // wake-up must not be dropped with it: same policy as
            // `rbSetTransform`. The pose is unchanged, so no write-through.
            self.map_mut_untracked(handle, |rb| rb.wake_up(true))
        }
    }

    /// Sets the rotation angle of this rigid-body.
    ///
    /// # Parameters
    /// - `angle`: the rotation angle, in radians.
    /// - `wakeUp`: forces the rigid-body to wake-up so it is properly affected by forces if it
    /// wasn't moving before modifying its position.
    #[cfg(feature = "dim2")]
    pub fn rbSetRotation(&mut self, handle: FlatHandle, angle: f32, wakeUp: bool) {
        self.map_mut(handle, |rb| rb.set_rotation(Rotation::new(angle), wakeUp))
    }

    /// Sets the linear velocity of this rigid-body.
    #[cfg(feature = "dim3")]
    pub fn rbSetLinvel(&mut self, handle: FlatHandle, x: f32, y: f32, z: f32, wakeUp: bool) {
        self.map_mut(handle, |rb| {
            rb.set_linvel(Vector::new(x, y, z), wakeUp);
        });
    }

    /// Sets the linear velocity of this rigid-body.
    #[cfg(feature = "dim2")]
    pub fn rbSetLinvel(&mut self, handle: FlatHandle, x: f32, y: f32, wakeUp: bool) {
        self.map_mut(handle, |rb| {
            rb.set_linvel(Vector::new(x, y), wakeUp);
        });
    }

    /// Sets the angular velocity of this rigid-body.
    #[cfg(feature = "dim2")]
    pub fn rbSetAngvel(&mut self, handle: FlatHandle, angvel: f32, wakeUp: bool) {
        self.map_mut(handle, |rb| {
            rb.set_angvel(angvel, wakeUp);
        });
    }

    /// Sets the angular velocity of this rigid-body.
    #[cfg(feature = "dim3")]
    pub fn rbSetAngvel(&mut self, handle: FlatHandle, x: f32, y: f32, z: f32, wakeUp: bool) {
        self.map_mut(handle, |rb| {
            rb.set_angvel(Vector::new(x, y, z), wakeUp);
        });
    }

    /// If this rigid body is kinematic, sets its future translation after the next timestep integration.
    ///
    /// This should be used instead of `rigidBody.setTranslation` to make the dynamic object
    /// interacting with this kinematic body behave as expected. Internally, Rapier will compute
    /// an artificial velocity for this rigid-body from its current position and its next kinematic
    /// position. This velocity will be used to compute forces on dynamic bodies interacting with
    /// this body.
    ///
    /// # Parameters
    /// - `x`: the world-space position of the rigid-body along the `x` axis.
    /// - `y`: the world-space position of the rigid-body along the `y` axis.
    /// - `z`: the world-space position of the rigid-body along the `z` axis.
    // The `rbSetNextKinematic*` setters only write the body's *next* pose, which
    // the transform buffer does not hold; the current pose and velocity are left
    // as they are until the step integrates them. They therefore go through
    // `map_mut_untracked`: routing them through `map_mut` would rewrite the slot
    // with unchanged values and, worse, count every kinematic body driven each
    // frame toward the pending list's `max(64, len / 2)` budget, after which the
    // incremental sync gives up and every step rewrites every body *and* collider
    // slot. Nothing is lost: a kinematic body is never asleep, so it is always in
    // the island manager's active set and the post-step sync refreshes it anyway.
    #[cfg(feature = "dim3")]
    pub fn rbSetNextKinematicTranslation(&mut self, handle: FlatHandle, x: f32, y: f32, z: f32) {
        self.map_mut_untracked(handle, |rb| {
            rb.set_next_kinematic_translation(Vector::new(x, y, z));
        })
    }

    /// If this rigid body is kinematic, sets its future translation after the next timestep integration.
    ///
    /// This should be used instead of `rigidBody.setTranslation` to make the dynamic object
    /// interacting with this kinematic body behave as expected. Internally, Rapier will compute
    /// an artificial velocity for this rigid-body from its current position and its next kinematic
    /// position. This velocity will be used to compute forces on dynamic bodies interacting with
    /// this body.
    ///
    /// # Parameters
    /// - `x`: the world-space position of the rigid-body along the `x` axis.
    /// - `y`: the world-space position of the rigid-body along the `y` axis.
    #[cfg(feature = "dim2")]
    pub fn rbSetNextKinematicTranslation(&mut self, handle: FlatHandle, x: f32, y: f32) {
        self.map_mut_untracked(handle, |rb| {
            rb.set_next_kinematic_translation(Vector::new(x, y));
        })
    }

    /// If this rigid body is kinematic, sets its future rotation after the next timestep integration.
    ///
    /// This should be used instead of `rigidBody.setRotation` to make the dynamic object
    /// interacting with this kinematic body behave as expected. Internally, Rapier will compute
    /// an artificial velocity for this rigid-body from its current position and its next kinematic
    /// position. This velocity will be used to compute forces on dynamic bodies interacting with
    /// this body.
    ///
    /// # Parameters
    /// - `x`: the first vector component of the quaternion.
    /// - `y`: the second vector component of the quaternion.
    /// - `z`: the third vector component of the quaternion.
    /// - `w`: the scalar component of the quaternion.
    #[cfg(feature = "dim3")]
    pub fn rbSetNextKinematicRotation(
        &mut self,
        handle: FlatHandle,
        x: f32,
        y: f32,
        z: f32,
        w: f32,
    ) {
        if let Some(q) = utils::unit_rotation(x, y, z, w) {
            self.map_mut_untracked(handle, |rb| {
                rb.set_next_kinematic_rotation(q);
            })
        }
    }

    /// If this rigid body is kinematic, sets its future rotation after the next timestep integration.
    ///
    /// This should be used instead of `rigidBody.setRotation` to make the dynamic object
    /// interacting with this kinematic body behave as expected. Internally, Rapier will compute
    /// an artificial velocity for this rigid-body from its current position and its next kinematic
    /// position. This velocity will be used to compute forces on dynamic bodies interacting with
    /// this body.
    ///
    /// # Parameters
    /// - `angle`: the rotation angle, in radians.
    #[cfg(feature = "dim2")]
    pub fn rbSetNextKinematicRotation(&mut self, handle: FlatHandle, angle: f32) {
        self.map_mut_untracked(handle, |rb| {
            rb.set_next_kinematic_rotation(Rotation::new(angle));
        })
    }

    /// Sets both the translation and rotation of this rigid-body in a single WASM call.
    ///
    /// # Parameters
    /// - `tx`, `ty`, `tz`: the world-space position of the rigid-body.
    /// - `rx`, `ry`, `rz`, `rw`: the rotation quaternion components.
    /// - `wakeUp`: forces the rigid-body to wake-up so it is properly affected by forces.
    #[cfg(feature = "dim3")]
    pub fn rbSetTransform(
        &mut self,
        handle: FlatHandle,
        tx: f32,
        ty: f32,
        tz: f32,
        rx: f32,
        ry: f32,
        rz: f32,
        rw: f32,
        wakeUp: bool,
    ) {
        let q = utils::unit_rotation(rx, ry, rz, rw);
        self.map_mut(handle, |rb| {
            // Rapier only wakes the body from inside its "component actually
            // changed" branch, so the wake-up has to ride on both calls: a
            // body given a new rotation at its current translation (or the
            // other way around) would otherwise stay asleep. `wake_up` is
            // idempotent, so passing it twice costs nothing.
            rb.set_translation(Vector::new(tx, ty, tz), wakeUp);
            if let Some(q) = q {
                rb.set_rotation(q, wakeUp);
            } else if wakeUp {
                // A rejected (zero) quaternion still must not drop the wake-up.
                rb.wake_up(true);
            }
        })
    }

    /// Sets both the translation and rotation of this rigid-body in a single WASM call.
    ///
    /// # Parameters
    /// - `tx`, `ty`: the world-space position of the rigid-body.
    /// - `angle`: the rotation angle, in radians.
    /// - `wakeUp`: forces the rigid-body to wake-up so it is properly affected by forces.
    #[cfg(feature = "dim2")]
    pub fn rbSetTransform(
        &mut self,
        handle: FlatHandle,
        tx: f32,
        ty: f32,
        angle: f32,
        wakeUp: bool,
    ) {
        self.map_mut(handle, |rb| {
            // See the 3D variant: the wake-up has to ride on both calls.
            rb.set_translation(Vector::new(tx, ty), wakeUp);
            rb.set_rotation(Rotation::new(angle), wakeUp);
        })
    }

    /// If this rigid body is kinematic, sets its future translation and rotation after the next
    /// timestep integration in a single WASM call.
    ///
    /// # Parameters
    /// - `tx`, `ty`, `tz`: the world-space position of the rigid-body.
    /// - `rx`, `ry`, `rz`, `rw`: the rotation quaternion components.
    #[cfg(feature = "dim3")]
    pub fn rbSetNextKinematicTransform(
        &mut self,
        handle: FlatHandle,
        tx: f32,
        ty: f32,
        tz: f32,
        rx: f32,
        ry: f32,
        rz: f32,
        rw: f32,
    ) {
        let q = utils::unit_rotation(rx, ry, rz, rw);
        self.map_mut_untracked(handle, |rb| {
            rb.set_next_kinematic_translation(Vector::new(tx, ty, tz));
            if let Some(q) = q {
                rb.set_next_kinematic_rotation(q);
            }
        })
    }

    /// If this rigid body is kinematic, sets its future translation and rotation after the next
    /// timestep integration in a single WASM call.
    ///
    /// # Parameters
    /// - `tx`, `ty`: the world-space position of the rigid-body.
    /// - `angle`: the rotation angle, in radians.
    #[cfg(feature = "dim2")]
    pub fn rbSetNextKinematicTransform(
        &mut self,
        handle: FlatHandle,
        tx: f32,
        ty: f32,
        angle: f32,
    ) {
        self.map_mut_untracked(handle, |rb| {
            rb.set_next_kinematic_translation(Vector::new(tx, ty));
            rb.set_next_kinematic_rotation(Rotation::new(angle));
        })
    }

    pub fn rbRecomputeMassPropertiesFromColliders(
        &mut self,
        handle: FlatHandle,
        colliders: &RawColliderSet,
    ) {
        self.map_mut_untracked(handle, |rb| {
            rb.recompute_mass_properties_from_colliders(&colliders.0)
        })
    }

    pub fn rbSetAdditionalMass(&mut self, handle: FlatHandle, mass: f32, wake_up: bool) {
        self.map_mut_untracked(handle, |rb| {
            rb.set_additional_mass(mass, wake_up);
        })
    }

    /// Sets the additional mass properties of this rigid-body.
    ///
    /// The vectors and the inertia frame are passed component-wise, like every
    /// other setter: a `RawVector`/`RawRotation` temporary costs a WASM
    /// allocation plus a `FinalizationRegistry` registration each. The frame is
    /// normalized like every other quaternion input, falling back to the
    /// identity when it has no direction to recover.
    #[cfg(feature = "dim3")]
    pub fn rbSetAdditionalMassProperties(
        &mut self,
        handle: FlatHandle,
        mass: f32,
        centerOfMass_x: f32,
        centerOfMass_y: f32,
        centerOfMass_z: f32,
        principalAngularInertia_x: f32,
        principalAngularInertia_y: f32,
        principalAngularInertia_z: f32,
        angularInertiaFrame_x: f32,
        angularInertiaFrame_y: f32,
        angularInertiaFrame_z: f32,
        angularInertiaFrame_w: f32,
        wake_up: bool,
    ) {
        self.map_mut_untracked(handle, |rb| {
            let mprops = MassProperties::with_principal_inertia_frame(
                Vector::new(centerOfMass_x, centerOfMass_y, centerOfMass_z).into(),
                mass,
                Vector::new(
                    principalAngularInertia_x,
                    principalAngularInertia_y,
                    principalAngularInertia_z,
                ),
                utils::unit_rotation(
                    angularInertiaFrame_x,
                    angularInertiaFrame_y,
                    angularInertiaFrame_z,
                    angularInertiaFrame_w,
                )
                .unwrap_or(Rotation::IDENTITY),
            );
            rb.set_additional_mass_properties(mprops, wake_up)
        })
    }

    /// Sets the additional mass properties of this rigid-body.
    ///
    /// See the 3D variant for why the center of mass is passed component-wise.
    #[cfg(feature = "dim2")]
    pub fn rbSetAdditionalMassProperties(
        &mut self,
        handle: FlatHandle,
        mass: f32,
        centerOfMass_x: f32,
        centerOfMass_y: f32,
        principalAngularInertia: f32,
        wake_up: bool,
    ) {
        self.map_mut_untracked(handle, |rb| {
            let props = MassProperties::new(
                Vector::new(centerOfMass_x, centerOfMass_y).into(),
                mass,
                principalAngularInertia,
            );
            rb.set_additional_mass_properties(props, wake_up)
        })
    }

    /// The linear velocity of this rigid-body, written to the scratch buffer.
    #[cfg(feature = "dim2")]
    pub fn rbLinvel(&self, handle: FlatHandle) {
        self.map(handle, |rb| {
            let v = rb.linvel();
            scratch::write(&[v.x, v.y]);
        });
    }

    /// The linear velocity of this rigid-body, written to the scratch buffer.
    #[cfg(feature = "dim3")]
    pub fn rbLinvel(&self, handle: FlatHandle) {
        self.map(handle, |rb| {
            let v = rb.linvel();
            scratch::write(&[v.x, v.y, v.z]);
        });
    }

    /// The angular velocity of this rigid-body.
    #[cfg(feature = "dim2")]
    pub fn rbAngvel(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |rb| rb.angvel())
    }

    /// The angular velocity of this rigid-body, written to the scratch buffer.
    #[cfg(feature = "dim3")]
    pub fn rbAngvel(&self, handle: FlatHandle) {
        self.map(handle, |rb| {
            let v = rb.angvel();
            scratch::write(&[v.x, v.y, v.z]);
        });
    }

    #[cfg(feature = "dim2")]
    /// The velocity of the given world-space point on this rigid-body, written to the scratch buffer.
    pub fn rbVelocityAtPoint(&self, handle: FlatHandle, px: f32, py: f32) {
        let point = Vector::new(px, py);
        self.map(handle, |rb| {
            scratch::write_vector(rb.velocity_at_point(point));
        })
    }

    #[cfg(feature = "dim3")]
    /// The velocity of the given world-space point on this rigid-body, written to the scratch buffer.
    pub fn rbVelocityAtPoint(&self, handle: FlatHandle, px: f32, py: f32, pz: f32) {
        let point = Vector::new(px, py, pz);
        self.map(handle, |rb| {
            scratch::write_vector(rb.velocity_at_point(point));
        })
    }

    pub fn rbLockTranslations(&mut self, handle: FlatHandle, locked: bool, wake_up: bool) {
        self.map_mut(handle, |rb| rb.lock_translations(locked, wake_up))
    }

    #[cfg(feature = "dim2")]
    pub fn rbSetEnabledTranslations(
        &mut self,
        handle: FlatHandle,
        allow_x: bool,
        allow_y: bool,
        wake_up: bool,
    ) {
        self.map_mut(handle, |rb| {
            rb.set_enabled_translations(allow_x, allow_y, wake_up)
        })
    }

    #[cfg(feature = "dim3")]
    pub fn rbSetEnabledTranslations(
        &mut self,
        handle: FlatHandle,
        allow_x: bool,
        allow_y: bool,
        allow_z: bool,
        wake_up: bool,
    ) {
        self.map_mut(handle, |rb| {
            rb.set_enabled_translations(allow_x, allow_y, allow_z, wake_up)
        })
    }

    pub fn rbLockRotations(&mut self, handle: FlatHandle, locked: bool, wake_up: bool) {
        self.map_mut(handle, |rb| rb.lock_rotations(locked, wake_up))
    }

    #[cfg(feature = "dim3")]
    pub fn rbSetEnabledRotations(
        &mut self,
        handle: FlatHandle,
        allow_x: bool,
        allow_y: bool,
        allow_z: bool,
        wake_up: bool,
    ) {
        self.map_mut(handle, |rb| {
            rb.set_enabled_rotations(allow_x, allow_y, allow_z, wake_up)
        })
    }

    pub fn rbDominanceGroup(&self, handle: FlatHandle) -> i8 {
        self.map(handle, |rb| rb.dominance_group())
    }

    pub fn rbSetDominanceGroup(&mut self, handle: FlatHandle, group: i8) {
        self.map_mut_untracked(handle, |rb| rb.set_dominance_group(group))
    }

    pub fn rbEnableCcd(&mut self, handle: FlatHandle, enabled: bool) {
        self.map_mut_untracked(handle, |rb| rb.enable_ccd(enabled))
    }

    pub fn rbSetSoftCcdPrediction(&mut self, handle: FlatHandle, prediction: f32) {
        self.map_mut_untracked(handle, |rb| rb.set_soft_ccd_prediction(prediction))
    }

    /// The mass of this rigid-body.
    pub fn rbMass(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |rb| rb.mass())
    }

    /// The inverse of the mass of a rigid-body.
    ///
    /// If this is zero, the rigid-body is assumed to have infinite mass.
    pub fn rbInvMass(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |rb| rb.mass_properties().local_mprops.inv_mass)
    }

    /// The inverse mass taking into account translation locking.
    pub fn rbEffectiveInvMass(&self, handle: FlatHandle) {
        self.map(handle, |rb| {
            scratch::write_vector(rb.mass_properties().effective_inv_mass);
        })
    }

    /// The center of mass of a rigid-body expressed in its local-space, written to the scratch buffer.
    #[cfg(feature = "dim2")]
    pub fn rbLocalCom(&self, handle: FlatHandle) {
        self.map(handle, |rb| {
            let c = rb.mass_properties().local_mprops.local_com;
            scratch::write(&[c.x, c.y]);
        });
    }

    /// The center of mass of a rigid-body expressed in its local-space, written to the scratch buffer.
    #[cfg(feature = "dim3")]
    pub fn rbLocalCom(&self, handle: FlatHandle) {
        self.map(handle, |rb| {
            let c = rb.mass_properties().local_mprops.local_com;
            scratch::write(&[c.x, c.y, c.z]);
        });
    }

    /// The world-space center of mass of the rigid-body, written to the scratch buffer.
    #[cfg(feature = "dim2")]
    pub fn rbWorldCom(&self, handle: FlatHandle) {
        self.map(handle, |rb| {
            let c = rb.mass_properties().world_com;
            scratch::write(&[c.x, c.y]);
        });
    }

    /// The world-space center of mass of the rigid-body, written to the scratch buffer.
    #[cfg(feature = "dim3")]
    pub fn rbWorldCom(&self, handle: FlatHandle) {
        self.map(handle, |rb| {
            let c = rb.mass_properties().world_com;
            scratch::write(&[c.x, c.y, c.z]);
        });
    }

    /// The inverse of the principal angular inertia of the rigid-body.
    ///
    /// Components set to zero are assumed to be infinite along the corresponding principal axis.
    #[cfg(feature = "dim2")]
    pub fn rbInvPrincipalInertia(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |rb| {
            rb.mass_properties()
                .local_mprops
                .inv_principal_inertia
                .into()
        })
    }

    /// The inverse of the principal angular inertia of the rigid-body.
    ///
    /// Components set to zero are assumed to be infinite along the corresponding principal axis.
    #[cfg(feature = "dim3")]
    pub fn rbInvPrincipalInertia(&self, handle: FlatHandle) {
        self.map(handle, |rb| {
            scratch::write_vector(rb.mass_properties().local_mprops.inv_principal_inertia);
        })
    }

    #[cfg(feature = "dim3")]
    /// The principal vectors of the local angular inertia tensor of the rigid-body.
    pub fn rbPrincipalInertiaLocalFrame(&self, handle: FlatHandle) {
        self.map(handle, |rb| {
            scratch::write_rotation(
                rb.mass_properties()
                    .local_mprops
                    .principal_inertia_local_frame,
            );
        })
    }

    /// The angular inertia along the principal inertia axes of the rigid-body.
    #[cfg(feature = "dim2")]
    pub fn rbPrincipalInertia(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |rb| {
            rb.mass_properties().local_mprops.principal_inertia().into()
        })
    }

    /// The angular inertia along the principal inertia axes of the rigid-body.
    #[cfg(feature = "dim3")]
    pub fn rbPrincipalInertia(&self, handle: FlatHandle) {
        self.map(handle, |rb| {
            scratch::write_vector(rb.mass_properties().local_mprops.principal_inertia());
        })
    }

    /// The world-space inverse angular inertia tensor of the rigid-body,
    /// taking into account rotation locking.
    #[cfg(feature = "dim2")]
    pub fn rbEffectiveWorldInvInertia(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |rb| {
            rb.mass_properties().effective_world_inv_inertia.into()
        })
    }

    /// The world-space inverse angular inertia tensor of the rigid-body,
    /// taking into account rotation locking.
    #[cfg(feature = "dim3")]
    pub fn rbEffectiveWorldInvInertia(&self, handle: FlatHandle) {
        self.map(handle, |rb| {
            crate::math::write_sdp_matrix3(rb.mass_properties().effective_world_inv_inertia);
        })
    }

    /// The effective world-space angular inertia (that takes the potential rotation locking into account) of
    /// this rigid-body.
    #[cfg(feature = "dim2")]
    pub fn rbEffectiveAngularInertia(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |rb| {
            rb.mass_properties().effective_angular_inertia().into()
        })
    }

    /// The effective world-space angular inertia (that takes the potential rotation locking into account) of
    /// this rigid-body.
    #[cfg(feature = "dim3")]
    pub fn rbEffectiveAngularInertia(&self, handle: FlatHandle) {
        self.map(handle, |rb| {
            crate::math::write_sdp_matrix3(rb.mass_properties().effective_angular_inertia());
        })
    }

    /// Wakes this rigid-body up.
    ///
    /// A dynamic rigid-body that does not move during several consecutive frames will
    /// be put to sleep by the physics engine, i.e., it will stop being simulated in order
    /// to avoid useless computations.
    /// This method forces a sleeping rigid-body to wake-up. This is useful, e.g., before modifying
    /// the position of a dynamic body so that it is properly simulated afterwards.
    pub fn rbWakeUp(&mut self, handle: FlatHandle) {
        // Waking only flips the activation flags; pose and velocity are
        // untouched, so this need not count toward the incremental-sync budget
        // (a game keeping a hundred bodies awake by hand every frame would
        // otherwise force a full re-sync each step).
        self.map_mut_untracked(handle, |rb| rb.wake_up(true))
    }

    /// Is Continuous Collision Detection enabled for this rigid-body?
    pub fn rbIsCcdEnabled(&self, handle: FlatHandle) -> bool {
        self.map(handle, |rb| rb.is_ccd_enabled())
    }
    pub fn rbSoftCcdPrediction(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |rb| rb.soft_ccd_prediction())
    }

    /// The number of colliders attached to this rigid-body.
    pub fn rbNumColliders(&self, handle: FlatHandle) -> usize {
        self.map(handle, |rb| rb.colliders().len())
    }

    /// Retrieves the `i-th` collider attached to this rigid-body.
    ///
    /// # Parameters
    /// - `at`: The index of the collider to retrieve. Must be a number in `[0, this.numColliders()[`.
    ///         This index is **not** the same as the unique identifier of the collider.
    pub fn rbCollider(&self, handle: FlatHandle, at: usize) -> Option<FlatHandle> {
        self.map(handle, |rb| {
            rb.colliders().get(at).map(|h| utils::flat_handle(h.0))
        })
    }

    /// The status of this rigid-body: fixed, dynamic, or kinematic.
    pub fn rbBodyType(&self, handle: FlatHandle) -> RawRigidBodyType {
        self.map(handle, |rb| rb.body_type().into())
    }

    /// Set a new status for this rigid-body: fixed, dynamic, or kinematic.
    pub fn rbSetBodyType(&mut self, handle: FlatHandle, status: RawRigidBodyType, wake_up: bool) {
        self.map_mut(handle, |rb| rb.set_body_type(status.into(), wake_up));
    }

    /// Is this rigid-body fixed?
    pub fn rbIsFixed(&self, handle: FlatHandle) -> bool {
        self.map(handle, |rb| rb.is_fixed())
    }

    /// Is this rigid-body kinematic?
    pub fn rbIsKinematic(&self, handle: FlatHandle) -> bool {
        self.map(handle, |rb| rb.is_kinematic())
    }

    /// Is this rigid-body dynamic?
    pub fn rbIsDynamic(&self, handle: FlatHandle) -> bool {
        self.map(handle, |rb| rb.is_dynamic())
    }

    /// The linear damping coefficient of this rigid-body.
    pub fn rbLinearDamping(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |rb| rb.linear_damping())
    }

    /// The angular damping coefficient of this rigid-body.
    pub fn rbAngularDamping(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |rb| rb.angular_damping())
    }

    pub fn rbSetLinearDamping(&mut self, handle: FlatHandle, factor: f32) {
        self.map_mut_untracked(handle, |rb| rb.set_linear_damping(factor));
    }

    pub fn rbSetAngularDamping(&mut self, handle: FlatHandle, factor: f32) {
        self.map_mut_untracked(handle, |rb| rb.set_angular_damping(factor));
    }

    pub fn rbSetEnabled(&mut self, handle: FlatHandle, enabled: bool) {
        self.map_mut(handle, |rb| rb.set_enabled(enabled))
    }

    pub fn rbIsEnabled(&self, handle: FlatHandle) -> bool {
        self.map(handle, |rb| rb.is_enabled())
    }

    pub fn rbGravityScale(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |rb| rb.gravity_scale())
    }

    pub fn rbSetGravityScale(&mut self, handle: FlatHandle, factor: f32, wakeUp: bool) {
        self.map_mut_untracked(handle, |rb| rb.set_gravity_scale(factor, wakeUp));
    }

    /// Resets to zero all user-added forces added to this rigid-body.
    pub fn rbResetForces(&mut self, handle: FlatHandle, wakeUp: bool) {
        self.map_mut_untracked(handle, |rb| {
            rb.reset_forces(wakeUp);
        })
    }

    /// Resets to zero all user-added torques added to this rigid-body.
    pub fn rbResetTorques(&mut self, handle: FlatHandle, wakeUp: bool) {
        self.map_mut_untracked(handle, |rb| {
            rb.reset_torques(wakeUp);
        })
    }

    /// Adds a force at the center-of-mass of this rigid-body.
    ///
    /// # Parameters
    /// - `force`: the world-space force to apply on the rigid-body.
    /// - `wakeUp`: should the rigid-body be automatically woken-up?
    #[cfg(feature = "dim3")]
    pub fn rbAddForce(&mut self, handle: FlatHandle, x: f32, y: f32, z: f32, wakeUp: bool) {
        self.map_mut_untracked(handle, |rb| {
            rb.add_force(Vector::new(x, y, z), wakeUp);
        })
    }

    /// Adds a force at the center-of-mass of this rigid-body.
    #[cfg(feature = "dim2")]
    pub fn rbAddForce(&mut self, handle: FlatHandle, x: f32, y: f32, wakeUp: bool) {
        self.map_mut_untracked(handle, |rb| {
            rb.add_force(Vector::new(x, y), wakeUp);
        })
    }

    /// Applies an impulse at the center-of-mass of this rigid-body.
    ///
    /// # Parameters
    /// - `impulse`: the world-space impulse to apply on the rigid-body.
    /// - `wakeUp`: should the rigid-body be automatically woken-up?
    #[cfg(feature = "dim3")]
    pub fn rbApplyImpulse(&mut self, handle: FlatHandle, x: f32, y: f32, z: f32, wakeUp: bool) {
        self.map_mut(handle, |rb| {
            rb.apply_impulse(Vector::new(x, y, z), wakeUp);
        })
    }

    /// Applies an impulse at the center-of-mass of this rigid-body.
    #[cfg(feature = "dim2")]
    pub fn rbApplyImpulse(&mut self, handle: FlatHandle, x: f32, y: f32, wakeUp: bool) {
        self.map_mut(handle, |rb| {
            rb.apply_impulse(Vector::new(x, y), wakeUp);
        })
    }

    /// Adds a torque at the center-of-mass of this rigid-body.
    ///
    /// # Parameters
    /// - `torque`: the torque to apply on the rigid-body.
    /// - `wakeUp`: should the rigid-body be automatically woken-up?
    #[cfg(feature = "dim2")]
    pub fn rbAddTorque(&mut self, handle: FlatHandle, torque: f32, wakeUp: bool) {
        self.map_mut_untracked(handle, |rb| {
            rb.add_torque(torque, wakeUp);
        })
    }

    /// Adds a torque at the center-of-mass of this rigid-body.
    ///
    /// # Parameters
    /// - `torque`: the world-space torque to apply on the rigid-body.
    /// - `wakeUp`: should the rigid-body be automatically woken-up?
    #[cfg(feature = "dim3")]
    pub fn rbAddTorque(&mut self, handle: FlatHandle, x: f32, y: f32, z: f32, wakeUp: bool) {
        self.map_mut_untracked(handle, |rb| {
            rb.add_torque(Vector::new(x, y, z), wakeUp);
        })
    }

    /// Applies an impulsive torque at the center-of-mass of this rigid-body.
    ///
    /// # Parameters
    /// - `torque impulse`: the torque impulse to apply on the rigid-body.
    /// - `wakeUp`: should the rigid-body be automatically woken-up?
    #[cfg(feature = "dim2")]
    pub fn rbApplyTorqueImpulse(&mut self, handle: FlatHandle, torque_impulse: f32, wakeUp: bool) {
        self.map_mut(handle, |rb| {
            rb.apply_torque_impulse(torque_impulse, wakeUp);
        })
    }

    /// Applies an impulsive torque at the center-of-mass of this rigid-body.
    ///
    /// # Parameters
    /// - `torque impulse`: the world-space torque impulse to apply on the rigid-body.
    /// - `wakeUp`: should the rigid-body be automatically woken-up?
    #[cfg(feature = "dim3")]
    pub fn rbApplyTorqueImpulse(
        &mut self,
        handle: FlatHandle,
        x: f32,
        y: f32,
        z: f32,
        wakeUp: bool,
    ) {
        self.map_mut(handle, |rb| {
            rb.apply_torque_impulse(Vector::new(x, y, z), wakeUp);
        })
    }

    /// Adds a force at the given world-space point of this rigid-body.
    ///
    /// # Parameters
    /// - `force`: the world-space force to apply on the rigid-body.
    /// - `point`: the world-space point where the impulse is to be applied on the rigid-body.
    /// - `wakeUp`: should the rigid-body be automatically woken-up?
    #[cfg(feature = "dim3")]
    pub fn rbAddForceAtPoint(
        &mut self,
        handle: FlatHandle,
        fx: f32,
        fy: f32,
        fz: f32,
        px: f32,
        py: f32,
        pz: f32,
        wakeUp: bool,
    ) {
        self.map_mut_untracked(handle, |rb| {
            rb.add_force_at_point(
                Vector::new(fx, fy, fz),
                Vector::new(px, py, pz).into(),
                wakeUp,
            );
        })
    }

    /// Adds a force at the given world-space point of this rigid-body.
    #[cfg(feature = "dim2")]
    pub fn rbAddForceAtPoint(
        &mut self,
        handle: FlatHandle,
        fx: f32,
        fy: f32,
        px: f32,
        py: f32,
        wakeUp: bool,
    ) {
        self.map_mut_untracked(handle, |rb| {
            rb.add_force_at_point(Vector::new(fx, fy), Vector::new(px, py).into(), wakeUp);
        })
    }

    /// Applies an impulse at the given world-space point of this rigid-body.
    ///
    /// # Parameters
    /// - `impulse`: the world-space impulse to apply on the rigid-body.
    /// - `point`: the world-space point where the impulse is to be applied on the rigid-body.
    /// - `wakeUp`: should the rigid-body be automatically woken-up?
    #[cfg(feature = "dim3")]
    pub fn rbApplyImpulseAtPoint(
        &mut self,
        handle: FlatHandle,
        ix: f32,
        iy: f32,
        iz: f32,
        px: f32,
        py: f32,
        pz: f32,
        wakeUp: bool,
    ) {
        self.map_mut(handle, |rb| {
            rb.apply_impulse_at_point(
                Vector::new(ix, iy, iz),
                Vector::new(px, py, pz).into(),
                wakeUp,
            );
        })
    }

    /// Applies an impulse at the given world-space point of this rigid-body.
    #[cfg(feature = "dim2")]
    pub fn rbApplyImpulseAtPoint(
        &mut self,
        handle: FlatHandle,
        ix: f32,
        iy: f32,
        px: f32,
        py: f32,
        wakeUp: bool,
    ) {
        self.map_mut(handle, |rb| {
            rb.apply_impulse_at_point(Vector::new(ix, iy), Vector::new(px, py).into(), wakeUp);
        })
    }

    pub fn rbAdditionalSolverIterations(&self, handle: FlatHandle) -> usize {
        self.map(handle, |rb| rb.additional_solver_iterations())
    }

    pub fn rbSetAdditionalSolverIterations(&mut self, handle: FlatHandle, iters: usize) {
        self.map_mut_untracked(handle, |rb| {
            rb.set_additional_solver_iterations(iters);
        })
    }

    /// Retrieves the constant force(s) the user added to this rigid-body.
    /// Returns zero if the rigid-body is not dynamic.
    pub fn rbUserForce(&self, handle: FlatHandle) {
        self.map(handle, |rb| scratch::write_vector(rb.user_force()))
    }

    /// Retrieves the constant torque(s) the user added to this rigid-body.
    /// Returns zero if the rigid-body is not dynamic.
    #[cfg(feature = "dim2")]
    pub fn rbUserTorque(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |rb| rb.user_torque())
    }

    /// Retrieves the constant torque(s) the user added to this rigid-body.
    /// Returns zero if the rigid-body is not dynamic.
    #[cfg(feature = "dim3")]
    pub fn rbUserTorque(&self, handle: FlatHandle) {
        self.map(handle, |rb| scratch::write_vector(rb.user_torque()))
    }
}
