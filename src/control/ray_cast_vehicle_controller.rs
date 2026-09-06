use crate::dynamics::RawRigidBodySet;
use crate::geometry::{RawBroadPhase, RawColliderSet, RawNarrowPhase};
use crate::utils::{self, FlatHandle};
use rapier::control::{DynamicRayCastVehicleController, WheelTuning};
use rapier::math::{Real, DIM};
use rapier::pipeline::{QueryFilter, QueryFilterFlags};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct RawDynamicRayCastVehicleController {
    controller: DynamicRayCastVehicleController,
}

#[wasm_bindgen]
impl RawDynamicRayCastVehicleController {
    #[wasm_bindgen(constructor)]
    pub fn new(chassis: FlatHandle) -> Self {
        Self {
            controller: DynamicRayCastVehicleController::new(utils::body_handle(chassis)),
        }
    }

    pub fn current_vehicle_speed(&self) -> Real {
        self.controller.current_vehicle_speed
    }

    pub fn chassis(&self) -> FlatHandle {
        utils::flat_handle(self.controller.chassis.0)
    }

    pub fn index_up_axis(&self) -> usize {
        self.controller.index_up_axis
    }
    /// Sets the chassis' local up axis; an index outside `0..DIM` is ignored,
    /// since rapier would turn it into a zero up vector and `NaN` velocities.
    pub fn set_index_up_axis(&mut self, axis: usize) {
        if axis < DIM {
            self.controller.index_up_axis = axis;
        }
    }

    pub fn index_forward_axis(&self) -> usize {
        self.controller.index_forward_axis
    }
    /// Sets the chassis' local forward axis; see [`Self::set_index_up_axis`].
    pub fn set_index_forward_axis(&mut self, axis: usize) {
        if axis < DIM {
            self.controller.index_forward_axis = axis;
        }
    }

    /// The vectors are passed component-wise so JS allocates no `RawVector`.
    #[allow(clippy::too_many_arguments)]
    pub fn add_wheel(
        &mut self,
        chassis_connection_cs_x: Real,
        chassis_connection_cs_y: Real,
        chassis_connection_cs_z: Real,
        direction_cs_x: Real,
        direction_cs_y: Real,
        direction_cs_z: Real,
        axle_cs_x: Real,
        axle_cs_y: Real,
        axle_cs_z: Real,
        suspension_rest_length: Real,
        radius: Real,
    ) {
        self.controller.add_wheel(
            [
                chassis_connection_cs_x,
                chassis_connection_cs_y,
                chassis_connection_cs_z,
            ]
            .into(),
            [direction_cs_x, direction_cs_y, direction_cs_z].into(),
            [axle_cs_x, axle_cs_y, axle_cs_z].into(),
            suspension_rest_length,
            radius,
            &WheelTuning::default(),
        );
    }

    pub fn num_wheels(&self) -> usize {
        self.controller.wheels().len()
    }

    pub fn update_vehicle(
        &mut self,
        dt: Real,
        broad_phase: &RawBroadPhase,
        narrow_phase: &RawNarrowPhase,
        bodies: &mut RawRigidBodySet,
        colliders: &mut RawColliderSet,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_predicate: &js_sys::Function,
    ) {
        // rapier indexes the chassis unchecked, which on wasm32 is a module-wide
        // trap once the body has been removed from the world.
        if !bodies.bodies.contains(self.controller.chassis) {
            return;
        }

        // `update_vehicle` drives the chassis directly, bypassing `map_mut`.
        bodies.mark_pending(self.controller.chassis);

        crate::utils::with_filter(filter_predicate, |predicate| {
            let query_filter = QueryFilter {
                flags: QueryFilterFlags::from_bits_truncate(filter_flags),
                groups: filter_groups.map(crate::geometry::unpack_interaction_groups),
                predicate,
                exclude_rigid_body: Some(self.controller.chassis),
                exclude_collider: None,
            };

            let query_pipeline = broad_phase.0.as_query_pipeline_mut(
                narrow_phase.narrow_phase.query_dispatcher(),
                &mut bodies.bodies,
                &mut colliders.0,
                query_filter,
            );

            self.controller.update_vehicle(dt, query_pipeline);
        });

        bodies.write_through(self.controller.chassis);
    }

    /*
     *
     * Access to wheel properties.
     *
     */
    /*
     * Getters + setters
     */
    /// Written to the scratch buffer; returns `false` for an out-of-range wheel.
    pub fn wheel_chassis_connection_point_cs(&self, i: usize) -> bool {
        self.controller
            .wheels()
            .get(i)
            .map(|w| crate::scratch::write_vector(w.chassis_connection_point_cs.into()))
            .is_some()
    }
    pub fn set_wheel_chassis_connection_point_cs(&mut self, i: usize, x: Real, y: Real, z: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.chassis_connection_point_cs = [x, y, z].into();
        }
    }

    pub fn wheel_suspension_rest_length(&self, i: usize) -> Option<Real> {
        self.controller
            .wheels()
            .get(i)
            .map(|w| w.suspension_rest_length)
    }
    pub fn set_wheel_suspension_rest_length(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.suspension_rest_length = value;
        }
    }

    pub fn wheel_max_suspension_travel(&self, i: usize) -> Option<Real> {
        self.controller
            .wheels()
            .get(i)
            .map(|w| w.max_suspension_travel)
    }
    pub fn set_wheel_max_suspension_travel(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.max_suspension_travel = value;
        }
    }

    pub fn wheel_radius(&self, i: usize) -> Option<Real> {
        self.controller.wheels().get(i).map(|w| w.radius)
    }
    pub fn set_wheel_radius(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.radius = value;
        }
    }

    pub fn wheel_suspension_stiffness(&self, i: usize) -> Option<Real> {
        self.controller
            .wheels()
            .get(i)
            .map(|w| w.suspension_stiffness)
    }
    pub fn set_wheel_suspension_stiffness(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.suspension_stiffness = value;
        }
    }

    pub fn wheel_suspension_compression(&self, i: usize) -> Option<Real> {
        self.controller
            .wheels()
            .get(i)
            .map(|w| w.damping_compression)
    }
    pub fn set_wheel_suspension_compression(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.damping_compression = value;
        }
    }

    pub fn wheel_suspension_relaxation(&self, i: usize) -> Option<Real> {
        self.controller
            .wheels()
            .get(i)
            .map(|w| w.damping_relaxation)
    }
    pub fn set_wheel_suspension_relaxation(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.damping_relaxation = value;
        }
    }

    pub fn wheel_max_suspension_force(&self, i: usize) -> Option<Real> {
        self.controller
            .wheels()
            .get(i)
            .map(|w| w.max_suspension_force)
    }
    pub fn set_wheel_max_suspension_force(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.max_suspension_force = value;
        }
    }

    pub fn wheel_brake(&self, i: usize) -> Option<Real> {
        self.controller.wheels().get(i).map(|w| w.brake)
    }
    pub fn set_wheel_brake(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.brake = value;
        }
    }

    pub fn wheel_steering(&self, i: usize) -> Option<Real> {
        self.controller.wheels().get(i).map(|w| w.steering)
    }
    pub fn set_wheel_steering(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.steering = value;
        }
    }

    pub fn wheel_engine_force(&self, i: usize) -> Option<Real> {
        self.controller.wheels().get(i).map(|w| w.engine_force)
    }
    pub fn set_wheel_engine_force(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.engine_force = value;
        }
    }

    /// Written to the scratch buffer; returns `false` for an out-of-range wheel.
    pub fn wheel_direction_cs(&self, i: usize) -> bool {
        self.controller
            .wheels()
            .get(i)
            .map(|w| crate::scratch::write_vector(w.direction_cs.into()))
            .is_some()
    }
    pub fn set_wheel_direction_cs(&mut self, i: usize, x: Real, y: Real, z: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.direction_cs = [x, y, z].into();
        }
    }

    /// Written to the scratch buffer; returns `false` for an out-of-range wheel.
    pub fn wheel_axle_cs(&self, i: usize) -> bool {
        self.controller
            .wheels()
            .get(i)
            .map(|w| crate::scratch::write_vector(w.axle_cs.into()))
            .is_some()
    }
    pub fn set_wheel_axle_cs(&mut self, i: usize, x: Real, y: Real, z: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.axle_cs = [x, y, z].into();
        }
    }

    pub fn wheel_friction_slip(&self, i: usize) -> Option<Real> {
        self.controller.wheels().get(i).map(|w| w.friction_slip)
    }
    pub fn set_wheel_friction_slip(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.friction_slip = value;
        }
    }

    pub fn wheel_side_friction_stiffness(&self, i: usize) -> Option<f32> {
        self.controller
            .wheels()
            .get(i)
            .map(|w| w.side_friction_stiffness)
    }

    pub fn set_wheel_side_friction_stiffness(&mut self, i: usize, stiffness: f32) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.side_friction_stiffness = stiffness;
        }
    }

    /*
     * Getters only.
     */
    pub fn wheel_rotation(&self, i: usize) -> Option<Real> {
        self.controller.wheels().get(i).map(|w| w.rotation)
    }

    pub fn wheel_forward_impulse(&self, i: usize) -> Option<Real> {
        self.controller.wheels().get(i).map(|w| w.forward_impulse)
    }

    pub fn wheel_side_impulse(&self, i: usize) -> Option<Real> {
        self.controller.wheels().get(i).map(|w| w.side_impulse)
    }

    pub fn wheel_suspension_force(&self, i: usize) -> Option<Real> {
        self.controller
            .wheels()
            .get(i)
            .map(|w| w.wheel_suspension_force)
    }

    /// Written to the scratch buffer; returns `false` for an out-of-range wheel.
    pub fn wheel_contact_normal_ws(&self, i: usize) -> bool {
        self.controller
            .wheels()
            .get(i)
            .map(|w| crate::scratch::write_vector(w.raycast_info().contact_normal_ws.into()))
            .is_some()
    }

    /// Written to the scratch buffer; returns `false` for an out-of-range wheel.
    pub fn wheel_contact_point_ws(&self, i: usize) -> bool {
        self.controller
            .wheels()
            .get(i)
            .map(|w| crate::scratch::write_vector(w.raycast_info().contact_point_ws.into()))
            .is_some()
    }

    pub fn wheel_suspension_length(&self, i: usize) -> Option<Real> {
        self.controller
            .wheels()
            .get(i)
            .map(|w| w.raycast_info().suspension_length)
    }

    /// Written to the scratch buffer; returns `false` for an out-of-range wheel.
    pub fn wheel_hard_point_ws(&self, i: usize) -> bool {
        self.controller
            .wheels()
            .get(i)
            .map(|w| crate::scratch::write_vector(w.raycast_info().hard_point_ws.into()))
            .is_some()
    }

    pub fn wheel_is_in_contact(&self, i: usize) -> bool {
        self.controller
            .wheels()
            .get(i)
            .map(|w| w.raycast_info().is_in_contact)
            .unwrap_or(false)
    }

    pub fn wheel_ground_object(&self, i: usize) -> Option<FlatHandle> {
        self.controller
            .wheels()
            .get(i)
            .and_then(|w| w.raycast_info().ground_object)
            .map(|h| utils::flat_handle(h.0))
    }
}
