use crate::dynamics::{
    RawCCDSolver, RawImpulseJointSet, RawIntegrationParameters, RawIslandManager,
    RawMultibodyJointSet, RawRigidBodySet,
};
use crate::geometry::{RawBroadPhase, RawColliderSet, RawNarrowPhase};
use crate::math::RawVector;
use crate::pipeline::{RawEventQueue, RawPhysicsHooks};
use crate::rapier::pipeline::PhysicsPipeline;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct RawPhysicsPipeline(pub(crate) PhysicsPipeline);

/// Refreshes both transform buffers after a step. The body pass runs first and
/// tells the collider pass which bodies moved, if it kept track.
fn sync_transforms(
    islands: &RawIslandManager,
    bodies: &mut RawRigidBodySet,
    colliders: &mut RawColliderSet,
) {
    let synced_all_bodies = bodies.sync_transform_data(&islands.0, colliders.0.len());
    let moved_bodies = if synced_all_bodies {
        None
    } else {
        Some(bodies.synced.as_slice())
    };
    colliders.sync_transform_data(&bodies.bodies, moved_bodies);
}

#[wasm_bindgen]
impl RawPhysicsPipeline {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        let mut pipeline = PhysicsPipeline::new();
        pipeline.counters.disable(); // Disable perf counters by default.
        RawPhysicsPipeline(pipeline)
    }

    pub fn set_profiler_enabled(&mut self, enabled: bool) {
        if enabled {
            self.0.counters.enable();
        } else {
            self.0.counters.disable();
        }
    }

    pub fn is_profiler_enabled(&self) -> bool {
        self.0.counters.enabled()
    }

    pub fn timing_step(&self) -> f64 {
        self.0.counters.step_time_ms()
    }

    pub fn timing_collision_detection(&self) -> f64 {
        self.0.counters.collision_detection_time_ms()
    }

    pub fn timing_broad_phase(&self) -> f64 {
        self.0.counters.broad_phase_time_ms()
    }

    pub fn timing_narrow_phase(&self) -> f64 {
        self.0.counters.narrow_phase_time_ms()
    }

    pub fn timing_solver(&self) -> f64 {
        self.0.counters.solver_time_ms()
    }

    pub fn timing_velocity_assembly(&self) -> f64 {
        self.0.counters.solver.velocity_assembly_time.time_ms()
    }

    pub fn timing_velocity_resolution(&self) -> f64 {
        self.0.counters.velocity_resolution_time_ms()
    }

    pub fn timing_velocity_update(&self) -> f64 {
        self.0.counters.velocity_update_time_ms()
    }

    pub fn timing_velocity_writeback(&self) -> f64 {
        self.0.counters.solver.velocity_writeback_time.time_ms()
    }

    pub fn timing_ccd(&self) -> f64 {
        self.0.counters.ccd_time_ms()
    }

    pub fn timing_ccd_toi_computation(&self) -> f64 {
        self.0.counters.ccd.toi_computation_time.time_ms()
    }

    pub fn timing_ccd_broad_phase(&self) -> f64 {
        self.0.counters.ccd.broad_phase_time.time_ms()
    }

    pub fn timing_ccd_narrow_phase(&self) -> f64 {
        self.0.counters.ccd.narrow_phase_time.time_ms()
    }

    pub fn timing_ccd_solver(&self) -> f64 {
        self.0.counters.ccd.solver_time.time_ms()
    }

    pub fn timing_island_construction(&self) -> f64 {
        self.0.counters.island_construction_time_ms()
    }

    pub fn timing_user_changes(&self) -> f64 {
        self.0.counters.stages.user_changes.time_ms()
    }

    pub fn step(
        &mut self,
        gravity: &RawVector,
        integrationParameters: &RawIntegrationParameters,
        islands: &mut RawIslandManager,
        broadPhase: &mut RawBroadPhase,
        narrowPhase: &mut RawNarrowPhase,
        bodies: &mut RawRigidBodySet,
        colliders: &mut RawColliderSet,
        joints: &mut RawImpulseJointSet,
        articulations: &mut RawMultibodyJointSet,
        ccd_solver: &mut RawCCDSolver,
    ) {
        self.0.step(
            gravity.0,
            &integrationParameters.0,
            &mut islands.0,
            &mut broadPhase.0,
            &mut narrowPhase.narrow_phase,
            &mut bodies.bodies,
            &mut colliders.0,
            &mut joints.0,
            &mut articulations.0,
            &mut ccd_solver.0,
            &(),
            &(),
        );

        sync_transforms(islands, bodies, colliders);
    }

    /// Steps with physics hooks but without an event queue.
    ///
    /// Kept separate from `step` so the common hookless path does not pay for
    /// marshalling the three hook values across the boundary on every step.
    pub fn stepWithHooks(
        &mut self,
        gravity: &RawVector,
        integrationParameters: &RawIntegrationParameters,
        islands: &mut RawIslandManager,
        broadPhase: &mut RawBroadPhase,
        narrowPhase: &mut RawNarrowPhase,
        bodies: &mut RawRigidBodySet,
        colliders: &mut RawColliderSet,
        joints: &mut RawImpulseJointSet,
        articulations: &mut RawMultibodyJointSet,
        ccd_solver: &mut RawCCDSolver,
        hookObject: js_sys::Object,
        hookFilterContactPair: Option<js_sys::Function>,
        hookFilterIntersectionPair: Option<js_sys::Function>,
        hookModifySolverContacts: Option<js_sys::Function>,
    ) {
        let hooks = RawPhysicsHooks {
            this: hookObject,
            filter_contact_pair: hookFilterContactPair,
            filter_intersection_pair: hookFilterIntersectionPair,
            modify_solver_contacts: hookModifySolverContacts,
        };

        self.0.step(
            gravity.0,
            &integrationParameters.0,
            &mut islands.0,
            &mut broadPhase.0,
            &mut narrowPhase.narrow_phase,
            &mut bodies.bodies,
            &mut colliders.0,
            &mut joints.0,
            &mut articulations.0,
            &mut ccd_solver.0,
            &hooks,
            &(),
        );

        sync_transforms(islands, bodies, colliders);
    }

    /// Steps with an event queue but without physics hooks: the common case
    /// for anyone consuming events, which used to marshal a hook object and
    /// three absent functions across the boundary on every step and hand
    /// rapier a hooks object that answered "no hook" for every flagged pair.
    pub fn stepWithEvents(
        &mut self,
        gravity: &RawVector,
        integrationParameters: &RawIntegrationParameters,
        islands: &mut RawIslandManager,
        broadPhase: &mut RawBroadPhase,
        narrowPhase: &mut RawNarrowPhase,
        bodies: &mut RawRigidBodySet,
        colliders: &mut RawColliderSet,
        joints: &mut RawImpulseJointSet,
        articulations: &mut RawMultibodyJointSet,
        ccd_solver: &mut RawCCDSolver,
        eventQueue: &mut RawEventQueue,
    ) {
        if eventQueue.auto_drain {
            eventQueue.clear();
        }

        self.0.step(
            gravity.0,
            &integrationParameters.0,
            &mut islands.0,
            &mut broadPhase.0,
            &mut narrowPhase.narrow_phase,
            &mut bodies.bodies,
            &mut colliders.0,
            &mut joints.0,
            &mut articulations.0,
            &mut ccd_solver.0,
            &(),
            &*eventQueue,
        );

        sync_transforms(islands, bodies, colliders);
    }

    /// Steps with both an event queue and physics hooks.
    pub fn stepWithEventsAndHooks(
        &mut self,
        gravity: &RawVector,
        integrationParameters: &RawIntegrationParameters,
        islands: &mut RawIslandManager,
        broadPhase: &mut RawBroadPhase,
        narrowPhase: &mut RawNarrowPhase,
        bodies: &mut RawRigidBodySet,
        colliders: &mut RawColliderSet,
        joints: &mut RawImpulseJointSet,
        articulations: &mut RawMultibodyJointSet,
        ccd_solver: &mut RawCCDSolver,
        eventQueue: &mut RawEventQueue,
        hookObject: js_sys::Object,
        hookFilterContactPair: Option<js_sys::Function>,
        hookFilterIntersectionPair: Option<js_sys::Function>,
        hookModifySolverContacts: Option<js_sys::Function>,
    ) {
        if eventQueue.auto_drain {
            eventQueue.clear();
        }

        let hooks = RawPhysicsHooks {
            this: hookObject,
            filter_contact_pair: hookFilterContactPair,
            filter_intersection_pair: hookFilterIntersectionPair,
            modify_solver_contacts: hookModifySolverContacts,
        };

        self.0.step(
            gravity.0,
            &integrationParameters.0,
            &mut islands.0,
            &mut broadPhase.0,
            &mut narrowPhase.narrow_phase,
            &mut bodies.bodies,
            &mut colliders.0,
            &mut joints.0,
            &mut articulations.0,
            &mut ccd_solver.0,
            &hooks,
            &*eventQueue,
        );

        sync_transforms(islands, bodies, colliders);
    }
}
