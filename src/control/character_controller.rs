use crate::dynamics::RawRigidBodySet;
use crate::geometry::{RawBroadPhase, RawColliderSet, RawNarrowPhase};
use crate::scratch;
use crate::utils::{self, FlatHandle};
use rapier::control::{
    CharacterAutostep, CharacterCollision, CharacterLength, EffectiveCharacterMovement,
    KinematicCharacterController,
};
use rapier::dynamics::RigidBodyHandle;
use rapier::math::{Real, Vector, DIM};
use rapier::parry::bounding_volume::BoundingVolume;
use rapier::pipeline::{QueryFilter, QueryFilterFlags};
use wasm_bindgen::prelude::*;

/// Scratch slots one character collision occupies: `1 + 6 * DIM` floats plus
/// the two `u32` halves of the hit collider's handle.
const CHARACTER_COLLISION_LEN: usize = 3 + 6 * DIM;

#[wasm_bindgen]
pub struct RawKinematicCharacterController {
    controller: KinematicCharacterController,
    result: EffectiveCharacterMovement,
    events: Vec<CharacterCollision>,
    /// Bodies the last `computeColliderMovement` may have applied an impulse
    /// to; kept around so the per-call collection does not allocate.
    pushed: Vec<RigidBodyHandle>,
}

fn length_value(length: CharacterLength) -> Real {
    match length {
        CharacterLength::Absolute(val) => val,
        CharacterLength::Relative(val) => val,
    }
}
#[wasm_bindgen]
impl RawKinematicCharacterController {
    #[wasm_bindgen(constructor)]
    pub fn new(offset: Real) -> Self {
        let controller = KinematicCharacterController {
            offset: CharacterLength::Absolute(offset),
            autostep: None,
            snap_to_ground: None,
            ..KinematicCharacterController::default()
        };

        Self {
            controller,
            result: EffectiveCharacterMovement {
                translation: Vector::ZERO,
                grounded: false,
                is_sliding_down_slope: false,
            },
            events: vec![],
            pushed: vec![],
        }
    }

    /// The up vector, written to the scratch buffer.
    pub fn up(&self) {
        scratch::write_vector(self.controller.up)
    }

    /// Sets the up direction, passed component-wise so JS allocates no `RawVector`
    /// (some games re-aim it every frame on curved ground).
    #[cfg(feature = "dim3")]
    pub fn setUp(&mut self, x: f32, y: f32, z: f32) {
        // Ignore a zero vector rather than setting a NaN up direction.
        if let Some(up) = Vector::new(x, y, z).try_normalize() {
            self.controller.up = up;
        }
    }

    /// Sets the up direction; see the 3D variant.
    #[cfg(feature = "dim2")]
    pub fn setUp(&mut self, x: f32, y: f32) {
        if let Some(up) = Vector::new(x, y).try_normalize() {
            self.controller.up = up;
        }
    }

    pub fn normalNudgeFactor(&self) -> Real {
        self.controller.normal_nudge_factor
    }

    pub fn setNormalNudgeFactor(&mut self, value: Real) {
        self.controller.normal_nudge_factor = value;
    }

    pub fn offset(&self) -> Real {
        length_value(self.controller.offset)
    }

    pub fn setOffset(&mut self, value: Real) {
        self.controller.offset = CharacterLength::Absolute(value);
    }

    pub fn slideEnabled(&self) -> bool {
        self.controller.slide
    }

    pub fn setSlideEnabled(&mut self, enabled: bool) {
        self.controller.slide = enabled
    }

    pub fn autostepMaxHeight(&self) -> Option<Real> {
        self.controller.autostep.map(|e| length_value(e.max_height))
    }

    pub fn autostepMinWidth(&self) -> Option<Real> {
        self.controller.autostep.map(|e| length_value(e.min_width))
    }

    pub fn autostepIncludesDynamicBodies(&self) -> Option<bool> {
        self.controller.autostep.map(|e| e.include_dynamic_bodies)
    }

    pub fn autostepEnabled(&self) -> bool {
        self.controller.autostep.is_some()
    }

    pub fn enableAutostep(&mut self, maxHeight: Real, minWidth: Real, includeDynamicBodies: bool) {
        self.controller.autostep = Some(CharacterAutostep {
            min_width: CharacterLength::Absolute(minWidth),
            max_height: CharacterLength::Absolute(maxHeight),
            include_dynamic_bodies: includeDynamicBodies,
        })
    }

    pub fn disableAutostep(&mut self) {
        self.controller.autostep = None;
    }

    pub fn maxSlopeClimbAngle(&self) -> Real {
        self.controller.max_slope_climb_angle
    }

    pub fn setMaxSlopeClimbAngle(&mut self, angle: Real) {
        self.controller.max_slope_climb_angle = angle;
    }

    pub fn minSlopeSlideAngle(&self) -> Real {
        self.controller.min_slope_slide_angle
    }

    pub fn setMinSlopeSlideAngle(&mut self, angle: Real) {
        self.controller.min_slope_slide_angle = angle
    }

    pub fn snapToGroundDistance(&self) -> Option<Real> {
        self.controller.snap_to_ground.map(length_value)
    }

    pub fn enableSnapToGround(&mut self, distance: Real) {
        self.controller.snap_to_ground = Some(CharacterLength::Absolute(distance));
    }

    pub fn disableSnapToGround(&mut self) {
        self.controller.snap_to_ground = None;
    }

    pub fn snapToGroundEnabled(&self) -> bool {
        self.controller.snap_to_ground.is_some()
    }

    /// See [`Self::do_compute_collider_movement`]; the desired translation is
    /// passed component-wise so the JS side allocates no `RawVector` per call.
    #[cfg(feature = "dim2")]
    pub fn computeColliderMovement(
        &mut self,
        dt: Real,
        broad_phase: &RawBroadPhase,
        narrow_phase: &RawNarrowPhase,
        bodies: &mut RawRigidBodySet,
        colliders: &mut RawColliderSet,
        collider_handle: FlatHandle,
        desired_translation_x: Real,
        desired_translation_y: Real,
        apply_impulses_to_dynamic_bodies: bool,
        character_mass: Option<Real>,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_predicate: &js_sys::Function,
    ) {
        self.do_compute_collider_movement(
            dt,
            broad_phase,
            narrow_phase,
            bodies,
            colliders,
            collider_handle,
            Vector::new(desired_translation_x, desired_translation_y),
            apply_impulses_to_dynamic_bodies,
            character_mass,
            filter_flags,
            filter_groups,
            filter_predicate,
        )
    }

    /// See [`Self::do_compute_collider_movement`]; the desired translation is
    /// passed component-wise so the JS side allocates no `RawVector` per call.
    #[cfg(feature = "dim3")]
    pub fn computeColliderMovement(
        &mut self,
        dt: Real,
        broad_phase: &RawBroadPhase,
        narrow_phase: &RawNarrowPhase,
        bodies: &mut RawRigidBodySet,
        colliders: &mut RawColliderSet,
        collider_handle: FlatHandle,
        desired_translation_x: Real,
        desired_translation_y: Real,
        desired_translation_z: Real,
        apply_impulses_to_dynamic_bodies: bool,
        character_mass: Option<Real>,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_predicate: &js_sys::Function,
    ) {
        self.do_compute_collider_movement(
            dt,
            broad_phase,
            narrow_phase,
            bodies,
            colliders,
            collider_handle,
            Vector::new(
                desired_translation_x,
                desired_translation_y,
                desired_translation_z,
            ),
            apply_impulses_to_dynamic_bodies,
            character_mass,
            filter_flags,
            filter_groups,
            filter_predicate,
        )
    }

    /// The movement computed by the last `computeColliderMovement` call, written to
    /// the scratch buffer.
    pub fn computedMovement(&self) {
        scratch::write_vector(self.result.translation);
    }

    pub fn computedGrounded(&self) -> bool {
        self.result.grounded
    }

    pub fn numComputedCollisions(&self) -> usize {
        self.events.len()
    }

    /// Writes the `i`-th collision of the last `computeColliderMovement` into the
    /// shared scratch buffer, or returns `false` if there is no such collision.
    ///
    /// Layout: `[toi, translationDeltaApplied, translationDeltaRemaining,
    /// worldWitness1, worldWitness2, worldNormal1, worldNormal2]` as floats
    /// (`1 + 6 * DIM` slots), then the hit collider's handle as its arena index
    /// and generation, two raw `u32` bit patterns JS reads back through a
    /// `Uint32Array` view. Witnesses and normals are all expressed in world-space.
    ///
    /// This used to copy the collision into a `RawCharacterCollision` first and
    /// read the handle back with a third call; the whole read is one crossing now.
    pub fn computedCollision(&self, i: usize) -> bool {
        let Some(coll) = self.events.get(i) else {
            return false;
        };

        let components = [
            coll.translation_applied,
            coll.translation_remaining,
            coll.hit.witness1, // Already in world-space.
            coll.character_pos * coll.hit.witness2,
            coll.hit.normal1, // Already in world-space.
            coll.character_pos.rotation * coll.hit.normal2,
        ];

        // Flattened on the stack, then written into WASM memory in one go — none of
        // it crosses the boundary.
        let mut flat = [0.0; CHARACTER_COLLISION_LEN];
        flat[0] = coll.hit.time_of_impact;

        for (i, u) in components.iter().enumerate() {
            flat[1 + i * DIM] = u.x;
            flat[2 + i * DIM] = u.y;
            #[cfg(feature = "dim3")]
            {
                flat[3 + i * DIM] = u.z;
            }
        }

        let handle = utils::flat_handle(coll.handle.0).to_bits();
        flat[1 + 6 * DIM] = scratch::u32_bits(handle as u32);
        flat[2 + 6 * DIM] = scratch::u32_bits((handle >> 32) as u32);

        scratch::write(&flat);
        true
    }
}

impl RawKinematicCharacterController {
    pub(crate) fn do_compute_collider_movement(
        &mut self,
        dt: Real,
        broad_phase: &RawBroadPhase,
        narrow_phase: &RawNarrowPhase,
        bodies: &mut RawRigidBodySet,
        colliders: &mut RawColliderSet,
        collider_handle: FlatHandle,
        desired_translation_delta: Vector,
        apply_impulses_to_dynamic_bodies: bool,
        character_mass: Option<Real>,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_predicate: &js_sys::Function,
    ) {
        let handle = crate::utils::collider_handle(collider_handle);
        if let Some(collider) = colliders.0.get(handle) {
            let collider_pose = *collider.position();
            let collider_shape = collider.shared_shape().clone();
            let collider_parent = collider.parent();

            crate::utils::with_filter(filter_predicate, |predicate| {
                let query_filter = QueryFilter {
                    flags: QueryFilterFlags::from_bits_truncate(filter_flags),
                    groups: filter_groups.map(crate::geometry::unpack_interaction_groups),
                    exclude_collider: Some(handle),
                    exclude_rigid_body: collider_parent,
                    predicate,
                };

                let character_mass = character_mass
                    .or_else(|| {
                        collider_parent
                            .and_then(|h| bodies.bodies.get(h))
                            .map(|b| b.mass())
                    })
                    .unwrap_or(0.0);

                let mut query_pipeline = broad_phase.0.as_query_pipeline_mut(
                    narrow_phase.narrow_phase.query_dispatcher(),
                    &mut bodies.bodies,
                    &mut colliders.0,
                    query_filter,
                );

                self.events.clear();
                let events = &mut self.events;
                self.result = self.controller.move_shape(
                    dt,
                    &query_pipeline.as_ref(),
                    &*collider_shape,
                    &collider_pose,
                    desired_translation_delta,
                    |event| events.push(event),
                );

                if apply_impulses_to_dynamic_bodies {
                    self.controller.solve_character_collision_impulses(
                        dt,
                        &mut query_pipeline,
                        &*collider_shape,
                        character_mass,
                        self.events.iter(),
                    );

                    // The impulses above went straight into rapier's bodies,
                    // bypassing `map_mut`, so their new velocities have to be
                    // published by hand below. Rapier does not only push the body
                    // behind each reported hit: for every collision it runs a
                    // contact query over the character's AABB (loosened by the
                    // same margin as here, see `solve_single_character_collision_impulse`)
                    // and applies an impulse to every dynamic body it finds, so
                    // the same query is repeated to know which bodies to publish.
                    // Over-collecting is harmless: a write-through of an
                    // untouched body is a no-op.
                    let queries = query_pipeline.as_ref();
                    let up_extent = collider_shape
                        .compute_local_aabb()
                        .extents()
                        .dot(self.controller.up.abs());
                    let prediction = match self.controller.offset {
                        CharacterLength::Absolute(offset) => offset,
                        CharacterLength::Relative(offset) => offset * up_extent,
                    } + 0.05;

                    self.pushed.clear();
                    for event in &self.events {
                        let aabb = collider_shape
                            .compute_aabb(&event.character_pos)
                            .loosened(prediction);
                        for (_, collider) in queries.intersect_aabb_conservative(aabb) {
                            if let Some(parent) = collider.parent() {
                                if queries.bodies.get(parent).is_some_and(|b| b.is_dynamic()) {
                                    self.pushed.push(parent);
                                }
                            }
                        }
                    }
                }
            });

            // Publish the new velocities so JS does not read the pre-impulse ones
            // out of the buffer until the next step.
            for &parent in &self.pushed {
                bodies.mark_pending(parent);
                bodies.write_through(parent);
            }
            self.pushed.clear();
        } else {
            // The collider is gone: report no movement, no contacts, not grounded,
            // rather than leaving the previous call's results in place.
            self.events.clear();
            self.result = EffectiveCharacterMovement {
                translation: Vector::ZERO,
                grounded: false,
                is_sliding_down_slope: false,
            };
        }
    }
}
