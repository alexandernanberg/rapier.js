use crate::dynamics::{RawImpulseJointSet, RawIslandManager, RawMultibodyJointSet};
use crate::geometry::RawColliderSet;
use crate::transform_buffer::{ArenaHandle, IndexSet, TransformBuffer};
use crate::utils::{self, FlatHandle};
use rapier::dynamics::{
    IslandManager, MassProperties, RigidBody, RigidBodyBuilder, RigidBodyHandle, RigidBodySet,
    RigidBodyType,
};
use rapier::math::{Pose, Rotation, Vector};
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
#[derive(Default)]
pub struct RawRigidBodySet {
    pub(crate) bodies: RigidBodySet,
    pub(crate) transforms: TransformBuffer<RigidBodyHandle>,
    /// Handles refreshed by the last sync, active ones first. Also drives the
    /// collider sync: a collider only moves when its parent body does.
    pub(crate) synced: Vec<RigidBodyHandle>,
    /// How many of the leading entries of `synced` came from the island manager's
    /// active set (the rest were carried over or mutated from JS).
    num_active: usize,
    /// The previous step's `synced`/`num_active`, swapped in each step so both
    /// allocations get reused.
    prev_synced: Vec<RigidBodyHandle>,
    prev_num_active: usize,
    /// Dedup set for `synced`, left empty between steps.
    synced_set: IndexSet,
}

impl RawRigidBodySet {
    /// Wraps an already-populated set (deserialization). The transform buffer
    /// starts empty, so the first sync fills it in one full pass.
    pub(crate) fn from_bodies(bodies: RigidBodySet) -> Self {
        Self {
            bodies,
            ..Default::default()
        }
    }

    /// Flags a body's buffered transform as stale.
    ///
    /// [`Self::map_mut`] covers every mutating accessor; this is for the few
    /// callers that reach into `bodies` directly (the PID and vehicle
    /// controllers). Pair it with [`Self::write_through`] once the mutation is
    /// done so JS reads the new state before the next step.
    pub(crate) fn mark_pending(&mut self, handle: RigidBodyHandle) {
        self.transforms.mark_pending(handle, self.bodies.len());
    }

    /// Publishes a body's current pose and velocity into its buffer slot.
    ///
    /// Rapier applies impulses, `sleep()` and body-type changes to the velocity
    /// immediately, and JS reads pose and velocity straight out of the buffer
    /// whenever it is live. Without this, `linvel()` right after `applyImpulse()`
    /// would return the pre-impulse value until the next `step()`.
    ///
    /// Only an existing slot is written: growing the buffer here could move it
    /// from under a live JS view. A body without a slot yet is still pending and
    /// gets written by the next sync.
    #[inline]
    pub(crate) fn write_through(&mut self, handle: RigidBodyHandle) {
        if let Some(body) = self.bodies.get(handle) {
            if let Some(slot) = self.transforms.existing_slot(handle.arena_index()) {
                write_body_transform(slot, body);
            }
        }
    }

    pub(crate) fn map<T>(&self, handle: FlatHandle, f: impl FnOnce(&RigidBody) -> T) -> T {
        let body = self.bodies.get(utils::body_handle(handle)).expect(
            "Invalid RigidBody reference. It may have been removed from the physics World.",
        );
        f(body)
    }

    /// Like [`Self::map_mut`], for mutations that cannot change anything the
    /// transform buffer holds (pose and velocities): forces, damping, CCD and
    /// dominance settings, mass properties, solver iterations.
    ///
    /// Routing those through `map_mut` would not be wrong, only wasteful: every
    /// call would rewrite the slot with unchanged values and, worse, add the body
    /// to the pending list. That list is capped at `max(64, len / 2)` entries
    /// before it gives up and schedules a full sync, so a scene applying a force
    /// to a hundred bodies every frame would lose the incremental sync entirely.
    pub(crate) fn map_mut_untracked<T>(
        &mut self,
        handle: FlatHandle,
        f: impl FnOnce(&mut RigidBody) -> T,
    ) -> T {
        let body = self.bodies.get_mut(utils::body_handle(handle)).expect(
            "Invalid RigidBody reference. It may have been removed from the physics World.",
        );
        f(body)
    }

    /// Records a freshly inserted body in the transform buffer.
    ///
    /// The slot is written immediately, growing the buffer if this is a new
    /// arena index. Growing may move the buffer, so the JS side re-reads
    /// `transformBufferInfo()` after every creation instead of falling back to
    /// per-body WASM calls until the next step — which used to make *every*
    /// body's reads cross the boundary for the rest of the frame whenever a
    /// single body was spawned.
    ///
    /// The body is still marked pending: a sync that is not a full pass has to
    /// know about it in case it is one the island manager never reports.
    fn publish_new_body(&mut self, handle: RigidBodyHandle) {
        self.transforms.mark_pending(handle, self.bodies.len());
        if let Some(body) = self.bodies.get(handle) {
            write_body_transform(self.transforms.slot(handle.arena_index()), body);
        }
    }

    pub(crate) fn map_mut<T>(
        &mut self,
        handle: FlatHandle,
        f: impl FnOnce(&mut RigidBody) -> T,
    ) -> T {
        let handle = utils::body_handle(handle);
        // Every mutating accessor goes through here, so this is the one place that
        // has to notice a body whose buffered transform (or velocity) was changed
        // from JS. A body mutated without waking up never shows up in the island
        // manager's active set, so the next sync would otherwise skip it.
        self.transforms.mark_pending(handle, self.bodies.len());

        let body = self.bodies.get_mut(handle).expect(
            "Invalid RigidBody reference. It may have been removed from the physics World.",
        );
        let result = f(body);

        // Publish the new state right away so the JS-side buffer reads stay
        // coherent with WASM until the next step refreshes the slot anyway.
        if let Some(slot) = self.transforms.existing_slot(handle.arena_index()) {
            write_body_transform(slot, body);
        }

        result
    }

    /// Collects the bodies whose buffered transform may have gone stale:
    ///
    /// - the bodies the island manager reports as active;
    /// - the bodies that were active during the *previous* step. An island is put
    ///   to sleep after its last pose was integrated, so a body that fell asleep
    ///   during this step is already gone from the active set by the time the sync
    ///   runs, and this is the only pass that will ever see that final pose;
    /// - the bodies JS created or mutated since the last sync.
    fn collect_synced(&mut self, islands: &IslandManager) {
        // Both lists are swapped rather than reallocated; only the leading
        // `prev_num_active` entries of `prev_synced` are last step's active set,
        // and only those need carrying over — carrying the whole list over would
        // make every entry immortal.
        core::mem::swap(&mut self.synced, &mut self.prev_synced);
        self.prev_num_active = self.num_active;
        self.synced.clear();

        for handle in islands.active_bodies() {
            // `active_bodies()` never repeats a handle; the insert is there to
            // seed the dedup set for the passes below.
            self.synced_set.insert(handle.arena_index());
            self.synced.push(handle);
        }
        self.num_active = self.synced.len();

        // Removed bodies have to be dropped here rather than skipped at write
        // time: `synced_set` is keyed by arena index, and an index outlives the
        // body that held it. A stale handle left in would take the dedup slot of
        // whatever new body recycled its index, and that body would then never be
        // written — permanently, if it is one the island manager never reports
        // (a fixed body, say).
        //
        // Indexed loops: `prev_synced` and `synced`/`synced_set` are disjoint
        // fields, but a `for` over one of them would still borrow all of `self`.
        for i in 0..self.prev_num_active {
            let handle = self.prev_synced[i];
            if self.bodies.contains(handle) && self.synced_set.insert(handle.arena_index()) {
                self.synced.push(handle);
            }
        }

        let pending = self.transforms.take_pending();
        for &handle in &pending {
            if self.bodies.contains(handle) && self.synced_set.insert(handle.arena_index()) {
                self.synced.push(handle);
            }
        }
        self.transforms.restore_pending(pending);

        // Leave the dedup set empty for the next step.
        for i in 0..self.synced.len() {
            self.synced_set.remove(self.synced[i].arena_index());
        }
    }

    /// Syncs rigid-body transforms into the contiguous buffer.
    ///
    /// Called internally from the physics pipeline step for cache locality.
    /// Not exposed via wasm-bindgen to avoid borrow tracking issues.
    ///
    /// Returns `true` if every slot was rewritten. Otherwise only the bodies in
    /// [`Self::synced`] were refreshed, and the collider sync can use that list
    /// to skip the colliders that cannot have moved.
    pub(crate) fn sync_transform_data(&mut self, islands: &IslandManager) -> bool {
        // A scattered pass pays a random arena lookup per body, so once most of
        // the set is moving the sequential walk is the cheaper one — and deciding
        // that is `O(1)`, so a scene where nothing ever sleeps never pays for a
        // refresh list it would only discard.
        if self.transforms.take_needs_full_sync()
            || mostly_moved(islands.num_active_bodies(), self.bodies.len())
        {
            // Every slot ends up correct, so the pending list is moot. The active
            // set still has to be recorded though: the *next* step needs it to
            // catch the bodies that fall asleep between now and then. Collecting
            // it is just a copy — none of the dedup work below.
            self.transforms.clear_pending();
            self.synced.clear();
            self.synced.extend(islands.active_bodies());
            self.num_active = self.synced.len();

            // Handles are iterated in increasing index order, so the buffer only
            // ever has to grow: a single pass can size it as it goes instead of
            // doing an extra pass just to find the highest index.
            for (handle, body) in self.bodies.iter() {
                write_body_transform(self.transforms.slot(handle.arena_index()), body);
            }
            return true;
        }

        self.collect_synced(islands);

        for i in 0..self.synced.len() {
            let handle = self.synced[i];
            let Some(body) = self.bodies.get(handle) else {
                continue;
            };
            write_body_transform(self.transforms.slot(handle.arena_index()), body);
        }

        false
    }
}

/// Whether a refresh list of `moved` entries covers enough of a `total`-entity
/// set that walking the whole arena in order beats looking each entry up.
#[inline]
pub(crate) fn mostly_moved(moved: usize, total: usize) -> bool {
    moved * 4 >= total * 3
}

/// Writes one body's pose and velocity into its buffer slot.
#[inline]
fn write_body_transform(slot: &mut [f32; BODY_STRIDE], body: &RigidBody) {
    let pos = body.position();
    let t = pos.translation;
    let lv = body.linvel();
    let av = body.angvel();

    #[cfg(feature = "dim3")]
    {
        let r = pos.rotation;
        *slot = [
            t.x, t.y, t.z, r.x, r.y, r.z, r.w, lv.x, lv.y, lv.z, av.x, av.y, av.z,
        ];
    }

    #[cfg(feature = "dim2")]
    {
        *slot = [t.x, t.y, pos.rotation.angle(), lv.x, lv.y, av];
    }
}

#[wasm_bindgen]
impl RawRigidBodySet {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Self::default()
    }

    /// Returns the transform buffer pointer and length packed into a single f64.
    /// Low 32 bits = byte offset in WASM memory, high 32 bits = f32 element count.
    pub fn transformBufferInfo(&self) -> f64 {
        self.transforms.info()
    }

    /// Returns the number of floats per body in the buffer.
    pub fn transformBufferStride(&self) -> usize {
        BODY_STRIDE
    }

    /// Creates a rigid-body from plain scalars.
    ///
    /// Vectors and rotations are passed component-wise instead of as `RawVector`/
    /// `RawRotation` handles: allocating those temporaries on the JS side costs
    /// far more than the extra arguments (each one is a WASM allocation plus a
    /// `FinalizationRegistry` registration).
    #[cfg(feature = "dim3")]
    pub fn createRigidBody(
        &mut self,
        enabled: bool,
        translation_x: f32,
        translation_y: f32,
        translation_z: f32,
        rotation_x: f32,
        rotation_y: f32,
        rotation_z: f32,
        rotation_w: f32,
        gravityScale: f32,
        mass: f32,
        massOnly: bool,
        centerOfMass_x: f32,
        centerOfMass_y: f32,
        centerOfMass_z: f32,
        linvel_x: f32,
        linvel_y: f32,
        linvel_z: f32,
        angvel_x: f32,
        angvel_y: f32,
        angvel_z: f32,
        principalAngularInertia_x: f32,
        principalAngularInertia_y: f32,
        principalAngularInertia_z: f32,
        angularInertiaFrame_x: f32,
        angularInertiaFrame_y: f32,
        angularInertiaFrame_z: f32,
        angularInertiaFrame_w: f32,
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
        let pos = Pose::from_parts(
            Vector::new(translation_x, translation_y, translation_z),
            // Same policy as `rbSetRotation`: a drifted quaternion is normalized,
            // a zero one falls back to the identity instead of skewing the pose.
            utils::unit_rotation(rotation_x, rotation_y, rotation_z, rotation_w)
                .unwrap_or(Rotation::IDENTITY),
        );

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
            .linvel(Vector::new(linvel_x, linvel_y, linvel_z))
            .angvel(Vector::new(angvel_x, angvel_y, angvel_z))
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
            rigid_body.additional_mass_properties(props)
        };

        let handle = self.bodies.insert(rigid_body.build());
        self.publish_new_body(handle);
        utils::flat_handle(handle.0)
    }

    /// Creates a rigid-body from plain scalars.
    ///
    /// See the 3D variant for why the components are passed individually.
    #[cfg(feature = "dim2")]
    pub fn createRigidBody(
        &mut self,
        enabled: bool,
        translation_x: f32,
        translation_y: f32,
        rotation_angle: f32,
        gravityScale: f32,
        mass: f32,
        massOnly: bool,
        centerOfMass_x: f32,
        centerOfMass_y: f32,
        linvel_x: f32,
        linvel_y: f32,
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
        let pos = Pose::from_parts(
            Vector::new(translation_x, translation_y),
            Rotation::new(rotation_angle),
        );
        let mut rigid_body = RigidBodyBuilder::new(rb_type.into())
            .enabled(enabled)
            .pose(pos)
            .gravity_scale(gravityScale)
            .enabled_translations(translationEnabledX, translationEnabledY)
            .linvel(Vector::new(linvel_x, linvel_y))
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
            let props = MassProperties::new(
                Vector::new(centerOfMass_x, centerOfMass_y).into(),
                mass,
                principalAngularInertia,
            );
            rigid_body.additional_mass_properties(props)
        };

        if !rotationsEnabled {
            rigid_body = rigid_body.lock_rotations();
        }

        let handle = self.bodies.insert(rigid_body.build());
        self.publish_new_body(handle);
        utils::flat_handle(handle.0)
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
        // The removed body's colliders go with it; release their arena indices
        // (and the body's) so recycled indices can be marked pending again.
        if let Some(body) = self.bodies.get(handle) {
            for &collider in body.colliders() {
                colliders.1.forget(collider);
            }
        }
        self.transforms.forget(handle);
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
