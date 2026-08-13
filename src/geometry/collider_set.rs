use crate::dynamics::{RawIslandManager, RawRigidBodySet};
use crate::geometry::RawShape;
use crate::utils::{self, FlatHandle};
use rapier::math::Pose;
use rapier::prelude::*;
use wasm_bindgen::prelude::*;

// NOTE: this MUST match the same enum on the TS side.
enum MassPropsMode {
    Density = 0,
    Mass,
    MassProps,
}

/// Number of f32 values per collider in the world-space transform buffer.
#[cfg(feature = "dim3")]
const COLLIDER_STRIDE: usize = 7; // translation(3) + rotation(4)
#[cfg(feature = "dim2")]
const COLLIDER_STRIDE: usize = 3; // translation(2) + rotation(1)

#[wasm_bindgen]
pub struct RawColliderSet(
    pub(crate) ColliderSet,
    /// Contiguous world-space transform buffer, read directly from JS.
    pub(crate) Vec<f32>,
);

impl RawColliderSet {
    pub(crate) fn map<T>(&self, handle: FlatHandle, f: impl FnOnce(&Collider) -> T) -> T {
        let collider = self
            .0
            .get(utils::collider_handle(handle))
            .expect("Invalid Collider reference. It may have been removed from the physics World.");
        f(collider)
    }

    pub(crate) fn map_mut<T>(
        &mut self,
        handle: FlatHandle,
        f: impl FnOnce(&mut Collider) -> T,
    ) -> T {
        let collider = self
            .0
            .get_mut(utils::collider_handle(handle))
            .expect("Invalid Collider reference. It may have been removed from the physics World.");
        f(collider)
    }

    pub(crate) fn map_pair_mut<T>(
        &mut self,
        handle1: FlatHandle,
        handle2: FlatHandle,
        f: impl FnOnce(Option<&mut Collider>, Option<&mut Collider>) -> T,
    ) -> T {
        let (collider1, collider2) = self.0.get_pair_mut(
            utils::collider_handle(handle1),
            utils::collider_handle(handle2),
        );
        f(collider1, collider2)
    }

    /// Writes a single collider's world-space transform into `buf` at `offset`.
    #[inline]
    fn write_collider_transform(buf: &mut [f32], offset: usize, collider: &Collider) {
        let pos = collider.position();

        #[cfg(feature = "dim3")]
        {
            let t = pos.translation;
            let r = pos.rotation;
            buf[offset] = t.x;
            buf[offset + 1] = t.y;
            buf[offset + 2] = t.z;
            buf[offset + 3] = r.x;
            buf[offset + 4] = r.y;
            buf[offset + 5] = r.z;
            buf[offset + 6] = r.w;
        }

        #[cfg(feature = "dim2")]
        {
            let t = pos.translation;
            let r = pos.rotation.angle();
            buf[offset] = t.x;
            buf[offset + 1] = t.y;
            buf[offset + 2] = r;
        }
    }

    /// Syncs all collider world transforms into the contiguous buffer, resizing
    /// it to fit the highest collider index.
    ///
    /// Called internally from the physics pipeline step for cache locality.
    /// Not exposed via wasm-bindgen to avoid borrow tracking issues. The JS side
    /// reads world translations/rotations directly from this buffer, falling back
    /// to per-collider WASM calls whenever the view is invalidated (a collider was
    /// created or mutated) or detached (WASM memory growth).
    pub(crate) fn sync_transform_data(&mut self) {
        // Handles are iterated in increasing index order, so the buffer only ever
        // has to grow: a single pass can size it as it goes instead of doing an
        // extra pass just to find the highest index.
        for (handle, collider) in self.0.iter() {
            let (index, _) = handle.0.into_raw_parts();
            let offset = index as usize * COLLIDER_STRIDE;
            if self.1.len() < offset + COLLIDER_STRIDE {
                self.1.resize(offset + COLLIDER_STRIDE, 0.0);
            }
            Self::write_collider_transform(&mut self.1, offset, collider);
        }
    }
}

impl RawColliderSet {
    // This is a workaround because wasm-bindgen doesn't support the `cfg(feature = ...)`
    // for the method arguments.
    pub fn do_create_collider(
        &mut self,
        enabled: bool,
        shape: &RawShape,
        translation_x: f32,
        translation_y: f32,
        #[cfg(feature = "dim3")] translation_z: f32,
        #[cfg(feature = "dim2")] rotation_angle: f32,
        #[cfg(feature = "dim3")] rotation_x: f32,
        #[cfg(feature = "dim3")] rotation_y: f32,
        #[cfg(feature = "dim3")] rotation_z: f32,
        #[cfg(feature = "dim3")] rotation_w: f32,
        massPropsMode: u32,
        mass: f32,
        centerOfMass_x: f32,
        centerOfMass_y: f32,
        #[cfg(feature = "dim3")] centerOfMass_z: f32,
        #[cfg(feature = "dim2")] principalAngularInertia: f32,
        #[cfg(feature = "dim3")] principalAngularInertia_x: f32,
        #[cfg(feature = "dim3")] principalAngularInertia_y: f32,
        #[cfg(feature = "dim3")] principalAngularInertia_z: f32,
        #[cfg(feature = "dim3")] angularInertiaFrame_x: f32,
        #[cfg(feature = "dim3")] angularInertiaFrame_y: f32,
        #[cfg(feature = "dim3")] angularInertiaFrame_z: f32,
        #[cfg(feature = "dim3")] angularInertiaFrame_w: f32,
        density: f32,
        friction: f32,
        restitution: f32,
        frictionCombineRule: u32,
        restitutionCombineRule: u32,
        isSensor: bool,
        collisionGroups: u32,
        solverGroups: u32,
        activeCollisionTypes: u16,
        activeHooks: u32,
        activeEvents: u32,
        contactForceEventThreshold: f32,
        contactSkin: f32,
        hasParent: bool,
        parent: FlatHandle,
        bodies: &mut RawRigidBodySet,
    ) -> Option<FlatHandle> {
        #[cfg(feature = "dim2")]
        let pos = Pose::from_parts(
            Vector::new(translation_x, translation_y),
            Rotation::new(rotation_angle),
        );
        #[cfg(feature = "dim3")]
        let pos = Pose::from_parts(
            Vector::new(translation_x, translation_y, translation_z),
            Rotation::from_xyzw(rotation_x, rotation_y, rotation_z, rotation_w),
        );

        let mut builder = ColliderBuilder::new(shape.0.clone())
            .enabled(enabled)
            .position(pos)
            .friction(friction)
            .restitution(restitution)
            .collision_groups(super::unpack_interaction_groups(collisionGroups))
            .solver_groups(super::unpack_interaction_groups(solverGroups))
            .active_hooks(ActiveHooks::from_bits(activeHooks).unwrap_or(ActiveHooks::empty()))
            .active_events(ActiveEvents::from_bits(activeEvents).unwrap_or(ActiveEvents::empty()))
            .active_collision_types(
                ActiveCollisionTypes::from_bits(activeCollisionTypes)
                    .unwrap_or(ActiveCollisionTypes::empty()),
            )
            .sensor(isSensor)
            .friction_combine_rule(super::combine_rule_from_u32(frictionCombineRule))
            .restitution_combine_rule(super::combine_rule_from_u32(restitutionCombineRule))
            .contact_force_event_threshold(contactForceEventThreshold)
            .contact_skin(contactSkin);

        if massPropsMode == MassPropsMode::MassProps as u32 {
            #[cfg(feature = "dim2")]
            let mprops = MassProperties::new(
                Vector::new(centerOfMass_x, centerOfMass_y).into(),
                mass,
                principalAngularInertia,
            );
            #[cfg(feature = "dim3")]
            let mprops = MassProperties::with_principal_inertia_frame(
                Vector::new(centerOfMass_x, centerOfMass_y, centerOfMass_z).into(),
                mass,
                Vector::new(
                    principalAngularInertia_x,
                    principalAngularInertia_y,
                    principalAngularInertia_z,
                ),
                Rotation::from_xyzw(
                    angularInertiaFrame_x,
                    angularInertiaFrame_y,
                    angularInertiaFrame_z,
                    angularInertiaFrame_w,
                ),
            );
            builder = builder.mass_properties(mprops);
        } else if massPropsMode == MassPropsMode::Density as u32 {
            builder = builder.density(density);
        } else {
            assert_eq!(massPropsMode, MassPropsMode::Mass as u32);
            builder = builder.mass(mass);
        };

        let collider = builder.build();

        if hasParent {
            Some(utils::flat_handle(
                self.0
                    .insert_with_parent(collider, utils::body_handle(parent), &mut bodies.bodies)
                    .0,
            ))
        } else {
            Some(utils::flat_handle(self.0.insert(collider).0))
        }
    }
}

#[wasm_bindgen]
impl RawColliderSet {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        RawColliderSet(ColliderSet::new(), Vec::new())
    }

    /// Returns the transform buffer pointer and length packed into a single f64.
    /// Low 32 bits = byte offset in WASM memory, high 32 bits = f32 element count.
    pub fn transformBufferInfo(&self) -> f64 {
        let ptr = self.1.as_ptr() as u32;
        let len = self.1.len() as u32;
        f64::from_bits(ptr as u64 | ((len as u64) << 32))
    }

    /// Returns the number of floats per collider in the buffer.
    pub fn transformBufferStride(&self) -> usize {
        COLLIDER_STRIDE
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn contains(&self, handle: FlatHandle) -> bool {
        self.0.get(utils::collider_handle(handle)).is_some()
    }

    /// Creates a collider from plain scalars.
    ///
    /// Vectors and rotations are passed component-wise instead of as `RawVector`/
    /// `RawRotation` handles: allocating those temporaries on the JS side costs
    /// far more than the extra arguments (each one is a WASM allocation plus a
    /// `FinalizationRegistry` registration).
    #[cfg(feature = "dim2")]
    pub fn createCollider(
        &mut self,
        enabled: bool,
        shape: &RawShape,
        translation_x: f32,
        translation_y: f32,
        rotation_angle: f32,
        massPropsMode: u32,
        mass: f32,
        centerOfMass_x: f32,
        centerOfMass_y: f32,
        principalAngularInertia: f32,
        density: f32,
        friction: f32,
        restitution: f32,
        frictionCombineRule: u32,
        restitutionCombineRule: u32,
        isSensor: bool,
        collisionGroups: u32,
        solverGroups: u32,
        activeCollisionTypes: u16,
        activeHooks: u32,
        activeEvents: u32,
        contactForceEventThreshold: f32,
        contactSkin: f32,
        hasParent: bool,
        parent: FlatHandle,
        bodies: &mut RawRigidBodySet,
    ) -> Option<FlatHandle> {
        self.do_create_collider(
            enabled,
            shape,
            translation_x,
            translation_y,
            rotation_angle,
            massPropsMode,
            mass,
            centerOfMass_x,
            centerOfMass_y,
            principalAngularInertia,
            density,
            friction,
            restitution,
            frictionCombineRule,
            restitutionCombineRule,
            isSensor,
            collisionGroups,
            solverGroups,
            activeCollisionTypes,
            activeHooks,
            activeEvents,
            contactForceEventThreshold,
            contactSkin,
            hasParent,
            parent,
            bodies,
        )
    }

    /// Creates a collider from plain scalars.
    ///
    /// See the 2D variant for why the components are passed individually.
    #[cfg(feature = "dim3")]
    pub fn createCollider(
        &mut self,
        enabled: bool,
        shape: &RawShape,
        translation_x: f32,
        translation_y: f32,
        translation_z: f32,
        rotation_x: f32,
        rotation_y: f32,
        rotation_z: f32,
        rotation_w: f32,
        massPropsMode: u32,
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
        density: f32,
        friction: f32,
        restitution: f32,
        frictionCombineRule: u32,
        restitutionCombineRule: u32,
        isSensor: bool,
        collisionGroups: u32,
        solverGroups: u32,
        activeCollisionTypes: u16,
        activeHooks: u32,
        activeEvents: u32,
        contactForceEventThreshold: f32,
        contactSkin: f32,
        hasParent: bool,
        parent: FlatHandle,
        bodies: &mut RawRigidBodySet,
    ) -> Option<FlatHandle> {
        self.do_create_collider(
            enabled,
            shape,
            translation_x,
            translation_y,
            translation_z,
            rotation_x,
            rotation_y,
            rotation_z,
            rotation_w,
            massPropsMode,
            mass,
            centerOfMass_x,
            centerOfMass_y,
            centerOfMass_z,
            principalAngularInertia_x,
            principalAngularInertia_y,
            principalAngularInertia_z,
            angularInertiaFrame_x,
            angularInertiaFrame_y,
            angularInertiaFrame_z,
            angularInertiaFrame_w,
            density,
            friction,
            restitution,
            frictionCombineRule,
            restitutionCombineRule,
            isSensor,
            collisionGroups,
            solverGroups,
            activeCollisionTypes,
            activeHooks,
            activeEvents,
            contactForceEventThreshold,
            contactSkin,
            hasParent,
            parent,
            bodies,
        )
    }

    /// Removes a collider from this set and wake-up the rigid-body it is attached to.
    pub fn remove(
        &mut self,
        handle: FlatHandle,
        islands: &mut RawIslandManager,
        bodies: &mut RawRigidBodySet,
        wakeUp: bool,
    ) {
        let handle = utils::collider_handle(handle);
        self.0
            .remove(handle, &mut islands.0, &mut bodies.bodies, wakeUp);
    }

    /// Checks if a collider with the given integer handle exists.
    pub fn isHandleValid(&self, handle: FlatHandle) -> bool {
        self.0.get(utils::collider_handle(handle)).is_some()
    }

    /// Applies the given JavaScript function to the integer handle of each collider managed by this collider set.
    ///
    /// # Parameters
    /// - `f(handle)`: the function to apply to the integer handle of each collider managed by this collider set. Called as `f(handle)`.
    pub fn forEachColliderHandle(&self, f: &js_sys::Function) {
        let this = JsValue::null();
        for (handle, _) in self.0.iter() {
            let _ = f.call1(&this, &JsValue::from(utils::flat_handle(handle.0)));
        }
    }
}
