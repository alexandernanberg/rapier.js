use crate::dynamics::RawRigidBodySet;
use crate::geometry::feature::IntoTypeValue;
use crate::geometry::{RawColliderSet, RawColliderShapeCastHit, RawNarrowPhase, RawShape};
use crate::math::{RawRotation, RawVector};
use crate::utils::{self, FlatHandle};
use rapier::geometry::DefaultBroadPhase;
use rapier::geometry::{Aabb, ColliderHandle, Ray, RayIntersection};
use rapier::math::{Pose, Real, Vector};
use rapier::parry::query::{PointProjection, ShapeCastOptions};
use rapier::pipeline::{QueryFilter, QueryFilterFlags};
use rapier::prelude::FeatureId;
use std::cell::Cell;
use wasm_bindgen::prelude::*;

/// Number of `f64` slots in the scratch buffer used to hand query results back
/// to JS. The largest payload is a ray intersection:
/// `handle, timeOfImpact, normal (2 or 3), featureType, featureId`.
const QUERY_RESULT_LEN: usize = 8;

#[wasm_bindgen]
pub struct RawBroadPhase(
    pub(crate) DefaultBroadPhase,
    /// Scratch buffer read directly from JS, so that a query result doesn't need
    /// a wasm-bindgen object (an allocation plus a `FinalizationRegistry`
    /// registration on the JS side) per call.
    ///
    /// It is allocated once with its final capacity, so its address never
    /// changes and the JS-side view only ever has to be re-created when WASM
    /// memory growth detaches it. The `Cell`s let the queries write their result
    /// through a shared reference, so they can keep taking `&self`: a `&mut self`
    /// would make wasm-bindgen reject any re-entrant call on the broad-phase,
    /// including a query issued from inside a filter or hit callback.
    Vec<Cell<f64>>,
);

impl RawBroadPhase {
    pub(crate) fn from_broad_phase(broad_phase: DefaultBroadPhase) -> Self {
        RawBroadPhase(broad_phase, vec![Cell::new(0.0); QUERY_RESULT_LEN])
    }

    /// Writes `handle, timeOfImpact` into the result buffer.
    #[inline]
    fn fill_ray_hit(buf: &[Cell<f64>], handle: ColliderHandle, time_of_impact: Real) {
        buf[0].set(utils::flat_handle(handle.0));
        buf[1].set(time_of_impact as f64);
    }

    /// Writes `handle, timeOfImpact, normal, featureType, featureId` into the
    /// result buffer. A missing feature id is written as `-1`.
    #[inline]
    fn fill_ray_intersection(buf: &[Cell<f64>], handle: ColliderHandle, inter: &RayIntersection) {
        buf[0].set(utils::flat_handle(handle.0));
        buf[1].set(inter.time_of_impact as f64);
        buf[2].set(inter.normal.x as f64);
        buf[3].set(inter.normal.y as f64);
        #[cfg(feature = "dim2")]
        let next = 4;
        #[cfg(feature = "dim3")]
        let next = {
            buf[4].set(inter.normal.z as f64);
            5
        };
        buf[next].set(inter.feature.into_type() as u32 as f64);
        buf[next + 1].set(
            inter
                .feature
                .into_value()
                .map(|id| id as f64)
                .unwrap_or(-1.0),
        );
    }

    /// Writes `handle, point, isInside, featureType, featureId` into the result
    /// buffer. A missing feature id is written as `-1`.
    #[inline]
    fn fill_point_projection(
        buf: &[Cell<f64>],
        handle: ColliderHandle,
        proj: &PointProjection,
        feature: FeatureId,
    ) {
        buf[0].set(utils::flat_handle(handle.0));
        buf[1].set(proj.point.x as f64);
        buf[2].set(proj.point.y as f64);
        #[cfg(feature = "dim2")]
        let next = 3;
        #[cfg(feature = "dim3")]
        let next = {
            buf[3].set(proj.point.z as f64);
            4
        };
        buf[next].set(proj.is_inside as u32 as f64);
        buf[next + 1].set(feature.into_type() as u32 as f64);
        buf[next + 2].set(feature.into_value().map(|id| id as f64).unwrap_or(-1.0));
    }
}

#[wasm_bindgen]
impl RawBroadPhase {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Self::from_broad_phase(DefaultBroadPhase::new())
    }

    /// Returns the query result buffer pointer and length packed into a single f64.
    /// Low 32 bits = byte offset in WASM memory, high 32 bits = f64 element count.
    pub fn queryResultBufferInfo(&self) -> f64 {
        let ptr = self.1.as_ptr() as u32;
        let len = self.1.len() as u32;
        f64::from_bits(ptr as u64 | ((len as u64) << 32))
    }

    #[cfg(feature = "dim2")]
    #[allow(clippy::too_many_arguments)]
    pub fn castRay(
        &self,
        narrow_phase: &RawNarrowPhase,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        ray_ox: f32,
        ray_oy: f32,
        ray_dx: f32,
        ray_dy: f32,
        maxToi: f32,
        solid: bool,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_exclude_collider: Option<FlatHandle>,
        filter_exclude_rigid_body: Option<FlatHandle>,
        filter_predicate: &js_sys::Function,
    ) -> bool {
        let Some((handle, timeOfImpact)) = utils::with_filter(filter_predicate, |predicate| {
            let query_filter = QueryFilter {
                flags: QueryFilterFlags::from_bits(filter_flags)
                    .unwrap_or(QueryFilterFlags::empty()),
                groups: filter_groups.map(crate::geometry::unpack_interaction_groups),
                exclude_collider: filter_exclude_collider.map(crate::utils::collider_handle),
                exclude_rigid_body: filter_exclude_rigid_body.map(crate::utils::body_handle),
                predicate,
            };

            let query_pipeline = self.0.as_query_pipeline(
                narrow_phase.0.query_dispatcher(),
                &bodies.bodies,
                &colliders.0,
                query_filter,
            );

            let ray = Ray::new([ray_ox, ray_oy].into(), [ray_dx, ray_dy].into());
            query_pipeline.cast_ray(&ray, maxToi, solid)
        }) else {
            return false;
        };

        Self::fill_ray_hit(&self.1, handle, timeOfImpact);
        true
    }

    #[cfg(feature = "dim3")]
    #[allow(clippy::too_many_arguments)]
    pub fn castRay(
        &self,
        narrow_phase: &RawNarrowPhase,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        ray_ox: f32,
        ray_oy: f32,
        ray_oz: f32,
        ray_dx: f32,
        ray_dy: f32,
        ray_dz: f32,
        maxToi: f32,
        solid: bool,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_exclude_collider: Option<FlatHandle>,
        filter_exclude_rigid_body: Option<FlatHandle>,
        filter_predicate: &js_sys::Function,
    ) -> bool {
        let Some((handle, timeOfImpact)) = utils::with_filter(filter_predicate, |predicate| {
            let query_filter = QueryFilter {
                flags: QueryFilterFlags::from_bits(filter_flags)
                    .unwrap_or(QueryFilterFlags::empty()),
                groups: filter_groups.map(crate::geometry::unpack_interaction_groups),
                exclude_collider: filter_exclude_collider.map(crate::utils::collider_handle),
                exclude_rigid_body: filter_exclude_rigid_body.map(crate::utils::body_handle),
                predicate,
            };

            let query_pipeline = self.0.as_query_pipeline(
                narrow_phase.0.query_dispatcher(),
                &bodies.bodies,
                &colliders.0,
                query_filter,
            );

            let ray = Ray::new(
                [ray_ox, ray_oy, ray_oz].into(),
                [ray_dx, ray_dy, ray_dz].into(),
            );
            query_pipeline.cast_ray(&ray, maxToi, solid)
        }) else {
            return false;
        };

        Self::fill_ray_hit(&self.1, handle, timeOfImpact);
        true
    }

    #[cfg(feature = "dim2")]
    #[allow(clippy::too_many_arguments)]
    pub fn castRayAndGetNormal(
        &self,
        narrow_phase: &RawNarrowPhase,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        ray_ox: f32,
        ray_oy: f32,
        ray_dx: f32,
        ray_dy: f32,
        maxToi: f32,
        solid: bool,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_exclude_collider: Option<FlatHandle>,
        filter_exclude_rigid_body: Option<FlatHandle>,
        filter_predicate: &js_sys::Function,
    ) -> bool {
        let Some((handle, inter)) = utils::with_filter(filter_predicate, |predicate| {
            let query_filter = QueryFilter {
                flags: QueryFilterFlags::from_bits(filter_flags)
                    .unwrap_or(QueryFilterFlags::empty()),
                groups: filter_groups.map(crate::geometry::unpack_interaction_groups),
                exclude_collider: filter_exclude_collider.map(crate::utils::collider_handle),
                exclude_rigid_body: filter_exclude_rigid_body.map(crate::utils::body_handle),
                predicate,
            };

            let query_pipeline = self.0.as_query_pipeline(
                narrow_phase.0.query_dispatcher(),
                &bodies.bodies,
                &colliders.0,
                query_filter,
            );

            let ray = Ray::new([ray_ox, ray_oy].into(), [ray_dx, ray_dy].into());
            query_pipeline.cast_ray_and_get_normal(&ray, maxToi, solid)
        }) else {
            return false;
        };

        Self::fill_ray_intersection(&self.1, handle, &inter);
        true
    }

    #[cfg(feature = "dim3")]
    #[allow(clippy::too_many_arguments)]
    pub fn castRayAndGetNormal(
        &self,
        narrow_phase: &RawNarrowPhase,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        ray_ox: f32,
        ray_oy: f32,
        ray_oz: f32,
        ray_dx: f32,
        ray_dy: f32,
        ray_dz: f32,
        maxToi: f32,
        solid: bool,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_exclude_collider: Option<FlatHandle>,
        filter_exclude_rigid_body: Option<FlatHandle>,
        filter_predicate: &js_sys::Function,
    ) -> bool {
        let Some((handle, inter)) = utils::with_filter(filter_predicate, |predicate| {
            let query_filter = QueryFilter {
                flags: QueryFilterFlags::from_bits(filter_flags)
                    .unwrap_or(QueryFilterFlags::empty()),
                groups: filter_groups.map(crate::geometry::unpack_interaction_groups),
                exclude_collider: filter_exclude_collider.map(crate::utils::collider_handle),
                exclude_rigid_body: filter_exclude_rigid_body.map(crate::utils::body_handle),
                predicate,
            };

            let query_pipeline = self.0.as_query_pipeline(
                narrow_phase.0.query_dispatcher(),
                &bodies.bodies,
                &colliders.0,
                query_filter,
            );

            let ray = Ray::new(
                [ray_ox, ray_oy, ray_oz].into(),
                [ray_dx, ray_dy, ray_dz].into(),
            );
            query_pipeline.cast_ray_and_get_normal(&ray, maxToi, solid)
        }) else {
            return false;
        };

        Self::fill_ray_intersection(&self.1, handle, &inter);
        true
    }

    // The callback is of type () => bool; each hit is written to the query
    // result buffer right before the callback is invoked.
    #[cfg(feature = "dim2")]
    #[allow(clippy::too_many_arguments)]
    pub fn intersectionsWithRay(
        &self,
        narrow_phase: &RawNarrowPhase,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        ray_ox: f32,
        ray_oy: f32,
        ray_dx: f32,
        ray_dy: f32,
        maxToi: f32,
        solid: bool,
        callback: &js_sys::Function,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_exclude_collider: Option<FlatHandle>,
        filter_exclude_rigid_body: Option<FlatHandle>,
        filter_predicate: &js_sys::Function,
    ) {
        utils::with_filter(filter_predicate, |predicate| {
            let query_filter = QueryFilter {
                flags: QueryFilterFlags::from_bits(filter_flags)
                    .unwrap_or(QueryFilterFlags::empty()),
                groups: filter_groups.map(crate::geometry::unpack_interaction_groups),
                exclude_collider: filter_exclude_collider.map(crate::utils::collider_handle),
                exclude_rigid_body: filter_exclude_rigid_body.map(crate::utils::body_handle),
                predicate,
            };

            let ray = Ray::new([ray_ox, ray_oy].into(), [ray_dx, ray_dy].into());
            let rcallback = |handle, inter: RayIntersection| {
                Self::fill_ray_intersection(&self.1, handle, &inter);
                match callback.call0(&JsValue::null()) {
                    Err(_) => true,
                    Ok(val) => val.as_bool().unwrap_or(true),
                }
            };

            let query_pipeline = self.0.as_query_pipeline(
                narrow_phase.0.query_dispatcher(),
                &bodies.bodies,
                &colliders.0,
                query_filter,
            );

            for (handle, _, inter) in query_pipeline.intersect_ray(ray, maxToi, solid) {
                if !rcallback(handle, inter) {
                    break;
                }
            }
        });
    }

    // The callback is of type () => bool; each hit is written to the query
    // result buffer right before the callback is invoked.
    #[cfg(feature = "dim3")]
    #[allow(clippy::too_many_arguments)]
    pub fn intersectionsWithRay(
        &self,
        narrow_phase: &RawNarrowPhase,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        ray_ox: f32,
        ray_oy: f32,
        ray_oz: f32,
        ray_dx: f32,
        ray_dy: f32,
        ray_dz: f32,
        maxToi: f32,
        solid: bool,
        callback: &js_sys::Function,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_exclude_collider: Option<FlatHandle>,
        filter_exclude_rigid_body: Option<FlatHandle>,
        filter_predicate: &js_sys::Function,
    ) {
        utils::with_filter(filter_predicate, |predicate| {
            let query_filter = QueryFilter {
                flags: QueryFilterFlags::from_bits(filter_flags)
                    .unwrap_or(QueryFilterFlags::empty()),
                groups: filter_groups.map(crate::geometry::unpack_interaction_groups),
                exclude_collider: filter_exclude_collider.map(crate::utils::collider_handle),
                exclude_rigid_body: filter_exclude_rigid_body.map(crate::utils::body_handle),
                predicate,
            };

            let ray = Ray::new(
                [ray_ox, ray_oy, ray_oz].into(),
                [ray_dx, ray_dy, ray_dz].into(),
            );
            let rcallback = |handle, inter: RayIntersection| {
                Self::fill_ray_intersection(&self.1, handle, &inter);
                match callback.call0(&JsValue::null()) {
                    Err(_) => true,
                    Ok(val) => val.as_bool().unwrap_or(true),
                }
            };

            let query_pipeline = self.0.as_query_pipeline(
                narrow_phase.0.query_dispatcher(),
                &bodies.bodies,
                &colliders.0,
                query_filter,
            );

            for (handle, _, inter) in query_pipeline.intersect_ray(ray, maxToi, solid) {
                if !rcallback(handle, inter) {
                    break;
                }
            }
        });
    }

    pub fn intersectionWithShape(
        &self,
        narrow_phase: &RawNarrowPhase,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        shapePos: &RawVector,
        shapeRot: &RawRotation,
        shape: &RawShape,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_exclude_collider: Option<FlatHandle>,
        filter_exclude_rigid_body: Option<FlatHandle>,
        filter_predicate: &js_sys::Function,
    ) -> Option<FlatHandle> {
        utils::with_filter(filter_predicate, |predicate| {
            let query_filter = QueryFilter {
                flags: QueryFilterFlags::from_bits(filter_flags)
                    .unwrap_or(QueryFilterFlags::empty()),
                groups: filter_groups.map(crate::geometry::unpack_interaction_groups),
                exclude_collider: filter_exclude_collider.map(crate::utils::collider_handle),
                exclude_rigid_body: filter_exclude_rigid_body.map(crate::utils::body_handle),
                predicate,
            };

            let query_pipeline = self.0.as_query_pipeline(
                narrow_phase.0.query_dispatcher(),
                &bodies.bodies,
                &colliders.0,
                query_filter,
            );

            let pos = Pose::from_parts(shapePos.0, shapeRot.0);

            // TODO: take a callback as argument so we can yield all the intersecting shapes?
            for (handle, _) in query_pipeline.intersect_shape(pos, &*shape.0) {
                // Return the first intersection we find.
                return Some(utils::flat_handle(handle.0));
            }

            None
        })
    }

    #[cfg(feature = "dim2")]
    #[allow(clippy::too_many_arguments)]
    pub fn projectPoint(
        &self,
        narrow_phase: &RawNarrowPhase,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        point_x: f32,
        point_y: f32,
        solid: bool,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_exclude_collider: Option<FlatHandle>,
        filter_exclude_rigid_body: Option<FlatHandle>,
        filter_predicate: &js_sys::Function,
    ) -> bool {
        let point = Vector::new(point_x, point_y);
        self.do_project_point(
            narrow_phase,
            bodies,
            colliders,
            point,
            solid,
            filter_flags,
            filter_groups,
            filter_exclude_collider,
            filter_exclude_rigid_body,
            filter_predicate,
        )
    }

    #[cfg(feature = "dim3")]
    #[allow(clippy::too_many_arguments)]
    pub fn projectPoint(
        &self,
        narrow_phase: &RawNarrowPhase,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        point_x: f32,
        point_y: f32,
        point_z: f32,
        solid: bool,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_exclude_collider: Option<FlatHandle>,
        filter_exclude_rigid_body: Option<FlatHandle>,
        filter_predicate: &js_sys::Function,
    ) -> bool {
        let point = Vector::new(point_x, point_y, point_z);
        self.do_project_point(
            narrow_phase,
            bodies,
            colliders,
            point,
            solid,
            filter_flags,
            filter_groups,
            filter_exclude_collider,
            filter_exclude_rigid_body,
            filter_predicate,
        )
    }

    #[cfg(feature = "dim2")]
    #[allow(clippy::too_many_arguments)]
    pub fn projectPointAndGetFeature(
        &self,
        narrow_phase: &RawNarrowPhase,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        point_x: f32,
        point_y: f32,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_exclude_collider: Option<FlatHandle>,
        filter_exclude_rigid_body: Option<FlatHandle>,
        filter_predicate: &js_sys::Function,
    ) -> bool {
        let point = Vector::new(point_x, point_y);
        self.do_project_point_and_get_feature(
            narrow_phase,
            bodies,
            colliders,
            point,
            filter_flags,
            filter_groups,
            filter_exclude_collider,
            filter_exclude_rigid_body,
            filter_predicate,
        )
    }

    #[cfg(feature = "dim3")]
    #[allow(clippy::too_many_arguments)]
    pub fn projectPointAndGetFeature(
        &self,
        narrow_phase: &RawNarrowPhase,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        point_x: f32,
        point_y: f32,
        point_z: f32,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_exclude_collider: Option<FlatHandle>,
        filter_exclude_rigid_body: Option<FlatHandle>,
        filter_predicate: &js_sys::Function,
    ) -> bool {
        let point = Vector::new(point_x, point_y, point_z);
        self.do_project_point_and_get_feature(
            narrow_phase,
            bodies,
            colliders,
            point,
            filter_flags,
            filter_groups,
            filter_exclude_collider,
            filter_exclude_rigid_body,
            filter_predicate,
        )
    }

    #[cfg(feature = "dim2")]
    #[allow(clippy::too_many_arguments)]
    pub fn intersectionsWithPoint(
        &self,
        narrow_phase: &RawNarrowPhase,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        point_x: f32,
        point_y: f32,
        callback: &js_sys::Function,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_exclude_collider: Option<FlatHandle>,
        filter_exclude_rigid_body: Option<FlatHandle>,
        filter_predicate: &js_sys::Function,
    ) {
        let point = Vector::new(point_x, point_y);
        self.do_intersections_with_point(
            narrow_phase,
            bodies,
            colliders,
            point,
            callback,
            filter_flags,
            filter_groups,
            filter_exclude_collider,
            filter_exclude_rigid_body,
            filter_predicate,
        )
    }

    #[cfg(feature = "dim3")]
    #[allow(clippy::too_many_arguments)]
    pub fn intersectionsWithPoint(
        &self,
        narrow_phase: &RawNarrowPhase,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        point_x: f32,
        point_y: f32,
        point_z: f32,
        callback: &js_sys::Function,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_exclude_collider: Option<FlatHandle>,
        filter_exclude_rigid_body: Option<FlatHandle>,
        filter_predicate: &js_sys::Function,
    ) {
        let point = Vector::new(point_x, point_y, point_z);
        self.do_intersections_with_point(
            narrow_phase,
            bodies,
            colliders,
            point,
            callback,
            filter_flags,
            filter_groups,
            filter_exclude_collider,
            filter_exclude_rigid_body,
            filter_predicate,
        )
    }
}

impl RawBroadPhase {
    #[allow(clippy::too_many_arguments)]
    fn do_project_point(
        &self,
        narrow_phase: &RawNarrowPhase,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        point: Vector,
        solid: bool,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_exclude_collider: Option<FlatHandle>,
        filter_exclude_rigid_body: Option<FlatHandle>,
        filter_predicate: &js_sys::Function,
    ) -> bool {
        let projection = utils::with_filter(filter_predicate, |predicate| {
            let query_filter = QueryFilter {
                flags: QueryFilterFlags::from_bits(filter_flags)
                    .unwrap_or(QueryFilterFlags::empty()),
                groups: filter_groups.map(crate::geometry::unpack_interaction_groups),
                exclude_collider: filter_exclude_collider.map(crate::utils::collider_handle),
                exclude_rigid_body: filter_exclude_rigid_body.map(crate::utils::body_handle),
                predicate,
            };

            let query_pipeline = self.0.as_query_pipeline(
                narrow_phase.0.query_dispatcher(),
                &bodies.bodies,
                &colliders.0,
                query_filter,
            );

            query_pipeline.project_point(point, f32::MAX, solid)
        });

        let Some((handle, proj)) = projection else {
            return false;
        };

        Self::fill_point_projection(&self.1, handle, &proj, FeatureId::Unknown);
        true
    }

    #[allow(clippy::too_many_arguments)]
    fn do_project_point_and_get_feature(
        &self,
        narrow_phase: &RawNarrowPhase,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        point: Vector,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_exclude_collider: Option<FlatHandle>,
        filter_exclude_rigid_body: Option<FlatHandle>,
        filter_predicate: &js_sys::Function,
    ) -> bool {
        let projection = utils::with_filter(filter_predicate, |predicate| {
            let query_filter = QueryFilter {
                flags: QueryFilterFlags::from_bits(filter_flags)
                    .unwrap_or(QueryFilterFlags::empty()),
                groups: filter_groups.map(crate::geometry::unpack_interaction_groups),
                exclude_collider: filter_exclude_collider.map(crate::utils::collider_handle),
                exclude_rigid_body: filter_exclude_rigid_body.map(crate::utils::body_handle),
                predicate,
            };

            let query_pipeline = self.0.as_query_pipeline(
                narrow_phase.0.query_dispatcher(),
                &bodies.bodies,
                &colliders.0,
                query_filter,
            );

            query_pipeline.project_point_and_get_feature(point)
        });

        let Some((handle, proj, feature)) = projection else {
            return false;
        };

        Self::fill_point_projection(&self.1, handle, &proj, feature);
        true
    }

    // The callback is of type (handle) => bool
    #[allow(clippy::too_many_arguments)]
    fn do_intersections_with_point(
        &self,
        narrow_phase: &RawNarrowPhase,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        point: Vector,
        callback: &js_sys::Function,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_exclude_collider: Option<FlatHandle>,
        filter_exclude_rigid_body: Option<FlatHandle>,
        filter_predicate: &js_sys::Function,
    ) {
        utils::with_filter(filter_predicate, |predicate| {
            let query_filter = QueryFilter {
                flags: QueryFilterFlags::from_bits(filter_flags)
                    .unwrap_or(QueryFilterFlags::empty()),
                groups: filter_groups.map(crate::geometry::unpack_interaction_groups),
                exclude_collider: filter_exclude_collider.map(crate::utils::collider_handle),
                exclude_rigid_body: filter_exclude_rigid_body.map(crate::utils::body_handle),
                predicate,
            };

            let query_pipeline = self.0.as_query_pipeline(
                narrow_phase.0.query_dispatcher(),
                &bodies.bodies,
                &colliders.0,
                query_filter,
            );

            let rcallback = |handle: ColliderHandle| match callback.call1(
                &JsValue::null(),
                &JsValue::from(utils::flat_handle(handle.0)),
            ) {
                Err(_) => true,
                Ok(val) => val.as_bool().unwrap_or(true),
            };

            for (handle, _) in query_pipeline.intersect_point(point.into()) {
                if !rcallback(handle) {
                    break;
                }
            }
        });
    }
}

#[wasm_bindgen]
impl RawBroadPhase {
    pub fn castShape(
        &self,
        narrow_phase: &RawNarrowPhase,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        shapePos: &RawVector,
        shapeRot: &RawRotation,
        shapeVel: &RawVector,
        shape: &RawShape,
        target_distance: f32,
        maxToi: f32,
        stop_at_penetration: bool,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_exclude_collider: Option<FlatHandle>,
        filter_exclude_rigid_body: Option<FlatHandle>,
        filter_predicate: &js_sys::Function,
    ) -> Option<RawColliderShapeCastHit> {
        utils::with_filter(filter_predicate, |predicate| {
            let query_filter = QueryFilter {
                flags: QueryFilterFlags::from_bits(filter_flags)
                    .unwrap_or(QueryFilterFlags::empty()),
                groups: filter_groups.map(crate::geometry::unpack_interaction_groups),
                exclude_collider: filter_exclude_collider.map(crate::utils::collider_handle),
                exclude_rigid_body: filter_exclude_rigid_body.map(crate::utils::body_handle),
                predicate,
            };

            let query_pipeline = self.0.as_query_pipeline(
                narrow_phase.0.query_dispatcher(),
                &bodies.bodies,
                &colliders.0,
                query_filter,
            );

            let pos = Pose::from_parts(shapePos.0, shapeRot.0);
            query_pipeline
                .cast_shape(
                    &pos,
                    shapeVel.0,
                    &*shape.0,
                    ShapeCastOptions {
                        max_time_of_impact: maxToi,
                        stop_at_penetration,
                        compute_impact_geometry_on_penetration: true,
                        target_distance,
                    },
                )
                .map(|(handle, hit)| RawColliderShapeCastHit { handle, hit })
        })
    }

    // The callback has type (u32) => boolean
    pub fn intersectionsWithShape(
        &self,
        narrow_phase: &RawNarrowPhase,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        shapePos: &RawVector,
        shapeRot: &RawRotation,
        shape: &RawShape,
        callback: &js_sys::Function,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_exclude_collider: Option<FlatHandle>,
        filter_exclude_rigid_body: Option<FlatHandle>,
        filter_predicate: &js_sys::Function,
    ) {
        utils::with_filter(filter_predicate, |predicate| {
            let query_filter = QueryFilter {
                flags: QueryFilterFlags::from_bits(filter_flags)
                    .unwrap_or(QueryFilterFlags::empty()),
                groups: filter_groups.map(crate::geometry::unpack_interaction_groups),
                exclude_collider: filter_exclude_collider.map(crate::utils::collider_handle),
                exclude_rigid_body: filter_exclude_rigid_body.map(crate::utils::body_handle),
                predicate,
            };

            let query_pipeline = self.0.as_query_pipeline(
                narrow_phase.0.query_dispatcher(),
                &bodies.bodies,
                &colliders.0,
                query_filter,
            );

            let rcallback = |handle: ColliderHandle| match callback.call1(
                &JsValue::null(),
                &JsValue::from(utils::flat_handle(handle.0)),
            ) {
                Err(_) => true,
                Ok(val) => val.as_bool().unwrap_or(true),
            };

            let pos = Pose::from_parts(shapePos.0, shapeRot.0);
            for (handle, _) in query_pipeline.intersect_shape(pos, &*shape.0) {
                if !rcallback(handle) {
                    break;
                }
            }
        })
    }

    pub fn collidersWithAabbIntersectingAabb(
        &self,
        narrow_phase: &RawNarrowPhase,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        aabbCenter: &RawVector,
        aabbHalfExtents: &RawVector,
        callback: &js_sys::Function,
    ) {
        let rcallback = |handle: &ColliderHandle| match callback.call1(
            &JsValue::null(),
            &JsValue::from(utils::flat_handle(handle.0)),
        ) {
            Err(_) => true,
            Ok(val) => val.as_bool().unwrap_or(true),
        };

        let query_pipeline = self.0.as_query_pipeline(
            narrow_phase.0.query_dispatcher(),
            &bodies.bodies,
            &colliders.0,
            Default::default(),
        );

        let center = aabbCenter.0;
        let aabb = Aabb::new(center - aabbHalfExtents.0, center + aabbHalfExtents.0);

        for (handle, _) in query_pipeline.intersect_aabb_conservative(aabb) {
            if !rcallback(&handle) {
                break;
            }
        }
    }
}
