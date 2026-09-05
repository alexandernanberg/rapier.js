#[cfg(feature = "dim3")]
use crate::geometry::shape::normalized_convex_polyhedron_mesh;
use crate::geometry::shape::SharedShapeUtility;
use crate::geometry::{
    write_contact, write_hit, write_point_projection, write_ray_intersection, RawColliderSet,
    RawShape, RawShapeType,
};
use crate::math::pose_from_scalars;
use crate::scratch;
use crate::utils::{self, FlatHandle};
use rapier::dynamics::MassProperties;
use rapier::geometry::{ActiveCollisionTypes, ShapeType};
use rapier::math::{IVector, Pose, Real, Rotation, Vector};
use rapier::parry::query;
use rapier::parry::query::ShapeCastOptions;
use rapier::pipeline::{ActiveEvents, ActiveHooks};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
impl RawColliderSet {
    /// The world-space translation of this collider, written to the scratch buffer.
    #[cfg(feature = "dim2")]
    pub fn coTranslation(&self, handle: FlatHandle) {
        self.map(handle, |co| {
            let t = co.position().translation;
            scratch::write(&[t.x, t.y]);
        });
    }

    /// The world-space translation of this collider, written to the scratch buffer.
    #[cfg(feature = "dim3")]
    pub fn coTranslation(&self, handle: FlatHandle) {
        self.map(handle, |co| {
            let t = co.position().translation;
            scratch::write(&[t.x, t.y, t.z]);
        });
    }

    /// The world-space orientation of this collider, written to the scratch buffer.
    #[cfg(feature = "dim2")]
    pub fn coRotation(&self, handle: FlatHandle) {
        self.map(handle, |co| {
            scratch::write(&[co.position().rotation.angle()]);
        });
    }

    /// The world-space orientation of this collider, written to the scratch buffer.
    #[cfg(feature = "dim3")]
    pub fn coRotation(&self, handle: FlatHandle) {
        self.map(handle, |co| {
            let r = co.position().rotation;
            scratch::write(&[r.x, r.y, r.z, r.w]);
        });
    }

    /// The translation of this collider relative to its parent rigid-body, written to the scratch buffer.
    /// Returns false if it doesn't have a parent.
    #[cfg(feature = "dim2")]
    pub fn coTranslationWrtParent(&self, handle: FlatHandle) -> bool {
        self.map(handle, |co| {
            if let Some(pose) = co.position_wrt_parent() {
                scratch::write(&[pose.translation.x, pose.translation.y]);
                true
            } else {
                false
            }
        })
    }

    /// The translation of this collider relative to its parent rigid-body, written to the scratch buffer.
    /// Returns false if it doesn't have a parent.
    #[cfg(feature = "dim3")]
    pub fn coTranslationWrtParent(&self, handle: FlatHandle) -> bool {
        self.map(handle, |co| {
            if let Some(pose) = co.position_wrt_parent() {
                scratch::write(&[pose.translation.x, pose.translation.y, pose.translation.z]);
                true
            } else {
                false
            }
        })
    }

    /// The orientation of this collider relative to its parent rigid-body, written to the scratch buffer.
    /// Returns false if it doesn't have a parent.
    #[cfg(feature = "dim2")]
    pub fn coRotationWrtParent(&self, handle: FlatHandle) -> bool {
        self.map(handle, |co| {
            if let Some(pose) = co.position_wrt_parent() {
                scratch::write(&[pose.rotation.angle()]);
                true
            } else {
                false
            }
        })
    }

    /// The orientation of this collider relative to its parent rigid-body, written to the scratch buffer.
    /// Returns false if it doesn't have a parent.
    #[cfg(feature = "dim3")]
    pub fn coRotationWrtParent(&self, handle: FlatHandle) -> bool {
        self.map(handle, |co| {
            if let Some(pose) = co.position_wrt_parent() {
                scratch::write(&[
                    pose.rotation.x,
                    pose.rotation.y,
                    pose.rotation.z,
                    pose.rotation.w,
                ]);
                true
            } else {
                false
            }
        })
    }

    /// Sets the translation of this collider.
    ///
    /// # Parameters
    /// - `x`: the world-space position of the collider along the `x` axis.
    /// - `y`: the world-space position of the collider along the `y` axis.
    /// - `z`: the world-space position of the collider along the `z` axis.
    /// - `wakeUp`: forces the collider to wake-up so it is properly affected by forces if it
    /// wasn't moving before modifying its position.
    #[cfg(feature = "dim3")]
    pub fn coSetTranslation(&mut self, handle: FlatHandle, x: f32, y: f32, z: f32) {
        self.map_mut(handle, |co| {
            co.set_translation(Vector::new(x, y, z));
        })
    }

    /// Sets the translation of this collider.
    ///
    /// # Parameters
    /// - `x`: the world-space position of the collider along the `x` axis.
    /// - `y`: the world-space position of the collider along the `y` axis.
    /// - `wakeUp`: forces the collider to wake-up so it is properly affected by forces if it
    /// wasn't moving before modifying its position.
    #[cfg(feature = "dim2")]
    pub fn coSetTranslation(&mut self, handle: FlatHandle, x: f32, y: f32) {
        self.map_mut(handle, |co| {
            co.set_translation(Vector::new(x, y));
        })
    }

    #[cfg(feature = "dim3")]
    pub fn coSetTranslationWrtParent(&mut self, handle: FlatHandle, x: f32, y: f32, z: f32) {
        self.map_mut(handle, |co| {
            co.set_translation_wrt_parent(Vector::new(x, y, z));
        })
    }

    #[cfg(feature = "dim2")]
    pub fn coSetTranslationWrtParent(&mut self, handle: FlatHandle, x: f32, y: f32) {
        self.map_mut(handle, |co| {
            co.set_translation_wrt_parent(Vector::new(x, y));
        })
    }

    /// Sets the rotation quaternion of this collider.
    ///
    /// This does nothing if a zero quaternion is provided.
    ///
    /// # Parameters
    /// - `x`: the first vector component of the quaternion.
    /// - `y`: the second vector component of the quaternion.
    /// - `z`: the third vector component of the quaternion.
    /// - `w`: the scalar component of the quaternion.
    /// - `wakeUp`: forces the collider to wake-up so it is properly affected by forces if it
    /// wasn't moving before modifying its position.
    #[cfg(feature = "dim3")]
    pub fn coSetRotation(&mut self, handle: FlatHandle, x: f32, y: f32, z: f32, w: f32) {
        if let Some(q) = utils::unit_rotation(x, y, z, w) {
            self.map_mut(handle, |co| co.set_rotation(q))
        }
    }

    /// Sets the rotation angle of this collider.
    ///
    /// # Parameters
    /// - `angle`: the rotation angle, in radians.
    /// - `wakeUp`: forces the collider to wake-up so it is properly affected by forces if it
    /// wasn't moving before modifying its position.
    #[cfg(feature = "dim2")]
    pub fn coSetRotation(&mut self, handle: FlatHandle, angle: f32) {
        self.map_mut(handle, |co| co.set_rotation(Rotation::new(angle)))
    }

    #[cfg(feature = "dim3")]
    pub fn coSetRotationWrtParent(&mut self, handle: FlatHandle, x: f32, y: f32, z: f32, w: f32) {
        if let Some(q) = utils::unit_rotation(x, y, z, w) {
            // Set through the full parent-relative pose: rapier's
            // `set_rotation_wrt_parent` takes a scaled axis, and the axis-angle
            // round trip it would force costs an `atan2`/`sqrt`/`sin`/`cos` and
            // can hand back a sign-flipped quaternion.
            self.map_mut(handle, |co| {
                let translation = co
                    .position_wrt_parent()
                    .map(|p| p.translation)
                    .unwrap_or(Vector::ZERO);
                co.set_position_wrt_parent(Pose::from_parts(translation, q))
            })
        }
    }

    #[cfg(feature = "dim2")]
    pub fn coSetRotationWrtParent(&mut self, handle: FlatHandle, angle: f32) {
        self.map_mut(handle, |co| co.set_rotation_wrt_parent(angle))
    }

    /// Is this collider a sensor?
    pub fn coIsSensor(&self, handle: FlatHandle) -> bool {
        self.map(handle, |co| co.is_sensor())
    }

    /// The type of the shape of this collider.
    pub fn coShapeType(&self, handle: FlatHandle) -> RawShapeType {
        self.map(handle, |co| match co.shape().shape_type() {
            ShapeType::Ball => RawShapeType::Ball,
            ShapeType::Cuboid => RawShapeType::Cuboid,
            ShapeType::Capsule => RawShapeType::Capsule,
            ShapeType::Segment => RawShapeType::Segment,
            ShapeType::Polyline => RawShapeType::Polyline,
            ShapeType::Triangle => RawShapeType::Triangle,
            ShapeType::TriMesh => RawShapeType::TriMesh,
            ShapeType::HeightField => RawShapeType::HeightField,
            ShapeType::Compound => RawShapeType::Compound,
            ShapeType::HalfSpace => RawShapeType::HalfSpace,
            ShapeType::Voxels => RawShapeType::Voxels,
            #[cfg(feature = "dim3")]
            ShapeType::ConvexPolyhedron => RawShapeType::ConvexPolyhedron,
            #[cfg(feature = "dim2")]
            ShapeType::ConvexPolygon => RawShapeType::ConvexPolygon,
            #[cfg(feature = "dim3")]
            ShapeType::Cylinder => RawShapeType::Cylinder,
            #[cfg(feature = "dim3")]
            ShapeType::Cone => RawShapeType::Cone,
            ShapeType::RoundCuboid => RawShapeType::RoundCuboid,
            ShapeType::RoundTriangle => RawShapeType::RoundTriangle,
            #[cfg(feature = "dim3")]
            ShapeType::RoundCylinder => RawShapeType::RoundCylinder,
            #[cfg(feature = "dim3")]
            ShapeType::RoundCone => RawShapeType::RoundCone,
            #[cfg(feature = "dim3")]
            ShapeType::RoundConvexPolyhedron => RawShapeType::RoundConvexPolyhedron,
            #[cfg(feature = "dim2")]
            ShapeType::RoundConvexPolygon => RawShapeType::RoundConvexPolygon,
            ShapeType::Custom => panic!("Not yet implemented."),
        })
    }

    /// The shape of this collider.
    pub fn coShape(&self, handle: FlatHandle) -> RawShape {
        self.map(handle, |co| RawShape(co.shared_shape().clone()))
    }

    /// The half-extents of a cuboid (or round cuboid) collider, written to the
    /// scratch buffer. Returns `false` (and writes nothing) for any other shape.
    pub fn coHalfExtents(&self, handle: FlatHandle) -> bool {
        self.map(handle, |co| {
            co.shape().as_cuboid().map(|c| c.half_extents).or_else(|| {
                co.shape()
                    .as_round_cuboid()
                    .map(|c| c.inner_shape.half_extents)
            })
        })
        .map(|half_extents| scratch::write_vector(half_extents))
        .is_some()
    }

    /// Set the half-extents of this collider if it has a cuboid shape.
    /// Sets the half-extents of a cuboid or round cuboid collider, passed
    /// component-wise so the JS side allocates no `RawVector` per call.
    #[cfg(feature = "dim2")]
    pub fn coSetHalfExtents(&mut self, handle: FlatHandle, x: f32, y: f32) {
        self.do_set_half_extents(handle, Vector::new(x, y));
    }

    /// Sets the half-extents of a cuboid or round cuboid collider, passed
    /// component-wise so the JS side allocates no `RawVector` per call.
    #[cfg(feature = "dim3")]
    pub fn coSetHalfExtents(&mut self, handle: FlatHandle, x: f32, y: f32, z: f32) {
        self.do_set_half_extents(handle, Vector::new(x, y, z));
    }

    /// The radius of this collider if it is a ball, capsule, cylinder, or cone shape.
    pub fn coRadius(&self, handle: FlatHandle) -> Option<f32> {
        self.map(handle, |co| match co.shape().shape_type() {
            ShapeType::Ball => co.shape().as_ball().map(|b| b.radius),
            ShapeType::Capsule => co.shape().as_capsule().map(|b| b.radius),
            #[cfg(feature = "dim3")]
            ShapeType::Cylinder => co.shape().as_cylinder().map(|b| b.radius),
            #[cfg(feature = "dim3")]
            ShapeType::RoundCylinder => {
                co.shape().as_round_cylinder().map(|b| b.inner_shape.radius)
            }
            #[cfg(feature = "dim3")]
            ShapeType::Cone => co.shape().as_cone().map(|b| b.radius),
            #[cfg(feature = "dim3")]
            ShapeType::RoundCone => co.shape().as_round_cone().map(|b| b.inner_shape.radius),
            _ => None,
        })
    }

    /// Set the radius of this collider if it is a ball, capsule, cylinder, or cone shape.
    pub fn coSetRadius(&mut self, handle: FlatHandle, newRadius: Real) {
        self.map_mut_untracked(handle, |co| match co.shape().shape_type() {
            ShapeType::Ball => co.shape_mut().as_ball_mut().map(|b| b.radius = newRadius),
            ShapeType::Capsule => co
                .shape_mut()
                .as_capsule_mut()
                .map(|b| b.radius = newRadius),
            #[cfg(feature = "dim3")]
            ShapeType::Cylinder => co
                .shape_mut()
                .as_cylinder_mut()
                .map(|b| b.radius = newRadius),
            #[cfg(feature = "dim3")]
            ShapeType::RoundCylinder => co
                .shape_mut()
                .as_round_cylinder_mut()
                .map(|b| b.inner_shape.radius = newRadius),
            #[cfg(feature = "dim3")]
            ShapeType::Cone => co.shape_mut().as_cone_mut().map(|b| b.radius = newRadius),
            #[cfg(feature = "dim3")]
            ShapeType::RoundCone => co
                .shape_mut()
                .as_round_cone_mut()
                .map(|b| b.inner_shape.radius = newRadius),
            _ => None,
        });
    }

    /// The half height of this collider if it is a capsule, cylinder, or cone shape.
    pub fn coHalfHeight(&self, handle: FlatHandle) -> Option<f32> {
        self.map(handle, |co| match co.shape().shape_type() {
            ShapeType::Capsule => co.shape().as_capsule().map(|b| b.half_height()),
            #[cfg(feature = "dim3")]
            ShapeType::Cylinder => co.shape().as_cylinder().map(|b| b.half_height),
            #[cfg(feature = "dim3")]
            ShapeType::RoundCylinder => co
                .shape()
                .as_round_cylinder()
                .map(|b| b.inner_shape.half_height),
            #[cfg(feature = "dim3")]
            ShapeType::Cone => co.shape().as_cone().map(|b| b.half_height),
            #[cfg(feature = "dim3")]
            ShapeType::RoundCone => co
                .shape()
                .as_round_cone()
                .map(|b| b.inner_shape.half_height),
            _ => None,
        })
    }

    /// Set the half height of this collider if it is a capsule, cylinder, or cone shape.
    pub fn coSetHalfHeight(&mut self, handle: FlatHandle, newHalfheight: Real) {
        self.map_mut_untracked(handle, |co| match co.shape().shape_type() {
            ShapeType::Capsule => co.shape_mut().as_capsule_mut().map(|b| {
                // Keep the capsule's axis (a deserialized world may hold one
                // that is not aligned with `Y`); only its length changes.
                let axis = (b.segment.b - b.segment.a).normalize_or(Vector::Y);
                let center = (b.segment.a + b.segment.b) * 0.5;
                let half = axis * newHalfheight;
                b.segment.a = center - half;
                b.segment.b = center + half;
            }),
            #[cfg(feature = "dim3")]
            ShapeType::Cylinder => co
                .shape_mut()
                .as_cylinder_mut()
                .map(|b| b.half_height = newHalfheight),
            #[cfg(feature = "dim3")]
            ShapeType::RoundCylinder => co
                .shape_mut()
                .as_round_cylinder_mut()
                .map(|b| b.inner_shape.half_height = newHalfheight),
            #[cfg(feature = "dim3")]
            ShapeType::Cone => co
                .shape_mut()
                .as_cone_mut()
                .map(|b| b.half_height = newHalfheight),
            #[cfg(feature = "dim3")]
            ShapeType::RoundCone => co
                .shape_mut()
                .as_round_cone_mut()
                .map(|b| b.inner_shape.half_height = newHalfheight),
            _ => None,
        });
    }

    /// The radius of the round edges of this collider.
    pub fn coRoundRadius(&self, handle: FlatHandle) -> Option<f32> {
        self.map(handle, |co| match co.shape().shape_type() {
            ShapeType::RoundCuboid => co.shape().as_round_cuboid().map(|b| b.border_radius),
            ShapeType::RoundTriangle => co.shape().as_round_triangle().map(|b| b.border_radius),
            #[cfg(feature = "dim3")]
            ShapeType::RoundCylinder => co.shape().as_round_cylinder().map(|b| b.border_radius),
            #[cfg(feature = "dim3")]
            ShapeType::RoundCone => co.shape().as_round_cone().map(|b| b.border_radius),
            #[cfg(feature = "dim3")]
            ShapeType::RoundConvexPolyhedron => co
                .shape()
                .as_round_convex_polyhedron()
                .map(|b| b.border_radius),
            #[cfg(feature = "dim2")]
            ShapeType::RoundConvexPolygon => co
                .shape()
                .as_round_convex_polygon()
                .map(|b| b.border_radius),
            _ => None,
        })
    }

    /// Set the radius of the round edges of this collider.
    pub fn coSetRoundRadius(&mut self, handle: FlatHandle, newBorderRadius: Real) {
        self.map_mut_untracked(handle, |co| match co.shape().shape_type() {
            ShapeType::RoundCuboid => co
                .shape_mut()
                .as_round_cuboid_mut()
                .map(|b| b.border_radius = newBorderRadius),
            ShapeType::RoundTriangle => co
                .shape_mut()
                .as_round_triangle_mut()
                .map(|b| b.border_radius = newBorderRadius),
            #[cfg(feature = "dim3")]
            ShapeType::RoundCylinder => co
                .shape_mut()
                .as_round_cylinder_mut()
                .map(|b| b.border_radius = newBorderRadius),
            #[cfg(feature = "dim3")]
            ShapeType::RoundCone => co
                .shape_mut()
                .as_round_cone_mut()
                .map(|b| b.border_radius = newBorderRadius),
            #[cfg(feature = "dim3")]
            ShapeType::RoundConvexPolyhedron => co
                .shape_mut()
                .as_round_convex_polyhedron_mut()
                .map(|b| b.border_radius = newBorderRadius),
            #[cfg(feature = "dim2")]
            ShapeType::RoundConvexPolygon => co
                .shape_mut()
                .as_round_convex_polygon_mut()
                .map(|b| b.border_radius = newBorderRadius),
            _ => None,
        });
    }

    #[cfg(feature = "dim2")]
    pub fn coSetVoxel(&mut self, handle: FlatHandle, ix: i32, iy: i32, filled: bool) {
        self.map_mut_untracked(handle, |co| {
            if let Some(vox) = co.shape_mut().as_voxels_mut() {
                vox.set_voxel(IVector::new(ix, iy), filled);
            }
        })
    }

    #[cfg(feature = "dim3")]
    pub fn coSetVoxel(&mut self, handle: FlatHandle, ix: i32, iy: i32, iz: i32, filled: bool) {
        self.map_mut_untracked(handle, |co| {
            if let Some(vox) = co.shape_mut().as_voxels_mut() {
                vox.set_voxel(IVector::new(ix, iy, iz), filled);
            }
        })
    }

    #[cfg(feature = "dim2")]
    pub fn coPropagateVoxelChange(
        &mut self,
        handle1: FlatHandle,
        handle2: FlatHandle,
        ix: i32,
        iy: i32,
        shift_x: i32,
        shift_y: i32,
    ) {
        self.map_pair_mut(handle1, handle2, |co1, co2| {
            if let (Some(co1), Some(co2)) = (co1, co2) {
                if let (Some(vox1), Some(vox2)) = (
                    co1.shape_mut().as_voxels_mut(),
                    co2.shape_mut().as_voxels_mut(),
                ) {
                    vox1.propagate_voxel_change(
                        vox2,
                        IVector::new(ix, iy),
                        IVector::new(shift_x, shift_y),
                    );
                }
            }
        })
    }

    #[cfg(feature = "dim3")]
    pub fn coPropagateVoxelChange(
        &mut self,
        handle1: FlatHandle,
        handle2: FlatHandle,
        ix: i32,
        iy: i32,
        iz: i32,
        shift_x: i32,
        shift_y: i32,
        shift_z: i32,
    ) {
        self.map_pair_mut(handle1, handle2, |co1, co2| {
            if let (Some(co1), Some(co2)) = (co1, co2) {
                if let (Some(vox1), Some(vox2)) = (
                    co1.shape_mut().as_voxels_mut(),
                    co2.shape_mut().as_voxels_mut(),
                ) {
                    vox1.propagate_voxel_change(
                        vox2,
                        IVector::new(ix, iy, iz),
                        IVector::new(shift_x, shift_y, shift_z),
                    );
                }
            }
        })
    }

    #[cfg(feature = "dim2")]
    pub fn coCombineVoxelStates(
        &mut self,
        handle1: FlatHandle,
        handle2: FlatHandle,
        shift_x: i32,
        shift_y: i32,
    ) {
        self.map_pair_mut(handle1, handle2, |co1, co2| {
            if let (Some(co1), Some(co2)) = (co1, co2) {
                if let (Some(vox1), Some(vox2)) = (
                    co1.shape_mut().as_voxels_mut(),
                    co2.shape_mut().as_voxels_mut(),
                ) {
                    vox1.combine_voxel_states(vox2, IVector::new(shift_x, shift_y));
                }
            }
        })
    }

    #[cfg(feature = "dim3")]
    pub fn coCombineVoxelStates(
        &mut self,
        handle1: FlatHandle,
        handle2: FlatHandle,
        shift_x: i32,
        shift_y: i32,
        shift_z: i32,
    ) {
        self.map_pair_mut(handle1, handle2, |co1, co2| {
            if let (Some(co1), Some(co2)) = (co1, co2) {
                if let (Some(vox1), Some(vox2)) = (
                    co1.shape_mut().as_voxels_mut(),
                    co2.shape_mut().as_voxels_mut(),
                ) {
                    vox1.combine_voxel_states(vox2, IVector::new(shift_x, shift_y, shift_z));
                }
            }
        })
    }

    /// The vertices of this triangle mesh, polyline, convex polyhedron, segment, triangle or convex polyhedron, if it is one.
    pub fn coVertices(&self, handle: FlatHandle) -> Option<Vec<f32>> {
        let flatten = |vertices: &[Vector]| {
            vertices
                .iter()
                .flat_map(|p| p.as_ref().iter())
                .copied()
                .collect()
        };
        self.map(handle, |co| match co.shape().shape_type() {
            ShapeType::TriMesh => co.shape().as_trimesh().map(|t| flatten(t.vertices())),
            ShapeType::Polyline => co.shape().as_polyline().map(|p| flatten(p.vertices())),
            #[cfg(feature = "dim3")]
            ShapeType::ConvexPolyhedron => co
                .shape()
                .as_convex_polyhedron()
                .and_then(normalized_convex_polyhedron_mesh)
                .map(|(points, _)| flatten(&points)),
            #[cfg(feature = "dim3")]
            ShapeType::RoundConvexPolyhedron => co
                .shape()
                .as_round_convex_polyhedron()
                .and_then(|p| normalized_convex_polyhedron_mesh(&p.inner_shape))
                .map(|(points, _)| flatten(&points)),
            #[cfg(feature = "dim2")]
            ShapeType::ConvexPolygon => co.shape().as_convex_polygon().map(|p| flatten(p.points())),
            #[cfg(feature = "dim2")]
            ShapeType::RoundConvexPolygon => co
                .shape()
                .as_round_convex_polygon()
                .map(|p| flatten(p.inner_shape.points())),
            ShapeType::Segment => co.shape().as_segment().map(|s| flatten(&[s.a, s.b])),
            ShapeType::RoundTriangle => co
                .shape()
                .as_round_triangle()
                .map(|t| flatten(&[t.inner_shape.a, t.inner_shape.b, t.inner_shape.c])),
            ShapeType::Triangle => co.shape().as_triangle().map(|t| flatten(&[t.a, t.b, t.c])),
            _ => None,
        })
    }

    /// The indices of this triangle mesh, polyline, or convex polyhedron, if it is one.
    pub fn coIndices(&self, handle: FlatHandle) -> Option<Vec<u32>> {
        self.map(handle, |co| match co.shape().shape_type() {
            ShapeType::TriMesh => co
                .shape()
                .as_trimesh()
                .map(|t| t.indices().iter().flat_map(|p| p.iter()).copied().collect()),
            ShapeType::Polyline => co
                .shape()
                .as_polyline()
                .map(|p| p.indices().iter().flat_map(|p| p.iter()).copied().collect()),
            #[cfg(feature = "dim3")]
            ShapeType::ConvexPolyhedron => co
                .shape()
                .as_convex_polyhedron()
                .and_then(normalized_convex_polyhedron_mesh)
                .map(|(_, indices)| indices),
            #[cfg(feature = "dim3")]
            ShapeType::RoundConvexPolyhedron => co
                .shape()
                .as_round_convex_polyhedron()
                .and_then(|p| normalized_convex_polyhedron_mesh(&p.inner_shape))
                .map(|(_, indices)| indices),
            _ => None,
        })
    }

    /// The height of this heightfield if it is one.
    #[cfg(feature = "dim2")]
    pub fn coHeightfieldHeights(&self, handle: FlatHandle) -> Option<Vec<f32>> {
        self.map(handle, |co| match co.shape().shape_type() {
            ShapeType::HeightField => co.shape().as_heightfield().map(|h| h.heights().to_vec()),
            _ => None,
        })
    }

    /// The height of this heightfield if it is one.
    #[cfg(feature = "dim3")]
    pub fn coHeightfieldHeights(&self, handle: FlatHandle) -> Option<Vec<f32>> {
        self.map(handle, |co| match co.shape().shape_type() {
            ShapeType::HeightField => co
                .shape()
                .as_heightfield()
                .map(|h| h.heights().data().to_vec()),
            _ => None,
        })
    }

    /// The scale of a heightfield collider, written to the scratch buffer.
    /// Returns `false` (and writes nothing) for any other shape.
    pub fn coHeightfieldScale(&self, handle: FlatHandle) -> bool {
        self.map(handle, |co| match co.shape().shape_type() {
            ShapeType::HeightField => co
                .shape()
                .as_heightfield()
                .map(|h| scratch::write_vector(h.scale()))
                .is_some(),
            _ => false,
        })
    }

    /// The number of rows on this heightfield's height matrix, if it is one.
    #[cfg(feature = "dim3")]
    pub fn coHeightfieldNRows(&self, handle: FlatHandle) -> Option<usize> {
        self.map(handle, |co| match co.shape().shape_type() {
            ShapeType::HeightField => co.shape().as_heightfield().map(|h| h.nrows()),
            _ => None,
        })
    }

    /// The number of columns on this heightfield's height matrix, if it is one.
    #[cfg(feature = "dim3")]
    pub fn coHeightfieldNCols(&self, handle: FlatHandle) -> Option<usize> {
        self.map(handle, |co| match co.shape().shape_type() {
            ShapeType::HeightField => co.shape().as_heightfield().map(|h| h.ncols()),
            _ => None,
        })
    }

    /// The unique integer identifier of the collider this collider is attached to.
    pub fn coParent(&self, handle: FlatHandle) -> Option<FlatHandle> {
        self.map(handle, |co| co.parent().map(|p| utils::flat_handle(p.0)))
    }

    pub fn coSetEnabled(&mut self, handle: FlatHandle, enabled: bool) {
        self.map_mut_untracked(handle, |co| co.set_enabled(enabled))
    }

    pub fn coIsEnabled(&self, handle: FlatHandle) -> bool {
        self.map(handle, |co| co.is_enabled())
    }

    pub fn coSetContactSkin(&mut self, handle: FlatHandle, contact_skin: f32) {
        self.map_mut_untracked(handle, |co| co.set_contact_skin(contact_skin))
    }

    pub fn coContactSkin(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |co| co.contact_skin())
    }

    /// The friction coefficient of this collider.
    pub fn coFriction(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |co| co.material().friction)
    }
    /// The restitution coefficient of this collider.
    pub fn coRestitution(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |co| co.material().restitution)
    }

    /// The density of this collider.
    pub fn coDensity(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |co| co.density())
    }

    /// The mass of this collider.
    pub fn coMass(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |co| co.mass())
    }

    /// The volume of this collider.
    pub fn coVolume(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |co| co.volume())
    }

    /// The collision groups of this collider.
    pub fn coCollisionGroups(&self, handle: FlatHandle) -> u32 {
        self.map(handle, |co| {
            super::pack_interaction_groups(co.collision_groups())
        })
    }

    /// The solver groups of this collider.
    pub fn coSolverGroups(&self, handle: FlatHandle) -> u32 {
        self.map(handle, |co| {
            super::pack_interaction_groups(co.solver_groups())
        })
    }

    /// The physics hooks enabled for this collider.
    pub fn coActiveHooks(&self, handle: FlatHandle) -> u32 {
        self.map(handle, |co| co.active_hooks().bits())
    }

    /// The collision types enabled for this collider.
    pub fn coActiveCollisionTypes(&self, handle: FlatHandle) -> u16 {
        self.map(handle, |co| co.active_collision_types().bits())
    }

    /// The events enabled for this collider.
    pub fn coActiveEvents(&self, handle: FlatHandle) -> u32 {
        self.map(handle, |co| co.active_events().bits())
    }

    /// The total force magnitude beyond which a contact force event can be emitted.
    pub fn coContactForceEventThreshold(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |co| co.contact_force_event_threshold())
    }

    #[cfg(feature = "dim2")]
    pub fn coContainsPoint(&self, handle: FlatHandle, px: f32, py: f32) -> bool {
        let point = Vector::new(px, py);
        self.map(handle, |co| {
            co.shared_shape().containsPoint(co.position(), point)
        })
    }

    #[cfg(feature = "dim3")]
    pub fn coContainsPoint(&self, handle: FlatHandle, px: f32, py: f32, pz: f32) -> bool {
        let point = Vector::new(px, py, pz);
        self.map(handle, |co| {
            co.shared_shape().containsPoint(co.position(), point)
        })
    }

    /// Casts this collider's shape against `shape2` and, on a hit, writes it into
    /// the scratch buffer (`[time_of_impact, witness1, witness2, normal1, normal2]`).
    ///
    /// The pose and velocities are passed component-wise so the JS side
    /// allocates nothing but `shape2` per call.
    #[cfg(feature = "dim2")]
    pub fn coCastShape(
        &self,
        handle: FlatHandle,
        vel1_x: f32,
        vel1_y: f32,
        shape2: &RawShape,
        pos2_x: f32,
        pos2_y: f32,
        rot2: f32,
        vel2_x: f32,
        vel2_y: f32,
        target_distance: f32,
        maxToi: f32,
        stop_at_penetration: bool,
    ) -> bool {
        self.do_cast_shape(
            handle,
            &Vector::new(vel1_x, vel1_y),
            shape2,
            &pose_from_scalars(pos2_x, pos2_y, rot2),
            &Vector::new(vel2_x, vel2_y),
            target_distance,
            maxToi,
            stop_at_penetration,
        )
    }

    /// Casts this collider's shape against `shape2` and, on a hit, writes it into
    /// the scratch buffer (`[time_of_impact, witness1, witness2, normal1, normal2]`).
    ///
    /// The pose and velocities are passed component-wise so the JS side
    /// allocates nothing but `shape2` per call.
    #[cfg(feature = "dim3")]
    pub fn coCastShape(
        &self,
        handle: FlatHandle,
        vel1_x: f32,
        vel1_y: f32,
        vel1_z: f32,
        shape2: &RawShape,
        pos2_x: f32,
        pos2_y: f32,
        pos2_z: f32,
        rot2_x: f32,
        rot2_y: f32,
        rot2_z: f32,
        rot2_w: f32,
        vel2_x: f32,
        vel2_y: f32,
        vel2_z: f32,
        target_distance: f32,
        maxToi: f32,
        stop_at_penetration: bool,
    ) -> bool {
        self.do_cast_shape(
            handle,
            &Vector::new(vel1_x, vel1_y, vel1_z),
            shape2,
            &pose_from_scalars(pos2_x, pos2_y, pos2_z, rot2_x, rot2_y, rot2_z, rot2_w),
            &Vector::new(vel2_x, vel2_y, vel2_z),
            target_distance,
            maxToi,
            stop_at_penetration,
        )
    }

    /// Casts this collider against another one and, on a hit, writes it into the
    /// scratch buffer. A removed second collider is a miss, not a trap.
    #[cfg(feature = "dim2")]
    pub fn coCastCollider(
        &self,
        handle: FlatHandle,
        vel1_x: f32,
        vel1_y: f32,
        collider2handle: FlatHandle,
        vel2_x: f32,
        vel2_y: f32,
        target_distance: f32,
        max_toi: f32,
        stop_at_penetration: bool,
    ) -> bool {
        self.do_cast_collider(
            handle,
            Vector::new(vel1_x, vel1_y),
            collider2handle,
            Vector::new(vel2_x, vel2_y),
            target_distance,
            max_toi,
            stop_at_penetration,
        )
    }

    /// Casts this collider against another one and, on a hit, writes it into the
    /// scratch buffer. A removed second collider is a miss, not a trap.
    #[cfg(feature = "dim3")]
    pub fn coCastCollider(
        &self,
        handle: FlatHandle,
        vel1_x: f32,
        vel1_y: f32,
        vel1_z: f32,
        collider2handle: FlatHandle,
        vel2_x: f32,
        vel2_y: f32,
        vel2_z: f32,
        target_distance: f32,
        max_toi: f32,
        stop_at_penetration: bool,
    ) -> bool {
        self.do_cast_collider(
            handle,
            Vector::new(vel1_x, vel1_y, vel1_z),
            collider2handle,
            Vector::new(vel2_x, vel2_y, vel2_z),
            target_distance,
            max_toi,
            stop_at_penetration,
        )
    }

    #[cfg(feature = "dim2")]
    pub fn coIntersectsShape(
        &self,
        handle: FlatHandle,
        shape2: &RawShape,
        pos2_x: f32,
        pos2_y: f32,
        rot2: f32,
    ) -> bool {
        let pos2 = pose_from_scalars(pos2_x, pos2_y, rot2);
        self.map(handle, |co| {
            co.shared_shape()
                .intersectsShape(co.position(), &*shape2.0, &pos2)
        })
    }

    #[cfg(feature = "dim3")]
    pub fn coIntersectsShape(
        &self,
        handle: FlatHandle,
        shape2: &RawShape,
        pos2_x: f32,
        pos2_y: f32,
        pos2_z: f32,
        rot2_x: f32,
        rot2_y: f32,
        rot2_z: f32,
        rot2_w: f32,
    ) -> bool {
        let pos2 = pose_from_scalars(pos2_x, pos2_y, pos2_z, rot2_x, rot2_y, rot2_z, rot2_w);
        self.map(handle, |co| {
            co.shared_shape()
                .intersectsShape(co.position(), &*shape2.0, &pos2)
        })
    }

    /// Computes the contact between this collider and `shape2` and, if there is
    /// one within `prediction`, writes it into the scratch buffer
    /// (`[distance, point1, point2, normal1, normal2]`).
    #[cfg(feature = "dim2")]
    pub fn coContactShape(
        &self,
        handle: FlatHandle,
        shape2: &RawShape,
        pos2_x: f32,
        pos2_y: f32,
        rot2: f32,
        prediction: f32,
    ) -> bool {
        self.do_contact_shape(
            handle,
            shape2,
            &pose_from_scalars(pos2_x, pos2_y, rot2),
            prediction,
        )
    }

    /// Computes the contact between this collider and `shape2` and, if there is
    /// one within `prediction`, writes it into the scratch buffer
    /// (`[distance, point1, point2, normal1, normal2]`).
    #[cfg(feature = "dim3")]
    pub fn coContactShape(
        &self,
        handle: FlatHandle,
        shape2: &RawShape,
        pos2_x: f32,
        pos2_y: f32,
        pos2_z: f32,
        rot2_x: f32,
        rot2_y: f32,
        rot2_z: f32,
        rot2_w: f32,
        prediction: f32,
    ) -> bool {
        self.do_contact_shape(
            handle,
            shape2,
            &pose_from_scalars(pos2_x, pos2_y, pos2_z, rot2_x, rot2_y, rot2_z, rot2_w),
            prediction,
        )
    }

    /// Computes the contact between two colliders and, if there is one within
    /// `prediction`, writes it into the scratch buffer. A removed second
    /// collider is a miss, not a trap.
    pub fn coContactCollider(
        &self,
        handle: FlatHandle,
        collider2handle: FlatHandle,
        prediction: f32,
    ) -> bool {
        let Some(co2) = self.0.get(utils::collider_handle(collider2handle)) else {
            return false;
        };

        self.map(handle, |co| {
            match query::contact(
                co.position(),
                co.shape(),
                &co2.position(),
                co2.shape(),
                prediction,
            )
            .ok()
            .flatten()
            {
                Some(contact) => {
                    write_contact(&contact);
                    true
                }
                None => false,
            }
        })
    }

    #[cfg(feature = "dim2")]
    /// Projects a point on this collider, writing `point, isInside` to the scratch buffer.
    pub fn coProjectPoint(&self, handle: FlatHandle, px: f32, py: f32, solid: bool) {
        let point = Vector::new(px, py);
        self.map(handle, |co| {
            write_point_projection(&co.shared_shape().projectPoint(co.position(), point, solid))
        })
    }

    #[cfg(feature = "dim3")]
    /// Projects a point on this collider, writing `point, isInside` to the scratch buffer.
    pub fn coProjectPoint(&self, handle: FlatHandle, px: f32, py: f32, pz: f32, solid: bool) {
        let point = Vector::new(px, py, pz);
        self.map(handle, |co| {
            write_point_projection(&co.shared_shape().projectPoint(co.position(), point, solid))
        })
    }

    #[cfg(feature = "dim2")]
    pub fn coIntersectsRay(
        &self,
        handle: FlatHandle,
        ox: f32,
        oy: f32,
        dx: f32,
        dy: f32,
        maxToi: f32,
    ) -> bool {
        let (orig, dir) = (Vector::new(ox, oy), Vector::new(dx, dy));
        self.map(handle, |co| {
            co.shared_shape()
                .intersectsRay(co.position(), orig, dir, maxToi)
        })
    }

    #[cfg(feature = "dim3")]
    pub fn coIntersectsRay(
        &self,
        handle: FlatHandle,
        ox: f32,
        oy: f32,
        oz: f32,
        dx: f32,
        dy: f32,
        dz: f32,
        maxToi: f32,
    ) -> bool {
        let (orig, dir) = (Vector::new(ox, oy, oz), Vector::new(dx, dy, dz));
        self.map(handle, |co| {
            co.shared_shape()
                .intersectsRay(co.position(), orig, dir, maxToi)
        })
    }

    #[cfg(feature = "dim2")]
    /// Casts a ray on this collider. Returns the time of impact, or a negative
    /// value if there is no hit.
    pub fn coCastRay(
        &self,
        handle: FlatHandle,
        ox: f32,
        oy: f32,
        dx: f32,
        dy: f32,
        maxToi: f32,
        solid: bool,
    ) -> f32 {
        let (orig, dir) = (Vector::new(ox, oy), Vector::new(dx, dy));
        self.map(handle, |co| {
            co.shared_shape()
                .castRay(co.position(), orig, dir, maxToi, solid)
        })
    }

    #[cfg(feature = "dim3")]
    /// Casts a ray on this collider. Returns the time of impact, or a negative
    /// value if there is no hit.
    pub fn coCastRay(
        &self,
        handle: FlatHandle,
        ox: f32,
        oy: f32,
        oz: f32,
        dx: f32,
        dy: f32,
        dz: f32,
        maxToi: f32,
        solid: bool,
    ) -> f32 {
        let (orig, dir) = (Vector::new(ox, oy, oz), Vector::new(dx, dy, dz));
        self.map(handle, |co| {
            co.shared_shape()
                .castRay(co.position(), orig, dir, maxToi, solid)
        })
    }

    #[cfg(feature = "dim2")]
    /// Casts a ray on this collider, writing `timeOfImpact, normal, featureType,
    /// featureId` to the scratch buffer. Returns `false` (and writes nothing) on
    /// a miss.
    pub fn coCastRayAndGetNormal(
        &self,
        handle: FlatHandle,
        ox: f32,
        oy: f32,
        dx: f32,
        dy: f32,
        maxToi: f32,
        solid: bool,
    ) -> bool {
        let (orig, dir) = (Vector::new(ox, oy), Vector::new(dx, dy));
        self.map(handle, |co| {
            match co
                .shared_shape()
                .castRayAndGetNormal(co.position(), orig, dir, maxToi, solid)
            {
                Some(inter) => {
                    write_ray_intersection(&inter);
                    true
                }
                None => false,
            }
        })
    }

    #[cfg(feature = "dim3")]
    /// Casts a ray on this collider, writing `timeOfImpact, normal, featureType,
    /// featureId` to the scratch buffer. Returns `false` (and writes nothing) on
    /// a miss.
    pub fn coCastRayAndGetNormal(
        &self,
        handle: FlatHandle,
        ox: f32,
        oy: f32,
        oz: f32,
        dx: f32,
        dy: f32,
        dz: f32,
        maxToi: f32,
        solid: bool,
    ) -> bool {
        let (orig, dir) = (Vector::new(ox, oy, oz), Vector::new(dx, dy, dz));
        self.map(handle, |co| {
            match co
                .shared_shape()
                .castRayAndGetNormal(co.position(), orig, dir, maxToi, solid)
            {
                Some(inter) => {
                    write_ray_intersection(&inter);
                    true
                }
                None => false,
            }
        })
    }

    pub fn coSetSensor(&mut self, handle: FlatHandle, is_sensor: bool) {
        self.map_mut_untracked(handle, |co| co.set_sensor(is_sensor))
    }

    pub fn coSetRestitution(&mut self, handle: FlatHandle, restitution: f32) {
        self.map_mut_untracked(handle, |co| co.set_restitution(restitution))
    }

    pub fn coSetFriction(&mut self, handle: FlatHandle, friction: f32) {
        self.map_mut_untracked(handle, |co| co.set_friction(friction))
    }

    pub fn coFrictionCombineRule(&self, handle: FlatHandle) -> u32 {
        self.map(handle, |co| co.friction_combine_rule() as u32)
    }

    pub fn coSetFrictionCombineRule(&mut self, handle: FlatHandle, rule: u32) {
        let rule = super::combine_rule_from_u32(rule);
        self.map_mut_untracked(handle, |co| co.set_friction_combine_rule(rule))
    }

    pub fn coRestitutionCombineRule(&self, handle: FlatHandle) -> u32 {
        self.map(handle, |co| co.restitution_combine_rule() as u32)
    }

    pub fn coSetRestitutionCombineRule(&mut self, handle: FlatHandle, rule: u32) {
        let rule = super::combine_rule_from_u32(rule);
        self.map_mut_untracked(handle, |co| co.set_restitution_combine_rule(rule))
    }

    pub fn coSetCollisionGroups(&mut self, handle: FlatHandle, groups: u32) {
        let groups = super::unpack_interaction_groups(groups);
        self.map_mut_untracked(handle, |co| co.set_collision_groups(groups))
    }

    pub fn coSetSolverGroups(&mut self, handle: FlatHandle, groups: u32) {
        let groups = super::unpack_interaction_groups(groups);
        self.map_mut_untracked(handle, |co| co.set_solver_groups(groups))
    }

    pub fn coSetActiveHooks(&mut self, handle: FlatHandle, hooks: u32) {
        let hooks = ActiveHooks::from_bits_truncate(hooks);
        self.map_mut_untracked(handle, |co| co.set_active_hooks(hooks));
    }

    pub fn coSetActiveEvents(&mut self, handle: FlatHandle, events: u32) {
        let events = ActiveEvents::from_bits_truncate(events);
        self.map_mut_untracked(handle, |co| co.set_active_events(events))
    }

    pub fn coSetActiveCollisionTypes(&mut self, handle: FlatHandle, types: u16) {
        let types = ActiveCollisionTypes::from_bits_truncate(types);
        self.map_mut_untracked(handle, |co| co.set_active_collision_types(types));
    }

    pub fn coSetShape(&mut self, handle: FlatHandle, shape: &RawShape) {
        self.map_mut_untracked(handle, |co| co.set_shape(shape.0.clone()));
    }

    pub fn coSetContactForceEventThreshold(&mut self, handle: FlatHandle, threshold: f32) {
        self.map_mut_untracked(handle, |co| co.set_contact_force_event_threshold(threshold))
    }

    pub fn coSetDensity(&mut self, handle: FlatHandle, density: f32) {
        self.map_mut_untracked(handle, |co| co.set_density(density))
    }

    pub fn coSetMass(&mut self, handle: FlatHandle, mass: f32) {
        self.map_mut_untracked(handle, |co| co.set_mass(mass))
    }

    /// Sets the mass properties of this collider, passed component-wise like
    /// every other setter (a `RawVector`/`RawRotation` temporary is a WASM
    /// allocation plus a `FinalizationRegistry` registration each). The inertia
    /// frame is normalized like every other quaternion input, falling back to the
    /// identity when it has no direction to recover.
    #[cfg(feature = "dim3")]
    pub fn coSetMassProperties(
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
    ) {
        self.map_mut_untracked(handle, |co| {
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

            co.set_mass_properties(mprops)
        })
    }

    /// Sets the mass properties of this collider; see the 3D variant.
    #[cfg(feature = "dim2")]
    pub fn coSetMassProperties(
        &mut self,
        handle: FlatHandle,
        mass: f32,
        centerOfMass_x: f32,
        centerOfMass_y: f32,
        principalAngularInertia: f32,
    ) {
        self.map_mut_untracked(handle, |co| {
            let props = MassProperties::new(
                Vector::new(centerOfMass_x, centerOfMass_y).into(),
                mass,
                principalAngularInertia,
            );
            co.set_mass_properties(props)
        })
    }
}

impl RawColliderSet {
    fn do_cast_shape(
        &self,
        handle: FlatHandle,
        vel1: &Vector,
        shape2: &RawShape,
        pos2: &Pose,
        vel2: &Vector,
        target_distance: f32,
        max_toi: f32,
        stop_at_penetration: bool,
    ) -> bool {
        self.map(handle, |co| {
            match co.shared_shape().castShape(
                co.position(),
                vel1,
                &*shape2.0,
                pos2,
                vel2,
                target_distance,
                max_toi,
                stop_at_penetration,
            ) {
                Some(hit) => {
                    write_hit(&hit);
                    true
                }
                None => false,
            }
        })
    }

    fn do_cast_collider(
        &self,
        handle: FlatHandle,
        vel1: Vector,
        collider2handle: FlatHandle,
        vel2: Vector,
        target_distance: f32,
        max_toi: f32,
        stop_at_penetration: bool,
    ) -> bool {
        let Some(co2) = self.0.get(utils::collider_handle(collider2handle)) else {
            return false;
        };

        self.map(handle, |co| {
            match query::cast_shapes(
                co.position(),
                vel1,
                co.shape(),
                co2.position(),
                vel2,
                co2.shape(),
                ShapeCastOptions {
                    max_time_of_impact: max_toi,
                    stop_at_penetration,
                    target_distance,
                    compute_impact_geometry_on_penetration: true,
                },
            )
            .ok()
            .flatten()
            {
                Some(hit) => {
                    write_hit(&hit);
                    true
                }
                None => false,
            }
        })
    }

    fn do_contact_shape(
        &self,
        handle: FlatHandle,
        shape2: &RawShape,
        pos2: &Pose,
        prediction: f32,
    ) -> bool {
        self.map(handle, |co| {
            match co
                .shared_shape()
                .contactShape(co.position(), &*shape2.0, pos2, prediction)
            {
                Some(contact) => {
                    write_contact(&contact);
                    true
                }
                None => false,
            }
        })
    }

    fn do_set_half_extents(&mut self, handle: FlatHandle, half_extents: Vector) {
        self.map_mut_untracked(handle, |co| match co.shape().shape_type() {
            ShapeType::Cuboid => co
                .shape_mut()
                .as_cuboid_mut()
                .map(|b| b.half_extents = half_extents.into()),
            ShapeType::RoundCuboid => co
                .shape_mut()
                .as_round_cuboid_mut()
                .map(|b| b.inner_shape.half_extents = half_extents.into()),
            _ => None,
        });
    }
}
