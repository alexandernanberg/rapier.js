use crate::geometry::{RawPointProjection, RawRayIntersection, RawShapeCastHit, RawShapeContact};
use crate::math::{RawRotation, RawVector};
use rapier::geometry::{Shape, SharedShape, TriMeshFlags};
use rapier::math::{IVector, Pose, Rotation, Vector, DIM};
use rapier::parry::query;
use rapier::parry::query::{Ray, ShapeCastOptions};
use rapier::parry::transformation::vhacd::{VHACDParameters, VHACD};
use wasm_bindgen::prelude::*;

/// The vertices and indices of a convex polyhedron's convex hull, recomputed with
/// `try_convex_hull` so that they are consistent with each other.
///
/// The points a `ConvexPolyhedron` stores and the triangulation it exposes don't
/// necessarily agree (collinear boundary vertices are dropped from one but not the
/// other), so reading them separately can yield a mesh that can't be rebuilt.
#[cfg(feature = "dim3")]
pub(crate) fn normalized_convex_polyhedron_mesh(
    polyhedron: &rapier::parry::shape::ConvexPolyhedron,
) -> Option<(Vec<Vector>, Vec<u32>)> {
    let (points, indices) =
        rapier::parry::transformation::try_convex_hull(polyhedron.points()).ok()?;
    let flat_indices = indices.iter().flat_map(|tri| tri.iter()).copied().collect();
    Some((points, flat_indices))
}

/// Splits a flat `[x, y, (z), ...]` array into points, or `None` if its length is
/// not a whole number of points.
///
/// These arrays come straight from user data (a mesh loader's buffers, a typed
/// array sliced by hand), so a bad length is an input error, not a bug. Letting
/// `chunks` hand a short slice to `Vector::from_slice` would panic, and a panic
/// aborts the module: it reaches JS as an uncatchable `RuntimeError: unreachable`,
/// and every later call fails too.
fn to_points(flat: &[f32]) -> Option<Vec<Vector>> {
    if flat.len() % DIM != 0 {
        return None;
    }

    Some(flat.chunks_exact(DIM).map(Vector::from_slice).collect())
}

/// Splits a flat index array into `N`-vertex elements, or `None` if its length is
/// not a whole number of elements. See [`to_points`] for why this is not a panic.
fn to_indices<const N: usize>(flat: &[u32]) -> Option<Vec<[u32; N]>> {
    if flat.len() % N != 0 {
        return None;
    }

    flat.chunks_exact(N)
        .map(|element| element.first_chunk::<N>().copied())
        .collect()
}

/// Whether every index addresses a vertex of a mesh with `num_vertices` vertices.
///
/// Parry indexes the vertex buffer directly when building a polyline, a convex mesh
/// or a convex decomposition — `TriMesh` is the only builder that validates its
/// indices and reports an error. Everywhere else an out-of-range index is an
/// out-of-bounds panic. Index buffers are user data, so this is an input error to
/// report, not a bug to trap on. See [`to_points`].
fn indices_in_range<const N: usize>(indices: &[[u32; N]], num_vertices: usize) -> bool {
    indices
        .iter()
        .flatten()
        .all(|index| (*index as usize) < num_vertices)
}

pub trait SharedShapeUtility {
    fn castShape(
        &self,
        shapePos1: &Pose,
        shapeVel1: &Vector,
        shape2: &dyn Shape,
        shapePos2: &Pose,
        shapeVel2: &Vector,
        target_distance: f32,
        maxToi: f32,
        stop_at_penetration: bool,
    ) -> Option<RawShapeCastHit>;

    fn intersectsShape(&self, shapePos1: &Pose, shape2: &dyn Shape, shapePos2: &Pose) -> bool;

    fn contactShape(
        &self,
        shapePos1: &Pose,
        shape2: &dyn Shape,
        shapePos2: &Pose,
        prediction: f32,
    ) -> Option<RawShapeContact>;

    fn containsPoint(&self, shapePos: &Pose, point: Vector) -> bool;

    fn projectPoint(&self, shapePos: &Pose, point: Vector, solid: bool) -> RawPointProjection;

    fn intersectsRay(&self, shapePos: &Pose, rayOrig: Vector, rayDir: Vector, maxToi: f32) -> bool;

    fn castRay(
        &self,
        shapePos: &Pose,
        rayOrig: Vector,
        rayDir: Vector,
        maxToi: f32,
        solid: bool,
    ) -> f32;

    fn castRayAndGetNormal(
        &self,
        shapePos: &Pose,
        rayOrig: Vector,
        rayDir: Vector,
        maxToi: f32,
        solid: bool,
    ) -> Option<RawRayIntersection>;
}

// for RawShape & Collider
impl SharedShapeUtility for SharedShape {
    fn castShape(
        &self,
        shapePos1: &Pose,
        shapeVel1: &Vector,
        shape2: &dyn Shape,
        shapePos2: &Pose,
        shapeVel2: &Vector,
        target_distance: f32,
        maxToi: f32,
        stop_at_penetration: bool,
    ) -> Option<RawShapeCastHit> {
        query::cast_shapes(
            shapePos1,
            *shapeVel1,
            &*self.0,
            shapePos2,
            *shapeVel2,
            shape2,
            ShapeCastOptions {
                max_time_of_impact: maxToi,
                target_distance,
                stop_at_penetration,
                compute_impact_geometry_on_penetration: true,
            },
        )
        .ok()
        .flatten()
        .map(|hit| RawShapeCastHit { hit })
    }

    fn intersectsShape(&self, shapePos1: &Pose, shape2: &dyn Shape, shapePos2: &Pose) -> bool {
        query::intersection_test(shapePos1, &*self.0, shapePos2, shape2).unwrap_or(false)
    }

    fn contactShape(
        &self,
        shapePos1: &Pose,
        shape2: &dyn Shape,
        shapePos2: &Pose,
        prediction: f32,
    ) -> Option<RawShapeContact> {
        query::contact(shapePos1, &*self.0, shapePos2, shape2, prediction)
            .ok()
            .flatten()
            .map(|contact| RawShapeContact { contact })
    }

    fn containsPoint(&self, shapePos: &Pose, point: Vector) -> bool {
        self.as_ref().contains_point(shapePos, point)
    }

    fn projectPoint(&self, shapePos: &Pose, point: Vector, solid: bool) -> RawPointProjection {
        RawPointProjection(self.as_ref().project_point(shapePos, point, solid))
    }

    fn intersectsRay(&self, shapePos: &Pose, rayOrig: Vector, rayDir: Vector, maxToi: f32) -> bool {
        self.as_ref()
            .intersects_ray(shapePos, &Ray::new(rayOrig, rayDir), maxToi)
    }

    fn castRay(
        &self,
        shapePos: &Pose,
        rayOrig: Vector,
        rayDir: Vector,
        maxToi: f32,
        solid: bool,
    ) -> f32 {
        self.as_ref()
            .cast_ray(shapePos, &Ray::new(rayOrig, rayDir), maxToi, solid)
            .unwrap_or(-1.0) // Negative value = no hit.
    }

    fn castRayAndGetNormal(
        &self,
        shapePos: &Pose,
        rayOrig: Vector,
        rayDir: Vector,
        maxToi: f32,
        solid: bool,
    ) -> Option<RawRayIntersection> {
        self.as_ref()
            .cast_ray_and_get_normal(shapePos, &Ray::new(rayOrig, rayDir), maxToi, solid)
            .map(|inter| RawRayIntersection(inter))
    }
}

#[wasm_bindgen]
#[cfg(feature = "dim2")]
pub enum RawShapeType {
    Ball = 0,
    Cuboid = 1,
    Capsule = 2,
    Segment = 3,
    Polyline = 4,
    Triangle = 5,
    TriMesh = 6,
    HeightField = 7,
    Compound = 8,
    ConvexPolygon = 9,
    RoundCuboid = 10,
    RoundTriangle = 11,
    RoundConvexPolygon = 12,
    HalfSpace = 13,
    Voxels = 14,
}

#[wasm_bindgen]
#[cfg(feature = "dim3")]
pub enum RawShapeType {
    Ball = 0,
    Cuboid = 1,
    Capsule = 2,
    Segment = 3,
    Polyline = 4,
    Triangle = 5,
    TriMesh = 6,
    HeightField = 7,
    Compound = 8,
    ConvexPolyhedron = 9,
    Cylinder = 10,
    Cone = 11,
    RoundCuboid = 12,
    RoundTriangle = 13,
    RoundCylinder = 14,
    RoundCone = 15,
    RoundConvexPolyhedron = 16,
    HalfSpace = 17,
    Voxels = 18,
}

/// Parameters of the VHACD convex-decomposition algorithm.
#[wasm_bindgen]
pub struct RawVHACDParameters(pub(crate) VHACDParameters);

#[wasm_bindgen]
impl RawVHACDParameters {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Self(VHACDParameters::default())
    }

    #[wasm_bindgen(getter)]
    pub fn concavity(&self) -> f32 {
        self.0.concavity
    }

    #[wasm_bindgen(setter)]
    pub fn set_concavity(&mut self, val: f32) {
        self.0.concavity = val.clamp(0.0, 1.0);
    }

    #[wasm_bindgen(getter)]
    pub fn alpha(&self) -> f32 {
        self.0.alpha
    }

    #[wasm_bindgen(setter)]
    pub fn set_alpha(&mut self, val: f32) {
        self.0.alpha = val.clamp(0.0, 1.0);
    }

    #[wasm_bindgen(getter)]
    pub fn beta(&self) -> f32 {
        self.0.beta
    }

    #[wasm_bindgen(setter)]
    pub fn set_beta(&mut self, val: f32) {
        self.0.beta = val.clamp(0.0, 1.0);
    }

    #[wasm_bindgen(getter)]
    pub fn resolution(&self) -> u32 {
        self.0.resolution
    }

    #[wasm_bindgen(setter)]
    pub fn set_resolution(&mut self, val: u32) {
        self.0.resolution = val;
    }

    #[wasm_bindgen(getter)]
    pub fn plane_downsampling(&self) -> u32 {
        self.0.plane_downsampling
    }

    #[wasm_bindgen(setter)]
    pub fn set_plane_downsampling(&mut self, val: u32) {
        self.0.plane_downsampling = val;
    }

    #[wasm_bindgen(getter)]
    pub fn convex_hull_downsampling(&self) -> u32 {
        self.0.convex_hull_downsampling
    }

    #[wasm_bindgen(setter)]
    pub fn set_convex_hull_downsampling(&mut self, val: u32) {
        self.0.convex_hull_downsampling = val;
    }

    #[wasm_bindgen(getter)]
    pub fn max_convex_hulls(&self) -> u32 {
        self.0.max_convex_hulls
    }

    #[wasm_bindgen(setter)]
    pub fn set_max_convex_hulls(&mut self, val: u32) {
        self.0.max_convex_hulls = val;
    }

    #[wasm_bindgen(getter)]
    pub fn convex_hull_approximation(&self) -> bool {
        self.0.convex_hull_approximation
    }

    #[wasm_bindgen(setter)]
    pub fn set_convex_hull_approximation(&mut self, val: bool) {
        self.0.convex_hull_approximation = val;
    }
}

/// The vertex/index buffers of a convex polyhedron's convex hull.
#[cfg(feature = "dim3")]
#[wasm_bindgen(getter_with_clone)]
pub struct RawConvexMeshData {
    pub vertices: Vec<f32>,
    pub indices: Vec<u32>,
}

#[wasm_bindgen]
pub struct RawShape(pub(crate) SharedShape);

#[wasm_bindgen]
impl RawShape {
    pub fn shapeType(&self) -> RawShapeType {
        use rapier::geometry::ShapeType;

        match self.0.shape_type() {
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
            ShapeType::RoundCuboid => RawShapeType::RoundCuboid,
            ShapeType::RoundTriangle => RawShapeType::RoundTriangle,
            #[cfg(feature = "dim2")]
            ShapeType::ConvexPolygon => RawShapeType::ConvexPolygon,
            #[cfg(feature = "dim2")]
            ShapeType::RoundConvexPolygon => RawShapeType::RoundConvexPolygon,
            #[cfg(feature = "dim3")]
            ShapeType::ConvexPolyhedron => RawShapeType::ConvexPolyhedron,
            #[cfg(feature = "dim3")]
            ShapeType::RoundConvexPolyhedron => RawShapeType::RoundConvexPolyhedron,
            #[cfg(feature = "dim3")]
            ShapeType::Cylinder => RawShapeType::Cylinder,
            #[cfg(feature = "dim3")]
            ShapeType::RoundCylinder => RawShapeType::RoundCylinder,
            #[cfg(feature = "dim3")]
            ShapeType::Cone => RawShapeType::Cone,
            #[cfg(feature = "dim3")]
            ShapeType::RoundCone => RawShapeType::RoundCone,
            ShapeType::Custom => panic!("Custom shapes are not supported by the JS bindings."),
        }
    }

    /// The outward normal of this shape if it is a half-space.
    pub fn halfspaceNormal(&self) -> Option<RawVector> {
        self.0.as_halfspace().map(|h| h.normal.into())
    }

    /// The half-extents of this shape if it is a cuboid or round cuboid.
    pub fn halfExtents(&self) -> Option<RawVector> {
        self.0
            .as_cuboid()
            .map(|c| c.half_extents.into())
            .or_else(|| {
                self.0
                    .as_round_cuboid()
                    .map(|c| c.inner_shape.half_extents.into())
            })
    }

    /// The radius of this shape if it is a ball, capsule, cylinder or cone.
    pub fn radius(&self) -> Option<f32> {
        if let Some(ball) = self.0.as_ball() {
            return Some(ball.radius);
        }
        if let Some(capsule) = self.0.as_capsule() {
            return Some(capsule.radius);
        }

        #[cfg(feature = "dim3")]
        {
            if let Some(cylinder) = self.0.as_cylinder() {
                return Some(cylinder.radius);
            }
            if let Some(cylinder) = self.0.as_round_cylinder() {
                return Some(cylinder.inner_shape.radius);
            }
            if let Some(cone) = self.0.as_cone() {
                return Some(cone.radius);
            }
            if let Some(cone) = self.0.as_round_cone() {
                return Some(cone.inner_shape.radius);
            }
        }

        None
    }

    /// The half-height of this shape if it is a capsule, cylinder or cone.
    pub fn halfHeight(&self) -> Option<f32> {
        if let Some(capsule) = self.0.as_capsule() {
            return Some(capsule.half_height());
        }

        #[cfg(feature = "dim3")]
        {
            if let Some(cylinder) = self.0.as_cylinder() {
                return Some(cylinder.half_height);
            }
            if let Some(cylinder) = self.0.as_round_cylinder() {
                return Some(cylinder.inner_shape.half_height);
            }
            if let Some(cone) = self.0.as_cone() {
                return Some(cone.half_height);
            }
            if let Some(cone) = self.0.as_round_cone() {
                return Some(cone.inner_shape.half_height);
            }
        }

        None
    }

    /// The border radius of this shape if it is a round shape.
    pub fn roundRadius(&self) -> Option<f32> {
        if let Some(cuboid) = self.0.as_round_cuboid() {
            return Some(cuboid.border_radius);
        }
        if let Some(triangle) = self.0.as_round_triangle() {
            return Some(triangle.border_radius);
        }

        #[cfg(feature = "dim2")]
        if let Some(polygon) = self.0.as_round_convex_polygon() {
            return Some(polygon.border_radius);
        }

        #[cfg(feature = "dim3")]
        {
            if let Some(cylinder) = self.0.as_round_cylinder() {
                return Some(cylinder.border_radius);
            }
            if let Some(cone) = self.0.as_round_cone() {
                return Some(cone.border_radius);
            }
            if let Some(polyhedron) = self.0.as_round_convex_polyhedron() {
                return Some(polyhedron.border_radius);
            }
        }

        None
    }

    /// The grid coordinates of this shape's non-empty voxels, if it is a voxels shape.
    pub fn voxelData(&self) -> Option<Vec<i32>> {
        let voxels = self.0.as_voxels()?;
        Some(
            voxels
                .voxels()
                .filter_map(|vox| (!vox.state.is_empty()).then_some(vox.grid_coords))
                .flat_map(|ids| ids.to_array())
                .collect(),
        )
    }

    /// The size of a single voxel, if this shape is a voxels shape.
    pub fn voxelSize(&self) -> Option<RawVector> {
        self.0
            .as_voxels()
            .map(|voxels| RawVector(voxels.voxel_size()))
    }

    /// The vertices of this shape, if it is vertex-based.
    ///
    /// For convex polyhedra, this returns the vertices of a convex hull recomputed with
    /// `try_convex_hull` (so they may differ in count and order from the points the shape
    /// was built from), ensuring the result can be fed back to `RawShape::convexMesh`.
    /// If both `vertices` and `indices` are needed, prefer `convexMeshData`, which computes
    /// the convex hull only once.
    pub fn vertices(&self) -> Option<Vec<f32>> {
        let flatten = |vertices: &[Vector]| -> Vec<f32> {
            vertices.iter().flat_map(|p| p.to_array()).collect()
        };

        if let Some(mesh) = self.0.as_trimesh() {
            return Some(flatten(mesh.vertices()));
        }
        if let Some(polyline) = self.0.as_polyline() {
            return Some(flatten(polyline.vertices()));
        }
        if let Some(segment) = self.0.as_segment() {
            return Some(flatten(&[segment.a, segment.b]));
        }
        if let Some(triangle) = self.0.as_triangle() {
            return Some(flatten(&[triangle.a, triangle.b, triangle.c]));
        }
        if let Some(triangle) = self.0.as_round_triangle() {
            let t = &triangle.inner_shape;
            return Some(flatten(&[t.a, t.b, t.c]));
        }

        #[cfg(feature = "dim2")]
        {
            if let Some(polygon) = self.0.as_convex_polygon() {
                return Some(flatten(polygon.points()));
            }
            if let Some(polygon) = self.0.as_round_convex_polygon() {
                return Some(flatten(polygon.inner_shape.points()));
            }
        }

        #[cfg(feature = "dim3")]
        {
            if let Some(polyhedron) = self.0.as_convex_polyhedron() {
                return normalized_convex_polyhedron_mesh(polyhedron).map(|(p, _)| flatten(&p));
            }
            if let Some(polyhedron) = self.0.as_round_convex_polyhedron() {
                return normalized_convex_polyhedron_mesh(&polyhedron.inner_shape)
                    .map(|(p, _)| flatten(&p));
            }
        }

        None
    }

    /// The indices of this shape, if it is an indexed mesh.
    ///
    /// For convex polyhedra, the indices refer to the convex hull recomputed with
    /// `try_convex_hull` (matching `vertices`), not to the original input mesh.
    pub fn indices(&self) -> Option<Vec<u32>> {
        if let Some(mesh) = self.0.as_trimesh() {
            return Some(
                mesh.indices()
                    .iter()
                    .flat_map(|t| t.iter())
                    .copied()
                    .collect(),
            );
        }
        if let Some(polyline) = self.0.as_polyline() {
            return Some(
                polyline
                    .indices()
                    .iter()
                    .flat_map(|s| s.iter())
                    .copied()
                    .collect(),
            );
        }

        #[cfg(feature = "dim3")]
        {
            if let Some(polyhedron) = self.0.as_convex_polyhedron() {
                return normalized_convex_polyhedron_mesh(polyhedron).map(|(_, idx)| idx);
            }
            if let Some(polyhedron) = self.0.as_round_convex_polyhedron() {
                return normalized_convex_polyhedron_mesh(&polyhedron.inner_shape)
                    .map(|(_, idx)| idx);
            }
        }

        None
    }

    /// The vertices and indices of the convex hull of this convex polyhedron, recomputed
    /// with `try_convex_hull` so that the result can always be fed back to
    /// `RawShape::convexMesh`.
    ///
    /// This computes the convex hull only once, unlike calling both `vertices` and `indices`.
    #[cfg(feature = "dim3")]
    pub fn convexMeshData(&self) -> Option<RawConvexMeshData> {
        let polyhedron = self.0.as_convex_polyhedron().or_else(|| {
            self.0
                .as_round_convex_polyhedron()
                .map(|polyhedron| &polyhedron.inner_shape)
        })?;
        let (points, indices) = normalized_convex_polyhedron_mesh(polyhedron)?;
        Some(RawConvexMeshData {
            vertices: points.iter().flat_map(|p| p.to_array()).collect(),
            indices,
        })
    }

    pub fn triMeshFlags(&self) -> Option<u32> {
        self.0.as_trimesh().map(|t| t.flags().bits() as u32)
    }

    #[cfg(feature = "dim3")]
    pub fn heightFieldFlags(&self) -> Option<u32> {
        self.0.as_heightfield().map(|h| h.flags().bits() as u32)
    }

    pub fn heightfieldHeights(&self) -> Option<Vec<f32>> {
        self.0.as_heightfield().map(|h| {
            #[cfg(feature = "dim2")]
            {
                h.heights().as_slice().to_vec()
            }
            #[cfg(feature = "dim3")]
            {
                h.heights().data().to_vec()
            }
        })
    }

    pub fn heightfieldScale(&self) -> Option<RawVector> {
        self.0.as_heightfield().map(|h| RawVector(h.scale()))
    }

    #[cfg(feature = "dim3")]
    pub fn heightfieldNRows(&self) -> Option<usize> {
        self.0.as_heightfield().map(|h| h.nrows())
    }

    #[cfg(feature = "dim3")]
    pub fn heightfieldNCols(&self) -> Option<usize> {
        self.0.as_heightfield().map(|h| h.ncols())
    }

    /// The number of sub-shapes of this shape, if it is a compound shape.
    pub fn compoundLen(&self) -> Option<usize> {
        self.0.as_compound().map(|c| c.shapes().len())
    }

    /// The `index`-th sub-shape of this compound shape.
    pub fn compoundShape(&self, index: usize) -> Option<RawShape> {
        self.0
            .as_compound()
            .and_then(|c| c.shapes().get(index))
            .map(|(_, shape)| RawShape(shape.clone()))
    }

    /// The translation of the `index`-th sub-shape of this compound shape.
    pub fn compoundTranslation(&self, index: usize) -> Option<RawVector> {
        self.0
            .as_compound()
            .and_then(|c| c.shapes().get(index))
            .map(|(pose, _)| pose.translation.into())
    }

    /// The rotation of the `index`-th sub-shape of this compound shape.
    pub fn compoundRotation(&self, index: usize) -> Option<RawRotation> {
        self.0
            .as_compound()
            .and_then(|c| c.shapes().get(index))
            .map(|(pose, _)| pose.rotation.into())
    }

    #[cfg(feature = "dim2")]
    pub fn cuboid(hx: f32, hy: f32) -> Self {
        Self(SharedShape::cuboid(hx, hy))
    }

    #[cfg(feature = "dim3")]
    pub fn cuboid(hx: f32, hy: f32, hz: f32) -> Self {
        Self(SharedShape::cuboid(hx, hy, hz))
    }

    #[cfg(feature = "dim2")]
    pub fn roundCuboid(hx: f32, hy: f32, borderRadius: f32) -> Self {
        Self(SharedShape::round_cuboid(hx, hy, borderRadius))
    }

    #[cfg(feature = "dim3")]
    pub fn roundCuboid(hx: f32, hy: f32, hz: f32, borderRadius: f32) -> Self {
        Self(SharedShape::round_cuboid(hx, hy, hz, borderRadius))
    }

    pub fn ball(radius: f32) -> Self {
        Self(SharedShape::ball(radius))
    }

    pub fn halfspace(normal: &RawVector) -> Self {
        Self(SharedShape::halfspace(normal.0.normalize()))
    }

    pub fn capsule(halfHeight: f32, radius: f32) -> Self {
        let p2 = Vector::Y * halfHeight;
        let p1 = -p2;
        Self(SharedShape::capsule(p1, p2, radius))
    }

    #[cfg(feature = "dim3")]
    pub fn cylinder(halfHeight: f32, radius: f32) -> Self {
        Self(SharedShape::cylinder(halfHeight, radius))
    }

    #[cfg(feature = "dim3")]
    pub fn roundCylinder(halfHeight: f32, radius: f32, borderRadius: f32) -> Self {
        Self(SharedShape::round_cylinder(
            halfHeight,
            radius,
            borderRadius,
        ))
    }

    #[cfg(feature = "dim3")]
    pub fn cone(halfHeight: f32, radius: f32) -> Self {
        Self(SharedShape::cone(halfHeight, radius))
    }

    #[cfg(feature = "dim3")]
    pub fn roundCone(halfHeight: f32, radius: f32, borderRadius: f32) -> Self {
        Self(SharedShape::round_cone(halfHeight, radius, borderRadius))
    }

    pub fn voxels(voxel_size: &RawVector, grid_coords: Vec<i32>) -> Self {
        let grid_coords: Vec<_> = grid_coords
            .chunks_exact(DIM)
            .map(IVector::from_slice)
            .collect();
        Self(SharedShape::voxels(voxel_size.0, &grid_coords))
    }

    pub fn voxelsFromPoints(voxel_size: &RawVector, points: Vec<f32>) -> Self {
        let points: Vec<_> = points.chunks_exact(DIM).map(Vector::from_slice).collect();
        Self(SharedShape::voxels_from_points(voxel_size.0, &points))
    }

    /// Builds a polyline, or returns `None` if the buffers are ragged or a segment
    /// index addresses a vertex the vertex buffer doesn't have.
    pub fn polyline(vertices: Vec<f32>, indices: Vec<u32>) -> Option<RawShape> {
        let vertices = to_points(&vertices)?;
        let indices = to_indices::<2>(&indices)?;

        // `Polyline::new` reads `vertices[index]` for every segment endpoint.
        if !indices_in_range(&indices, vertices.len()) {
            return None;
        }

        // An empty index buffer means "connect the vertices into a line strip".
        let indices = (!indices.is_empty()).then_some(indices);
        Some(Self(SharedShape::polyline(vertices, indices)))
    }

    pub fn trimesh(vertices: Vec<f32>, indices: Vec<u32>, flags: u32) -> Option<RawShape> {
        let flags = TriMeshFlags::from_bits(flags as u16).unwrap_or_default();
        let vertices = to_points(&vertices)?;
        let indices = to_indices::<3>(&indices)?;
        SharedShape::trimesh_with_flags(vertices, indices, flags)
            .ok()
            .map(Self)
    }

    /// Builds a heightfield, or returns `None` if there are fewer than two heights.
    ///
    /// `HeightField::new` asserts on a degenerate grid: one height describes no
    /// segment, and the constructor computes `heights.len() - 1` of them.
    #[cfg(feature = "dim2")]
    pub fn heightfield(heights: Vec<f32>, scale: &RawVector) -> Option<RawShape> {
        if heights.len() < 2 {
            return None;
        }

        Some(Self(SharedShape::heightfield(heights, scale.0)))
    }

    /// Builds a heightfield, or returns `None` if the grid is degenerate or the
    /// height buffer doesn't hold exactly `(nrows + 1) * (ncols + 1)` entries.
    ///
    /// Both are asserts inside parry (in `Array2::new` and `HeightField::with_flags`
    /// respectively), and the grid dimensions come from user data.
    #[cfg(feature = "dim3")]
    pub fn heightfield(
        nrows: u32,
        ncols: u32,
        heights: Vec<f32>,
        scale: &RawVector,
        flags: u32,
    ) -> Option<RawShape> {
        use rapier::parry::utils::Array2;

        // `usize` is 32-bit on wasm32, so the row/column counts are one `u32::MAX`
        // away from wrapping and the product overflows well before that.
        let nrows = (nrows as usize).checked_add(1)?;
        let ncols = (ncols as usize).checked_add(1)?;
        let expected_heights = nrows.checked_mul(ncols)?;

        if nrows < 2 || ncols < 2 || heights.len() != expected_heights {
            return None;
        }

        let flags =
            rapier::parry::shape::HeightFieldFlags::from_bits(flags as u8).unwrap_or_default();
        let heights = Array2::new(nrows, ncols, heights);
        Some(Self(SharedShape::heightfield_with_flags(
            heights, scale.0, flags,
        )))
    }

    pub fn segment(p1: &RawVector, p2: &RawVector) -> Self {
        Self(SharedShape::segment(p1.0.into(), p2.0.into()))
    }

    pub fn triangle(p1: &RawVector, p2: &RawVector, p3: &RawVector) -> Self {
        Self(SharedShape::triangle(p1.0.into(), p2.0.into(), p3.0.into()))
    }

    pub fn roundTriangle(
        p1: &RawVector,
        p2: &RawVector,
        p3: &RawVector,
        borderRadius: f32,
    ) -> Self {
        Self(SharedShape::round_triangle(
            p1.0.into(),
            p2.0.into(),
            p3.0.into(),
            borderRadius,
        ))
    }

    pub fn convexHull(points: Vec<f32>) -> Option<RawShape> {
        let points = to_points(&points)?;
        SharedShape::convex_hull(&points).map(|s| Self(s))
    }

    pub fn roundConvexHull(points: Vec<f32>, borderRadius: f32) -> Option<RawShape> {
        let points = to_points(&points)?;
        SharedShape::round_convex_hull(&points, borderRadius).map(|s| Self(s))
    }

    #[cfg(feature = "dim2")]
    pub fn convexPolyline(vertices: Vec<f32>) -> Option<RawShape> {
        let vertices = to_points(&vertices)?;
        SharedShape::convex_polyline(vertices).map(|s| Self(s))
    }

    #[cfg(feature = "dim2")]
    pub fn roundConvexPolyline(vertices: Vec<f32>, borderRadius: f32) -> Option<RawShape> {
        let vertices = to_points(&vertices)?;
        SharedShape::round_convex_polyline(vertices, borderRadius).map(|s| Self(s))
    }

    #[cfg(feature = "dim3")]
    pub fn convexMesh(vertices: Vec<f32>, indices: Vec<u32>) -> Option<RawShape> {
        let vertices = to_points(&vertices)?;
        let indices = to_indices::<3>(&indices)?;

        // `ConvexPolyhedron::from_convex_mesh` reads `points[index]` while walking
        // the faces, so it panics rather than returning `None` on a bad index.
        if !indices_in_range(&indices, vertices.len()) {
            return None;
        }

        SharedShape::convex_mesh(vertices, &indices).map(|s| Self(s))
    }

    #[cfg(feature = "dim3")]
    pub fn roundConvexMesh(
        vertices: Vec<f32>,
        indices: Vec<u32>,
        borderRadius: f32,
    ) -> Option<RawShape> {
        let vertices = to_points(&vertices)?;
        let indices = to_indices::<3>(&indices)?;

        if !indices_in_range(&indices, vertices.len()) {
            return None;
        }

        SharedShape::round_convex_mesh(vertices, &indices, borderRadius).map(|s| Self(s))
    }

    /// Builds a compound shape from the given sub-shapes and their local poses, or
    /// returns `None` if the inputs don't describe a compound parry can build.
    ///
    /// `positions` must contain `DIM` entries per shape. `rotations` must contain one
    /// angle per shape in 2D, and one `{x, y, z, w}` quaternion (4 entries) per shape in 3D.
    /// Wrongly sized arrays used to be asserts here, and the two rules `Compound::new`
    /// enforces — at least one part, and no composite part — used to be panics inside
    /// parry. All four are input errors, so they are reported rather than trapped.
    pub fn compound(
        shapes: Vec<RawShape>,
        positions: Vec<f32>,
        rotations: Vec<f32>,
    ) -> Option<RawShape> {
        let num_shapes = shapes.len();

        #[cfg(feature = "dim2")]
        const ROTATION_LEN: usize = 1;
        #[cfg(feature = "dim3")]
        const ROTATION_LEN: usize = 4;

        if num_shapes == 0
            || positions.len() != num_shapes * DIM
            || rotations.len() != num_shapes * ROTATION_LEN
        {
            return None;
        }

        // A compound, trimesh or polyline part makes `Compound::new` panic with
        // "Nested composite shapes are not allowed."
        if shapes
            .iter()
            .any(|shape| shape.0.as_composite_shape().is_some())
        {
            return None;
        }

        let mut parts = Vec::with_capacity(num_shapes);

        for (i, shape) in shapes.iter().enumerate() {
            let translation = Vector::from_slice(&positions[i * DIM..(i + 1) * DIM]);

            #[cfg(feature = "dim2")]
            let rotation = Rotation::new(rotations[i]);
            #[cfg(feature = "dim3")]
            let rotation = {
                let o = i * 4;
                Rotation::from_xyzw(
                    rotations[o],
                    rotations[o + 1],
                    rotations[o + 2],
                    rotations[o + 3],
                )
            };

            parts.push((Pose::from_parts(translation, rotation), shape.0.clone()));
        }

        Some(Self(SharedShape::compound(parts)))
    }

    /// Decomposes the given mesh into a compound of convex parts, using VHACD with its
    /// default parameters.
    pub fn convexDecomposition(vertices: Vec<f32>, indices: Vec<u32>) -> Option<RawShape> {
        Self::convexDecompositionWithParams(vertices, indices, &RawVHACDParameters::new())
    }

    /// Decomposes the given mesh into a compound of convex parts, using VHACD with the
    /// given parameters.
    pub fn convexDecompositionWithParams(
        vertices: Vec<f32>,
        indices: Vec<u32>,
        params: &RawVHACDParameters,
    ) -> Option<RawShape> {
        let vertices = to_points(&vertices)?;
        #[cfg(feature = "dim2")]
        let indices = to_indices::<2>(&indices)?;
        #[cfg(feature = "dim3")]
        let indices = to_indices::<3>(&indices)?;

        // The voxelization pass reads `points[index]` for every element corner.
        if !indices_in_range(&indices, vertices.len()) {
            return None;
        }

        // Same as `SharedShape::convex_decomposition_with_params`, except that a
        // decomposition yielding no convex part returns `None` instead of panicking when
        // building the empty compound.
        let decomp = VHACD::decompose(&params.0, &vertices, &indices, true);
        let mut parts = vec![];

        #[cfg(feature = "dim2")]
        for hull_vertices in decomp.compute_exact_convex_hulls(&vertices, &indices) {
            if let Some(convex) = SharedShape::convex_polyline(hull_vertices) {
                parts.push((Pose::IDENTITY, convex));
            }
        }

        #[cfg(feature = "dim3")]
        for (hull_vertices, hull_indices) in decomp.compute_exact_convex_hulls(&vertices, &indices)
        {
            if let Some(convex) = SharedShape::convex_mesh(hull_vertices, &hull_indices) {
                parts.push((Pose::IDENTITY, convex));
            }
        }

        if parts.is_empty() {
            return None;
        }

        Some(Self(SharedShape::compound(parts)))
    }

    pub fn castShape(
        &self,
        shapePos1: &RawVector,
        shapeRot1: &RawRotation,
        shapeVel1: &RawVector,
        shape2: &RawShape,
        shapePos2: &RawVector,
        shapeRot2: &RawRotation,
        shapeVel2: &RawVector,
        target_distance: f32,
        maxToi: f32,
        stop_at_penetration: bool,
    ) -> Option<RawShapeCastHit> {
        let pos1 = Pose::from_parts(shapePos1.0, shapeRot1.0);
        let pos2 = Pose::from_parts(shapePos2.0, shapeRot2.0);

        self.0.castShape(
            &pos1,
            &shapeVel1.0,
            &*shape2.0,
            &pos2,
            &shapeVel2.0,
            target_distance,
            maxToi,
            stop_at_penetration,
        )
    }

    pub fn intersectsShape(
        &self,
        shapePos1: &RawVector,
        shapeRot1: &RawRotation,
        shape2: &RawShape,
        shapePos2: &RawVector,
        shapeRot2: &RawRotation,
    ) -> bool {
        let pos1 = Pose::from_parts(shapePos1.0, shapeRot1.0);
        let pos2 = Pose::from_parts(shapePos2.0, shapeRot2.0);

        self.0.intersectsShape(&pos1, &*shape2.0, &pos2)
    }

    pub fn contactShape(
        &self,
        shapePos1: &RawVector,
        shapeRot1: &RawRotation,
        shape2: &RawShape,
        shapePos2: &RawVector,
        shapeRot2: &RawRotation,
        prediction: f32,
    ) -> Option<RawShapeContact> {
        let pos1 = Pose::from_parts(shapePos1.0, shapeRot1.0);
        let pos2 = Pose::from_parts(shapePos2.0, shapeRot2.0);

        self.0.contactShape(&pos1, &*shape2.0, &pos2, prediction)
    }

    pub fn containsPoint(
        &self,
        shapePos: &RawVector,
        shapeRot: &RawRotation,
        point: &RawVector,
    ) -> bool {
        let pos = Pose::from_parts(shapePos.0, shapeRot.0);

        self.0.containsPoint(&pos, point.0)
    }

    pub fn projectPoint(
        &self,
        shapePos: &RawVector,
        shapeRot: &RawRotation,
        point: &RawVector,
        solid: bool,
    ) -> RawPointProjection {
        let pos = Pose::from_parts(shapePos.0, shapeRot.0);

        self.0.projectPoint(&pos, point.0, solid)
    }

    pub fn intersectsRay(
        &self,
        shapePos: &RawVector,
        shapeRot: &RawRotation,
        rayOrig: &RawVector,
        rayDir: &RawVector,
        maxToi: f32,
    ) -> bool {
        let pos = Pose::from_parts(shapePos.0, shapeRot.0);

        self.0.intersectsRay(&pos, rayOrig.0, rayDir.0, maxToi)
    }

    pub fn castRay(
        &self,
        shapePos: &RawVector,
        shapeRot: &RawRotation,
        rayOrig: &RawVector,
        rayDir: &RawVector,
        maxToi: f32,
        solid: bool,
    ) -> f32 {
        let pos = Pose::from_parts(shapePos.0, shapeRot.0);

        self.0.castRay(&pos, rayOrig.0, rayDir.0, maxToi, solid)
    }

    pub fn castRayAndGetNormal(
        &self,
        shapePos: &RawVector,
        shapeRot: &RawRotation,
        rayOrig: &RawVector,
        rayDir: &RawVector,
        maxToi: f32,
        solid: bool,
    ) -> Option<RawRayIntersection> {
        let pos = Pose::from_parts(shapePos.0, shapeRot.0);

        self.0
            .castRayAndGetNormal(&pos, rayOrig.0, rayDir.0, maxToi, solid)
    }
}
