use crate::scratch;
use rapier::geometry::PointProjection;

/// Writes a point projection into the scratch buffer as
/// `point (2 or 3), isInside (0 or 1)`.
#[inline]
pub(crate) fn write_point_projection(proj: &PointProjection) {
    let p = proj.point;
    let inside = if proj.is_inside { 1.0 } else { 0.0 };

    #[cfg(feature = "dim2")]
    scratch::write(&[p.x, p.y, inside]);
    #[cfg(feature = "dim3")]
    scratch::write(&[p.x, p.y, p.z, inside]);
}
