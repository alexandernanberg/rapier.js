use crate::geometry::feature::IntoTypeValue;
use crate::scratch;
use rapier::geometry::RayIntersection;

/// Writes a ray intersection into the scratch buffer as
/// `timeOfImpact, normal (2 or 3), featureType, featureId`.
///
/// `featureType` and `featureId` are stored as raw `u32` bit patterns (see
/// [`scratch::u32_bits`]); a missing feature id is [`scratch::NO_FEATURE_ID`].
#[inline]
pub(crate) fn write_ray_intersection(inter: &RayIntersection) {
    let n = inter.normal;
    let ty = scratch::u32_bits(inter.feature.into_type() as u32);
    let id = scratch::u32_bits(inter.feature.into_value().unwrap_or(scratch::NO_FEATURE_ID));

    #[cfg(feature = "dim2")]
    scratch::write(&[inter.time_of_impact, n.x, n.y, ty, id]);
    #[cfg(feature = "dim3")]
    scratch::write(&[inter.time_of_impact, n.x, n.y, n.z, ty, id]);
}
