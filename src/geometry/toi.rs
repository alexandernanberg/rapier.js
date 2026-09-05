use crate::scratch;
use rapier::geometry::ShapeCastHit;

/// Writes `hit` into the scratch buffer as
/// `[time_of_impact, witness1, witness2, normal1, normal2]`.
///
/// Shape casts used to hand a boxed `RawShapeCastHit` back to JS, which then
/// cost two more boundary crossings (one to read the components, one to free
/// it) plus the allocation. The querying call now returns a flag and JS reads
/// the hit out of the scratch buffer directly.
pub(crate) fn write_hit(hit: &ShapeCastHit) {
    let w1 = hit.witness1;
    let w2 = hit.witness2;
    let n1 = hit.normal1;
    let n2 = hit.normal2;

    #[cfg(feature = "dim2")]
    let components = [
        hit.time_of_impact,
        w1.x,
        w1.y,
        w2.x,
        w2.y,
        n1.x,
        n1.y,
        n2.x,
        n2.y,
    ];

    #[cfg(feature = "dim3")]
    let components = [
        hit.time_of_impact,
        w1.x,
        w1.y,
        w1.z,
        w2.x,
        w2.y,
        w2.z,
        n1.x,
        n1.y,
        n1.z,
        n2.x,
        n2.y,
        n2.z,
    ];

    scratch::write(&components);
}
