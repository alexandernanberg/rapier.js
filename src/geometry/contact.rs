use crate::scratch;
use rapier::parry::query;

/// Writes `contact` into the scratch buffer as
/// `[distance, point1, point2, normal1, normal2]`; see `toi::write_hit`.
pub(crate) fn write_contact(contact: &query::Contact) {
    let p1 = contact.point1;
    let p2 = contact.point2;
    let n1 = contact.normal1;
    let n2 = contact.normal2;

    #[cfg(feature = "dim2")]
    let components = [contact.dist, p1.x, p1.y, p2.x, p2.y, n1.x, n1.y, n2.x, n2.y];

    #[cfg(feature = "dim3")]
    let components = [
        contact.dist,
        p1.x,
        p1.y,
        p1.z,
        p2.x,
        p2.y,
        p2.z,
        n1.x,
        n1.y,
        n1.z,
        n2.x,
        n2.y,
        n2.z,
    ];

    scratch::write(&components);
}
