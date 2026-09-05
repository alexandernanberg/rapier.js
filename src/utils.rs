use rapier::data::Index;
use rapier::dynamics::{ImpulseJointHandle, MultibodyJointHandle, RigidBodyHandle};
use rapier::geometry::{Collider, ColliderHandle};
#[cfg(feature = "dim3")]
use rapier::math::Rotation;
use wasm_bindgen::JsValue;

pub type FlatHandle = f64;

#[inline(always)]
pub fn collider_handle(id: FlatHandle) -> ColliderHandle {
    ColliderHandle::from_raw_parts(id.to_bits() as u32, (id.to_bits() >> 32) as u32)
}

#[inline(always)]
pub fn body_handle(id: FlatHandle) -> RigidBodyHandle {
    RigidBodyHandle::from_raw_parts(id.to_bits() as u32, (id.to_bits() >> 32) as u32)
}

#[inline(always)]
pub fn impulse_joint_handle(id: FlatHandle) -> ImpulseJointHandle {
    ImpulseJointHandle::from_raw_parts(id.to_bits() as u32, (id.to_bits() >> 32) as u32)
}

#[inline(always)]
pub fn multibody_joint_handle(id: FlatHandle) -> MultibodyJointHandle {
    MultibodyJointHandle::from_raw_parts(id.to_bits() as u32, (id.to_bits() >> 32) as u32)
}

#[inline(always)]
pub fn flat_handle(id: Index) -> FlatHandle {
    let (i, g) = id.into_raw_parts();
    FlatHandle::from_bits(i as u64 | ((g as u64) << 32))
}

/// Builds a unit rotation from raw quaternion components.
///
/// A quaternion that drifted slightly off unit length (the usual state of one
/// coming out of a JS math library after a few multiplications) is normalized
/// rather than rejected, so it behaves the same whether it reaches the body
/// through its descriptor or through a setter. A zero or non-finite quaternion
/// has no direction to recover, so `None` is returned and the caller leaves the
/// rotation as it was.
#[cfg(feature = "dim3")]
#[inline]
pub fn unit_rotation(x: f32, y: f32, z: f32, w: f32) -> Option<Rotation> {
    let q = Rotation::from_xyzw(x, y, z, w);
    if q.is_normalized() {
        return Some(q);
    }

    let len_sq = q.length_squared();
    if len_sq.is_finite() && len_sq > 0.0 {
        Some(q / len_sq.sqrt())
    } else {
        None
    }
}

#[inline(always)]
pub fn with_filter<T>(
    filter: &js_sys::Function,
    f: impl FnOnce(Option<&dyn Fn(ColliderHandle, &Collider) -> bool>) -> T,
) -> T {
    if filter.is_function() {
        let filtercb = move |handle: ColliderHandle, _: &Collider| match filter
            .call1(&JsValue::null(), &JsValue::from(flat_handle(handle.0)))
        {
            Err(_) => true,
            Ok(val) => val.as_bool().unwrap_or(true),
        };

        f(Some(&filtercb))
    } else {
        f(None)
    }
}
