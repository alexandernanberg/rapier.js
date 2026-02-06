/// Stride per body in 3D: translation(3) + rotation(4) + linvel(3) + angvel(3) = 13
#[cfg(feature = "dim3")]
pub const BODY_STRIDE: usize = 13;

/// Stride per body in 2D: translation(2) + rotation(1) + linvel(2) + angvel(1) = 6
#[cfg(feature = "dim2")]
pub const BODY_STRIDE: usize = 6;
