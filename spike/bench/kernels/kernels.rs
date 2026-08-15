#![no_std]

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

/// Memory-bound streaming kernel: position += velocity * dt.
/// 6 f32 reads, 3 f32 writes, 3 mul + 3 add per entity.
#[no_mangle]
pub unsafe extern "C" fn integrate(
    px: *mut f32,
    py: *mut f32,
    pz: *mut f32,
    vx: *const f32,
    vy: *const f32,
    vz: *const f32,
    n: usize,
    dt: f32,
) {
    for i in 0..n {
        *px.add(i) += *vx.add(i) * dt;
        *py.add(i) += *vy.add(i) * dt;
        *pz.add(i) += *vz.add(i) * dt;
    }
}

/// Compute-denser kernel: quaternion product, as in transform propagation.
/// 8 f32 reads, 4 f32 writes, 16 mul + 12 add per entity.
#[no_mangle]
pub unsafe extern "C" fn qmul(
    ax: *const f32,
    ay: *const f32,
    az: *const f32,
    aw: *const f32,
    bx: *const f32,
    by: *const f32,
    bz: *const f32,
    bw: *const f32,
    ox: *mut f32,
    oy: *mut f32,
    oz: *mut f32,
    ow: *mut f32,
    n: usize,
) {
    for i in 0..n {
        let (x1, y1, z1, w1) = (*ax.add(i), *ay.add(i), *az.add(i), *aw.add(i));
        let (x2, y2, z2, w2) = (*bx.add(i), *by.add(i), *bz.add(i), *bw.add(i));
        *ox.add(i) = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
        *oy.add(i) = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
        *oz.add(i) = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
        *ow.add(i) = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
    }
}

/// Trivial function, for measuring raw JS -> WASM call overhead.
#[no_mangle]
pub extern "C" fn noop(x: i32) -> i32 {
    x + 1
}
