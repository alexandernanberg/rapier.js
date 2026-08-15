//! A physics backend shaped for an ECS rather than for JavaScript objects.
//!
//! The existing rapier.js bindings expose one JS object per body and marshal
//! through wasm-bindgen. Spike 01 measured what that costs: the call itself is
//! ~3ns, but allocating across it is 15.6ns because the call defeats V8's escape
//! analysis. So this crate exposes no objects at all.
//!
//! Instead there is a raw C ABI over bulk operations, and the ECS arena is
//! allocated *inside this module's linear memory*. Rapier writes transforms
//! straight into the ECS columns — no marshalling, no copy, one call per frame.

use rapier3d::math::{Pose, Rotation, Vector};
use rapier3d::prelude::*;

pub const KIND_DYNAMIC: u32 = 0;
pub const KIND_FIXED: u32 = 1;
pub const KIND_KINEMATIC: u32 = 2;

pub const SHAPE_BALL: u32 = 0;
pub const SHAPE_CUBOID: u32 = 1;

pub struct PhysicsWorld {
    gravity: Vector,
    params: IntegrationParameters,
    pipeline: PhysicsPipeline,
    islands: IslandManager,
    broad: DefaultBroadPhase,
    narrow: NarrowPhase,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    joints: ImpulseJointSet,
    multibody: MultibodyJointSet,
    ccd: CCDSolver,
    /// Dense handle table: the u32 the ECS stores is an index into this.
    handles: Vec<RigidBodyHandle>,
    /// The ECS arena, owned here so that component columns live in this
    /// module's memory and Rapier can write into them directly.
    arena: Vec<u8>,
}

/// Create a world, reserve the ECS arena, and claim allocator headroom.
///
/// The arena is allocated *before* any body exists, so its pointer stays valid
/// for the world's lifetime. `reserve_bytes` then forces the allocator to grow
/// linear memory once, up front, and immediately frees it — so later Rapier
/// allocations come out of that free pool instead of calling `memory.grow`.
///
/// That matters more than it looks. Growing a non-shared `WebAssembly.Memory`
/// detaches every JS view over it, and a detached typed array reports
/// `byteOffset === 0` rather than throwing — so the failure mode is Rust being
/// handed a null pointer, not an exception. Reserving up front avoids the whole
/// class of problem; `phys_buffer_generation` lets the host detect it anyway.
#[no_mangle]
pub extern "C" fn phys_new(
    gx: f32,
    gy: f32,
    gz: f32,
    arena_bytes: usize,
    reserve_bytes: usize,
) -> *mut PhysicsWorld {
    let mut arena = Vec::new();
    arena.resize(arena_bytes, 0u8);

    if reserve_bytes > 0 {
        let mut headroom: Vec<u8> = Vec::new();
        headroom.resize(reserve_bytes, 0u8);
        drop(headroom);
    }

    let world = PhysicsWorld {
        gravity: Vector::new(gx, gy, gz),
        params: IntegrationParameters::default(),
        pipeline: PhysicsPipeline::new(),
        islands: IslandManager::new(),
        broad: DefaultBroadPhase::new(),
        narrow: NarrowPhase::new(),
        bodies: RigidBodySet::new(),
        colliders: ColliderSet::new(),
        joints: ImpulseJointSet::new(),
        multibody: MultibodyJointSet::new(),
        ccd: CCDSolver::new(),
        handles: Vec::new(),
        arena,
    };
    Box::into_raw(Box::new(world))
}

#[no_mangle]
pub extern "C" fn phys_free(w: *mut PhysicsWorld) {
    if !w.is_null() {
        unsafe { drop(Box::from_raw(w)) };
    }
}

/// Base pointer of the ECS arena inside this module's linear memory.
#[no_mangle]
pub extern "C" fn phys_arena_ptr(w: *mut PhysicsWorld) -> *mut u8 {
    let w = unsafe { &mut *w };
    w.arena.as_mut_ptr()
}

#[no_mangle]
pub extern "C" fn phys_arena_len(w: *mut PhysicsWorld) -> usize {
    let w = unsafe { &*w };
    w.arena.len()
}

/// Add a body plus its collider. Returns the dense index the ECS should store.
#[no_mangle]
pub extern "C" fn phys_add_body(
    w: *mut PhysicsWorld,
    kind: u32,
    shape: u32,
    sx: f32,
    sy: f32,
    sz: f32,
    x: f32,
    y: f32,
    z: f32,
    restitution: f32,
    friction: f32,
) -> u32 {
    let w = unsafe { &mut *w };

    let builder = match kind {
        KIND_FIXED => RigidBodyBuilder::fixed(),
        KIND_KINEMATIC => RigidBodyBuilder::kinematic_position_based(),
        _ => RigidBodyBuilder::dynamic(),
    };
    let body = builder.translation(Vector::new(x, y, z)).build();
    let handle = w.bodies.insert(body);

    let collider = match shape {
        SHAPE_CUBOID => ColliderBuilder::cuboid(sx, sy, sz),
        _ => ColliderBuilder::ball(sx),
    }
    .restitution(restitution)
    .friction(friction)
    .build();
    w.colliders
        .insert_with_parent(collider, handle, &mut w.bodies);

    w.handles.push(handle);
    (w.handles.len() - 1) as u32
}

#[no_mangle]
pub extern "C" fn phys_body_count(w: *mut PhysicsWorld) -> u32 {
    let w = unsafe { &*w };
    w.handles.len() as u32
}

/// Advance the simulation. One call per tick, regardless of body count.
#[no_mangle]
pub extern "C" fn phys_step(w: *mut PhysicsWorld, dt: f32) {
    let w = unsafe { &mut *w };
    w.params.dt = dt;
    w.pipeline.step(
        w.gravity,
        &w.params,
        &mut w.islands,
        &mut w.broad,
        &mut w.narrow,
        &mut w.bodies,
        &mut w.colliders,
        &mut w.joints,
        &mut w.multibody,
        &mut w.ccd,
        &(),
        &(),
    );
}

/// Bulk read-back into ECS columns. `handles` is the ECS handle column; the
/// position and rotation pointers are the ECS columns themselves, so this writes
/// the results into their final home with no intermediate buffer.
///
/// # Safety
/// All pointers must address at least `n` elements inside this module's memory.
#[no_mangle]
pub unsafe extern "C" fn phys_pull_transforms(
    w: *mut PhysicsWorld,
    handles: *const u32,
    n: usize,
    px: *mut f32,
    py: *mut f32,
    pz: *mut f32,
    qx: *mut f32,
    qy: *mut f32,
    qz: *mut f32,
    qw: *mut f32,
) {
    let w = &*w;
    let handles = core::slice::from_raw_parts(handles, n);
    for i in 0..n {
        let Some(&h) = w.handles.get(handles[i] as usize) else {
            continue;
        };
        let Some(body) = w.bodies.get(h) else {
            continue;
        };
        let iso = body.position();
        let t = iso.translation;
        let r = iso.rotation;
        *px.add(i) = t.x;
        *py.add(i) = t.y;
        *pz.add(i) = t.z;
        *qx.add(i) = r.x;
        *qy.add(i) = r.y;
        *qz.add(i) = r.z;
        *qw.add(i) = r.w;
    }
}

/// Bulk write of kinematic targets, read straight out of ECS columns.
///
/// # Safety
/// All pointers must address at least `n` elements inside this module's memory.
#[no_mangle]
pub unsafe extern "C" fn phys_push_kinematic(
    w: *mut PhysicsWorld,
    handles: *const u32,
    n: usize,
    px: *const f32,
    py: *const f32,
    pz: *const f32,
) {
    let w = &mut *w;
    let handles = core::slice::from_raw_parts(handles, n);
    for i in 0..n {
        let Some(&h) = w.handles.get(handles[i] as usize) else {
            continue;
        };
        let Some(body) = w.bodies.get_mut(h) else {
            continue;
        };
        let pose = Pose::from_parts(
            Vector::new(*px.add(i), *py.add(i), *pz.add(i)),
            Rotation::IDENTITY,
        );
        body.set_next_kinematic_position(pose);
    }
}

/// Apply an impulse to a single body. Per-body by nature, and cheap enough:
/// a bare boundary call measured ~3ns, so a few hundred a frame is free.
#[no_mangle]
pub extern "C" fn phys_apply_impulse(w: *mut PhysicsWorld, handle: u32, ix: f32, iy: f32, iz: f32) {
    let w = unsafe { &mut *w };
    if let Some(&h) = w.handles.get(handle as usize) {
        if let Some(body) = w.bodies.get_mut(h) {
            body.apply_impulse(Vector::new(ix, iy, iz), true);
        }
    }
}

#[no_mangle]
pub extern "C" fn phys_set_linvel(w: *mut PhysicsWorld, handle: u32, vx: f32, vy: f32, vz: f32) {
    let w = unsafe { &mut *w };
    if let Some(&h) = w.handles.get(handle as usize) {
        if let Some(body) = w.bodies.get_mut(h) {
            body.set_linvel(Vector::new(vx, vy, vz), true);
        }
    }
}

/// Number of contact pairs currently touching — a cheap way for a test to assert
/// that collision actually happened without draining an event queue.
#[no_mangle]
pub extern "C" fn phys_contact_count(w: *mut PhysicsWorld) -> u32 {
    let w = unsafe { &*w };
    w.narrow
        .contact_pairs()
        .filter(|p| p.has_any_active_contact())
        .count() as u32
}
