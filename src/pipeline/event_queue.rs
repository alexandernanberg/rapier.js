use crate::utils::{self, FlatHandle};
use rapier::dynamics::RigidBodySet;
use rapier::geometry::{ColliderSet, CollisionEvent, ContactForceEvent, ContactPair};
use rapier::math::{Real, DIM};
use rapier::pipeline::EventHandler;
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

/// `f32` slots one drained collision event occupies: the two collider handles,
/// each split into its arena index and generation, plus the started flag.
const COLLISION_EVENT_STRIDE: usize = 5;

/// `f32` slots one drained contact-force event occupies: two split handles, the
/// total force and its magnitude, the max force direction and its magnitude,
/// and the started flag.
const CONTACT_FORCE_EVENT_STRIDE: usize = 7 + 2 * DIM;

/// The stride of the collision-event buffer, so the JS side can walk it without
/// hard-coding the layout.
#[wasm_bindgen]
pub fn collisionEventStride() -> u32 {
    COLLISION_EVENT_STRIDE as u32
}

/// The stride of the contact-force-event buffer. Dimension-dependent, so JS
/// reads it from here rather than keeping its own copy of `DIM`.
#[wasm_bindgen]
pub fn contactForceEventStride() -> u32 {
    CONTACT_FORCE_EVENT_STRIDE as u32
}

/// The event buffers, behind a `RefCell` because rapier hands the handler out
/// as `&dyn EventHandler` and calls it through a shared reference.
struct EventBuffers {
    /// Collision events, `COLLISION_EVENT_STRIDE` slots each.
    collision: Vec<f32>,
    /// Contact-force events, `CONTACT_FORCE_EVENT_STRIDE` slots each.
    contact_force: Vec<f32>,
    /// Set by a drain: JS has read the buffer, so the next event written to it
    /// starts a fresh batch instead of appending to the one already delivered.
    collision_drained: bool,
    contact_force_drained: bool,
}

/// A structure responsible for collecting events generated
/// by the physics engine.
///
/// The queue is the event handler itself: rapier calls it once per event and
/// it writes the event straight into the buffer JS later walks, in its final
/// layout. Routing the events through an `mpsc` channel first (the
/// `ChannelEventCollector` way) cost a heap allocation and two copies per
/// event; the `unsync-callbacks` feature lifts the `Sync` bound that made the
/// channel necessary.
#[wasm_bindgen]
pub struct RawEventQueue {
    buffers: RefCell<EventBuffers>,
    pub(crate) auto_drain: bool,
}

impl EventHandler for RawEventQueue {
    fn handle_collision_event(
        &self,
        _bodies: &RigidBodySet,
        _colliders: &ColliderSet,
        event: CollisionEvent,
        _contact_pair: Option<&ContactPair>,
    ) {
        let (co1, co2, started) = match event {
            CollisionEvent::Started(co1, co2, _) => (co1, co2, true),
            CollisionEvent::Stopped(co1, co2, _) => (co1, co2, false),
        };

        let mut buffers = self.buffers.borrow_mut();
        if buffers.collision_drained {
            buffers.collision.clear();
            buffers.collision_drained = false;
        }
        let out = &mut buffers.collision;
        push_handle(out, utils::flat_handle(co1.0));
        push_handle(out, utils::flat_handle(co2.0));
        out.push(if started { 1.0 } else { 0.0 });
    }

    fn handle_contact_force_event(
        &self,
        dt: Real,
        _bodies: &RigidBodySet,
        _colliders: &ColliderSet,
        contact_pair: &ContactPair,
        total_force_magnitude: Real,
    ) {
        let event = ContactForceEvent::from_contact_pair(dt, contact_pair, total_force_magnitude);

        let mut buffers = self.buffers.borrow_mut();
        if buffers.contact_force_drained {
            buffers.contact_force.clear();
            buffers.contact_force_drained = false;
        }
        let out = &mut buffers.contact_force;
        push_handle(out, utils::flat_handle(event.collider1.0));
        push_handle(out, utils::flat_handle(event.collider2.0));
        out.extend_from_slice(event.total_force.as_ref());
        out.push(event.total_force_magnitude);
        out.extend_from_slice(event.max_force_direction.as_ref());
        out.push(event.max_force_magnitude);
        out.push(if event.started { 1.0 } else { 0.0 });
    }
}

/// Packs a buffer's pointer and element count into one `f64`: low 32 bits the
/// byte offset in WASM memory, high 32 bits the `f32` element count. Same
/// encoding as the transform, query-result and scratch buffers.
fn pack_buffer_info(data: &[f32]) -> f64 {
    let ptr = data.as_ptr() as u32;
    let len = data.len() as u32;
    f64::from_bits(ptr as u64 | ((len as u64) << 32))
}

/// Appends a handle to `out` as its arena index and generation, each carried as
/// a raw `u32` bit pattern.
///
/// A `FlatHandle` is an `f64` holding `index | generation << 32`, so it does not
/// survive an `f32`. JS reads the two halves back through a `Uint32Array` view
/// and reassembles the `f64` exactly.
#[inline]
fn push_handle(out: &mut Vec<f32>, handle: FlatHandle) {
    let bits = handle.to_bits();
    out.push(f32::from_bits(bits as u32));
    out.push(f32::from_bits((bits >> 32) as u32));
}

#[wasm_bindgen]
impl RawEventQueue {
    /// Creates a new event collector.
    ///
    /// # Parameters
    /// - `autoDrain`: setting this to `true` is strongly recommended. If true, the collector will
    /// be automatically drained before each `world.step(collector)`. If false, the collector will
    /// keep all events in memory unless it is manually drained/cleared; this may lead to unbounded use of
    /// RAM if no drain is performed.
    #[wasm_bindgen(constructor)]
    pub fn new(autoDrain: bool) -> Self {
        Self {
            buffers: RefCell::new(EventBuffers {
                collision: Vec::new(),
                contact_force: Vec::new(),
                collision_drained: false,
                contact_force_drained: false,
            }),
            auto_drain: autoDrain,
        }
    }

    /// Publishes the collision events collected since the last drain and returns
    /// the buffer's pointer and length packed into a single `f64`.
    ///
    /// Each event takes `COLLISION_EVENT_STRIDE` slots: collider 1's arena index
    /// and generation, collider 2's, and `1` if the collision started or `0` if it
    /// stopped — the first four as raw `u32` bit patterns, the flag as a float.
    ///
    /// Calling the JS handler from here instead meant one WASM→JS call per event,
    /// each boxing three values; JS now makes that call itself while walking a
    /// view onto the buffer, so the drain costs one boundary crossing in total.
    ///
    /// The buffer is reused (and may be reallocated) by the next step, so the
    /// returned pointer is only valid until then.
    pub fn drainCollisionEvents(&mut self) -> f64 {
        let buffers = self.buffers.get_mut();
        // Already delivered: a drain removes the events it hands out, so the
        // next one finds nothing until the step writes a new batch.
        let len = if buffers.collision_drained {
            0
        } else {
            buffers.collision.len()
        };
        buffers.collision_drained = true;
        pack_buffer_info(&buffers.collision[..len])
    }

    /// Publishes the contact-force events collected since the last drain and
    /// returns the buffer's pointer and length packed into a single `f64`, the
    /// same way as [`Self::drainCollisionEvents`].
    ///
    /// Each event takes `CONTACT_FORCE_EVENT_STRIDE` slots: the two collider
    /// handles (split as above), the total force, the total force magnitude, the
    /// max force direction, the max force magnitude, and `1` if this is the first
    /// step the pair's force crossed its threshold or `0` if it stayed above it.
    ///
    /// Handing each event to JS as a `RawContactForceEvent` instead meant boxing
    /// one WASM object per event — allocated, passed across, read back through
    /// four more calls, then freed. The whole drain is now one crossing and no
    /// allocation.
    pub fn drainContactForceEvents(&mut self) -> f64 {
        let buffers = self.buffers.get_mut();
        let len = if buffers.contact_force_drained {
            0
        } else {
            buffers.contact_force.len()
        };
        buffers.contact_force_drained = true;
        pack_buffer_info(&buffers.contact_force[..len])
    }

    /// Removes all events contained by this collector.
    pub fn clear(&mut self) {
        // Contents only, not the capacity: a stale event must not be read back
        // through a view JS still holds, and the next step should not reallocate.
        let buffers = self.buffers.get_mut();
        buffers.collision.clear();
        buffers.contact_force.clear();
        buffers.collision_drained = false;
        buffers.contact_force_drained = false;
    }
}
