use crate::utils::{self, FlatHandle};
use rapier::geometry::{CollisionEvent, ContactForceEvent};
use rapier::math::DIM;
use rapier::pipeline::ChannelEventCollector;
use std::sync::mpsc::Receiver;
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

/// A structure responsible for collecting events generated
/// by the physics engine.
#[wasm_bindgen]
pub struct RawEventQueue {
    pub(crate) collector: ChannelEventCollector,
    collision_events: Receiver<CollisionEvent>,
    contact_force_events: Receiver<ContactForceEvent>,
    /// Drained collision events, `COLLISION_EVENT_STRIDE` slots each.
    collision_buffer: Vec<f32>,
    /// Drained contact-force events, `CONTACT_FORCE_EVENT_STRIDE` slots each.
    contact_force_buffer: Vec<f32>,
    pub(crate) auto_drain: bool,
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
        let collision_channel = std::sync::mpsc::channel();
        let contact_force_channel = std::sync::mpsc::channel();
        let collector = ChannelEventCollector::new(collision_channel.0, contact_force_channel.0);

        Self {
            collector,
            collision_events: collision_channel.1,
            contact_force_events: contact_force_channel.1,
            collision_buffer: Vec::new(),
            contact_force_buffer: Vec::new(),
            auto_drain: autoDrain,
        }
    }

    /// Moves every pending collision event into this queue's collision buffer and
    /// returns that buffer's pointer and length packed into a single `f64`.
    ///
    /// Each event takes `COLLISION_EVENT_STRIDE` slots: collider 1's arena index
    /// and generation, collider 2's, and `1` if the collision started or `0` if it
    /// stopped — the first four as raw `u32` bit patterns, the flag as a float.
    ///
    /// Calling the JS handler from here instead meant one WASM→JS call per event,
    /// each boxing three values; JS now makes that call itself while walking a
    /// view onto the buffer, so the drain costs one boundary crossing in total.
    ///
    /// The buffer may be reallocated by this call, so the returned pointer is only
    /// valid until the next drain.
    pub fn drainCollisionEvents(&mut self) -> f64 {
        self.collision_buffer.clear();

        while let Ok(event) = self.collision_events.try_recv() {
            let (co1, co2, started) = match event {
                CollisionEvent::Started(co1, co2, _) => (co1, co2, true),
                CollisionEvent::Stopped(co1, co2, _) => (co1, co2, false),
            };

            push_handle(&mut self.collision_buffer, utils::flat_handle(co1.0));
            push_handle(&mut self.collision_buffer, utils::flat_handle(co2.0));
            self.collision_buffer.push(if started { 1.0 } else { 0.0 });
        }

        pack_buffer_info(&self.collision_buffer)
    }

    /// Moves every pending contact-force event into this queue's contact-force
    /// buffer and returns that buffer's pointer and length packed into a single
    /// `f64`, the same way as [`Self::drainCollisionEvents`].
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
        self.contact_force_buffer.clear();

        while let Ok(event) = self.contact_force_events.try_recv() {
            let out = &mut self.contact_force_buffer;

            push_handle(out, utils::flat_handle(event.collider1.0));
            push_handle(out, utils::flat_handle(event.collider2.0));
            out.extend_from_slice(event.total_force.as_ref());
            out.push(event.total_force_magnitude);
            out.extend_from_slice(event.max_force_direction.as_ref());
            out.push(event.max_force_magnitude);
            out.push(if event.started { 1.0 } else { 0.0 });
        }

        pack_buffer_info(&self.contact_force_buffer)
    }

    /// Removes all events contained by this collector.
    pub fn clear(&mut self) {
        while self.collision_events.try_recv().is_ok() {}
        while self.contact_force_events.try_recv().is_ok() {}

        // The drain buffers hold whatever the last drain published. Dropping the
        // contents (but not the capacity) keeps a stale event from being read back
        // through a view that JS still holds.
        self.collision_buffer.clear();
        self.contact_force_buffer.clear();
    }
}
