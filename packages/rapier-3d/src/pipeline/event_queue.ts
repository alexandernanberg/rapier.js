import {ColliderHandle} from "../geometry";
import {Vector, VectorOps} from "../math";
import {collisionEventStride, contactForceEventStride, RawEventQueue} from "../raw";
import {handleFromParts, WasmBuffer} from "../wasm_buffer";

// Both strides are compile-time constants on the Rust side; they are fetched
// lazily (the module has to be initialized first) and then never again, rather
// than crossing the boundary on every drain.
let _collisionStride = 0;
let _contactForceStride = 0;

/**
 * Flags indicating what events are enabled for colliders.
 */
export enum ActiveEvents {
    NONE = 0,
    /**
     * Enable collision events.
     */
    COLLISION_EVENTS = 0b0001,
    /**
     * Enable contact force events.
     */
    CONTACT_FORCE_EVENTS = 0b0010,
}

/**
 * Event occurring when the sum of the magnitudes of the
 * contact forces between two colliders exceed a threshold.
 *
 * This object should **not** be stored anywhere. Its properties can only be
 * read from within the closure given to `EventHandler.drainContactForceEvents`.
 */
export class TempContactForceEvent {
    private _buffer: WasmBuffer = null!;
    private _offset = 0;
    /** Slot offsets within one event, derived from the dimension-dependent stride. */
    private _forceOffset = 0;
    private _magnitudeOffset = 0;
    private _directionOffset = 0;
    private _maxMagnitudeOffset = 0;

    /** @internal */
    public _bind(buffer: WasmBuffer, offset: number, stride: number) {
        this._buffer = buffer;
        this._offset = offset;

        // Layout per event: two split handles, the total force, its magnitude,
        // the max force direction, its magnitude — so `stride = 6 + 2 * DIM`.
        const dim = (stride - 6) / 2;
        this._forceOffset = offset + 4;
        this._magnitudeOffset = offset + 4 + dim;
        this._directionOffset = offset + 5 + dim;
        this._maxMagnitudeOffset = offset + 5 + 2 * dim;
    }

    /**
     * The first collider involved in the contact.
     */
    public collider1(): ColliderHandle {
        const u32 = this._buffer.u32();
        return handleFromParts(u32[this._offset], u32[this._offset + 1]);
    }

    /**
     * The second collider involved in the contact.
     */
    public collider2(): ColliderHandle {
        const u32 = this._buffer.u32();
        return handleFromParts(u32[this._offset + 2], u32[this._offset + 3]);
    }

    /**
     * The sum of all the forces between the two colliders.
     */
    public totalForce(target?: Vector): Vector {
        return VectorOps.fromBufferAt(this._buffer.f32(), this._forceOffset, target);
    }

    /**
     * The sum of the magnitudes of each force between the two colliders.
     *
     * Note that this is **not** the same as the magnitude of `self.total_force`.
     * Here we are summing the magnitude of all the forces, instead of taking
     * the magnitude of their sum.
     */
    public totalForceMagnitude(): number {
        return this._buffer.f32()[this._magnitudeOffset];
    }

    /**
     * The world-space (unit) direction of the force with strongest magnitude.
     */
    public maxForceDirection(target?: Vector): Vector {
        return VectorOps.fromBufferAt(this._buffer.f32(), this._directionOffset, target);
    }

    /**
     * The magnitude of the largest force at a contact point of this contact pair.
     */
    public maxForceMagnitude(): number {
        return this._buffer.f32()[this._maxMagnitudeOffset];
    }
}

/**
 * A structure responsible for collecting events generated
 * by the physics engine.
 *
 * To avoid leaking WASM resources, this MUST be freed manually with `eventQueue.free()`
 * once you are done using it.
 */
export class EventQueue {
    raw: RawEventQueue;

    private _collisions = new WasmBuffer();
    private _contactForces = new WasmBuffer();
    private _event = new TempContactForceEvent();

    /**
     * Creates a new event collector.
     *
     * @param autoDrain -setting this to `true` is strongly recommended. If true, the collector will
     * be automatically drained before each `world.step(collector)`. If false, the collector will
     * keep all events in memory unless it is manually drained/cleared; this may lead to unbounded use of
     * RAM if no drain is performed.
     */
    constructor(autoDrain: boolean, raw?: RawEventQueue) {
        this.raw = raw || new RawEventQueue(autoDrain);
    }

    /**
     * Release the WASM memory occupied by this event-queue.
     */
    public free() {
        if (!!this.raw) {
            this.raw.free();
        }
        this.raw = undefined!;
        // Both views point into the buffers that were just freed.
        this._collisions.release();
        this._contactForces.release();
    }

    /**
     * Applies the given javascript closure on each collision event of this collector, then clear
     * the internal collision event buffer.
     *
     * @param f - JavaScript closure applied to each collision event. The
     * closure must take three arguments: two integers representing the handles of the colliders
     * involved in the collision, and a boolean indicating if the collision started (true) or stopped
     * (false).
     */
    public drainCollisionEvents(
        f: (handle1: ColliderHandle, handle2: ColliderHandle, started: boolean) => void,
    ) {
        // One call moves every pending event into WASM-side storage; the loop below
        // reads it out of a view, so no further boundary crossing happens per event.
        this._collisions.reset(this.raw.drainCollisionEvents());

        const stride = (_collisionStride ||= collisionEventStride());
        const len = this._collisions.length;
        // A throwing handler must not swallow the events behind it: the queue is
        // already empty, so anything skipped here would never be seen again. Every
        // event is delivered and the first error is re-thrown once the walk is done.
        let error: unknown;
        let failed = false;

        for (let offset = 0; offset < len; offset += stride) {
            const u32 = this._collisions.u32();
            const handle1 = handleFromParts(u32[offset], u32[offset + 1]);
            const handle2 = handleFromParts(u32[offset + 2], u32[offset + 3]);
            const started = this._collisions.f32()[offset + 4] !== 0;

            try {
                f(handle1, handle2, started);
            } catch (e) {
                if (!failed) {
                    failed = true;
                    error = e;
                }
            }
        }

        if (failed) throw error;
    }

    /**
     * Applies the given javascript closure on each contact force event of this collector, then clear
     * the internal collision event buffer.
     *
     * @param f - JavaScript closure applied to each collision event. The
     *            closure must take one `TempContactForceEvent` argument.
     */
    public drainContactForceEvents(f: (event: TempContactForceEvent) => void) {
        this._contactForces.reset(this.raw.drainContactForceEvents());

        const stride = (_contactForceStride ||= contactForceEventStride());
        const len = this._contactForces.length;
        const event = this._event;
        let error: unknown;
        let failed = false;

        for (let offset = 0; offset < len; offset += stride) {
            event._bind(this._contactForces, offset, stride);
            try {
                f(event);
            } catch (e) {
                if (!failed) {
                    failed = true;
                    error = e;
                }
            }
        }

        if (failed) throw error;
    }

    /**
     * Removes all events contained by this collector
     */
    public clear() {
        this.raw.clear();
    }
}
