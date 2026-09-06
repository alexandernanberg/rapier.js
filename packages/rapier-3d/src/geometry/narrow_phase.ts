import {RigidBodySet} from "../dynamics";
import {Vector, VectorOps} from "../math";
import {
    contactManifoldStride,
    contactPointStride,
    solverContactStride,
    RawNarrowPhase,
} from "../raw";
import {WasmBuffer} from "../wasm_buffer";
import {ColliderHandle} from "./collider";

// The strides are compile-time constants on the Rust side; they are fetched
// lazily (the module has to be initialized first) and then never again, rather
// than crossing the boundary on every `contactPair` call. The dimension falls out
// of the contact stride (`3 * DIM + 3`), which sizes every vector slot below.
let _manifoldStride = 0;
let _contactStride = 0;
let _solverContactStride = 0;
let _dim = 0;

function loadStrides() {
    if (_manifoldStride !== 0) return;
    _manifoldStride = contactManifoldStride();
    _contactStride = contactPointStride();
    _solverContactStride = solverContactStride();
    _dim = (_contactStride - 3) / 3;
}

/** The buffer and cursor serving one nesting depth of `contactPair`. */
interface PairLevel {
    buffer: WasmBuffer;
    manifold: TempContactManifold;
}

/**
 * The narrow-phase used for precise collision-detection.
 *
 * To avoid leaking WASM resources, this MUST be freed manually with `narrowPhase.free()`
 * once you are done using it.
 */
export class NarrowPhase {
    raw: RawNarrowPhase;
    // One buffer view and manifold cursor per nesting depth of `contactPair`:
    // a nested call (from inside the callback) gets its own so it does not
    // overwrite the manifolds the outer callback is still reading. The Rust side
    // keeps a matching buffer per depth.
    private _pairLevels: PairLevel[] = [];
    private _contactPairDepth = 0;

    /**
     * Release the WASM memory occupied by this narrow-phase.
     */
    public free() {
        if (!!this.raw) {
            this.raw.free();
        }
        this.raw = undefined!;
        // The views point into the pair buffers that were just freed.
        for (const level of this._pairLevels) level.buffer.release();
    }

    constructor(raw?: RawNarrowPhase) {
        this.raw = raw || new RawNarrowPhase();
    }

    /**
     * Enumerates all the colliders potentially in contact with the given collider.
     *
     * @param collider1 - The second collider involved in the contact.
     * @param f - Closure that will be called on each collider that is in contact with `collider1`.
     */
    public contactPairsWith(
        collider1: ColliderHandle,
        f: (collider2: ColliderHandle) => boolean | void,
    ) {
        this.raw.contact_pairs_with(collider1, this.guard(f));
        this.rethrowCallbackError();
    }

    /**
     * Enumerates all the colliders intersecting the given colliders, assuming one of them
     * is a sensor.
     *
     * A closure that returns exactly `false` ends the enumeration early.
     */
    public intersectionPairsWith(
        collider1: ColliderHandle,
        f: (collider2: ColliderHandle) => boolean | void,
    ) {
        this.raw.intersection_pairs_with(collider1, this.guard(f));
        this.rethrowCallbackError();
    }

    // A user callback that threw from inside an enumeration. The exception
    // cannot cross the WASM boundary, so the wrapper below catches it, stops the
    // walk, and keeps it for `rethrowCallbackError` once the walk has returned.
    private _callbackError: unknown = undefined;
    private _callbackFailed = false;

    /**
     * Wraps `f` so that an exception it throws ends the enumeration and reaches
     * the caller. `f`'s own answer is passed through (an explicit `false` ends
     * the enumeration early); once `f` has thrown the wrapper answers `false`
     * itself so the remaining pairs are not handed to a failed callback.
     */
    private guard(
        f: (collider2: ColliderHandle) => boolean | void,
    ): (collider2: ColliderHandle) => boolean | void {
        this._callbackFailed = false;
        return (collider2) => {
            if (this._callbackFailed) return false;
            try {
                return f(collider2);
            } catch (e) {
                this._callbackFailed = true;
                this._callbackError = e;
                return false;
            }
        };
    }

    private rethrowCallbackError() {
        if (this._callbackFailed) {
            const error = this._callbackError;
            this._callbackFailed = false;
            this._callbackError = undefined;
            throw error;
        }
    }

    /**
     * Iterates through all the contact manifolds between the given pair of colliders.
     *
     * The manifolds are read out of a WASM-resident buffer that one call fills with
     * every manifold of the pair, contacts and solver contacts included, so the
     * whole walk costs a single boundary crossing and allocates nothing. The
     * `manifold` object handed to `f` is a cursor into that buffer, reused for
     * every manifold and overwritten by the next `contactPair` call: read what you
     * need inside the closure rather than storing the object.
     *
     * @param collider1 - The first collider involved in the contact.
     * @param collider2 - The second collider involved in the contact.
     * @param bodies - The set of rigid-bodies the colliders are attached to. Solver contacts are
     *                 anchored in body-local space, so this is needed to read them back in world-space.
     * @param f - Closure that will be called on each contact manifold between the two colliders. If the second argument
     *            passed to this closure is `true`, then the contact manifold data is flipped, i.e., methods like `localNormal1`
     *            actually apply to the `collider2` and fields like `localNormal2` apply to the `collider1`.
     */
    public contactPair(
        collider1: ColliderHandle,
        collider2: ColliderHandle,
        bodies: RigidBodySet,
        f: (manifold: TempContactManifold, flipped: boolean) => void,
    ) {
        loadStrides();

        const depth = this._contactPairDepth;
        let level = this._pairLevels[depth];
        if (level === undefined) {
            level = {buffer: new WasmBuffer(), manifold: new TempContactManifold()};
            this._pairLevels[depth] = level;
        }

        // One call publishes the pair's manifolds; the loop below reads them out
        // of a view, so nothing else crosses the boundary per manifold or field.
        // The published length is the buffer's capacity rather than the pair's
        // size, so the view is only rebuilt when the buffer grows.
        level.buffer.reset(this.raw.contact_pair(collider1, collider2, bodies.raw, depth));

        const u32 = level.buffer.u32();
        const flipped = u32[0] !== 0;
        const numManifolds = u32[1];
        if (numManifolds === 0) return;
        const manifold = level.manifold;

        this._contactPairDepth = depth + 1;
        try {
            let offset = 2;
            for (let i = 0; i < numManifolds; ++i) {
                offset = manifold._bind(level.buffer, offset);
                f(manifold, flipped);
            }
        } finally {
            this._contactPairDepth = depth;
        }
    }

    /**
     * Returns `true` if `collider1` and `collider2` intersect and at least one of them is a sensor.
     * @param collider1 − The first collider involved in the intersection.
     * @param collider2 − The second collider involved in the intersection.
     */
    public intersectionPair(collider1: ColliderHandle, collider2: ColliderHandle): boolean {
        return this.raw.intersection_pair(collider1, collider2);
    }
}

/**
 * One contact manifold between two colliders, as handed to the closure given to
 * `NarrowPhase.contactPair` / `World.contactPair`.
 *
 * This object should **not** be stored anywhere: it is a cursor into a buffer
 * that the next `contactPair` call overwrites, so its getters are only meaningful
 * inside that closure. Its vector getters take the same optional `target` as the
 * rigid-body getters, and a contact index out of range reads as `null` (vectors)
 * or `0` (scalars).
 */
export class TempContactManifold {
    private _buffer: WasmBuffer = null!;
    /** Slot of the manifold's fixed part. */
    private _offset = 0;
    private _numContacts = 0;
    private _numSolverContacts = 0;
    /** Slot of the first contact point, `_contactStride` slots each. */
    private _contactsOffset = 0;
    /** Slot of the first solver contact, `_solverContactStride` slots each. */
    private _solverContactsOffset = 0;

    /**
     * Points this cursor at the manifold starting at `offset` and returns the
     * offset of the manifold after it.
     *
     * Layout of the fixed part: the world-space normal, both local normals, then
     * the user data, both subshape indices, friction, restitution, and the
     * number of contacts and of solver contacts that follow (the integers as raw
     * `u32` bit patterns).
     *
     * @internal
     */
    public _bind(buffer: WasmBuffer, offset: number): number {
        this._buffer = buffer;
        this._offset = offset;
        const u32 = buffer.u32();
        const counts = offset + 3 * _dim + 5;
        this._numContacts = u32[counts];
        this._numSolverContacts = u32[counts + 1];
        this._contactsOffset = offset + _manifoldStride;
        this._solverContactsOffset = this._contactsOffset + this._numContacts * _contactStride;
        return this._solverContactsOffset + this._numSolverContacts * _solverContactStride;
    }

    /**
     * The slot of contact `i`, or `-1` if there is no such contact. The index is
     * converted the way the WASM boundary used to convert it to a `usize`, so a
     * negative or fractional `i` behaves as it always has.
     */
    private contactSlot(i: number): number {
        const index = i >>> 0;
        if (index >= this._numContacts) return -1;
        return this._contactsOffset + index * _contactStride;
    }

    private solverContactSlot(i: number): number {
        const index = i >>> 0;
        if (index >= this._numSolverContacts) return -1;
        return this._solverContactsOffset + index * _solverContactStride;
    }

    /** The world-space contact normal, pointing from the first collider towards the second. */
    public normal(target?: Vector): Vector {
        return VectorOps.fromBufferAt(this._buffer.f32(), this._offset, target);
    }

    public localNormal1(target?: Vector): Vector {
        return VectorOps.fromBufferAt(this._buffer.f32(), this._offset + _dim, target);
    }

    public localNormal2(target?: Vector): Vector {
        return VectorOps.fromBufferAt(this._buffer.f32(), this._offset + 2 * _dim, target);
    }

    /**
     * The user-defined 32-bit integer attached to this manifold by a
     * `PhysicsHooks.modifySolverContacts` hook. Preserved across steps, and `0`
     * if no hook ever set it.
     */
    public userData(): number {
        return this._buffer.u32()[this._offset + 3 * _dim];
    }

    public subshape1(): number {
        return this._buffer.u32()[this._offset + 3 * _dim + 1];
    }

    public subshape2(): number {
        return this._buffer.u32()[this._offset + 3 * _dim + 2];
    }

    public numContacts(): number {
        return this._numContacts;
    }

    public localContactPoint1(i: number, target?: Vector): Vector | null {
        const slot = this.contactSlot(i);
        if (slot < 0) return null;
        return VectorOps.fromBufferAt(this._buffer.f32(), slot, target);
    }

    public localContactPoint2(i: number, target?: Vector): Vector | null {
        const slot = this.contactSlot(i);
        if (slot < 0) return null;
        return VectorOps.fromBufferAt(this._buffer.f32(), slot + _dim, target);
    }

    public contactDist(i: number): number {
        const slot = this.contactSlot(i);
        return slot < 0 ? 0 : this._buffer.f32()[slot + 2 * _dim];
    }

    public contactFid1(i: number): number {
        const slot = this.contactSlot(i);
        return slot < 0 ? 0 : this._buffer.u32()[slot + 2 * _dim + 1];
    }

    public contactFid2(i: number): number {
        const slot = this.contactSlot(i);
        return slot < 0 ? 0 : this._buffer.u32()[slot + 2 * _dim + 2];
    }

    public contactImpulse(i: number): number {
        const slot = this.contactSlot(i);
        return slot < 0 ? 0 : this._buffer.f32()[slot + 2 * _dim + 3];
    }

    public contactTangentImpulseX(i: number): number {
        const slot = this.contactSlot(i);
        return slot < 0 ? 0 : this._buffer.f32()[slot + 2 * _dim + 4];
    }

    public contactTangentImpulseY(i: number): number {
        const slot = this.contactSlot(i);
        return slot < 0 ? 0 : this._buffer.f32()[slot + 2 * _dim + 5];
    }

    public numSolverContacts(): number {
        return this._numSolverContacts;
    }

    /**
     * The contact point on the first body's surface.
     *
     * This is expressed in that body's center-of-mass-centered local frame, or in
     * world-space when the first side has no solver body (no rigid-body, or world-attached
     * by dominance — fixed bodies included).
     */
    public solverContactAnchor1(i: number, target?: Vector): Vector | null {
        const slot = this.solverContactSlot(i);
        if (slot < 0) return null;
        return VectorOps.fromBufferAt(this._buffer.f32(), slot, target);
    }

    /**
     * The contact point on the second body's surface, expressed like
     * {@link solverContactAnchor1}.
     */
    public solverContactAnchor2(i: number, target?: Vector): Vector | null {
        const slot = this.solverContactSlot(i);
        if (slot < 0) return null;
        return VectorOps.fromBufferAt(this._buffer.f32(), slot + _dim, target);
    }

    /**
     * The world-space contact point the solver acts on, midway between both surfaces.
     *
     * Returns `null` if `i` is out of bounds.
     */
    public solverContactPoint(i: number, target?: Vector): Vector | null {
        const slot = this.solverContactSlot(i);
        if (slot < 0) return null;
        return VectorOps.fromBufferAt(this._buffer.f32(), slot + 2 * _dim, target);
    }

    public solverContactTangentVelocity(i: number, target?: Vector): Vector | null {
        const slot = this.solverContactSlot(i);
        if (slot < 0) return null;
        return VectorOps.fromBufferAt(this._buffer.f32(), slot + 3 * _dim, target);
    }

    public solverContactDist(i: number): number {
        const slot = this.solverContactSlot(i);
        return slot < 0 ? 0 : this._buffer.f32()[slot + 4 * _dim];
    }

    /**
     * The effective friction coefficient of this manifold's contacts.
     *
     * Friction is stored per-manifold, so it is identical for every contact of this manifold.
     */
    public friction(): number {
        return this._buffer.f32()[this._offset + 3 * _dim + 3];
    }

    /**
     * The effective restitution coefficient of this manifold's contacts.
     *
     * Restitution is stored per-manifold, so it is identical for every contact of this manifold.
     */
    public restitution(): number {
        return this._buffer.f32()[this._offset + 3 * _dim + 4];
    }
}
