import {Coarena} from "../coarena";
import {IslandManager, RigidBodyHandle} from "../dynamics";
import {RigidBodySet} from "../dynamics";
import {RawColliderSet, wasmMemory} from "../raw";
import {
    createTransformBufferRef,
    invalidateTransformBuffer,
    refreshTransformBuffer,
    type TransformBufferRef,
} from "../transform_buffer";
import {Collider, ColliderDesc, ColliderHandle} from "./collider";

/**
 * @internal Container for the collider transform buffer, shared with Collider instances.
 */
export type ColliderTransformBufferRef = TransformBufferRef;

/**
 * A set of rigid bodies that can be handled by a physics pipeline.
 *
 * To avoid leaking WASM resources, this MUST be freed manually with `colliderSet.free()`
 * once you are done using it (and all the rigid-bodies it created).
 */
export class ColliderSet {
    raw: RawColliderSet;
    private map: Coarena<Collider>;
    /** @internal */
    _bufferRef: ColliderTransformBufferRef = createTransformBufferRef();
    private _wasmMemory: WebAssembly.Memory | null = null;

    /**
     * Release the WASM memory occupied by this collider set.
     */
    public free() {
        if (!!this.raw) {
            this.raw.free();
        }
        this.raw = undefined!;
        // Bodies/colliders handed out earlier still hold the old ref: mark it
        // stale so they can't re-attach a view over the WASM memory just freed.
        invalidateTransformBuffer(this._bufferRef);
        this._bufferRef = createTransformBufferRef();

        if (!!this.map) {
            this.map.clear();
        }
        this.map = undefined!;
    }

    constructor(raw?: RawColliderSet) {
        this.raw = raw || new RawColliderSet();
        this.map = new Coarena<Collider>();
        // Initialize the map with the existing elements, if any.
        if (raw) {
            raw.forEachColliderHandle((handle: ColliderHandle) => {
                this.map.set(handle, new Collider(this, handle, null));
            });
        }
    }

    /** @internal */
    // A user callback that threw from inside a query. The Rust side cannot let
    // the exception through (it has to be caught at the boundary), so it used to
    // treat a throwing predicate as "include this collider" and a throwing hit
    // callback as "keep going", and the error was lost. The wrappers built below
    // catch it, make the query stop, and keep it here for `rethrowCallbackError`
    // to throw once the query has returned and released its WASM borrows and
    // temporaries.
    private _callbackError: unknown = undefined;
    private _callbackFailed = false;

    /**
     * Wraps a callback handed to a WASM query so that an exception it throws
     * stops the query and is re-thrown by {@link rethrowCallbackError} instead
     * of being swallowed at the boundary.
     *
     * @internal
     */
    public guardCallback<Res>(f: () => Res): () => Res {
        this._callbackFailed = false;
        return () => {
            if (this._callbackFailed) return false as unknown as Res;
            try {
                return f();
            } catch (e) {
                this._callbackFailed = true;
                this._callbackError = e;
                // `false` excludes the collider from a predicate and stops a
                // hit callback's iteration.
                return false as unknown as Res;
            }
        };
    }

    /**
     * Turns a callback taking a `Collider` into one taking a handle, guarded the
     * same way as {@link guardCallback}.
     *
     * @internal
     */
    public castClosure<Res>(
        f?: (collider: Collider) => Res,
    ): ((handle: ColliderHandle) => Res) | undefined {
        if (!f) return undefined;
        this._callbackFailed = false;
        return (handle) => {
            if (this._callbackFailed) return false as unknown as Res;
            try {
                return f(this.get(handle)!);
            } catch (e) {
                this._callbackFailed = true;
                this._callbackError = e;
                return false as unknown as Res;
            }
        };
    }

    /**
     * Like {@link castClosure}, for callbacks whose return value means nothing to
     * the caller: the wrapper answers `undefined` normally and `false` once the
     * user's callback has thrown. The Rust enumerations stop on `false`, so the
     * remaining items are not handed to a callback that has already failed —
     * and whatever the user's callback happens to return (`Set.delete`'s
     * boolean, `Array.push`'s length) can never end the walk by accident.
     *
     * @internal
     */
    public castVoidClosure(
        f: (collider: Collider) => void,
    ): (handle: ColliderHandle) => boolean | void {
        this._callbackFailed = false;
        return (handle) => {
            if (this._callbackFailed) return false;
            try {
                f(this.get(handle)!);
            } catch (e) {
                this._callbackFailed = true;
                this._callbackError = e;
                return false;
            }
        };
    }

    /**
     * Throws the exception a guarded callback raised during the query that just
     * returned, if any. Called by every query wrapper after its WASM call.
     *
     * @internal
     */
    public rethrowCallbackError() {
        if (this._callbackFailed) {
            const error = this._callbackError;
            this._callbackFailed = false;
            this._callbackError = undefined;
            throw error;
        }
    }

    /**
     * Refreshes the JS-side Float32Array view into the WASM collider transform
     * buffer. The data sync happens inside the Rust step(); this rebuilds the
     * view from the current ptr+len (the WASM memory may have grown).
     *
     * Called automatically by `World.step()`.
     *
     * @internal
     */
    public syncTransformBuffer() {
        if (!this._wasmMemory) {
            this._wasmMemory = wasmMemory() as unknown as WebAssembly.Memory;
        }
        refreshTransformBuffer(this._bufferRef, this.raw.transformBufferInfo(), this._wasmMemory);
    }

    /** @internal */
    public finalizeDeserialization(bodies: RigidBodySet) {
        this.map.forEach((collider) => collider.finalizeDeserialization(bodies));
    }

    /**
     * Creates a new collider and return its integer handle.
     *
     * @param bodies - The set of bodies where the collider's parent can be found.
     * @param desc - The collider's description.
     * @param parentHandle - The integer handle of the rigid-body this collider is attached to.
     */
    public createCollider(
        bodies: RigidBodySet,
        desc: ColliderDesc,
        parentHandle?: RigidBodyHandle,
    ): Collider {
        let hasParent = parentHandle != undefined && parentHandle != null;

        if (hasParent && isNaN(parentHandle!))
            throw Error(
                "Cannot create a collider with a parent rigid-body handle that is not a number.",
            );

        let rawShape = desc.shape.intoRaw();

        const tra = desc.translation;
        const rot = desc.rotation;
        const com = desc.centerOfMass;
        const pai = desc.principalAngularInertia;
        const aif = desc.angularInertiaLocalFrame;

        let handle: ColliderHandle | undefined;
        try {
            handle = this.raw.createCollider(
                desc.enabled,
                rawShape,
                tra.x,
                tra.y,
                tra.z,
                rot.x,
                rot.y,
                rot.z,
                rot.w,
                desc.massPropsMode,
                desc.mass,
                com.x,
                com.y,
                com.z,
                pai.x,
                pai.y,
                pai.z,
                aif.x,
                aif.y,
                aif.z,
                aif.w,
                desc.density,
                desc.friction,
                desc.restitution,
                desc.frictionCombineRule,
                desc.restitutionCombineRule,
                desc.isSensor,
                desc.collisionGroups,
                desc.solverGroups,
                desc.activeCollisionTypes,
                desc.activeHooks,
                desc.activeEvents,
                desc.contactForceEventThreshold,
                desc.contactSkin,
                hasParent,
                hasParent ? parentHandle! : 0,
                bodies.raw,
            );
        } finally {
            // A trimesh or heightfield shape is a large allocation; the call can
            // throw (a re-entrant use of the set from inside a callback, say), and
            // the shape must not outlive the attempt either way.
            rawShape.free();
        }

        if (handle === undefined) {
            throw Error(
                "Cannot create the collider: its parent rigid-body is not part of the world (it may have been removed), or its descriptor holds an invalid mass-properties mode.",
            );
        }

        // The Rust side wrote the new collider's slot, which may have grown
        // (and so moved) the buffer: re-point the view rather than invalidate
        // it. See `RigidBodySet.createRigidBody` for the not-yet-live case.
        if (this._bufferRef.ptr !== 0) {
            this.syncTransformBuffer();
        }

        let parent = hasParent ? bodies.get(parentHandle!) : null;
        let collider = new Collider(this, handle, parent, desc.shape);
        this.map.set(handle, collider);
        return collider;
    }

    /**
     * Remove a collider from this set.
     *
     * @param handle - The integer handle of the collider to remove.
     * @param bodies - The set of rigid-body containing the rigid-body the collider is attached to.
     * @param wakeUp - If `true`, the rigid-body the removed collider is attached to will be woken-up automatically.
     */
    public remove(
        handle: ColliderHandle,
        islands: IslandManager,
        bodies: RigidBodySet,
        wakeUp: boolean,
    ) {
        // Already removed (or a stale handle whose index was recycled): nothing
        // to do, and the collider that owns the index now must be left alone.
        if (!this.map.get(handle)) return;
        this.raw.remove(handle, islands.raw, bodies.raw, wakeUp);
        this.unmap(handle);
    }

    /**
     * Internal function, do not call directly.
     * @param handle
     */
    public unmap(handle: ColliderHandle) {
        const collider = this.map.get(handle);
        if (collider) {
            // The arena slot and the raw set both outlive the collider; see
            // `Collider._markRemoved`.
            collider._markRemoved();
            this.map.delete(handle);
        }
    }

    /**
     * Gets the rigid-body with the given handle.
     *
     * @param handle - The handle of the rigid-body to retrieve.
     */
    public get(handle: ColliderHandle): Collider | null {
        return this.map.get(handle);
    }

    /**
     * The number of colliders on this set.
     */
    public len(): number {
        return this.map.len();
    }

    /**
     * Does this set contain a collider with the given handle?
     *
     * @param handle - The collider handle to check.
     */
    public contains(handle: ColliderHandle): boolean {
        return this.get(handle) != null;
    }

    /**
     * Applies the given closure to each collider contained by this set.
     *
     * @param f - The closure to apply.
     */
    public forEach(f: (collider: Collider) => void) {
        this.map.forEach(f);
    }

    /**
     * Gets all colliders in the list.
     *
     * @returns collider list.
     */
    public getAll(): Collider[] {
        return this.map.getAll();
    }
}
