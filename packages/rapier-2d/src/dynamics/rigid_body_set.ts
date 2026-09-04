import {Coarena} from "../coarena";
import {ColliderSet} from "../geometry";
import {RawRigidBodySet, RawRigidBodyType, wasmMemory} from "../raw";
import {
    createTransformBufferRef,
    DEAD_TRANSFORM_BUFFER_REF,
    invalidateTransformBuffer,
    refreshTransformBuffer,
    type TransformBufferRef,
} from "../transform_buffer";
import {ImpulseJointSet} from "./impulse_joint_set";
import {IslandManager} from "./island_manager";
import {MultibodyJointSet} from "./multibody_joint_set";
import {RigidBody, RigidBodyDesc, RigidBodyHandle} from "./rigid_body";

/**
 * A set of rigid bodies that can be handled by a physics pipeline.
 *
 * To avoid leaking WASM resources, this MUST be freed manually with `rigidBodySet.free()`
 * once you are done using it (and all the rigid-bodies it created).
 */
export class RigidBodySet {
    raw: RawRigidBodySet;
    private map: Coarena<RigidBody>;
    /** @internal */
    _bufferRef: TransformBufferRef = createTransformBufferRef();
    private _wasmMemory: WebAssembly.Memory | null = null;

    /**
     * Release the WASM memory occupied by this rigid-body set.
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

    constructor(raw?: RawRigidBodySet) {
        this.raw = raw || new RawRigidBodySet();
        this.map = new Coarena<RigidBody>();
        // deserialize
        if (raw) {
            raw.forEachRigidBodyHandle((handle: RigidBodyHandle) => {
                this.map.set(handle, new RigidBody(this.raw, this._bufferRef, null!, handle));
            });
        }
    }

    /**
     * Internal method, do not call this explicitly.
     */
    public finalizeDeserialization(colliderSet: ColliderSet) {
        this.map.forEach((rb) => rb.finalizeDeserialization(colliderSet));
    }

    /**
     * Refreshes the JS-side Float32Array view into the WASM transform buffer.
     *
     * The data sync happens inside the Rust step() for cache locality.
     * This creates the Float32Array view directly from WASM memory using
     * ptr+len, bypassing wasm-bindgen borrow tracking entirely.
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

    /**
     * Creates a new rigid-body and return its integer handle.
     *
     * @param desc - The description of the rigid-body to create.
     */
    public createRigidBody(colliderSet: ColliderSet, desc: RigidBodyDesc): RigidBody {
        const tra = desc.translation;
        const com = desc.centerOfMass;
        const lv = desc.linvel;

        let handle = this.raw.createRigidBody(
            desc.enabled,
            tra.x,
            tra.y,
            desc.rotation,
            desc.gravityScale,
            desc.mass,
            desc.massOnly,
            com.x,
            com.y,
            lv.x,
            lv.y,
            desc.angvel,
            desc.principalAngularInertia,
            desc.translationsEnabledX,
            desc.translationsEnabledY,
            desc.rotationsEnabled,
            desc.linearDamping,
            desc.angularDamping,
            desc.status as number as RawRigidBodyType,
            desc.canSleep,
            desc.sleeping,
            desc.softCcdPrediction,
            desc.ccdEnabled,
            desc.dominanceGroup,
            desc.additionalSolverIterations,
        );

        // The Rust side wrote the new body's slot, which may have grown (and so
        // moved) the buffer: re-point the view rather than invalidate it, so a
        // body spawned mid-frame does not push every other body's reads onto the
        // WASM path until the next step. A buffer that is not live yet (nothing
        // synced since the world was created or restored) stays that way: some
        // of its slots have never been written.
        if (this._bufferRef.ptr !== 0) {
            this.syncTransformBuffer();
        }

        const body = new RigidBody(this.raw, this._bufferRef, colliderSet, handle);
        body.userData = desc.userData;

        this.map.set(handle, body);

        return body;
    }

    /**
     * Removes a rigid-body from this set.
     *
     * This will also remove all the colliders and joints attached to the rigid-body.
     *
     * @param handle - The integer handle of the rigid-body to remove.
     * @param colliders - The set of colliders that may contain colliders attached to the removed rigid-body.
     * @param impulseJoints - The set of impulse joints that may contain joints attached to the removed rigid-body.
     * @param multibodyJoints - The set of multibody joints that may contain joints attached to the removed rigid-body.
     */
    public remove(
        handle: RigidBodyHandle,
        islands: IslandManager,
        colliders: ColliderSet,
        impulseJoints: ImpulseJointSet,
        multibodyJoints: MultibodyJointSet,
    ) {
        // Unmap the entities that will be removed automatically because of the rigid-body removals.
        const numColliders = this.raw.rbNumColliders(handle);
        for (let i = 0; i < numColliders; i += 1) {
            colliders.unmap(this.raw.rbCollider(handle, i)!);
        }

        impulseJoints.forEachJointHandleAttachedToRigidBody(handle, (handle) =>
            impulseJoints.unmap(handle),
        );
        multibodyJoints.forEachJointHandleAttachedToRigidBody(handle, (handle) =>
            multibodyJoints.unmap(handle),
        );

        // Remove the rigid-body.
        this.raw.remove(handle, islands.raw, colliders.raw, impulseJoints.raw, multibodyJoints.raw);
        const body = this.map.get(handle);
        if (body) {
            // See `DEAD_TRANSFORM_BUFFER_REF`: the slot outlives the body.
            body._bufferRef = DEAD_TRANSFORM_BUFFER_REF;
        }
        this.map.delete(handle);
    }

    /**
     * The number of rigid-bodies on this set.
     */
    public len(): number {
        return this.map.len();
    }

    /**
     * Does this set contain a rigid-body with the given handle?
     *
     * @param handle - The rigid-body handle to check.
     */
    public contains(handle: RigidBodyHandle): boolean {
        return this.get(handle) != null;
    }

    /**
     * Gets the rigid-body with the given handle.
     *
     * @param handle - The handle of the rigid-body to retrieve.
     */
    public get(handle: RigidBodyHandle): RigidBody | null {
        return this.map.get(handle);
    }

    /**
     * Applies the given closure to each rigid-body contained by this set.
     *
     * @param f - The closure to apply.
     */
    public forEach(f: (body: RigidBody) => void) {
        this.map.forEach(f);
    }

    /**
     * Applies the given closure to each active rigid-bodies contained by this set.
     *
     * A rigid-body is active if it is not sleeping, i.e., if it moved recently.
     *
     * @param f - The closure to apply.
     */
    public forEachActiveRigidBody(islands: IslandManager, f: (body: RigidBody) => void) {
        islands.forEachActiveRigidBodyHandle((handle) => {
            f(this.get(handle)!);
        });
    }

    /**
     * Gets all rigid-bodies in the list.
     *
     * @returns rigid-bodies list.
     */
    public getAll(): RigidBody[] {
        return this.map.getAll();
    }
}
