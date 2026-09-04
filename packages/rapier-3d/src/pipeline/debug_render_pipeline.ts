import {ImpulseJointSet, MultibodyJointSet, RigidBodySet} from "../dynamics";
import {Collider, ColliderSet, NarrowPhase} from "../geometry";
import {RawDebugRenderPipeline, wasmMemory} from "../raw";
import {
    createTransformBufferRef,
    liveTransformBuffer,
    refreshTransformBuffer,
    type TransformBufferRef,
} from "../transform_buffer";
import {QueryFilterFlags} from "./query_pipeline";

const EMPTY = new Float32Array(0);

/**
 * Grows `store` to hold at least `len` elements, doubling rather than fitting
 * exactly so a slowly growing line count does not reallocate every frame.
 */
function reserve(store: Float32Array, len: number): Float32Array {
    if (store.length >= len) return store;
    return new Float32Array(Math.max(len, store.length * 2));
}

/**
 * Returns a view of exactly `len` elements over `store`, reusing `previous` when
 * it already describes that window.
 *
 * A `subarray` allocates, so the common case — a scene whose collider set is not
 * changing, and which therefore renders the same number of lines every frame —
 * must not take one.
 */
function viewOf(store: Float32Array, len: number, previous: Float32Array): Float32Array {
    if (previous.length === len && previous.buffer === store.buffer) return previous;
    return len === store.length ? store : store.subarray(0, len);
}

/**
 * The vertex and color buffers for debug-redering the physics scene.
 *
 * Pass an instance back to `World.debugRender` as its `target` to reuse its
 * buffers instead of allocating a new pair every frame.
 */
export class DebugRenderBuffers {
    /**
     * The lines to render. This is a flat array containing all the lines
     * to render. Each line is described as two consecutive point. Each
     * point is described as two (in 2D) or three (in 3D) consecutive
     * floats. For example, in 2D, the array: `[1, 2, 3, 4, 5, 6, 7, 8]`
     * describes the two segments `[[1, 2], [3, 4]]` and `[[5, 6], [7, 8]]`.
     */
    public vertices: Float32Array;
    /**
     * The color buffer. There is one color per vertex, and each color
     * has four consecutive components (in RGBA format).
     */
    public colors: Float32Array;

    /**
     * Backing stores the exposed views window into. Kept across frames, and only
     * ever grown, so a reused target settles into allocating nothing.
     */
    private _vertexStore: Float32Array;
    private _colorStore: Float32Array;

    constructor(vertices?: Float32Array, colors?: Float32Array) {
        this._vertexStore = vertices ?? EMPTY;
        this._colorStore = colors ?? EMPTY;
        this.vertices = this._vertexStore;
        this.colors = this._colorStore;
    }

    /**
     * Copies the pipeline's WASM-resident line buffers into this object's own
     * storage, so the result stays valid across later WASM calls.
     *
     * @internal
     */
    public _copyFrom(vertices: Float32Array, colors: Float32Array) {
        this._vertexStore = reserve(this._vertexStore, vertices.length);
        this._vertexStore.set(vertices);
        this.vertices = viewOf(this._vertexStore, vertices.length, this.vertices);

        this._colorStore = reserve(this._colorStore, colors.length);
        this._colorStore.set(colors);
        this.colors = viewOf(this._colorStore, colors.length, this.colors);
    }
}

/**
 * A pipeline for rendering the physics scene.
 *
 * To avoid leaking WASM resources, this MUST be freed manually with `debugRenderPipeline.free()`
 * once you are done using it (and all the rigid-bodies it created).
 */
export class DebugRenderPipeline {
    raw: RawDebugRenderPipeline;

    /** @internal */
    _vertexRef: TransformBufferRef = createTransformBufferRef();
    /** @internal */
    _colorRef: TransformBufferRef = createTransformBufferRef();
    private _wasmMemory: WebAssembly.Memory | null = null;

    /**
     * Release the WASM memory occupied by this serialization pipeline.
     */
    free() {
        if (!!this.raw) {
            this.raw.free();
        }
        this.raw = undefined!;
        this._vertexRef = createTransformBufferRef();
        this._colorRef = createTransformBufferRef();
    }

    constructor(raw?: RawDebugRenderPipeline) {
        this.raw = raw || new RawDebugRenderPipeline();
    }

    /**
     * The lines produced by the last {@link render} call, as a view straight into
     * WASM memory.
     *
     * Nothing is copied, so the view is only valid until the next call into WASM:
     * a later call can grow the linear memory (which detaches the view) or run
     * another render (which overwrites, and may move, the buffer). Copy out of it,
     * or use `World.debugRender`'s `target` parameter, to keep the data.
     */
    public get vertices(): Float32Array {
        // A zero-length view is indistinguishable from a detached one (both report
        // a `byteLength` of 0), so an empty frame is answered without going near
        // `liveTransformBuffer`, which would otherwise rebuild a view every call.
        if (this._vertexRef.len === 0) return EMPTY;
        return liveTransformBuffer(this._vertexRef) ?? EMPTY;
    }

    /**
     * The colors produced by the last {@link render} call, one per vertex, with
     * the same lifetime caveats as {@link vertices}.
     */
    public get colors(): Float32Array {
        if (this._colorRef.len === 0) return EMPTY;
        return liveTransformBuffer(this._colorRef) ?? EMPTY;
    }

    public render(
        bodies: RigidBodySet,
        colliders: ColliderSet,
        impulse_joints: ImpulseJointSet,
        multibody_joints: MultibodyJointSet,
        narrow_phase: NarrowPhase,
        filterFlags?: QueryFilterFlags,
        filterPredicate?: (collider: Collider) => boolean,
    ) {
        this.raw.render(
            bodies.raw,
            colliders.raw,
            impulse_joints.raw,
            multibody_joints.raw,
            narrow_phase.raw,
            filterFlags ?? 0,
            colliders.castClosure(filterPredicate) as unknown as Function,
        );

        if (!this._wasmMemory) {
            this._wasmMemory = wasmMemory() as unknown as WebAssembly.Memory;
        }

        // The render just rebuilt both buffers, and either may have been grown
        // (and therefore moved) to fit this frame's lines, so both views have to
        // be re-pointed at what WASM now holds.
        refreshTransformBuffer(this._vertexRef, this.raw.verticesInfo(), this._wasmMemory);
        refreshTransformBuffer(this._colorRef, this.raw.colorsInfo(), this._wasmMemory);
    }
}
