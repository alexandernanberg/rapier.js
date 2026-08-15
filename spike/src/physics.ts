/**
 * The physics seam.
 *
 * Everything that touches Rapier lives behind this module — no abstraction
 * layer, no polymorphic backend interface, just concrete calls in one place with
 * a lint rule holding the line. The interface is extracted when a second backend
 * actually exists, not before.
 *
 * The hot path is bulk-only: push kinematic targets, step once, pull transforms
 * once. Per-body entry points exist for setup and for impulses, which are cheap
 * (a bare boundary call measured ~3ns) and never run per-entity per-frame.
 */

import {readFileSync} from "node:fs";

export const BodyKind = {dynamic: 0, fixed: 1, kinematic: 2} as const;
export const Shape = {ball: 0, cuboid: 1} as const;

interface Exports {
    memory: WebAssembly.Memory;
    phys_new(gx: number, gy: number, gz: number, arenaBytes: number, reserveBytes: number): number;
    phys_free(w: number): void;
    phys_arena_ptr(w: number): number;
    phys_arena_len(w: number): number;
    phys_add_body(
        w: number,
        kind: number,
        shape: number,
        sx: number,
        sy: number,
        sz: number,
        x: number,
        y: number,
        z: number,
        restitution: number,
        friction: number,
    ): number;
    phys_body_count(w: number): number;
    phys_step(w: number, dt: number): void;
    phys_pull_transforms(
        w: number,
        handles: number,
        n: number,
        px: number,
        py: number,
        pz: number,
        qx: number,
        qy: number,
        qz: number,
        qw: number,
    ): void;
    phys_push_kinematic(
        w: number,
        handles: number,
        n: number,
        px: number,
        py: number,
        pz: number,
    ): void;
    phys_apply_impulse(w: number, h: number, ix: number, iy: number, iz: number): void;
    phys_set_linvel(w: number, h: number, vx: number, vy: number, vz: number): void;
    phys_contact_count(w: number): number;
}

export interface BodyDesc {
    kind?: number;
    shape?: number;
    /** ball: [radius]; cuboid: [hx, hy, hz] */
    size?: [number, number, number];
    pos?: [number, number, number];
    restitution?: number;
    friction?: number;
}

const WASM_URL = new URL(
    "../../target/wasm32-unknown-unknown/debug/physics_ecs.wasm",
    import.meta.url,
);

/**
 * A detached typed array silently reports byteOffset 0, which would hand Rust a
 * null pointer instead of raising. Catch it at the seam with a message that
 * names the actual cause.
 */
function assertLive(view: {byteOffset: number; length: number}, what: string): void {
    if (view.length > 0 && view.byteOffset === 0) {
        throw new Error(
            `${what} is detached — the wasm memory grew and invalidated this view. ` +
                `Re-derive columns after any call that can allocate (body creation), ` +
                `or raise the reserve passed to Physics.create().`,
        );
    }
}

export class Physics {
    private readonly ex: Exports;
    private readonly world: number;
    readonly memory: WebAssembly.Memory;
    readonly arenaPtr: number;
    readonly arenaBytes: number;
    /** Bumped whenever the module's memory may have moved. */
    bufferGeneration = 0;
    private lastBuffer: ArrayBufferLike;

    private constructor(ex: Exports, world: number, arenaPtr: number, arenaBytes: number) {
        this.ex = ex;
        this.world = world;
        this.memory = ex.memory;
        this.arenaPtr = arenaPtr;
        this.arenaBytes = arenaBytes;
        this.lastBuffer = ex.memory.buffer;
    }

    static async create(
        gravity: [number, number, number] = [0, -9.81, 0],
        arenaBytes = 8 << 20,
        reserveBytes = 64 << 20,
    ): Promise<Physics> {
        const bytes = readFileSync(WASM_URL);
        const {instance} = await WebAssembly.instantiate(bytes, {});
        const ex = instance.exports as unknown as Exports;
        const world = ex.phys_new(gravity[0], gravity[1], gravity[2], arenaBytes, reserveBytes);
        const ptr = ex.phys_arena_ptr(world);
        return new Physics(ex, world, ptr, ex.phys_arena_len(world));
    }

    /**
     * Whether the module's memory has been replaced since the last check. Rust
     * allocations (body creation) can grow the heap, and growing a non-shared
     * memory detaches every existing view. Stepping and pulling never allocate,
     * so this only ever needs checking around structural changes.
     */
    checkBuffer(): boolean {
        if (this.memory.buffer !== this.lastBuffer) {
            this.lastBuffer = this.memory.buffer;
            this.bufferGeneration++;
            return true;
        }
        return false;
    }

    addBody(d: BodyDesc = {}): number {
        const [sx, sy, sz] = d.size ?? [0.5, 0.5, 0.5];
        const [x, y, z] = d.pos ?? [0, 0, 0];
        return this.ex.phys_add_body(
            this.world,
            d.kind ?? BodyKind.dynamic,
            d.shape ?? Shape.ball,
            sx,
            sy,
            sz,
            x,
            y,
            z,
            d.restitution ?? 0,
            d.friction ?? 0.5,
        );
    }

    step(dt: number): void {
        this.ex.phys_step(this.world, dt);
    }

    /** One call per frame. `cols` are ECS columns living in this same memory. */
    pullTransforms(
        handles: Uint32Array,
        n: number,
        px: Float32Array,
        py: Float32Array,
        pz: Float32Array,
        qx: Float32Array,
        qy: Float32Array,
        qz: Float32Array,
        qw: Float32Array,
    ): void {
        assertLive(handles, "handle column");
        assertLive(px, "position column");
        this.ex.phys_pull_transforms(
            this.world,
            handles.byteOffset,
            n,
            px.byteOffset,
            py.byteOffset,
            pz.byteOffset,
            qx.byteOffset,
            qy.byteOffset,
            qz.byteOffset,
            qw.byteOffset,
        );
    }

    pushKinematic(
        handles: Uint32Array,
        n: number,
        px: Float32Array,
        py: Float32Array,
        pz: Float32Array,
    ): void {
        assertLive(handles, "handle column");
        assertLive(px, "position column");
        this.ex.phys_push_kinematic(
            this.world,
            handles.byteOffset,
            n,
            px.byteOffset,
            py.byteOffset,
            pz.byteOffset,
        );
    }

    applyImpulse(handle: number, ix: number, iy: number, iz: number): void {
        this.ex.phys_apply_impulse(this.world, handle, ix, iy, iz);
    }

    setLinvel(handle: number, vx: number, vy: number, vz: number): void {
        this.ex.phys_set_linvel(this.world, handle, vx, vy, vz);
    }

    get bodyCount(): number {
        return this.ex.phys_body_count(this.world);
    }

    get contactCount(): number {
        return this.ex.phys_contact_count(this.world);
    }

    destroy(): void {
        this.ex.phys_free(this.world);
    }
}
