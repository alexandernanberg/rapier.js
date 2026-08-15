/**
 * Archetype storage over a single linear memory.
 *
 * Every column is a typed-array view into one arena, so the layout is identical
 * whether that arena is a plain ArrayBuffer or a shared WASM memory. The spike
 * measures both, because the whole design assumes they perform the same.
 */

import {CTOR, type ComponentDef, type FieldType} from "./schema.ts";

const PAGE = 65536;

export class Arena {
    readonly buf: ArrayBufferLike;
    readonly shared: boolean;
    /** Byte offset of the arena within `buf`. Non-zero when borrowed. */
    readonly base: number;
    readonly limit: number;
    private off: number;

    constructor(pages: number, shared: boolean) {
        this.shared = shared;
        if (shared) {
            const mem = new WebAssembly.Memory({initial: pages, maximum: pages, shared: true});
            this.buf = mem.buffer;
        } else {
            this.buf = new ArrayBuffer(pages * PAGE);
        }
        this.base = 0;
        this.off = 0;
        this.limit = this.buf.byteLength;
    }

    /**
     * Borrow a region of somebody else's linear memory — in practice the physics
     * module's, so that component columns live where Rapier can write into them
     * directly. This is what makes the tier-1 zero-copy path real rather than
     * aspirational.
     */
    static borrow(buf: ArrayBufferLike, base: number, bytes: number): Arena {
        const a: Arena = Object.create(Arena.prototype);
        (a as {buf: ArrayBufferLike}).buf = buf;
        (a as {shared: boolean}).shared = !(buf instanceof ArrayBuffer);
        (a as {base: number}).base = base;
        (a as {limit: number}).limit = base + bytes;
        (a as unknown as {off: number}).off = base;
        return a;
    }

    alloc(type: FieldType, len: number): Float32Array | Uint32Array {
        const Ctor = CTOR[type];
        const bytes = len * Ctor.BYTES_PER_ELEMENT;
        // keep every column 8-byte aligned
        const at = (this.off + 7) & ~7;
        if (at + bytes > this.limit) {
            throw new Error(
                `arena exhausted: need ${at + bytes - this.base}, have ${this.limit - this.base}`,
            );
        }
        this.off = at + bytes;
        return new Ctor(this.buf as ArrayBuffer, at, len);
    }

    get used() {
        return this.off - this.base;
    }
}

export type Column = Float32Array | Uint32Array;

export class Archetype {
    readonly key: string;
    readonly comps: ComponentDef[];
    /** `${componentId}:${field}` -> column */
    readonly columns = new Map<string, Column>();
    readonly entities: Uint32Array;
    count = 0;

    constructor(comps: ComponentDef[], cap: number, arena: Arena) {
        this.comps = comps;
        this.key = comps
            .map((c) => c.id)
            .sort((a, b) => a - b)
            .join(",");
        this.entities = arena.alloc("u32", cap) as Uint32Array;
        for (const c of comps) {
            for (const [field, type] of c.fields) {
                this.columns.set(`${c.id}:${field}`, arena.alloc(type, cap));
            }
        }
    }

    /** Columns for one component, in declaration order. */
    bind(c: ComponentDef): Column[] {
        return c.fields.map(([f]) => this.columns.get(`${c.id}:${f}`)!);
    }

    push(entity: number): number {
        const row = this.count++;
        this.entities[row] = entity;
        return row;
    }

    /** Swap-remove; returns the entity that moved into `row`, or -1. */
    remove(row: number): number {
        const last = --this.count;
        if (row === last) return -1;
        for (const col of this.columns.values()) col[row] = col[last];
        const moved = this.entities[last];
        this.entities[row] = moved;
        return moved;
    }
}

interface Location {
    arch: Archetype;
    row: number;
}

export class World {
    readonly arena: Arena;
    private readonly cap: number;
    private readonly archetypes = new Map<string, Archetype>();
    private readonly loc: (Location | undefined)[] = [];
    private readonly generation: number[] = [];
    private nextEntity = 0;
    private readonly free: number[] = [];

    /** Deferred structural changes, applied at a stage boundary. */
    private readonly cmds: Array<() => void> = [];

    constructor(opts: {pages?: number; capacity?: number; shared?: boolean; arena?: Arena} = {}) {
        this.cap = opts.capacity ?? 1_100_000;
        this.arena = opts.arena ?? new Arena(opts.pages ?? 3072, opts.shared ?? true);
    }

    archetype(comps: ComponentDef[]): Archetype {
        const key = comps
            .map((c) => c.id)
            .sort((a, b) => a - b)
            .join(",");
        let a = this.archetypes.get(key);
        if (!a) {
            a = new Archetype(comps, this.cap, this.arena);
            this.archetypes.set(key, a);
        }
        return a;
    }

    allArchetypes(): Archetype[] {
        return [...this.archetypes.values()];
    }

    spawn(comps: ComponentDef[]): number {
        const entity = this.free.pop() ?? this.nextEntity++;
        this.generation[entity] = (this.generation[entity] ?? 0) + 1;
        const arch = this.archetype(comps);
        this.loc[entity] = {arch, row: arch.push(entity)};
        return entity;
    }

    despawn(entity: number): void {
        const l = this.loc[entity];
        if (!l) return;
        const moved = l.arch.remove(l.row);
        if (moved >= 0) this.loc[moved]!.row = l.row;
        this.loc[entity] = undefined;
        this.free.push(entity);
    }

    /** Move an entity to a different archetype — the expensive structural change. */
    move(entity: number, comps: ComponentDef[]): void {
        const from = this.loc[entity];
        if (!from) return;
        const to = this.archetype(comps);
        if (to === from.arch) return;
        const row = to.push(entity);
        // copy the columns the two archetypes share
        for (const [key, dst] of to.columns) {
            const src = from.arch.columns.get(key);
            if (src) dst[row] = src[from.row];
        }
        const moved = from.arch.remove(from.row);
        if (moved >= 0) this.loc[moved]!.row = from.row;
        this.loc[entity] = {arch: to, row};
    }

    defer(fn: () => void): void {
        this.cmds.push(fn);
    }

    flush(): number {
        const n = this.cmds.length;
        for (let i = 0; i < n; i++) this.cmds[i]();
        this.cmds.length = 0;
        return n;
    }

    locate(entity: number): Location | undefined {
        return this.loc[entity];
    }
}
