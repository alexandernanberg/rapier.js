/**
 * Queries and the candidate iteration APIs.
 *
 * The microbenchmark that settled this earlier used closure-captured module
 * constants. A real query resolves columns dynamically, per archetype, which is
 * where monomorphism can quietly collapse. That is the thing under test.
 */

import type {ComponentDef} from "./schema.ts";
import type {Archetype, Column, World} from "./world.ts";

const EMPTY = new Float32Array(0);

export class Query {
    readonly comps: ComponentDef[];
    /** flattened field names, in query order */
    readonly fields: string[];
    archetypes: Archetype[] = [];
    /** per archetype, columns flattened in the same order as `fields` */
    bindings: Column[][] = [];

    private readonly world: World;

    constructor(world: World, comps: ComponentDef[]) {
        this.world = world;
        this.comps = comps;
        this.fields = comps.flatMap((c) => c.fields.map(([f]) => f));
        this.refresh();
    }

    /** Rebuild the archetype match list. Real engines cache this by generation. */
    refresh(): this {
        this.archetypes = [];
        this.bindings = [];
        const want = this.comps.map((c) => c.id);
        for (const a of this.world.allArchetypes()) {
            const has = new Set(a.comps.map((c) => c.id));
            if (!want.every((id) => has.has(id))) continue;
            this.archetypes.push(a);
            this.bindings.push(this.comps.flatMap((c) => a.bind(c)));
        }
        return this;
    }
}

/* ------------------------------------------------------------------ cursors */

export interface Cursor {
    i: number;
    bind(cols: Column[]): void;
}

/** Columns held in an array; each getter does `this.c[k][this.i]`. */
export function arrayCursor(fields: string[]): new () => Cursor {
    class C {
        i = 0;
        c: Column[] = [];
        bind(cols: Column[]) {
            this.c = cols;
        }
    }
    fields.forEach((name, k) => {
        Object.defineProperty(C.prototype, name, {
            get(this: C) {
                return this.c[k][this.i];
            },
            set(this: C, v: number) {
                this.c[k][this.i] = v;
            },
        });
    });
    return C as new () => Cursor;
}

/** Columns held in named slots; each getter does `this._k[this.i]`. */
export function slotCursor(fields: string[]): new () => Cursor {
    if (fields.length > 8) throw new Error("slotCursor: max 8 columns");
    class C {
        i = 0;
        _0: Column = EMPTY;
        _1: Column = EMPTY;
        _2: Column = EMPTY;
        _3: Column = EMPTY;
        _4: Column = EMPTY;
        _5: Column = EMPTY;
        _6: Column = EMPTY;
        _7: Column = EMPTY;
        bind(cols: Column[]) {
            this._0 = cols[0] ?? EMPTY;
            this._1 = cols[1] ?? EMPTY;
            this._2 = cols[2] ?? EMPTY;
            this._3 = cols[3] ?? EMPTY;
            this._4 = cols[4] ?? EMPTY;
            this._5 = cols[5] ?? EMPTY;
            this._6 = cols[6] ?? EMPTY;
            this._7 = cols[7] ?? EMPTY;
        }
    }
    fields.forEach((name, k) => {
        const slot = `_${k}` as const;
        Object.defineProperty(C.prototype, name, {
            get(this: any) {
                return this[slot][this.i];
            },
            set(this: any, v: number) {
                this[slot][this.i] = v;
            },
        });
    });
    return C as new () => Cursor;
}

/**
 * Same shape as slotCursor, but the accessors are generated so that `this._3` is
 * a real named property load rather than a computed `this[slot]` keyed load.
 * Built once per query signature, at registration time — not a build step.
 *
 * Caveat: `new Function` is blocked under a strict CSP, so a browser build would
 * need this moved to a compile-time codegen step.
 */
export function codegenCursor(fields: string[]): new () => Cursor {
    const init = fields.map((_, k) => `this._${k} = E`).join("; ");
    const bind = fields.map((_, k) => `this._${k} = c[${k}] || E`).join("; ");
    const accessors = fields
        .map(
            (f, k) =>
                `get ${f}() { return this._${k}[this.i] }\n` +
                `set ${f}(v) { this._${k}[this.i] = v }`,
        )
        .join("\n");
    const src = `return class GeneratedCursor {
        constructor() { this.i = 0; ${init} }
        bind(c) { ${bind} }
        ${accessors}
    }`;
    return new Function("E", src)(EMPTY);
}

/* ---------------------------------------------------------------- iteration */

/** Per-entity callback — the most ergonomic form, and the one to be suspicious of. */
export function eachEntity(q: Query, cursor: Cursor, fn: (e: any) => void): void {
    for (let a = 0; a < q.archetypes.length; a++) {
        const arch = q.archetypes[a];
        cursor.bind(q.bindings[a]);
        const n = arch.count;
        for (let i = 0; i < n; i++) {
            cursor.i = i;
            fn(cursor);
        }
    }
}

/** Per-archetype callback handing over raw columns — the escape hatch. */
export function eachChunk(q: Query, fn: (cols: Column[], count: number) => void): void {
    for (let a = 0; a < q.archetypes.length; a++) {
        fn(q.bindings[a], q.archetypes[a].count);
    }
}
