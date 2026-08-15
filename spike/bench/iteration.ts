/**
 * Spike 02 — does the cursor survive a real archetype?
 *
 * The earlier microbenchmark iterated closure-captured module constants. This one
 * goes through dynamic archetype resolution, multiple archetypes, and columns
 * backed by shared WASM memory. If the cursor holds up here, the ergonomic query
 * API is safe to build on. If it doesn't, the fallback is schema codegen.
 */

import {
    arrayCursor,
    codegenCursor,
    eachChunk,
    eachEntity,
    Query,
    slotCursor,
} from "../src/query.ts";
import {component, tag, type ComponentDef} from "../src/schema.ts";
import {World} from "../src/world.ts";

/* ------------------------------------------------------------------ harness */

function best(fn: () => void, reps = 9, warmup = 4): number {
    for (let i = 0; i < warmup; i++) fn();
    let b = Infinity;
    for (let i = 0; i < reps; i++) {
        const t0 = process.hrtime.bigint();
        fn();
        const s = Number(process.hrtime.bigint() - t0) / 1e9;
        if (s < b) b = s;
    }
    return b;
}

let sink = 0;
const DT = 0.016;

/* --------------------------------------------------------------- components */

const Position = component("Position", {px: "f32", py: "f32", pz: "f32"});
const Velocity = component("Velocity", {vx: "f32", vy: "f32", vz: "f32"});
const Health = component("Health", {hp: "f32", hpMax: "f32"});
const Sprite = component("Sprite", {tex: "u32"});
const Player = tag("Player");
const Enemy = tag("Enemy");

const FIELDS = ["px", "py", "pz", "vx", "vy", "vz"];

/* ------------------------------------------------------------------ fixture */

function build(n: number, shapes: ComponentDef[][], shared: boolean) {
    const world = new World({capacity: n + 16, shared, pages: 4096});
    for (let i = 0; i < n; i++) world.spawn(shapes[i % shapes.length]);
    const q = new Query(world, [Position, Velocity]);
    // seed
    for (const cols of q.bindings) {
        for (const c of cols) for (let i = 0; i < c.length; i++) c[i] = (i % 97) * 0.01;
    }
    return {world, q};
}

/* -------------------------------------------------------------- the variants */

function runVariants(label: string, n: number, shapes: ComponentDef[][], shared: boolean) {
    const {q} = build(n, shapes, shared);
    const arrCur = new (arrayCursor(FIELDS))();
    const sloCur = new (slotCursor(FIELDS))();
    const genCur = new (codegenCursor(FIELDS))();

    const variants: Record<string, () => void> = {
        // A — raw columns, loop written by hand inside a per-archetype callback
        "raw columns": () => {
            eachChunk(q, (c, count) => {
                const px = c[0],
                    py = c[1],
                    pz = c[2],
                    vx = c[3],
                    vy = c[4],
                    vz = c[5];
                for (let i = 0; i < count; i++) {
                    px[i] += vx[i] * DT;
                    py[i] += vy[i] * DT;
                    pz[i] += vz[i] * DT;
                }
            });
        },

        // B — cursor with array-indexed columns, inline loop
        "cursor[] inline": () => {
            for (let a = 0; a < q.archetypes.length; a++) {
                const cur = arrCur as any;
                cur.bind(q.bindings[a]);
                const count = q.archetypes[a].count;
                for (let i = 0; i < count; i++) {
                    cur.i = i;
                    cur.px += cur.vx * DT;
                    cur.py += cur.vy * DT;
                    cur.pz += cur.vz * DT;
                }
            }
        },

        // C — cursor with named slots, inline loop
        "cursor_slot inline": () => {
            for (let a = 0; a < q.archetypes.length; a++) {
                const cur = sloCur as any;
                cur.bind(q.bindings[a]);
                const count = q.archetypes[a].count;
                for (let i = 0; i < count; i++) {
                    cur.i = i;
                    cur.px += cur.vx * DT;
                    cur.py += cur.vy * DT;
                    cur.pz += cur.vz * DT;
                }
            }
        },

        // D — generated cursor: named slot loads instead of computed keyed loads
        "cursor_gen inline": () => {
            for (let a = 0; a < q.archetypes.length; a++) {
                const cur = genCur as any;
                cur.bind(q.bindings[a]);
                const count = q.archetypes[a].count;
                for (let i = 0; i < count; i++) {
                    cur.i = i;
                    cur.px += cur.vx * DT;
                    cur.py += cur.vy * DT;
                    cur.pz += cur.vz * DT;
                }
            }
        },

        // E — per-entity callback over the generated cursor: the ergonomic API
        "each(cb) + gen": () => {
            eachEntity(q, genCur, (e) => {
                e.px += e.vx * DT;
                e.py += e.vy * DT;
                e.pz += e.vz * DT;
            });
        },

        // F — per-entity callback over the array cursor, to separate the two costs
        "each(cb) + array": () => {
            eachEntity(q, arrCur, (e) => {
                e.px += e.vx * DT;
                e.py += e.vy * DT;
                e.pz += e.vz * DT;
            });
        },
    };

    const res: [string, number][] = [];
    for (const [k, fn] of Object.entries(variants)) res.push([k, best(fn)]);
    sink += q.bindings[0][0][3];

    const base = res[0][1];
    console.log(
        `\n  \x1b[1m${label}\x1b[0m  (${q.archetypes.length} archetype${q.archetypes.length > 1 ? "s" : ""}, ${n.toLocaleString()} entities, ${shared ? "shared" : "plain"} buffer)`,
    );
    console.log("  " + "-".repeat(66));
    for (const [k, t] of res) {
        const rel = t / base;
        const flag = rel > 1.5 ? "\x1b[31m" : rel < 1.1 ? "\x1b[32m" : "";
        console.log(
            "  " +
                k.padEnd(22) +
                (t * 1000).toFixed(2).padStart(8) +
                " ms" +
                (n / t / 1e6).toFixed(0).padStart(8) +
                " M/s" +
                `${flag}${rel.toFixed(2)}x\x1b[0m`.padStart(16),
        );
    }
    return res;
}

/* ------------------------------------------------------------ structural ops */

function structural(n: number) {
    console.log(`\n  \x1b[1mStructural change\x1b[0m  (${n.toLocaleString()} ops)`);
    console.log("  " + "-".repeat(66));

    const world = new World({capacity: n * 2 + 16, shared: true, pages: 4096});
    const ents: number[] = [];
    const tSpawn = best(
        () => {
            for (let i = 0; i < n; i++) ents.push(world.spawn([Position, Velocity]));
        },
        1,
        0,
    );

    // move between archetypes — the expensive one
    const half = ents.slice(0, Math.min(n, 100_000));
    const tMove = best(
        () => {
            for (const e of half) world.move(e, [Position, Velocity, Health]);
            for (const e of half) world.move(e, [Position, Velocity]);
        },
        3,
        1,
    );

    const tDespawn = best(
        () => {
            for (const e of ents) world.despawn(e);
        },
        1,
        0,
    );

    const row = (k: string, t: number, ops: number) =>
        console.log(
            "  " +
                k.padEnd(22) +
                (t * 1000).toFixed(2).padStart(8) +
                " ms" +
                (ops / t / 1e6).toFixed(1).padStart(8) +
                " M ops/s",
        );
    row("spawn", tSpawn, n);
    row("add+remove component", tMove, half.length * 2);
    row("despawn", tDespawn, n);
}

/* --------------------------------------------------------------------- main */

const N = 1_000_000;
console.log("\n\x1b[1m═══ Spike 02 · cursor through a real archetype ═══\x1b[0m");
console.log(`  node ${process.version} · columns are views into one arena`);

// 1 archetype — closest to the original microbenchmark
runVariants("Single archetype", N, [[Position, Velocity]], true);

// 5 archetypes — the megamorphism risk: getters now see many column instances
const many = runVariants(
    "Five archetypes",
    N,
    [
        [Position, Velocity],
        [Position, Velocity, Health],
        [Position, Velocity, Sprite],
        [Position, Velocity, Health, Player],
        [Position, Velocity, Sprite, Enemy],
    ],
    true,
);

// same, over a plain ArrayBuffer — does SharedArrayBuffer cost anything?
const plain = runVariants(
    "Five archetypes",
    N,
    [
        [Position, Velocity],
        [Position, Velocity, Health],
        [Position, Velocity, Sprite],
        [Position, Velocity, Health, Player],
        [Position, Velocity, Sprite, Enemy],
    ],
    false,
);

structural(500_000);

/* ------------------------------------------------------------------ verdict */

console.log("\n  \x1b[1mVerdict\x1b[0m");
console.log("  " + "-".repeat(66));
const byName = (r: [string, number][], k: string) => r.find((x) => x[0] === k)![1];
const rawT = byName(many, "raw columns");
const ratio = byName(many, "cursor_gen inline") / rawT;
const cbRatio = byName(many, "each(cb) + gen") / rawT;
const sabRatio = rawT / byName(plain, "raw columns");
console.log(
    `  best cursor vs raw, 5 archetypes : ${ratio.toFixed(2)}x  ${ratio <= 1.15 ? "\x1b[32mPASS\x1b[0m — ship the cursor" : ratio <= 1.5 ? "\x1b[33mMARGINAL\x1b[0m" : "\x1b[31mFAIL\x1b[0m — fall back to codegen"}`,
);
console.log(
    `  ergonomic each(cb) vs raw        : ${cbRatio.toFixed(2)}x  ${cbRatio <= 1.3 ? "\x1b[32mPASS\x1b[0m" : "\x1b[33mcost is real\x1b[0m"}`,
);
console.log(
    `  shared vs plain buffer           : ${sabRatio.toFixed(2)}x  ${Math.abs(sabRatio - 1) < 0.12 ? "\x1b[32mPASS\x1b[0m — SAB is free" : "\x1b[33minvestigate\x1b[0m"}`,
);
console.log(`\n  (checksum ${sink.toFixed(4)})\n`);
