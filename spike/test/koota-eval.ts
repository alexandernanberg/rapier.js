/**
 * Spike 06 — can Koota replace the custom ECS?
 *
 * Koota converges on the same shape we designed: schema traits, SoA stores, a
 * callback query API. The question is whether it can carry the two things this
 * engine actually needs from its storage layer — cheap snapshots (rewind, replay,
 * netcode) and determinism — and what it costs at the target scale.
 *
 * Compared throughout against the custom ECS numbers from spike 02.
 */

import {createWorld, getStore, trait, unpackEntity, type Trait, type World} from "koota";

/* ------------------------------------------------------------ tiny runner */

let passed = 0;
let failed = 0;
const failures: string[] = [];

function check(name: string, fn: () => void) {
    try {
        fn();
        passed++;
        console.log(`  \x1b[32m✓\x1b[0m ${name}`);
    } catch (err) {
        failed++;
        const msg = err instanceof Error ? err.message : String(err);
        failures.push(`${name}: ${msg}`);
        console.log(`  \x1b[31m✗\x1b[0m ${name}\n      ${msg}`);
    }
}

const ok = (c: boolean, what: string) => {
    if (!c) throw new Error(what);
};
const section = (t: string) => console.log(`\n  \x1b[1m${t}\x1b[0m\n  ${"-".repeat(66)}`);

function best(fn: () => void, reps = 7, warmup = 3): number {
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

/* ------------------------------------------------------------------ traits */

const Position = trait({px: 0, py: 0, pz: 0});
const Velocity = trait({vx: 0, vy: 0, vz: 0});
const Health = trait({hp: 100});
const Sprite = trait({tex: 0});
const Player = trait();
const DT = 0.016;

const ALL: [string, Trait][] = [
    ["Position", Position],
    ["Velocity", Velocity],
    ["Health", Health],
    ["Sprite", Sprite],
];

/* --------------------------------------------------------------- snapshot */

type Snap = Record<string, Record<string, unknown[]>>;

/** Deep-copy every SoA store. The engine's rewind/replay depends on this. */
function snapshot(world: World): Snap {
    const out: Snap = {};
    for (const [name, t] of ALL) {
        const store = getStore(world, t) as Record<string, unknown[]>;
        const cols: Record<string, unknown[]> = {};
        for (const k of Object.keys(store)) cols[k] = store[k].slice();
        out[name] = cols;
    }
    return out;
}

function restore(world: World, snap: Snap): void {
    for (const [name, t] of ALL) {
        const store = getStore(world, t) as Record<string, unknown[]>;
        const cols = snap[name];
        for (const k of Object.keys(cols)) {
            const src = cols[k];
            const dst = store[k];
            dst.length = src.length;
            for (let i = 0; i < src.length; i++) dst[i] = src[i];
        }
    }
}

function diffSnap(a: Snap, b: Snap): string | null {
    for (const name of Object.keys(a)) {
        for (const k of Object.keys(a[name])) {
            const x = a[name][k];
            const y = b[name][k];
            if (x.length !== y.length) return `${name}.${k} length ${x.length} vs ${y.length}`;
            for (let i = 0; i < x.length; i++) {
                if (x[i] !== y[i]) return `${name}.${k}[${i}] ${String(x[i])} vs ${String(y[i])}`;
            }
        }
    }
    return null;
}

/* --------------------------------------------------------------- fixtures */

function build(n: number, mixed: boolean): World {
    const w = createWorld();
    for (let i = 0; i < n; i++) {
        const extra = !mixed
            ? []
            : i % 5 === 1
              ? [Health]
              : i % 5 === 2
                ? [Sprite]
                : i % 5 === 3
                  ? [Health, Player]
                  : i % 5 === 4
                    ? [Sprite, Player]
                    : [];
        w.spawn(
            Position({px: (i % 97) * 0.01, py: 0, pz: 0}),
            Velocity({vx: 0.5, vy: 0.25, vz: 0.125}),
            ...extra,
        );
    }
    return w;
}

const integrate = (w: World) =>
    w.query(Position, Velocity).updateEach(([p, v]) => {
        p.px += v.vx * DT;
        p.py += v.vy * DT;
        p.pz += v.vz * DT;
    });

/* ==================================================================== */

console.log("\n\x1b[1m═══ Spike 06 · can Koota replace the custom ECS? ═══\x1b[0m");
console.log(`  node ${process.version} · koota 0.6.6`);

section("Storage reality check");

check("SoA stores are plain Arrays, not typed arrays", () => {
    const w = build(4, false);
    const s = getStore(w, Position) as Record<string, unknown[]>;
    ok(Array.isArray(s.px), "expected a plain Array");
    ok(!ArrayBuffer.isView(s.px as never), "unexpectedly a typed array");
    console.log(
        `      store.px is ${(s.px as unknown[]).constructor.name}, ` +
            `indexed by entity id (length ${(s.px as unknown[]).length} for 4 entities)`,
    );
});

check("stores are addressable and writable through getStore", () => {
    const w = build(4, false);
    const s = getStore(w, Position) as Record<string, number[]>;
    const before = s.px.slice();
    integrate(w);
    const after = getStore(w, Position) as Record<string, number[]>;
    ok(
        after.px.some((v, i) => v !== before[i]),
        "store did not change after updateEach",
    );
});

section("Iteration throughput \u2014 ergonomic vs raw, 1M entities");

{
    const N = 1_000_000;
    const w = build(N, true);
    const variants: Record<string, () => void> = {
        "updateEach (default)": () => integrate(w),
        "updateEach, no change-det": () =>
            w.query(Position, Velocity).updateEach(
                ([p, v]) => {
                    p.px += v.vx * DT;
                    p.py += v.vy * DT;
                    p.pz += v.vz * DT;
                },
                {changeDetection: "never"},
            ),
        "raw getStore loop": () => {
            const P = getStore(w, Position) as Record<string, number[]>;
            const V = getStore(w, Velocity) as Record<string, number[]>;
            const ents = w.query(Position, Velocity);
            for (let i = 0; i < ents.length; i++) {
                // the entity value packs a generation; the store is indexed by entityId
                const e = unpackEntity(ents[i]).entityId;
                P.px[e] += V.vx[e] * DT;
                P.py[e] += V.vy[e] * DT;
                P.pz[e] += V.vz[e] * DT;
            }
        },
    };
    for (const [k, fn] of Object.entries(variants)) {
        const t = best(fn, 5, 2);
        console.log(
            `  ${k.padEnd(26)}${(t * 1000).toFixed(1).padStart(8)} ms   ${(N / t / 1e6).toFixed(0).padStart(4)} M/s`,
        );
    }
    console.log(
        "  \x1b[2mcustom ECS, 1M, 5 archetypes: raw 170 M/s \u00b7 each(cb)+gen 68 M/s\x1b[0m",
    );
}

section("The two things the engine actually needs");

check("snapshot round-trips exactly", () => {
    const w = build(2000, true);
    for (let i = 0; i < 30; i++) integrate(w);
    const mark = snapshot(w);
    for (let i = 0; i < 30; i++) integrate(w);
    ok(diffSnap(mark, snapshot(w)) !== null, "world did not advance — test is vacuous");
    restore(w, mark);
    const d = diffSnap(mark, snapshot(w));
    ok(d === null, `restore did not round-trip: ${d}`);
});

check("two identical runs stay bit-identical", () => {
    const run = () => {
        const w = build(2000, true);
        for (let i = 0; i < 120; i++) integrate(w);
        return snapshot(w);
    };
    const d = diffSnap(run(), run());
    ok(d === null, `diverged: ${d}`);
});

section("Cost at the target scale (2,000 entities)");

{
    const w = build(2000, true);
    const tIter = best(() => integrate(w));
    const tSnap = best(() => snapshot(w));
    const mark = snapshot(w);
    const tRestore = best(() => restore(w, mark));
    console.log(`  iterate 2,000            ${(tIter * 1000).toFixed(3)} ms`);
    console.log(`  snapshot                 ${(tSnap * 1000).toFixed(3)} ms`);
    console.log(`  restore                  ${(tRestore * 1000).toFixed(3)} ms`);
    console.log(
        `  → snapshot every frame would cost ${(((tSnap * 1000) / 16.6) * 100).toFixed(1)}% of a 60Hz budget`,
    );
}

section("Structural change — where a sparse layout should win");

{
    const w = build(200_000, false);
    const ents = w.query(Position).slice(0, 100_000);
    const tAdd = best(
        () => {
            for (const e of ents) e.add(Health);
            for (const e of ents) e.remove(Health);
        },
        3,
        1,
    );
    const ops = ents.length * 2;
    console.log(
        `  add+remove trait         ${(tAdd * 1000).toFixed(2)} ms for ${ops.toLocaleString()} ops ` +
            `(${(ops / tAdd / 1e6).toFixed(1)} M ops/s)`,
    );
    console.log(`  \x1b[2mcustom ECS archetype move: 1.6 M ops/s\x1b[0m`);
}

/* ------------------------------------------------------------------ report */

console.log(`\n  ${"-".repeat(68)}`);
if (failed === 0) {
    console.log(`  \x1b[32m${passed} passed\x1b[0m, 0 failed\n`);
} else {
    console.log(`  \x1b[32m${passed} passed\x1b[0m, \x1b[31m${failed} failed\x1b[0m`);
    for (const f of failures) console.log(`    - ${f}`);
    console.log();
}
process.exit(failed === 0 ? 0 : 1);
