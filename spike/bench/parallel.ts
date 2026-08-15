/**
 * Spike 03 — the parallel executor, and whether the equivalence claim survives.
 *
 * Two questions:
 *   1. Does chunking across workers actually pay at this core count, and does it
 *      pay differently for memory-bound versus compute-dense systems?
 *   2. Does a parallel run produce a byte-identical world to a serial one — and
 *      do the two documented traps really bite if the rules are ignored?
 *
 * Simplified to a single archetype on purpose: archetype iteration was measured
 * in spike 02, and what is under test here is chunking and synchronisation.
 */

import {JOB} from "../src/kernels.ts";
import {Pool, runSerial} from "../src/parallel.ts";

const N = 1_000_000;
const COLS = 6;
const PAGE = 65536;

/* ------------------------------------------------------------------- fixture */

function makeWorld() {
    const bytes = COLS * N * 4;
    const mem = new WebAssembly.Memory({
        initial: Math.ceil(bytes / PAGE) + 4,
        maximum: Math.ceil(bytes / PAGE) + 4,
        shared: true,
    });
    const buffer = mem.buffer as SharedArrayBuffer;
    const offsets: number[] = [];
    for (let c = 0; c < COLS; c++) offsets.push(c * N * 4);
    const cols = offsets.map((o) => new Float32Array(buffer, o, N));
    return {buffer, offsets, cols};
}

function seed(cols: Float32Array[]) {
    let s = 2463534242;
    const rnd = () => {
        s ^= s << 13;
        s >>>= 0;
        s ^= s >> 17;
        s ^= s << 5;
        s >>>= 0;
        return s / 4294967296;
    };
    for (const c of cols) for (let i = 0; i < N; i++) c[i] = rnd();
}

function snapshot(buffer: SharedArrayBuffer): Uint8Array {
    return new Uint8Array(new Uint8Array(buffer).slice(0, COLS * N * 4));
}

function firstDiff(a: Uint8Array, b: Uint8Array): number {
    if (a.length !== b.length) return 0;
    for (let i = 0; i < a.length; i++) if (a[i] !== b[i]) return i;
    return -1;
}

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

/* ---------------------------------------------------------------------- main */

const MAX = Math.min(4, (await import("node:os")).cpus().length);
console.log("\n\x1b[1m═══ Spike 03 · parallel executor ═══\x1b[0m");
console.log(`  node ${process.version} · ${MAX} hardware threads · ${N.toLocaleString()} entities`);

/* ---- 1. scaling ---- */

console.log("\n  \x1b[1mScaling\x1b[0m  (main thread counts as one chunk)");
console.log("  " + "-".repeat(68));
console.log(
    "  " +
        "job".padEnd(14) +
        ["1", "2", "3", "4"].map((t) => `${t}t`.padStart(11)).join("") +
        "   speedup",
);

for (const [name, job] of [
    ["integrate", JOB.integrate],
    ["heavy", JOB.heavy],
] as const) {
    const times: number[] = [];
    for (let t = 1; t <= MAX; t++) {
        const {buffer, offsets, cols} = makeWorld();
        seed(cols);
        const pool = await Pool.create(buffer, offsets, N, t);
        times.push(best(() => pool.run(job)));
        await pool.destroy();
    }
    const sp = times[0] / times[times.length - 1];
    const colr = sp > 2.5 ? "\x1b[32m" : sp > 1.6 ? "\x1b[33m" : "\x1b[31m";
    console.log(
        "  " +
            name.padEnd(14) +
            times.map((x) => `${(x * 1000).toFixed(2)}ms`.padStart(11)).join("") +
            `${colr}${sp.toFixed(2)}x\x1b[0m`.padStart(15),
    );
}

/* ---- 2. equivalence ---- */

console.log("\n  \x1b[1mSerial / parallel equivalence\x1b[0m");
console.log("  " + "-".repeat(68));

for (const [name, job] of [
    ["integrate", JOB.integrate],
    ["heavy", JOB.heavy],
] as const) {
    // serial
    const a = makeWorld();
    seed(a.cols);
    const rs = new Float64Array(MAX);
    const os = new Int32Array(8);
    for (let k = 0; k < 10; k++) runSerial(a.cols, N, MAX, job, rs, os, 1);
    const serialSnap = snapshot(a.buffer);

    // parallel, identical seed
    const b = makeWorld();
    seed(b.cols);
    const pool = await Pool.create(b.buffer, b.offsets, N, MAX);
    for (let k = 0; k < 10; k++) pool.run(job);
    const parallelSnap = snapshot(b.buffer);
    await pool.destroy();

    const d = firstDiff(serialSnap, parallelSnap);
    console.log(
        `  ${name.padEnd(14)} 10 ticks x ${MAX} threads  ` +
            (d < 0 ? "\x1b[32mbyte-identical\x1b[0m" : `\x1b[31mDIVERGED at byte ${d}\x1b[0m`),
    );
}

/* ---- 3. the two traps ---- */

console.log("\n  \x1b[1mThe traps — do the rules actually matter?\x1b[0m");
console.log("  " + "-".repeat(68));

{
    const {buffer, offsets, cols} = makeWorld();
    seed(cols);
    const pool = await Pool.create(buffer, offsets, N, MAX);

    // (a) reduction: the observable is the *chunk count*, not just merge order
    const f = Math.fround;
    const combine = (vals: ArrayLike<number>, n: number, seq?: Int32Array) => {
        let acc = 0;
        for (let i = 0; i < n; i++) acc = f(acc + vals[seq ? seq[i] : i]);
        return acc;
    };

    pool.run(JOB.sumY);
    const parN = combine(pool.results, MAX);
    const parByCompletion = combine(pool.results, MAX, pool.order);

    const sc = makeWorld();
    seed(sc.cols);
    const rs = new Float64Array(MAX);
    runSerial(sc.cols, N, 1, JOB.sumY, rs, new Int32Array(8), 1);
    const serial1 = f(rs[0]);
    runSerial(sc.cols, N, MAX, JOB.sumY, rs, new Int32Array(8), 1);
    const serialN = combine(rs, MAX);

    console.log(`  reduction, serial as 1 chunk      : ${serial1.toPrecision(9)}`);
    console.log(`  reduction, serial as ${MAX} chunks     : ${serialN.toPrecision(9)}`);
    console.log(`  reduction, parallel ${MAX} threads     : ${parN.toPrecision(9)}`);
    console.log(`  reduction, merged by completion   : ${parByCompletion.toPrecision(9)}`);
    console.log(
        `\n  parallel == serial at same chunk count : ${
            parN === serialN ? "\x1b[32mYES\x1b[0m" : "\x1b[31mNO\x1b[0m"
        }`,
    );
    console.log(
        `  chunk count changes the answer        : ${
            serial1 !== serialN ? "\x1b[33mYES\x1b[0m — so it must not track core count" : "no"
        }`,
    );
    console.log(
        `  merge order changes the answer        : ${
            parN !== parByCompletion ? "\x1b[33mYES\x1b[0m" : `not at ${MAX} partials this run`
        }   (completion [${Array.from(pool.order).join(",")}])`,
    );

    // (b) command buffer: merge per-chunk regions by index vs by completion
    pool.run(JOB.collect);
    const merge = (seq: number[]) => {
        const acc: number[] = [];
        for (const w of seq) {
            const n = pool.results[w];
            for (let k = 0; k < n; k++) acc.push(pool.out[w * pool.outStride + k]);
        }
        return acc;
    };
    const byIdx = merge([...Array(MAX).keys()]);
    const byComp = merge(Array.from(pool.order));
    const sameOrder = byIdx.length === byComp.length && byIdx.every((v, i) => v === byComp[i]);
    console.log(`\n  command buffer, entities matched  : ${byIdx.length.toLocaleString()}`);
    console.log(
        `  merge by index vs by completion   : ${
            sameOrder
                ? "identical on this run (completion happened to be in order)"
                : "\x1b[33mDIFFERENT\x1b[0m — merge order is load-bearing"
        }`,
    );
    await pool.destroy();
}

console.log();
