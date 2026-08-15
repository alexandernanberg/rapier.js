import {readFileSync} from "node:fs";

// ---------------------------------------------------------------- harness

const NS = 1e9;
function timeBest(fn, reps, warmup = 3) {
    for (let i = 0; i < warmup; i++) fn();
    let best = Infinity;
    for (let i = 0; i < reps; i++) {
        const t0 = process.hrtime.bigint();
        fn();
        const t1 = process.hrtime.bigint();
        const s = Number(t1 - t0) / NS;
        if (s < best) best = s;
    }
    return best;
}

let sink = 0;
const fmt = (n, d = 2) => n.toFixed(d).padStart(9);

// ---------------------------------------------------------------- wasm setup

async function loadWasm(path, pages) {
    const bytes = readFileSync(path);
    const {instance} = await WebAssembly.instantiate(bytes, {});
    const mem = instance.exports.memory;
    const need = pages - mem.buffer.byteLength / 65536;
    if (need > 0) mem.grow(Math.ceil(need));
    return {ex: instance.exports, mem};
}

const N_MAX = 1_000_000;
const BASE = 1 << 20; // past static data
const PAGES = Math.ceil((BASE + 12 * N_MAX * 4) / 65536) + 16;

const scalar = await loadWasm(new URL("./kernels_scalar.wasm", import.meta.url), PAGES);
const simd = await loadWasm(new URL("./kernels_simd.wasm", import.meta.url), PAGES);

// ---------------------------------------------------------------- layout

function layout(mem, n) {
    const cols = {};
    const names = ["px", "py", "pz", "vx", "vy", "vz", "ax", "ay", "az", "aw", "ox", "oy"];
    let off = BASE;
    for (const nm of names) {
        cols[nm] = {ptr: off, arr: new Float32Array(mem.buffer, off, n)};
        off += n * 4;
    }
    return cols;
}

function seed(c, n) {
    let s = 12345;
    const rnd = () => (s = (s * 1103515245 + 12345) & 0x7fffffff) / 0x7fffffff - 0.5;
    for (const k of Object.keys(c)) {
        const a = c[k].arr;
        for (let i = 0; i < n; i++) a[i] = rnd();
    }
}

// ---------------------------------------------------------------- JS kernels

function jsIntegrate(px, py, pz, vx, vy, vz, n, dt) {
    for (let i = 0; i < n; i++) {
        px[i] += vx[i] * dt;
        py[i] += vy[i] * dt;
        pz[i] += vz[i] * dt;
    }
}

function jsQmul(ax, ay, az, aw, bx, by, bz, bw, ox, oy, oz, ow, n) {
    for (let i = 0; i < n; i++) {
        const x1 = ax[i],
            y1 = ay[i],
            z1 = az[i],
            w1 = aw[i];
        const x2 = bx[i],
            y2 = by[i],
            z2 = bz[i],
            w2 = bw[i];
        ox[i] = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
        oy[i] = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
        oz[i] = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
        ow[i] = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
    }
}

// ---------------------------------------------------------------- run

const REPS = 7;

function runSize(n) {
    console.log(`\n\x1b[1m  N = ${n.toLocaleString()} entities\x1b[0m`);
    console.log("  " + "-".repeat(74));

    const cS = layout(scalar.mem, n);
    const cV = layout(simd.mem, n);
    seed(cS, n);
    seed(cV, n);

    const P = (c) => [c.px.ptr, c.py.ptr, c.pz.ptr, c.vx.ptr, c.vy.ptr, c.vz.ptr];
    const A = (c) => [c.px.arr, c.py.arr, c.pz.arr, c.vx.arr, c.vy.arr, c.vz.arr];
    const QP = (c) => [
        c.ax.ptr,
        c.ay.ptr,
        c.az.ptr,
        c.aw.ptr,
        c.vx.ptr,
        c.vy.ptr,
        c.vz.ptr,
        c.px.ptr,
        c.ox.ptr,
        c.oy.ptr,
        c.py.ptr,
        c.pz.ptr,
    ];
    const QA = (c) => [
        c.ax.arr,
        c.ay.arr,
        c.az.arr,
        c.aw.arr,
        c.vx.arr,
        c.vy.arr,
        c.vz.arr,
        c.px.arr,
        c.ox.arr,
        c.oy.arr,
        c.py.arr,
        c.pz.arr,
    ];

    const rows = [];
    const M = n / 1e6;

    // --- integrate (memory-bound)
    const tJsInt = timeBest(() => jsIntegrate(...A(cS), n, 0.016), REPS);
    const tWsInt = timeBest(() => scalar.ex.integrate(...P(cS), n, 0.016), REPS);
    const tWvInt = timeBest(() => simd.ex.integrate(...P(cV), n, 0.016), REPS);
    sink += cS.px.arr[7] + cV.px.arr[7];
    rows.push(["integrate (mem-bound)", tJsInt, tWsInt, tWvInt, M]);

    // --- qmul (compute-denser)
    const tJsQ = timeBest(() => jsQmul(...QA(cS), n), REPS);
    const tWsQ = timeBest(() => scalar.ex.qmul(...QP(cS), n), REPS);
    const tWvQ = timeBest(() => simd.ex.qmul(...QP(cV), n), REPS);
    sink += cS.ox.arr[7] + cV.ox.arr[7];
    rows.push(["qmul (compute-dense)", tJsQ, tWsQ, tWvQ, M]);

    console.log(
        "  " +
            "kernel".padEnd(24) +
            "JS".padStart(11) +
            "wasm".padStart(11) +
            "wasm+simd".padStart(11) +
            "   wasm/JS".padStart(12) +
            " simd/JS".padStart(9),
    );
    for (const [name, js, ws, wv] of rows) {
        console.log(
            "  " +
                name.padEnd(24) +
                fmt(js * 1000).padStart(9) +
                "ms" +
                fmt(ws * 1000).padStart(9) +
                "ms" +
                fmt(wv * 1000).padStart(9) +
                "ms" +
                `${(js / ws).toFixed(2)}x`.padStart(12) +
                `${(js / wv).toFixed(2)}x`.padStart(9),
        );
    }
}

// ---------------------------------------------------------------- iteration APIs

function iterationSpike(n) {
    console.log(`\n\x1b[1m  Iteration API — N = ${n.toLocaleString()}\x1b[0m`);
    console.log("  " + "-".repeat(74));

    const px = new Float32Array(n),
        py = new Float32Array(n),
        pz = new Float32Array(n);
    const vx = new Float32Array(n),
        vy = new Float32Array(n),
        vz = new Float32Array(n);
    for (let i = 0; i < n; i++) {
        px[i] = i * 0.1;
        vx[i] = 0.5;
        vy[i] = 0.25;
        vz[i] = 0.125;
    }

    // A: raw column loop
    const raw = () => {
        for (let i = 0; i < n; i++) {
            px[i] += vx[i] * 0.016;
            py[i] += vy[i] * 0.016;
            pz[i] += vz[i] * 0.016;
        }
    };

    // B: reused proxy object with getters/setters onto the columns
    class Proxy2 {
        constructor() {
            this.i = 0;
        }
        get x() {
            return px[this.i];
        }
        set x(v) {
            px[this.i] = v;
        }
        get y() {
            return py[this.i];
        }
        set y(v) {
            py[this.i] = v;
        }
        get z() {
            return pz[this.i];
        }
        set z(v) {
            pz[this.i] = v;
        }
        get lx() {
            return vx[this.i];
        }
        get ly() {
            return vy[this.i];
        }
        get lz() {
            return vz[this.i];
        }
    }
    const pxy = new Proxy2();
    const proxy = () => {
        for (let i = 0; i < n; i++) {
            pxy.i = i;
            pxy.x += pxy.lx * 0.016;
            pxy.y += pxy.ly * 0.016;
            pxy.z += pxy.lz * 0.016;
        }
    };

    // C: generator-based iteration over the proxy (the ergonomic `for..of` form)
    function* each() {
        for (let i = 0; i < n; i++) {
            pxy.i = i;
            yield pxy;
        }
    }
    const gen = () => {
        for (const e of each()) {
            e.x += e.lx * 0.016;
            e.y += e.ly * 0.016;
            e.z += e.lz * 0.016;
        }
    };

    // D: array-of-structs, the naive baseline
    const aos = new Array(n);
    for (let i = 0; i < n; i++) aos[i] = {x: i * 0.1, y: 0, z: 0, lx: 0.5, ly: 0.25, lz: 0.125};
    const aosRun = () => {
        for (let i = 0; i < n; i++) {
            const e = aos[i];
            e.x += e.lx * 0.016;
            e.y += e.ly * 0.016;
            e.z += e.lz * 0.016;
        }
    };

    const results = [
        ["raw columns", timeBest(raw, REPS)],
        ["reused proxy object", timeBest(proxy, REPS)],
        ["generator + proxy", timeBest(gen, REPS)],
        ["array-of-structs", timeBest(aosRun, REPS)],
    ];
    sink += px[3] + aos[3].x;
    const base = results[0][1];
    console.log(
        "  " +
            "pattern".padEnd(26) +
            "time".padStart(12) +
            "ent/sec".padStart(16) +
            "vs raw".padStart(10),
    );
    for (const [name, t] of results) {
        console.log(
            "  " +
                name.padEnd(26) +
                fmt(t * 1000).padStart(10) +
                "ms" +
                (n / t / 1e6).toFixed(0).padStart(13) +
                "M" +
                `${(t / base).toFixed(2)}x`.padStart(10),
        );
    }
}

// ---------------------------------------------------------------- call overhead

function callOverhead() {
    console.log(`\n\x1b[1m  JS → WASM call overhead\x1b[0m`);
    console.log("  " + "-".repeat(74));
    const ITER = 20_000_000;
    const noop = scalar.ex.noop;
    const jsNoop = (x) => x + 1;

    const tW = timeBest(() => {
        let a = 0;
        for (let i = 0; i < ITER; i++) a = noop(a);
        sink += a;
    }, 5);
    const tJ = timeBest(() => {
        let a = 0;
        for (let i = 0; i < ITER; i++) a = jsNoop(a);
        sink += a;
    }, 5);

    const nsW = (tW / ITER) * 1e9;
    const nsJ = (tJ / ITER) * 1e9;
    console.log(`  wasm call (imported fn)   ${nsW.toFixed(2).padStart(8)} ns/call`);
    console.log(`  js call (inlinable)       ${nsJ.toFixed(2).padStart(8)} ns/call`);
    console.log(`  overhead attributable     ${(nsW - nsJ).toFixed(2).padStart(8)} ns/call`);
    console.log(
        `\n  → at 10,000 bodies, per-body calls cost ~${(((nsW - nsJ) * 10000) / 1e6).toFixed(2)} ms/frame in overhead alone`,
    );
    console.log(
        `  → 16.6 ms budget is exhausted by ~${Math.round(16.6e6 / (nsW - nsJ)).toLocaleString()} boundary calls`,
    );
}

// ---------------------------------------------------------------- main

console.log("\n\x1b[1m═══ ECS kernel: JS vs Rust/WASM, identical memory ═══\x1b[0m");
console.log(
    `  node ${process.version} · ${process.arch} · kernels operate on the same wasm linear memory`,
);
runSize(100_000);
runSize(1_000_000);
iterationSpike(100_000);
iterationSpike(1_000_000);
callOverhead();
console.log(`\n  (checksum ${sink.toFixed(4)})\n`);
