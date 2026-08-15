import {readFileSync} from "node:fs";

const NS = 1e9;
function timeBest(fn, reps, warmup = 5) {
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
const N = 1_000_000;
const REPS = 15;

// ============================================================ 1. is the JS loop fair?

console.log("\n\x1b[1m1. JS integrate variants — is the baseline unfairly slow?\x1b[0m");
console.log("-".repeat(72));

const bytes = readFileSync(new URL("./kernels_scalar.wasm", import.meta.url));
const {instance} = await WebAssembly.instantiate(bytes, {});
const mem = instance.exports.memory;
const BASE = 1 << 20;
mem.grow(Math.ceil((BASE + 12 * N * 4) / 65536) + 16 - mem.buffer.byteLength / 65536);

const mk = (o) => new Float32Array(mem.buffer, BASE + o * N * 4, N);
const px = mk(0),
    py = mk(1),
    pz = mk(2),
    vx = mk(3),
    vy = mk(4),
    vz = mk(5);
for (let i = 0; i < N; i++) {
    px[i] = i * 1e-3;
    vx[i] = 0.5;
    vy[i] = 0.25;
    vz[i] = 0.125;
}
const PTR = (o) => BASE + o * N * 4;

const variants = {
    "fused loop (original)": () => {
        for (let i = 0; i < N; i++) {
            px[i] += vx[i] * 0.016;
            py[i] += vy[i] * 0.016;
            pz[i] += vz[i] * 0.016;
        }
    },
    "three split loops": () => {
        for (let i = 0; i < N; i++) px[i] += vx[i] * 0.016;
        for (let i = 0; i < N; i++) py[i] += vy[i] * 0.016;
        for (let i = 0; i < N; i++) pz[i] += vz[i] * 0.016;
    },
    "Math.fround (f32 math)": () => {
        const f = Math.fround;
        for (let i = 0; i < N; i++) {
            px[i] = f(px[i] + f(vx[i] * 0.016));
            py[i] = f(py[i] + f(vy[i] * 0.016));
            pz[i] = f(pz[i] + f(vz[i] * 0.016));
        }
    },
    "local aliases + len hoist": () => {
        const a = px,
            b = py,
            c = pz,
            d = vx,
            e = vy,
            g = vz,
            n = N,
            dt = 0.016;
        for (let i = 0; i < n; i++) {
            a[i] += d[i] * dt;
            b[i] += e[i] * dt;
            c[i] += g[i] * dt;
        }
    },
    "wasm (scalar)": () =>
        instance.exports.integrate(PTR(0), PTR(1), PTR(2), PTR(3), PTR(4), PTR(5), N, 0.016),
};

const r1 = {};
for (const [k, fn] of Object.entries(variants)) r1[k] = timeBest(fn, REPS);
sink += px[5];
const fastestJs = Math.min(
    ...Object.entries(r1)
        .filter(([k]) => !k.startsWith("wasm"))
        .map(([, v]) => v),
);
for (const [k, t] of Object.entries(r1)) {
    const tag = k.startsWith("wasm") ? `  ← ${(fastestJs / t).toFixed(2)}x vs best JS` : "";
    console.log(`  ${k.padEnd(26)}${(t * 1000).toFixed(2).padStart(8)} ms${tag}`);
}

// ============================================================ 2. raw vs proxy, interleaved

console.log("\n\x1b[1m2. Iteration API — interleaved trials, is proxy really free?\x1b[0m");
console.log("-".repeat(72));

const qx = new Float32Array(N),
    qy = new Float32Array(N),
    qz = new Float32Array(N);
const wx = new Float32Array(N),
    wy = new Float32Array(N),
    wz = new Float32Array(N);
for (let i = 0; i < N; i++) {
    qx[i] = i * 1e-3;
    wx[i] = 0.5;
    wy[i] = 0.25;
    wz[i] = 0.125;
}

const raw = () => {
    for (let i = 0; i < N; i++) {
        qx[i] += wx[i] * 0.016;
        qy[i] += wy[i] * 0.016;
        qz[i] += wz[i] * 0.016;
    }
};

class Cursor {
    constructor() {
        this.i = 0;
    }
    get x() {
        return qx[this.i];
    }
    set x(v) {
        qx[this.i] = v;
    }
    get y() {
        return qy[this.i];
    }
    set y(v) {
        qy[this.i] = v;
    }
    get z() {
        return qz[this.i];
    }
    set z(v) {
        qz[this.i] = v;
    }
    get lx() {
        return wx[this.i];
    }
    get ly() {
        return wy[this.i];
    }
    get lz() {
        return wz[this.i];
    }
}
const cur = new Cursor();
const proxy = () => {
    for (let i = 0; i < N; i++) {
        cur.i = i;
        cur.x += cur.lx * 0.016;
        cur.y += cur.ly * 0.016;
        cur.z += cur.lz * 0.016;
    }
};

// interleave to cancel drift / thermal effects
const rawT = [],
    proxyT = [];
for (let r = 0; r < 12; r++) {
    rawT.push(timeBest(raw, 1, 0));
    proxyT.push(timeBest(proxy, 1, 0));
}
sink += qx[5];
const med = (a) => a.slice().sort((x, y) => x - y)[Math.floor(a.length / 2)];
const mn = (a) => Math.min(...a);
console.log(
    `  raw columns          min ${(mn(rawT) * 1000).toFixed(2).padStart(7)} ms   median ${(med(rawT) * 1000).toFixed(2).padStart(7)} ms`,
);
console.log(
    `  reused cursor        min ${(mn(proxyT) * 1000).toFixed(2).padStart(7)} ms   median ${(med(proxyT) * 1000).toFixed(2).padStart(7)} ms`,
);
console.log(`  → cursor / raw       ${(med(proxyT) / med(rawT)).toFixed(3)}x (median)`);

// ============================================================ 3. what actually costs at the boundary

console.log("\n\x1b[1m3. Per-entity boundary cost — the call, or what surrounds it?\x1b[0m");
console.log("-".repeat(72));

const ITER = 5_000_000;
const noop = instance.exports.noop;
const scratch = new Float32Array(mem.buffer, BASE + 11 * N * 4, 4);

const pat = {
    "bare wasm call": () => {
        let a = 0;
        for (let i = 0; i < ITER; i++) a = noop(a);
        sink += a;
    },
    "wasm call + scratch read": () => {
        let a = 0;
        for (let i = 0; i < ITER; i++) {
            a += noop(i) + scratch[0] + scratch[1] + scratch[2];
        }
        sink += a;
    },
    "wasm call + {x,y,z} alloc": () => {
        let a = 0;
        for (let i = 0; i < ITER; i++) {
            const o = {x: scratch[0], y: scratch[1], z: scratch[2]};
            a += noop(i) + o.x + o.y + o.z;
        }
        sink += a;
    },
    "pure JS object alloc only": () => {
        let a = 0;
        for (let i = 0; i < ITER; i++) {
            const o = {x: scratch[0], y: scratch[1], z: scratch[2]};
            a += o.x + o.y + o.z;
        }
        sink += a;
    },
};
const r3 = {};
for (const [k, fn] of Object.entries(pat)) r3[k] = timeBest(fn, 5);
for (const [k, t] of Object.entries(r3)) {
    console.log(`  ${k.padEnd(28)}${((t / ITER) * 1e9).toFixed(2).padStart(7)} ns/entity`);
}
const perFrame = (ns) => ((ns * 10000) / 1e6).toFixed(3);
console.log(`\n  cost of 10k per-entity crossings per frame:`);
for (const [k, t] of Object.entries(r3)) {
    console.log(`    ${k.padEnd(28)}${perFrame((t / ITER) * 1e9).padStart(7)} ms/frame`);
}

// ============================================================ 4. f32 vs f64 divergence

console.log("\n\x1b[1m4. Do JS and Rust agree bit-for-bit?\x1b[0m");
console.log("-".repeat(72));
const A = new Float32Array(mem.buffer, BASE, 3);
const B = new Float32Array(3);
A[0] = 0.1;
A[1] = 0.1;
A[2] = 0.1;
B[0] = 0.1;
B[1] = 0.1;
B[2] = 0.1;
// JS: f64 intermediate, rounded on store. Rust: f32 throughout.
for (let k = 0; k < 1000; k++) B[0] += 0.1 * 0.016;
instance.exports.integrate(BASE, BASE + 4, BASE + 8, BASE, BASE + 4, BASE + 8, 0, 0.016);
console.log(`  JS f64-intermediate accumulate (1000 steps): ${B[0].toPrecision(12)}`);
let C = Math.fround(0.1);
for (let k = 0; k < 1000; k++)
    C = Math.fround(C + Math.fround(Math.fround(0.1) * Math.fround(0.016)));
console.log(`  strict f32 accumulate       (1000 steps): ${C.toPrecision(12)}`);
console.log(`  identical: ${B[0] === C}`);

console.log(`\n  (checksum ${sink.toFixed(3)})\n`);
