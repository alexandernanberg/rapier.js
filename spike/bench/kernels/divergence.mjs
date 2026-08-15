import {readFileSync} from "node:fs";
const bytes = readFileSync(new URL("./kernels_scalar.wasm", import.meta.url));
const {instance} = await WebAssembly.instantiate(bytes, {});
const mem = instance.exports.memory,
    BASE = 1 << 20;
const N = 10000;
mem.grow(Math.ceil((BASE + 16 * N * 4) / 65536) + 4);
const col = (o) => new Float32Array(mem.buffer, BASE + o * N * 4, N);
const ptr = (o) => BASE + o * N * 4;
const [ax, ay, az, aw, bx, by, bz, bw, ox, oy, oz, ow] = Array.from({length: 12}, (_, i) => col(i));

let s = 987654321;
const rnd = () => (s = (s * 1103515245 + 12345) & 0x7fffffff) / 0x7fffffff - 0.5;
for (let i = 0; i < N; i++) {
    ax[i] = rnd();
    ay[i] = rnd();
    az[i] = rnd();
    aw[i] = rnd();
    bx[i] = rnd();
    by[i] = rnd();
    bz[i] = rnd();
    bw[i] = rnd();
}

// Rust computes it, into columns 8..11
instance.exports.qmul(
    ptr(0),
    ptr(1),
    ptr(2),
    ptr(3),
    ptr(4),
    ptr(5),
    ptr(6),
    ptr(7),
    ptr(8),
    ptr(9),
    ptr(10),
    ptr(11),
    N,
);
const rust = [
    Float32Array.from(ox),
    Float32Array.from(oy),
    Float32Array.from(oz),
    Float32Array.from(ow),
];

// JS computes the same, f64 intermediates rounded on store
for (let i = 0; i < N; i++) {
    const x1 = ax[i],
        y1 = ay[i],
        z1 = az[i],
        w1 = aw[i],
        x2 = bx[i],
        y2 = by[i],
        z2 = bz[i],
        w2 = bw[i];
    ox[i] = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
    oy[i] = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
    oz[i] = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
    ow[i] = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
}
const js = [ox, oy, oz, ow];

let diff = 0,
    maxUlp = 0;
const ib = new Int32Array(1),
    fb = new Float32Array(ib.buffer);
const bits = (v) => {
    fb[0] = v;
    return ib[0];
};
for (let c = 0; c < 4; c++)
    for (let i = 0; i < N; i++) {
        if (rust[c][i] !== js[c][i]) {
            diff++;
            maxUlp = Math.max(maxUlp, Math.abs(bits(rust[c][i]) - bits(js[c][i])));
        }
    }
console.log(`\n  qmul over ${N} entities x 4 components = ${4 * N} results`);
console.log(`  bit-identical mismatches : ${diff} (${((100 * diff) / (4 * N)).toFixed(1)}%)`);
console.log(`  max divergence           : ${maxUlp} ULP`);
console.log(`  => JS and Rust ${diff ? "DIVERGE" : "agree"} on multi-term f32 math\n`);
