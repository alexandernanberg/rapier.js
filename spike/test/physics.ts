/**
 * Spike 05 — Rapier driven from ECS columns, with a purpose-built ABI.
 *
 * The design claims a "tier 1" physics backend can share the engine's linear
 * memory so transforms are written into their final home with no copy. This
 * checks whether that is real, what it costs, and where it breaks.
 */

import {BodyKind, Physics, Shape} from "../src/physics.ts";
import {Query} from "../src/query.ts";
import {component} from "../src/schema.ts";
import {Arena, World} from "../src/world.ts";

/* ------------------------------------------------------------ tiny runner */

let passed = 0;
let failed = 0;
const failures: string[] = [];

function check(name: string, fn: () => void | Promise<void>) {
    return Promise.resolve()
        .then(fn)
        .then(() => {
            passed++;
            console.log(`  \x1b[32m✓\x1b[0m ${name}`);
        })
        .catch((err: unknown) => {
            failed++;
            const msg = err instanceof Error ? err.message : String(err);
            failures.push(`${name}: ${msg}`);
            console.log(`  \x1b[31m✗\x1b[0m ${name}\n      ${msg}`);
        });
}

const ok = (c: boolean, what: string) => {
    if (!c) throw new Error(what);
};
const close = (a: number, b: number, tol: number, what: string) => {
    if (Math.abs(a - b) > tol) throw new Error(`${what}: expected ~${b} (±${tol}), got ${a}`);
};
const section = (t: string) => console.log(`\n  \x1b[1m${t}\x1b[0m\n  ${"-".repeat(66)}`);

/* --------------------------------------------------------------- fixture */

const Transform = component("Transform", {
    tx: "f32",
    ty: "f32",
    tz: "f32",
    rx: "f32",
    ry: "f32",
    rz: "f32",
    rw: "f32",
});
const RigidBody = component("RigidBody", {handle: "u32"});

interface Rig {
    phys: Physics;
    world: World;
    q: Query;
    cols: {
        h: Uint32Array;
        tx: Float32Array;
        ty: Float32Array;
        tz: Float32Array;
        rx: Float32Array;
        ry: Float32Array;
        rz: Float32Array;
        rw: Float32Array;
    };
    count: number;
    sync(): void;
}

/** Build an ECS whose arena lives inside the physics module's memory. */
async function makeRig(bodies: number, opts: {ground?: boolean} = {}): Promise<Rig> {
    const phys = await Physics.create([0, -9.81, 0], 4 << 20);
    const arena = Arena.borrow(phys.memory.buffer, phys.arenaPtr, phys.arenaBytes);
    const world = new World({capacity: bodies + 8, arena});

    if (opts.ground !== false) {
        phys.addBody({
            kind: BodyKind.fixed,
            shape: Shape.cuboid,
            size: [50, 0.5, 50],
            pos: [0, -0.5, 0],
        });
    }

    for (let i = 0; i < bodies; i++) {
        const handle = phys.addBody({
            shape: Shape.ball,
            size: [0.5, 0, 0],
            pos: [(i % 10) * 1.5 - 7, 5 + Math.floor(i / 10) * 1.5, 0],
        });
        const e = world.spawn([Transform, RigidBody]);
        const loc = world.locate(e)!;
        loc.arch.columns.get(`${RigidBody.id}:handle`)![loc.row] = handle;
    }

    // Body creation allocates in Rust; re-derive views afterwards if it grew.
    phys.checkBuffer();

    const q = new Query(world, [Transform, RigidBody]);
    const arch = q.archetypes[0];
    const col = (c: {id: number}, f: string) => arch.columns.get(`${c.id}:${f}`)!;
    const cols = {
        h: col(RigidBody, "handle") as Uint32Array,
        tx: col(Transform, "tx") as Float32Array,
        ty: col(Transform, "ty") as Float32Array,
        tz: col(Transform, "tz") as Float32Array,
        rx: col(Transform, "rx") as Float32Array,
        ry: col(Transform, "ry") as Float32Array,
        rz: col(Transform, "rz") as Float32Array,
        rw: col(Transform, "rw") as Float32Array,
    };
    const count = arch.count;

    return {
        phys,
        world,
        q,
        cols,
        count,
        sync() {
            phys.pullTransforms(
                cols.h,
                count,
                cols.tx,
                cols.ty,
                cols.tz,
                cols.rx,
                cols.ry,
                cols.rz,
                cols.rw,
            );
        },
    };
}

/* ==================================================================== */

console.log("\n\x1b[1m═══ Spike 05 · Rapier over ECS columns ═══\x1b[0m");
console.log(`  node ${process.version} · raw C ABI, no wasm-bindgen`);

section("Is the zero-copy claim real?");

await check("the ECS arena lives inside the physics module's memory", async () => {
    const r = await makeRig(4);
    ok(r.cols.tx.buffer === r.phys.memory.buffer, "column buffer is not the wasm memory buffer");
    ok(
        r.cols.tx.byteOffset >= r.phys.arenaPtr,
        `column at ${r.cols.tx.byteOffset} is below arena base ${r.phys.arenaPtr}`,
    );
    r.phys.destroy();
});

await check("Rapier writes transforms straight into ECS columns", async () => {
    const r = await makeRig(4);
    ok(r.cols.ty[0] === 0, "column should start zeroed");
    r.phys.step(1 / 60);
    r.sync();
    ok(r.cols.ty[0] !== 0, "column was not written by the pull");
    ok(r.cols.ty[0] < 5, `body should have fallen, got y=${r.cols.ty[0]}`);
    r.phys.destroy();
});

await check("growing the heap is the only thing that invalidates views", async () => {
    const phys = await Physics.create([0, -9.81, 0], 1 << 20);
    const before = phys.memory.buffer;
    for (let i = 0; i < 200; i++) phys.addBody({pos: [i * 0.1, 10, 0]});
    const grewOnCreate = phys.memory.buffer !== before;

    const after = phys.memory.buffer;
    for (let i = 0; i < 120; i++) phys.step(1 / 60);
    ok(phys.memory.buffer === after, "stepping must never move the memory");
    console.log(
        `      body creation grew memory: ${grewOnCreate ? "yes" : "no"}; ` +
            `stepping grew memory: no`,
    );
    phys.destroy();
});

section("Physics behaviour, asserted headlessly");

await check("a ball falls and comes to rest on the ground", async () => {
    const r = await makeRig(1);
    for (let i = 0; i < 240; i++) {
        r.phys.step(1 / 60);
    }
    r.sync();
    // ball radius 0.5 resting on a box whose top face is y=0
    close(r.cols.ty[0], 0.5, 0.05, "resting height");
    ok(r.phys.contactCount >= 1, "expected at least one contact pair");
    r.phys.destroy();
});

await check("no ground means no resting — the test can tell the difference", async () => {
    const r = await makeRig(1, {ground: false});
    for (let i = 0; i < 240; i++) r.phys.step(1 / 60);
    r.sync();
    ok(r.cols.ty[0] < -50, `expected free fall, got y=${r.cols.ty[0]}`);
    r.phys.destroy();
});

await check("two identical worlds stay bit-identical", async () => {
    const run = async () => {
        const r = await makeRig(60);
        for (let i = 0; i < 180; i++) r.phys.step(1 / 60);
        r.sync();
        const out = new Float32Array(r.count * 3);
        out.set(r.cols.tx.subarray(0, r.count), 0);
        out.set(r.cols.ty.subarray(0, r.count), r.count);
        out.set(r.cols.tz.subarray(0, r.count), r.count * 2);
        r.phys.destroy();
        return out;
    };
    const a = await run();
    const b = await run();
    for (let i = 0; i < a.length; i++) {
        if (a[i] !== b[i]) throw new Error(`diverged at ${i}: ${a[i]} vs ${b[i]}`);
    }
});

await check("60 balls settle into a pile without tunnelling", async () => {
    const r = await makeRig(60);
    for (let i = 0; i < 400; i++) r.phys.step(1 / 60);
    r.sync();
    let below = 0;
    for (let i = 0; i < r.count; i++) if (r.cols.ty[i] < -0.1) below++;
    ok(below === 0, `${below} bodies fell through the ground`);
    r.phys.destroy();
});

section("What the bulk sync costs");

{
    const N = 20_000;
    const r = await makeRig(N);
    for (let i = 0; i < 10; i++) {
        r.phys.step(1 / 60);
        r.sync();
    }

    const time = (fn: () => void, reps = 20) => {
        let best = Infinity;
        for (let i = 0; i < reps; i++) {
            const t0 = process.hrtime.bigint();
            fn();
            const s = Number(process.hrtime.bigint() - t0) / 1e6;
            if (s < best) best = s;
        }
        return best;
    };

    const tStep = time(() => r.phys.step(1 / 60));
    const tPull = time(() => r.sync());
    console.log(`  bodies                    ${N.toLocaleString()}`);
    console.log(`  phys_step                 ${tStep.toFixed(3)} ms`);
    console.log(`  pull_transforms (bulk)    ${tPull.toFixed(3)} ms`);
    console.log(`  per body                  ${((tPull * 1e6) / N).toFixed(1)} ns`);
    console.log(`  → sync is ${((tPull / tStep) * 100).toFixed(1)}% of the step it follows`);
    r.phys.destroy();
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
