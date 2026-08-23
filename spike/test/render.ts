/**
 * Spike 07 — the render seam.
 *
 * The claim being tested is not "rendering works" — there is no GPU here. It is
 * that rendering can be driven entirely from ECS columns, that a frame is
 * assertable without pixels, and that no render backend holds truth the ECS does
 * not. If those hold, swapping Three for a custom WebGPU renderer later is a
 * contained change rather than a rewrite.
 */

import {readdirSync, readFileSync} from "node:fs";
import {Hidden, identityTransform, Renderable, RigidBodyRef, Transform} from "../src/components.ts";
import {BodyKind, Physics, Shape} from "../src/physics.ts";
import {Extractor, makeCamera} from "../src/render/extract.ts";
import {RecordingBackend} from "../src/render/recording.ts";
import {CONVENTIONS} from "../src/render/types.ts";
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
const eq = (a: unknown, b: unknown, what: string) => {
    if (a !== b) throw new Error(`${what}: expected ${String(b)}, got ${String(a)}`);
};
const close = (a: number, b: number, tol: number, what: string) => {
    if (Math.abs(a - b) > tol) throw new Error(`${what}: expected ~${b} (±${tol}), got ${a}`);
};
const section = (t: string) => console.log(`\n  \x1b[1m${t}\x1b[0m\n  ${"-".repeat(66)}`);

/* --------------------------------------------------------------- fixtures */

function setTransform(world: World, e: number, v: Partial<Record<string, number>>) {
    const loc = world.locate(e)!;
    const all = {...identityTransform(), ...v};
    for (const [f, val] of Object.entries(all)) {
        loc.arch.columns.get(`${Transform.id}:${f}`)![loc.row] = val;
    }
}

function setRenderable(world: World, e: number, mesh: number, material: number) {
    const loc = world.locate(e)!;
    loc.arch.columns.get(`${Renderable.id}:mesh`)![loc.row] = mesh;
    loc.arch.columns.get(`${Renderable.id}:material`)![loc.row] = material;
}

function scene() {
    const world = new World({capacity: 4096, pages: 64, shared: true});
    const gpu = new RecordingBackend();
    const cube = gpu.createMesh({name: "cube", positions: new Float32Array(24)});
    const ball = gpu.createMesh({name: "ball", positions: new Float32Array(24)});
    const steel = gpu.createMaterial({
        name: "steel",
        baseColor: [0.6, 0.6, 0.62],
        metallic: 1,
        roughness: 0.3,
    });
    const wood = gpu.createMaterial({
        name: "wood",
        baseColor: [0.45, 0.3, 0.16],
        metallic: 0,
        roughness: 0.8,
    });
    return {world, gpu, cube, ball, steel, wood};
}

/* ==================================================================== */

console.log("\n\x1b[1m═══ Spike 07 · the render seam ═══\x1b[0m");
console.log(`  node ${process.version} · no GPU, no canvas`);

section("The seam holds");

await check("no engine code outside render/ imports a render library", () => {
    const offenders: string[] = [];
    const walk = (dir: string) => {
        for (const entry of readdirSync(dir, {withFileTypes: true})) {
            const p = `${dir}/${entry.name}`;
            if (entry.isDirectory()) {
                if (entry.name === "node_modules") continue;
                walk(p);
                continue;
            }
            if (!entry.name.endsWith(".ts")) continue;
            if (p.includes("/render/")) continue;
            const src = readFileSync(p, "utf8");
            if (/from\s+["'](three|@react-three)/.test(src)) offenders.push(p);
        }
    };
    walk("spike/src");
    ok(offenders.length === 0, `render library imported outside render/: ${offenders.join(", ")}`);
});

await check("the backend holds resources, never scene truth", () => {
    const {world, gpu, cube, steel} = scene();
    const e = world.spawn([Transform, Renderable]);
    setTransform(world, e, {tx: 3});
    setRenderable(world, e, cube, steel);

    const ex = new Extractor(world);
    gpu.submit(ex.extract(), makeCamera([0, 5, 10], [0, 0, 0]));

    // the backend knows about meshes and materials, and nothing else
    eq(gpu.meshes.length, 2, "mesh count");
    eq(gpu.materials.length, 2, "material count");
    ok(
        !Object.keys(gpu).includes("scene") && !Object.keys(gpu).includes("objects"),
        "backend appears to hold scene state",
    );
    // moving the entity changes the next frame with no backend call at all
    setTransform(world, e, {tx: 9});
    gpu.submit(ex.extract(), makeCamera([0, 5, 10], [0, 0, 0]));
    eq(gpu.translationOf(gpu.lastFrame, 0, 0)[0], 9, "translation after ECS-only edit");
});

section("Conventions, asserted so a swap cannot change them");

await check("a 90° yaw maps +X to -Z (right-handed, Y-up)", () => {
    const {world, gpu, cube, steel} = scene();
    const e = world.spawn([Transform, Renderable]);
    const s = Math.SQRT1_2; // sin/cos of 45° — quaternion for a 90° rotation
    setTransform(world, e, {ry: s, rw: s});
    setRenderable(world, e, cube, steel);
    const ex = new Extractor(world);
    gpu.submit(ex.extract(), makeCamera([0, 5, 10], [0, 0, 0]));

    const m = gpu.lastFrame.batches[0].instances;
    // column 0 is the image of the +X basis vector
    close(m[0], 0, 1e-6, "col0.x");
    close(m[1], 0, 1e-6, "col0.y");
    close(m[2], -1, 1e-6, "col0.z");
    eq(CONVENTIONS.handedness, "right", "declared handedness");
    eq(CONVENTIONS.matrixLayout, "column-major", "declared layout");
});

await check("scale lands on the correct columns", () => {
    const {world, gpu, cube, steel} = scene();
    const e = world.spawn([Transform, Renderable]);
    setTransform(world, e, {sx: 2, sy: 3, sz: 4, tx: 1, ty: 2, tz: 3});
    setRenderable(world, e, cube, steel);
    const ex = new Extractor(world);
    gpu.submit(ex.extract(), makeCamera([0, 5, 10], [0, 0, 0]));
    const m = gpu.lastFrame.batches[0].instances;
    close(m[0], 2, 1e-6, "scale x");
    close(m[5], 3, 1e-6, "scale y");
    close(m[10], 4, 1e-6, "scale z");
    close(m[12], 1, 1e-6, "translate x");
    close(m[15], 1, 1e-6, "homogeneous w");
});

section("Frames are assertable without pixels");

await check("entities batch by mesh and material", () => {
    const {world, gpu, cube, ball, steel, wood} = scene();
    const combos: [number, number][] = [
        [cube, steel],
        [cube, wood],
        [ball, steel],
    ];
    for (let i = 0; i < 30; i++) {
        const e = world.spawn([Transform, Renderable]);
        setTransform(world, e, {tx: i});
        const [m, mat] = combos[i % combos.length];
        setRenderable(world, e, m, mat);
    }
    const ex = new Extractor(world);
    gpu.submit(ex.extract(), makeCamera([0, 5, 20], [0, 0, 0]));

    const f = gpu.lastFrame;
    eq(f.batches.length, 3, "batch count (one draw call each)");
    eq(f.instanceCount, 30, "total instances");
    for (const b of f.batches) eq(b.count, 10, `instances in batch ${b.mesh}/${b.material}`);
});

await check("hidden entities are excluded by archetype, not by branch", () => {
    const {world, gpu, cube, steel} = scene();
    for (let i = 0; i < 10; i++) {
        const e = world.spawn(i < 4 ? [Transform, Renderable, Hidden] : [Transform, Renderable]);
        setTransform(world, e, {tx: i});
        setRenderable(world, e, cube, steel);
    }
    const ex = new Extractor(world, {without: [Hidden]});
    gpu.submit(ex.extract(), makeCamera([0, 5, 20], [0, 0, 0]));
    eq(gpu.lastFrame.instanceCount, 6, "visible instances");
});

await check("a recorded frame keeps its own copy of the instance data", () => {
    const {world, gpu, cube, steel} = scene();
    const e = world.spawn([Transform, Renderable]);
    setTransform(world, e, {ty: 1});
    setRenderable(world, e, cube, steel);
    const ex = new Extractor(world);
    const cam = makeCamera([0, 5, 10], [0, 0, 0]);

    gpu.submit(ex.extract(), cam);
    const first = gpu.lastFrame;
    setTransform(world, e, {ty: 99});
    gpu.submit(ex.extract(), cam);

    eq(gpu.translationOf(first, 0, 0)[1], 1, "earlier frame must not be mutated");
    eq(gpu.translationOf(gpu.lastFrame, 0, 0)[1], 99, "latest frame");
});

section("A physics-driven frame, end to end");

await check("a falling body's rendered transform tracks the simulation", async () => {
    const phys = await Physics.create([0, -9.81, 0], 2 << 20);
    const arena = Arena.borrow(phys.memory.buffer, phys.arenaPtr, phys.arenaBytes);
    const world = new World({capacity: 256, arena});
    const gpu = new RecordingBackend();
    const ball = gpu.createMesh({name: "ball", positions: new Float32Array(24)});
    const mat = gpu.createMaterial({
        name: "rubber",
        baseColor: [0.8, 0.2, 0.2],
        metallic: 0,
        roughness: 0.9,
    });

    phys.addBody({
        kind: BodyKind.fixed,
        shape: Shape.cuboid,
        size: [50, 0.5, 50],
        pos: [0, -0.5, 0],
    });
    const handle = phys.addBody({shape: Shape.ball, size: [0.5, 0, 0], pos: [0, 8, 0]});
    phys.checkBuffer();

    const e = world.spawn([Transform, Renderable, RigidBodyRef]);
    setTransform(world, e, {ty: 8});
    setRenderable(world, e, ball, mat);
    const loc = world.locate(e)!;
    loc.arch.columns.get(`${RigidBodyRef.id}:handle`)![loc.row] = handle;

    const col = (f: string) => loc.arch.columns.get(`${Transform.id}:${f}`)!;
    const handles = loc.arch.columns.get(`${RigidBodyRef.id}:handle`)! as Uint32Array;
    const ex = new Extractor(world);
    const cam = makeCamera([0, 5, 15], [0, 0, 0]);

    for (let i = 0; i < 240; i++) {
        phys.step(1 / 60);
        phys.pullTransforms(
            handles,
            1,
            col("tx") as Float32Array,
            col("ty") as Float32Array,
            col("tz") as Float32Array,
            col("rx") as Float32Array,
            col("ry") as Float32Array,
            col("rz") as Float32Array,
            col("rw") as Float32Array,
        );
        gpu.submit(ex.extract(), cam);
    }

    const y = gpu.translationOf(gpu.lastFrame, 0, 0)[1];
    close(y, 0.5, 0.05, "rendered resting height");
    ok(gpu.frames.length > 1, "frames were recorded");
    phys.destroy();
});

section("Extraction cost");

{
    const {world, gpu, cube, ball, steel, wood} = scene();
    const combos: [number, number][] = [
        [cube, steel],
        [cube, wood],
        [ball, steel],
        [ball, wood],
    ];
    const N = 20_000;
    const measure = (label: string, pick: (i: number) => [number, number]) => {
        const big = new World({capacity: N + 16, pages: 256, shared: true});
        for (let i = 0; i < N; i++) {
            const e = big.spawn([Transform, Renderable]);
            setTransform(big, e, {tx: i % 50, ty: (i * 7) % 30, tz: (i * 3) % 40});
            const [m, mat] = pick(i);
            setRenderable(big, e, m, mat);
        }
        const ex = new Extractor(big);
        for (let i = 0; i < 5; i++) ex.extract();
        let bst = Infinity;
        for (let i = 0; i < 20; i++) {
            const t = process.hrtime.bigint();
            ex.extract();
            const sec = Number(process.hrtime.bigint() - t) / 1e6;
            if (sec < bst) bst = sec;
        }
        console.log(
            "  " +
                label.padEnd(22) +
                bst.toFixed(3).padStart(7) +
                " ms   " +
                ((bst * 1e6) / N).toFixed(1).padStart(5) +
                " ns/inst   " +
                ((bst / 16.6) * 100).toFixed(2) +
                "% of a frame",
        );
        return bst;
    };
    console.log("  " + N.toLocaleString() + " instances across 4 batches:");
    measure("materials interleaved", (i) => combos[i % combos.length]);
    measure("materials grouped", (i) => combos[Math.floor(i / (N / combos.length))]);
    console.log("  at the 2,000-entity target this is well under 0.1 ms either way");
    void gpu;
    void world;
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
