import type {MemoryBench} from "../memory.js";
import {createSparseWorld} from "../worlds/sparse.js";

/**
 * Allocation-focused workloads: the read paths a frame loop runs every tick,
 * each in its allocating and (where supported) its `target` form.
 *
 * `world.step()` is included as a canary — it should allocate nothing, since the
 * transform sync writes into a buffer shared with WASM.
 *
 * This fork has only the `target` form; the official packages have only the
 * allocating one. Each row is emitted for whichever API is under test, so a
 * fork run reports the `[reuse]` rows and `--official` the allocating ones.
 */
/**
 * Deterministic stand-in for `Math.random`, installed while the scene and the
 * queries are built. How much a query allocates depends on how often it hits, so
 * a scene that differs run to run makes the numbers wander by ~50% — far more
 * than any regression worth catching.
 */
function withSeededRandom<T>(seed: number, build: () => T): T {
    const original = Math.random;
    let state = seed;
    Math.random = () => {
        // xorshift32
        state ^= state << 13;
        state ^= state >>> 17;
        state ^= state << 5;
        return ((state >>> 0) % 1_000_000) / 1_000_000;
    };

    try {
        return build();
    } finally {
        Math.random = original;
    }
}

export function allocationBenches(RAPIER: any, is3D: boolean, quick: boolean): MemoryBench[] {
    const bodyCount = quick ? 1000 : 5000;
    const queryCount = quick ? 250 : 1000;

    // `any`: this file runs against both this fork (targets required) and the
    // official packages (allocating), whose signatures differ.
    const world: any = withSeededRandom(0x5eed, () => createSparseWorld(RAPIER, is3D, bodyCount));
    for (let i = 0; i < 60; i++) world.step();

    // A separate, smaller world for the stepping canary: allocation per step
    // barely depends on body count, and a cheap step means the measurement window
    // can hold enough of them to rise above the noise floor. Sleeping is off, so
    // this measures a step that actually simulates rather than one that skips.
    const stepBodyCount = 100;
    const stepWorld = withSeededRandom(0xb0d1, () =>
        createSparseWorld(RAPIER, is3D, stepBodyCount, {canSleep: false}),
    );
    for (let i = 0; i < 60; i++) stepWorld.step();

    const bodies: any[] = [];
    world.bodies.forEach((body: any) => bodies.push(body));

    // 2D vectors are `{x, y}`; the `z` argument is ignored there.
    const vec = (x: number, y: number, z: number): any => (is3D ? {x, y, z} : {x, y});

    const rays: any[] = [];
    const points: any[] = [];
    withSeededRandom(0x7a45, () => {
        for (let i = 0; i < queryCount; i++) {
            const x = (Math.random() - 0.5) * 80;
            const z = (Math.random() - 0.5) * 80;
            rays.push(new RAPIER.Ray(vec(x, 50, z), vec(0, -1, 0)));
            points.push(vec(x, 5, z));
        }
    });

    // Results have to escape the benchmark, or V8's escape analysis scalar-
    // replaces them and the allocation being measured never happens. A small
    // rotating sink is enough to defeat it without retaining the whole run.
    const sink: any[] = new Array(16).fill(null);
    let sinkIndex = 0;
    const keep = (value: any) => {
        sink[sinkIndex++ & 15] = value;
    };

    const shape = new RAPIER.Ball(0.5);
    // 2D takes a scalar angle where 3D takes a quaternion.
    const identity: any = is3D ? {x: 0, y: 0, z: 0, w: 1} : 0;
    const down = vec(0, -1, 0);

    const benches: MemoryBench[] = [
        {
            name: `world.step() (${stepBodyCount} bodies)`,
            opsPerCall: 1,
            maxCalls: quick ? 500 : 2_000,
            fn: () => stepWorld.step(),
        },
    ];

    // This fork's result types are zero-arg constructible and pre-fill their
    // vectors; the official ones leave them undefined.
    const targetApi = (() => {
        try {
            return new RAPIER.PointProjection().point !== undefined;
        } catch {
            return false;
        }
    })();

    // --- body.translation() -------------------------------------------------
    if (targetApi) {
        const translationTarget = vec(0, 0, 0);
        benches.push({
            name: `body.translation() x${bodies.length} [reuse]`,
            opsPerCall: bodies.length,
            fn: () => {
                for (const b of bodies) keep(b.translation(translationTarget));
            },
        });
    } else {
        benches.push({
            name: `body.translation() x${bodies.length}`,
            opsPerCall: bodies.length,
            fn: () => {
                for (const b of bodies) keep(b.translation());
            },
        });
    }

    // --- castRay ------------------------------------------------------------
    if (targetApi) {
        const rayHit = new RAPIER.RayColliderHit();
        benches.push({
            name: `castRay x${queryCount} [reuse]`,
            opsPerCall: queryCount,
            fn: () => {
                for (let i = 0; i < queryCount; i++)
                    keep(world.castRay(rays[i], 100, true, rayHit));
            },
        });
    } else {
        benches.push({
            name: `castRay x${queryCount}`,
            opsPerCall: queryCount,
            fn: () => {
                for (let i = 0; i < queryCount; i++) keep(world.castRay(rays[i], 100, true));
            },
        });
    }

    // --- castRayAndGetNormal ------------------------------------------------
    if (targetApi) {
        const hitTarget = new RAPIER.RayColliderIntersection();
        benches.push({
            name: `castRayAndGetNormal x${queryCount} [reuse]`,
            opsPerCall: queryCount,
            fn: () => {
                for (let i = 0; i < queryCount; i++)
                    keep(world.castRayAndGetNormal(rays[i], 100, true, hitTarget));
            },
        });
    } else {
        benches.push({
            name: `castRayAndGetNormal x${queryCount}`,
            opsPerCall: queryCount,
            fn: () => {
                for (let i = 0; i < queryCount; i++)
                    keep(world.castRayAndGetNormal(rays[i], 100, true));
            },
        });
    }

    // --- projectPoint -------------------------------------------------------
    if (targetApi) {
        const projTarget = new RAPIER.PointColliderProjection();
        benches.push({
            name: `projectPoint x${queryCount} [reuse]`,
            opsPerCall: queryCount,
            fn: () => {
                for (let i = 0; i < queryCount; i++)
                    keep(world.projectPoint(points[i], true, projTarget));
            },
        });
    } else {
        benches.push({
            name: `projectPoint x${queryCount}`,
            opsPerCall: queryCount,
            fn: () => {
                for (let i = 0; i < queryCount; i++) keep(world.projectPoint(points[i], true));
            },
        });
    }

    // --- castShape ----------------------------------------------------------
    if (targetApi) {
        const castTarget = new RAPIER.ColliderShapeCastHit();
        benches.push({
            name: `castShape x${queryCount} [reuse]`,
            opsPerCall: queryCount,
            fn: () => {
                for (let i = 0; i < queryCount; i++)
                    keep(
                        world.castShape(points[i], identity, down, shape, 0, 100, true, castTarget),
                    );
            },
        });
    } else {
        benches.push({
            name: `castShape x${queryCount}`,
            opsPerCall: queryCount,
            fn: () => {
                for (let i = 0; i < queryCount; i++)
                    keep(world.castShape(points[i], identity, down, shape, 0, 100, true));
            },
        });
    }

    return benches;
}
