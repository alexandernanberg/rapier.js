import type {BenchResult} from "../runner.js";
import {bench} from "../runner.js";
import {createSparseWorld} from "../worlds/sparse.js";

export async function benchQueries(
    RAPIER: any,
    is3D: boolean,
    quick: boolean,
): Promise<BenchResult[]> {
    const results: BenchResult[] = [];
    const bodyCount = quick ? 500 : 2000;
    const opts = quick ? {warmup: 50, iterations: 200} : {warmup: 100, iterations: 1000};

    const world = createSparseWorld(RAPIER, is3D, bodyCount);

    // Let bodies settle
    for (let i = 0; i < 60; i++) world.step();

    const origin = is3D ? {x: 0, y: 50, z: 0} : {x: 0, y: 50};
    const dir = is3D ? {x: 0, y: -1, z: 0} : {x: 0, y: -1};
    const ray = new RAPIER.Ray(origin, dir);

    // Single ray cast
    results.push(
        bench(
            `castRay (${bodyCount} bodies)`,
            () => {
                world.castRay(ray, 100, true);
            },
            opts,
        ),
    );

    // Ray cast with normal
    results.push(
        bench(
            `castRayAndGetNormal`,
            () => {
                world.castRayAndGetNormal(ray, 100, true);
            },
            opts,
        ),
    );

    // All intersections with ray
    let hitCount = 0;
    results.push(
        bench(
            `intersectionsWithRay`,
            () => {
                world.intersectionsWithRay(ray, 100, true, () => {
                    hitCount++;
                    return true;
                });
            },
            opts,
        ),
    );

    // Point projection
    const point: any = is3D ? {x: 0, y: 5, z: 0} : {x: 0, y: 5};
    results.push(
        bench(
            `projectPoint`,
            () => {
                world.projectPoint(point, true);
            },
            opts,
        ),
    );

    // Intersections with point
    results.push(
        bench(
            `intersectionsWithPoint`,
            () => {
                world.intersectionsWithPoint(point, () => true);
            },
            opts,
        ),
    );

    world.free();

    return results;
}
