import type {BenchResult} from "../runner.js";
import {bench} from "../runner.js";
import {createSparseWorld} from "../worlds/sparse.js";

const RAY_COUNT = 100;

function createRays(RAPIER: any, is3D: boolean, count: number) {
    const rays = [];
    for (let i = 0; i < count; i++) {
        // Random origins spread across the scene, casting downward with slight variation
        const x = (Math.random() - 0.5) * 80;
        const dirX = (Math.random() - 0.5) * 0.2;
        if (is3D) {
            const z = (Math.random() - 0.5) * 80;
            const dirZ = (Math.random() - 0.5) * 0.2;
            rays.push(new RAPIER.Ray({x, y: 50, z}, {x: dirX, y: -1, z: dirZ}));
        } else {
            rays.push(new RAPIER.Ray({x, y: 50}, {x: dirX, y: -1}));
        }
    }
    return rays;
}

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

    const rays = createRays(RAPIER, is3D, RAY_COUNT);

    // Cast 100 rays
    results.push(
        bench(
            `castRay x${RAY_COUNT} (${bodyCount} bodies)`,
            () => {
                for (let i = 0; i < RAY_COUNT; i++) {
                    world.castRay(rays[i], 100, true);
                }
            },
            opts,
        ),
    );

    // Ray cast with normal
    results.push(
        bench(
            `castRayAndGetNormal x${RAY_COUNT}`,
            () => {
                for (let i = 0; i < RAY_COUNT; i++) {
                    world.castRayAndGetNormal(rays[i], 100, true);
                }
            },
            opts,
        ),
    );

    // All intersections with ray
    let _hitCount = 0;
    results.push(
        bench(
            `intersectionsWithRay x${RAY_COUNT}`,
            () => {
                for (let i = 0; i < RAY_COUNT; i++) {
                    world.intersectionsWithRay(rays[i], 100, true, () => {
                        _hitCount++;
                        return true;
                    });
                }
            },
            opts,
        ),
    );

    // Point projection
    const points: any[] = [];
    for (let i = 0; i < RAY_COUNT; i++) {
        const x = (Math.random() - 0.5) * 80;
        if (is3D) {
            const z = (Math.random() - 0.5) * 80;
            points.push({x, y: 5, z});
        } else {
            points.push({x, y: 5});
        }
    }

    results.push(
        bench(
            `projectPoint x${RAY_COUNT}`,
            () => {
                for (let i = 0; i < RAY_COUNT; i++) {
                    world.projectPoint(points[i], true);
                }
            },
            opts,
        ),
    );

    // Intersections with point
    results.push(
        bench(
            `intersectionsWithPoint x${RAY_COUNT}`,
            () => {
                for (let i = 0; i < RAY_COUNT; i++) {
                    world.intersectionsWithPoint(points[i], () => true);
                }
            },
            opts,
        ),
    );

    world.free();

    return results;
}
