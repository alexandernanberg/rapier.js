import {bench, summary} from "mitata";
import {createSparseWorld} from "../worlds/sparse.js";

function createRays(RAPIER: any, is3D: boolean, count: number) {
    const rays = [];
    for (let i = 0; i < count; i++) {
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

export function benchQueries(RAPIER: any, is3D: boolean, quick: boolean): void {
    const bodyCount = quick ? 1000 : 5000;
    // Enough casts that one iteration is hundreds of microseconds of real query
    // work, rather than a handful of casts wrapped in loop and timer overhead.
    const RAY_COUNT = quick ? 250 : 1000;

    // `any`: this file runs against both this fork (targets required) and the
    // official packages (allocating), whose signatures differ.
    const world: any = createSparseWorld(RAPIER, is3D, bodyCount);

    // This fork's result types are zero-arg constructible and pre-fill their
    // vectors; the official ones leave them undefined.
    const targetApi = (() => {
        try {
            return new RAPIER.PointProjection().point !== undefined;
        } catch {
            return false;
        }
    })();

    // Let bodies settle
    for (let i = 0; i < 60; i++) world.step();

    const rays = createRays(RAPIER, is3D, RAY_COUNT);

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

    const rayHit = targetApi ? new RAPIER.RayColliderHit() : undefined;
    const rayNormalHit = targetApi ? new RAPIER.RayColliderIntersection() : undefined;
    const pointProj = targetApi ? new RAPIER.PointColliderProjection() : undefined;

    summary(() => {
        bench(`castRay x${RAY_COUNT} (${bodyCount} bodies)`, () => {
            for (let i = 0; i < RAY_COUNT; i++) {
                targetApi
                    ? world.castRay(rays[i], 100, true, rayHit)
                    : world.castRay(rays[i], 100, true);
            }
        });

        bench(`castRayAndGetNormal x${RAY_COUNT}`, () => {
            for (let i = 0; i < RAY_COUNT; i++) {
                targetApi
                    ? world.castRayAndGetNormal(rays[i], 100, true, rayNormalHit)
                    : world.castRayAndGetNormal(rays[i], 100, true);
            }
        });

        bench(`intersectionsWithRay x${RAY_COUNT}`, () => {
            for (let i = 0; i < RAY_COUNT; i++) {
                targetApi
                    ? world.intersectionsWithRay(rays[i], 100, true, () => true, rayNormalHit)
                    : world.intersectionsWithRay(rays[i], 100, true, () => true);
            }
        });

        bench(`projectPoint x${RAY_COUNT}`, () => {
            for (let i = 0; i < RAY_COUNT; i++) {
                targetApi
                    ? world.projectPoint(points[i], true, pointProj)
                    : world.projectPoint(points[i], true);
            }
        });

        bench(`intersectionsWithPoint x${RAY_COUNT}`, () => {
            for (let i = 0; i < RAY_COUNT; i++) {
                world.intersectionsWithPoint(points[i], () => true);
            }
        });
    });
}
