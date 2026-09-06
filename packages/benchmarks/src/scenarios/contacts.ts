import {bench, summary} from "mitata";
import {createPyramidWorld} from "../worlds/pyramid.js";

/**
 * Reading contact manifolds back out of the narrow phase: the per-pair walk a
 * game loop runs for footstep sounds, decals or damage, over every touching pair
 * of a settled pyramid.
 *
 * Both packages expose the same `contactPair` shape, so the official build runs
 * the allocating rows for comparison; the `[reuse]` rows only exist on this fork.
 */
export function benchContacts(RAPIER: any, is3D: boolean, quick: boolean): void {
    const bodyCount = quick ? 250 : 1000;
    const world = createPyramidWorld(RAPIER, is3D, bodyCount);
    for (let i = 0; i < 120; i++) world.step();

    // Every touching pair once, in a fixed order.
    const pairs: [any, any][] = [];
    world.forEachCollider((collider: any) => {
        world.contactPairsWith(collider, (other: any) => {
            if (collider.handle < other.handle) pairs.push([collider, other]);
        });
    });

    const vec = (): any => (is3D ? {x: 0, y: 0, z: 0} : {x: 0, y: 0});
    let probe: any = null;
    const target = vec();
    world.contactPair(pairs[0][0], pairs[0][1], (manifold: any) => {
        probe = manifold.normal(target);
    });
    // The official package ignores the argument and allocates; only a returned
    // `target` proves the zero-allocation path is real.
    const supportsTargetParam = probe === target;

    // `sink` keeps the reads observable so they are not optimized away. The
    // callbacks are hoisted so the per-pair cost is the API's, not a closure's.
    let sink = 0;

    const readAlloc = (manifold: any) => {
        sink += manifold.normal().y;
        const n = manifold.numContacts();
        for (let i = 0; i < n; i++) {
            sink += manifold.localContactPoint1(i).x + manifold.contactDist(i);
        }
    };
    const normal = vec();
    const point = vec();
    const readInto = (manifold: any) => {
        sink += manifold.normal(normal).y;
        const n = manifold.numContacts();
        for (let i = 0; i < n; i++) {
            sink += manifold.localContactPoint1(i, point).x + manifold.contactDist(i);
        }
    };
    const readSolver = (manifold: any) => {
        const n = manifold.numSolverContacts();
        for (let i = 0; i < n; i++) {
            sink += manifold.solverContactDist(i) + manifold.solverContactPoint(i).y;
        }
    };

    summary(() => {
        bench(`world.contactPair() normal+points (${bodyCount}-body pyramid) [alloc]`, () => {
            for (const [a, b] of pairs) world.contactPair(a, b, readAlloc);
        });

        if (supportsTargetParam) {
            bench(`world.contactPair() normal+points (${bodyCount}-body pyramid) [reuse]`, () => {
                for (const [a, b] of pairs) world.contactPair(a, b, readInto);
            });
        }
    });

    summary(() => {
        bench(`world.contactPair() solver contacts (${bodyCount}-body pyramid)`, () => {
            for (const [a, b] of pairs) world.contactPair(a, b, readSolver);
        });
    });

    // Referenced so the accumulator cannot be dropped as dead.
    if (sink === Number.NaN) console.log(sink);
}
