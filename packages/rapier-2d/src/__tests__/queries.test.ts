import RAPIER, {init} from "@alexandernanberg/rapier2d/compat";
import {describe, test, expect, beforeAll} from "vitest";

beforeAll(async () => {
    await init();
});

function worldWithBall(y = 0, radius = 1) {
    const world = new RAPIER.World({x: 0, y: 0});
    const body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(0, y));
    const collider = world.createCollider(RAPIER.ColliderDesc.ball(radius), body);
    world.step();
    return {world, body, collider};
}

/**
 * Scene queries hand their results back through a scratch buffer shared with
 * WASM, so these check that every field makes it across intact.
 */
describe("scene queries", () => {
    test("castRay reports the collider and the time of impact", () => {
        const {world, collider} = worldWithBall();

        const ray = new RAPIER.Ray({x: 0, y: 10}, {x: 0, y: -1});
        const hit = world.castRay(ray, 100, true);

        expect(hit).not.toBeNull();
        expect(hit!.collider.handle).toBe(collider.handle);
        // The ball has a radius of 1 and sits at the origin.
        expect(hit!.timeOfImpact).toBeCloseTo(9, 4);

        world.free();
    });

    test("castRay misses when nothing is in the way", () => {
        const {world} = worldWithBall();

        const ray = new RAPIER.Ray({x: 50, y: 10}, {x: 0, y: -1});
        expect(world.castRay(ray, 100, true)).toBeNull();

        world.free();
    });

    test("castRayAndGetNormal also reports the normal at the hit point", () => {
        const {world, collider} = worldWithBall();

        const ray = new RAPIER.Ray({x: 0, y: 10}, {x: 0, y: -1});
        const hit = world.castRayAndGetNormal(ray, 100, true);

        expect(hit).not.toBeNull();
        expect(hit!.collider.handle).toBe(collider.handle);
        expect(hit!.timeOfImpact).toBeCloseTo(9, 4);
        expect(hit!.normal.x).toBeCloseTo(0, 4);
        expect(hit!.normal.y).toBeCloseTo(1, 4);

        world.free();
    });

    test("intersectionsWithRay reports every collider along the ray", () => {
        const world = new RAPIER.World({x: 0, y: 0});
        const handles = [];
        for (const y of [0, 5, 10]) {
            const body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(0, y));
            handles.push(world.createCollider(RAPIER.ColliderDesc.ball(1), body).handle);
        }
        world.step();

        const ray = new RAPIER.Ray({x: 0, y: 20}, {x: 0, y: -1});
        const hits: {handle: number; timeOfImpact: number; normalY: number}[] = [];
        world.intersectionsWithRay(ray, 100, true, (inter) => {
            hits.push({
                handle: inter.collider.handle,
                timeOfImpact: inter.timeOfImpact,
                normalY: inter.normal.y,
            });
            return true;
        });

        expect(hits.map((h) => h.handle).sort((a, b) => a - b)).toEqual(
            handles.sort((a, b) => a - b),
        );
        for (const hit of hits) {
            expect(hit.timeOfImpact).toBeGreaterThan(0);
            expect(hit.normalY).toBeCloseTo(1, 4);
        }

        world.free();
    });

    test("projectPoint reports the projection and whether the point is inside", () => {
        const {world, collider} = worldWithBall();

        const outside = world.projectPoint({x: 0, y: 10}, true);
        expect(outside).not.toBeNull();
        expect(outside!.collider.handle).toBe(collider.handle);
        expect(outside!.isInside).toBe(false);
        expect(outside!.point.x).toBeCloseTo(0, 4);
        expect(outside!.point.y).toBeCloseTo(1, 4);

        const inside = world.projectPoint({x: 0, y: 0.25}, true);
        expect(inside!.isInside).toBe(true);

        world.free();
    });

    test("projectPointAndGetFeature reports the projected-on feature", () => {
        const world = new RAPIER.World({x: 0, y: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        const collider = world.createCollider(RAPIER.ColliderDesc.cuboid(1, 1), body);
        world.step();

        const proj = world.projectPointAndGetFeature({x: 0, y: 10});
        expect(proj).not.toBeNull();
        expect(proj!.collider.handle).toBe(collider.handle);
        expect(proj!.point.y).toBeCloseTo(1, 4);
        expect(proj!.featureId).toBeTypeOf("number");

        world.free();
    });

    test("intersectionsWithPoint reports the colliders containing the point", () => {
        const {world, collider} = worldWithBall();

        const hits: number[] = [];
        world.intersectionsWithPoint({x: 0, y: 0}, (found) => {
            hits.push(found.handle);
            return true;
        });
        expect(hits).toEqual([collider.handle]);

        hits.length = 0;
        world.intersectionsWithPoint({x: 20, y: 0}, (found) => {
            hits.push(found.handle);
            return true;
        });
        expect(hits).toEqual([]);

        world.free();
    });

    test("a query can be issued from inside a query callback", () => {
        const {world, collider} = worldWithBall();

        const ray = new RAPIER.Ray({x: 0, y: 10}, {x: 0, y: -1});
        const nested: (number | null)[] = [];
        world.intersectionsWithRay(ray, 100, true, (inter) => {
            const hit = world.castRay(ray, 100, true);
            nested.push(hit ? hit.collider.handle : null);
            // The outer hit must not be clobbered by the nested query.
            expect(inter.collider.handle).toBe(collider.handle);
            return true;
        });

        expect(nested).toEqual([collider.handle]);

        world.free();
    });

    test("queries keep working after WASM memory growth", () => {
        const {world} = worldWithBall();
        const ray = new RAPIER.Ray({x: 0, y: 10}, {x: 0, y: -1});

        const before = world.castRayAndGetNormal(ray, 100, true)!;

        // Creating bodies grows WASM memory, which detaches the JS view over the
        // query result buffer; the next query must re-create it.
        for (let i = 0; i < 2000; i++) {
            world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(100, i));
        }

        const after = world.castRayAndGetNormal(ray, 100, true)!;
        expect(after.collider.handle).toBe(before.collider.handle);
        expect(after.timeOfImpact).toBeCloseTo(before.timeOfImpact, 6);
        expect(after.normal.y).toBeCloseTo(before.normal.y, 6);

        world.free();
    });
});
