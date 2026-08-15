import RAPIER, {init} from "@alexandernanberg/rapier2d/compat";
import {describe, test, expect, beforeAll} from "vitest";

const GRAVITY = {x: 0, y: -9.81};

beforeAll(async () => {
    await init();
});

/**
 * Every shape family builds its raw counterpart differently — several take
 * typed arrays that have to be copied into WASM memory. The suite otherwise
 * only ever creates balls, cuboids and capsules.
 */
describe("shape families", () => {
    test("a convex hull collider round-trips through the shape wrapper", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());

        const desc = RAPIER.ColliderDesc.convexHull(new Float32Array([0, 0, 1, 0, 1, 1, 0, 1]));
        expect(desc).not.toBeNull();

        const collider = world.createCollider(desc!, body);
        expect(collider.shape).toBeInstanceOf(RAPIER.ConvexPolygon);
        world.step();

        world.free();
    });

    test("a trimesh collider round-trips through the shape wrapper", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());

        const collider = world.createCollider(
            RAPIER.ColliderDesc.trimesh(
                new Float32Array([0, 0, 1, 0, 1, 1]),
                new Uint32Array([0, 1, 2]),
            ),
            body,
        );
        expect(collider.shape).toBeInstanceOf(RAPIER.TriMesh);
        world.step();

        world.free();
    });

    test("a heightfield collider round-trips through the shape wrapper", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());

        const collider = world.createCollider(
            RAPIER.ColliderDesc.heightfield(new Float32Array([0, 1, 0]), {x: 10, y: 1}),
            body,
        );
        expect(collider.shape).toBeInstanceOf(RAPIER.Heightfield);
        world.step();

        world.free();
    });

    test("a polyline collider round-trips through the shape wrapper", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());

        const collider = world.createCollider(
            RAPIER.ColliderDesc.polyline(new Float32Array([0, 0, 1, 0, 2, 0])),
            body,
        );
        expect(collider.shape).toBeInstanceOf(RAPIER.Polyline);
        world.step();

        world.free();
    });

    test("a ball lands on a trimesh floor", () => {
        const world = new RAPIER.World(GRAVITY);
        const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        world.createCollider(
            RAPIER.ColliderDesc.trimesh(
                new Float32Array([-10, 0, 10, 0, 10, -1, -10, -1]),
                new Uint32Array([0, 1, 2, 0, 2, 3]),
            ),
            ground,
        );

        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 3));
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        for (let i = 0; i < 120; i++) world.step();

        expect(body.translation().y).toBeGreaterThan(0);

        world.free();
    });
});
