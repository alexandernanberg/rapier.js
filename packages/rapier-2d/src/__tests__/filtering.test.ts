import RAPIER, {init} from "@alexandernanberg/rapier2d/compat";
import {describe, test, expect, beforeAll} from "vitest";

const GRAVITY = {x: 0, y: -9.81};

// Membership in the high 16 bits, the mask of groups to interact with in the
// low 16 bits.
const GROUP_A = 0x0001_0001;
const GROUP_B = 0x0002_0002;

beforeAll(async () => {
    await init();
});

describe("collision groups", () => {
    test("colliders in non-overlapping groups pass through each other", () => {
        const world = new RAPIER.World(GRAVITY);
        const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        world.createCollider(
            RAPIER.ColliderDesc.cuboid(20, 0.5).setCollisionGroups(GROUP_A),
            ground,
        );

        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 3));
        world.createCollider(RAPIER.ColliderDesc.ball(0.5).setCollisionGroups(GROUP_B), body);

        for (let i = 0; i < 60; i++) world.step();

        expect(body.translation().y).toBeLessThan(0);

        world.free();
    });

    test("colliders in the same group still collide", () => {
        const world = new RAPIER.World(GRAVITY);
        const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        world.createCollider(
            RAPIER.ColliderDesc.cuboid(20, 0.5).setCollisionGroups(GROUP_A),
            ground,
        );

        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 3));
        world.createCollider(RAPIER.ColliderDesc.ball(0.5).setCollisionGroups(GROUP_A), body);

        for (let i = 0; i < 60; i++) world.step();

        expect(body.translation().y).toBeGreaterThan(0);

        world.free();
    });
});

describe("query filters", () => {
    test("filterGroups excludes a collider from a raycast", () => {
        const world = new RAPIER.World({x: 0, y: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        world.createCollider(RAPIER.ColliderDesc.ball(1).setCollisionGroups(GROUP_B), body);
        world.step();

        const ray = new RAPIER.Ray({x: 0, y: 10}, {x: 0, y: -1});

        expect(world.castRay(ray, 100, true)).not.toBeNull();
        expect(world.castRay(ray, 100, true, undefined, GROUP_A)).toBeNull();

        world.free();
    });

    test("filterExcludeCollider skips the named collider", () => {
        const world = new RAPIER.World({x: 0, y: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        const collider = world.createCollider(RAPIER.ColliderDesc.ball(1), body);
        world.step();

        const ray = new RAPIER.Ray({x: 0, y: 10}, {x: 0, y: -1});

        expect(world.castRay(ray, 100, true)).not.toBeNull();
        expect(world.castRay(ray, 100, true, undefined, undefined, collider)).toBeNull();

        world.free();
    });

    test("filterPredicate can reject every candidate", () => {
        const world = new RAPIER.World({x: 0, y: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        world.createCollider(RAPIER.ColliderDesc.ball(1), body);
        world.step();

        const ray = new RAPIER.Ray({x: 0, y: 10}, {x: 0, y: -1});

        expect(
            world.castRay(ray, 100, true, undefined, undefined, undefined, undefined, () => false),
        ).toBeNull();
        expect(
            world.castRay(ray, 100, true, undefined, undefined, undefined, undefined, () => true),
        ).not.toBeNull();

        world.free();
    });

    test("QueryFilterFlags.EXCLUDE_DYNAMIC skips dynamic colliders", () => {
        const world = new RAPIER.World({x: 0, y: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        world.createCollider(RAPIER.ColliderDesc.ball(1), body);
        world.step();

        const ray = new RAPIER.Ray({x: 0, y: 10}, {x: 0, y: -1});

        expect(world.castRay(ray, 100, true)).not.toBeNull();
        expect(world.castRay(ray, 100, true, RAPIER.QueryFilterFlags.EXCLUDE_DYNAMIC)).toBeNull();

        world.free();
    });
});
