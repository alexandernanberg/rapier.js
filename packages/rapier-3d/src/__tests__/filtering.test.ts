import RAPIER, {init} from "@alexandernanberg/rapier3d/compat";
import {describe, test, expect, beforeAll} from "vitest";
import {_v} from "./_target";

const GRAVITY = {x: 0, y: -9.81, z: 0};

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
            RAPIER.ColliderDesc.cuboid(20, 0.5, 20).setCollisionGroups(GROUP_A),
            ground,
        );

        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 3, 0));
        world.createCollider(RAPIER.ColliderDesc.ball(0.5).setCollisionGroups(GROUP_B), body);

        for (let i = 0; i < 60; i++) world.step();

        expect(body.translation(_v()).y).toBeLessThan(0);

        world.free();
    });

    test("colliders in the same group still collide", () => {
        const world = new RAPIER.World(GRAVITY);
        const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        world.createCollider(
            RAPIER.ColliderDesc.cuboid(20, 0.5, 20).setCollisionGroups(GROUP_A),
            ground,
        );

        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 3, 0));
        world.createCollider(RAPIER.ColliderDesc.ball(0.5).setCollisionGroups(GROUP_A), body);

        for (let i = 0; i < 60; i++) world.step();

        expect(body.translation(_v()).y).toBeGreaterThan(0);

        world.free();
    });
});

describe("query filters", () => {
    function ballWorld(groups?: number, dynamic = false) {
        const world = new RAPIER.World({x: 0, y: 0, z: 0});
        const desc = dynamic ? RAPIER.RigidBodyDesc.dynamic() : RAPIER.RigidBodyDesc.fixed();
        const body = world.createRigidBody(desc);
        let colliderDesc = RAPIER.ColliderDesc.ball(1);
        if (groups !== undefined) colliderDesc = colliderDesc.setCollisionGroups(groups);
        const collider = world.createCollider(colliderDesc, body);
        world.step();
        return {world, collider};
    }

    const ray = () => new RAPIER.Ray({x: 0, y: 10, z: 0}, {x: 0, y: -1, z: 0});

    test("filterGroups excludes a collider from a raycast", () => {
        const {world} = ballWorld(GROUP_B);

        expect(world.castRay(ray(), 100, true, new RAPIER.RayColliderHit())).not.toBeNull();
        expect(
            world.castRay(ray(), 100, true, new RAPIER.RayColliderHit(), undefined, GROUP_A),
        ).toBeNull();

        world.free();
    });

    test("filterExcludeCollider skips the named collider", () => {
        const {world, collider} = ballWorld();

        expect(world.castRay(ray(), 100, true, new RAPIER.RayColliderHit())).not.toBeNull();
        expect(
            world.castRay(
                ray(),
                100,
                true,
                new RAPIER.RayColliderHit(),
                undefined,
                undefined,
                collider,
            ),
        ).toBeNull();

        world.free();
    });

    test("filterPredicate can reject every candidate", () => {
        const {world} = ballWorld();

        expect(
            world.castRay(
                ray(),
                100,
                true,
                new RAPIER.RayColliderHit(),
                undefined,
                undefined,
                undefined,
                undefined,
                () => false,
            ),
        ).toBeNull();
        expect(
            world.castRay(
                ray(),
                100,
                true,
                new RAPIER.RayColliderHit(),
                undefined,
                undefined,
                undefined,
                undefined,
                () => true,
            ),
        ).not.toBeNull();

        world.free();
    });

    test("QueryFilterFlags.EXCLUDE_DYNAMIC skips dynamic colliders", () => {
        const {world} = ballWorld(undefined, true);

        expect(world.castRay(ray(), 100, true, new RAPIER.RayColliderHit())).not.toBeNull();
        expect(
            world.castRay(
                ray(),
                100,
                true,
                new RAPIER.RayColliderHit(),
                RAPIER.QueryFilterFlags.EXCLUDE_DYNAMIC,
            ),
        ).toBeNull();

        world.free();
    });
});
