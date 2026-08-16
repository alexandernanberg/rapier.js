import RAPIER, {init} from "@alexandernanberg/rapier3d/compat";
import {describe, test, expect, beforeAll} from "vitest";

const GRAVITY = {x: 0, y: -9.81, z: 0};
const IDENTITY = {x: 0, y: 0, z: 0, w: 1};

beforeAll(async () => {
    await init();
});

/**
 * Joints cross the WASM boundary through `JointData`, which builds raw anchors
 * that have to be freed after the joint is created. These check both that the
 * constraint actually holds and that the handle bookkeeping on the JS side
 * survives creation and removal.
 */
describe("impulse joints", () => {
    test("a fixed joint holds a dynamic body against gravity", () => {
        const world = new RAPIER.World(GRAVITY);
        const anchor = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(0, 5, 0));
        const hung = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(1, 5, 0));
        world.createCollider(RAPIER.ColliderDesc.ball(0.2), anchor);
        world.createCollider(RAPIER.ColliderDesc.ball(0.2), hung);

        world.createImpulseJoint(
            RAPIER.JointData.fixed({x: 1, y: 0, z: 0}, IDENTITY, {x: 0, y: 0, z: 0}, IDENTITY),
            anchor,
            hung,
            true,
        );

        for (let i = 0; i < 60; i++) world.step();

        // A full second of free fall would put it near y = -0.9.
        expect(hung.translation().y).toBeGreaterThan(4.5);

        world.free();
    });

    test("a revolute joint keeps the bob at the anchor distance", () => {
        const world = new RAPIER.World(GRAVITY);
        const anchor = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(0, 5, 0));
        const bob = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(2, 5, 0));
        world.createCollider(RAPIER.ColliderDesc.ball(0.2), bob);

        world.createImpulseJoint(
            RAPIER.JointData.revolute({x: 0, y: 0, z: 0}, {x: -2, y: 0, z: 0}, {x: 0, y: 0, z: 1}),
            anchor,
            bob,
            true,
        );

        for (let i = 0; i < 120; i++) world.step();

        const t = bob.translation();
        expect(Math.hypot(t.x, t.y - 5, t.z)).toBeCloseTo(2, 1);

        world.free();
    });

    test("a spherical joint keeps the anchor distance", () => {
        const world = new RAPIER.World(GRAVITY);
        const anchor = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(0, 5, 0));
        const bob = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(1.5, 5, 0));
        world.createCollider(RAPIER.ColliderDesc.ball(0.2), bob);

        world.createImpulseJoint(
            RAPIER.JointData.spherical({x: 0, y: 0, z: 0}, {x: -1.5, y: 0, z: 0}),
            anchor,
            bob,
            true,
        );

        for (let i = 0; i < 120; i++) world.step();

        const t = bob.translation();
        expect(Math.hypot(t.x, t.y - 5, t.z)).toBeCloseTo(1.5, 1);

        world.free();
    });

    test("an impulse joint is reachable by handle until it is removed", () => {
        const world = new RAPIER.World(GRAVITY);
        const a = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        const b = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(1, 0, 0));

        const joint = world.createImpulseJoint(
            RAPIER.JointData.fixed({x: 1, y: 0, z: 0}, IDENTITY, {x: 0, y: 0, z: 0}, IDENTITY),
            a,
            b,
            true,
        );
        expect(world.getImpulseJoint(joint.handle)).toBe(joint);

        world.removeImpulseJoint(joint, true);
        expect(world.getImpulseJoint(joint.handle)).toBeNull();

        world.free();
    });
});

describe("multibody joints", () => {
    test("a multibody joint can be created, stepped and removed", () => {
        const world = new RAPIER.World(GRAVITY);
        const a = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        const b = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(1, 0, 0));
        world.createCollider(RAPIER.ColliderDesc.ball(0.2), b);

        const joint = world.createMultibodyJoint(
            RAPIER.JointData.revolute({x: 0, y: 0, z: 0}, {x: -1, y: 0, z: 0}, {x: 0, y: 0, z: 1}),
            a,
            b,
            true,
        );
        expect(world.getMultibodyJoint(joint.handle)).toBe(joint);

        for (let i = 0; i < 30; i++) world.step();
        const t = b.translation();
        expect(Math.hypot(t.x, t.y, t.z)).toBeCloseTo(1, 1);

        world.removeMultibodyJoint(joint, true);
        expect(world.getMultibodyJoint(joint.handle)).toBeNull();

        world.free();
    });

    test("a rejected multibody topology throws instead of trapping", () => {
        // Rapier refuses an insert that would give `parent2` a second parent joint,
        // or close a loop inside one multibody. The rejection used to come back as a
        // sentinel handle that `newTyped` immediately looked up, panicking inside
        // WASM — which takes the whole module down.
        const world = new RAPIER.World(GRAVITY);
        const a = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        const b = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(1, 0, 0));
        const c = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(2, 0, 0));

        const data = () =>
            RAPIER.JointData.revolute({x: 0, y: 0, z: 0}, {x: -1, y: 0, z: 0}, {x: 0, y: 0, z: 1});

        world.createMultibodyJoint(data(), a, b, true);

        // `b` already has a parent joint.
        expect(() => world.createMultibodyJoint(data(), c, b, true)).toThrow();
        // `a` and `b` are already in the same multibody, so this would close a loop.
        expect(() => world.createMultibodyJoint(data(), b, a, true)).toThrow();

        // The world still steps afterwards, which a trap would have made impossible.
        world.step();
        world.free();
    });
});
