import RAPIER, {init} from "@alexandernanberg/rapier2d/compat";
import {describe, test, expect, beforeAll} from "vitest";

beforeAll(async () => {
    await init();
});

describe("RigidBody", () => {
    test("creates dynamic body", () => {
        const world = new RAPIER.World({x: 0, y: -9.81});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        expect(body.isDynamic()).toBe(true);
        expect(body.isFixed()).toBe(false);
        expect(body.isKinematic()).toBe(false);
        world.free();
    });

    test("creates fixed body", () => {
        const world = new RAPIER.World({x: 0, y: -9.81});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        expect(body.isFixed()).toBe(true);
        expect(body.isDynamic()).toBe(false);
        world.free();
    });

    test("creates kinematic position-based body", () => {
        const world = new RAPIER.World({x: 0, y: -9.81});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.kinematicPositionBased());
        expect(body.isKinematic()).toBe(true);
        world.free();
    });

    test("creates kinematic velocity-based body", () => {
        const world = new RAPIER.World({x: 0, y: -9.81});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.kinematicVelocityBased());
        expect(body.isKinematic()).toBe(true);
        world.free();
    });

    test("translation returns correct initial values", () => {
        const world = new RAPIER.World({x: 0, y: -9.81});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(1, 2));

        const pos = body.translation();
        expect(pos.x).toBe(1);
        expect(pos.y).toBe(2);

        world.free();
    });

    test("rotation returns zero by default", () => {
        const world = new RAPIER.World({x: 0, y: -9.81});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());

        // 2D rotation is a scalar (angle in radians)
        const rot = body.rotation();
        expect(rot).toBe(0);

        world.free();
    });

    test("setTranslation updates position", () => {
        const world = new RAPIER.World({x: 0, y: -9.81});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());

        body.setTranslation({x: 5, y: 10}, true);
        const pos = body.translation();
        expect(pos.x).toBe(5);
        expect(pos.y).toBe(10);

        world.free();
    });

    test("setRotation updates orientation", () => {
        const world = new RAPIER.World({x: 0, y: -9.81});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());

        // 2D: rotation is a single angle in radians
        body.setRotation(Math.PI / 4, true);
        const rot = body.rotation();
        expect(rot).toBeCloseTo(Math.PI / 4, 4);

        world.free();
    });

    test("setLinvel and linvel work correctly", () => {
        const world = new RAPIER.World({x: 0, y: 0}); // No gravity
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 0));
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        body.setLinvel({x: 1, y: 0}, true);

        const vel = body.linvel();
        expect(vel.x).toBe(1);
        expect(vel.y).toBe(0);

        world.step();
        expect(body.translation().x).toBeGreaterThan(0);

        world.free();
    });

    test("setAngvel and angvel work correctly", () => {
        const world = new RAPIER.World({x: 0, y: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        // 2D: angular velocity is a scalar
        body.setAngvel(2, true);
        const angvel = body.angvel();
        expect(angvel).toBeCloseTo(2, 4);

        world.free();
    });

    test("zero-allocation translation with target parameter", () => {
        const world = new RAPIER.World({x: 0, y: -9.81});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(3, 7));
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        world.step();

        const target = {x: 0, y: 0};
        const result = body.translation(target);

        expect(result).toBe(target);
        expect(target.x).toBeCloseTo(3, 1);
        expect(target.y).toBeLessThan(7);

        world.free();
    });

    test("zero-allocation linvel with target parameter", () => {
        const world = new RAPIER.World({x: 0, y: -9.81});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        world.step();

        const target = {x: 0, y: 0};
        const result = body.linvel(target);

        expect(result).toBe(target);
        expect(target.y).toBeLessThan(0);

        world.free();
    });

    test("translation reads from WASM when buffer not yet populated", () => {
        const world = new RAPIER.World({x: 0, y: -9.81});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(5, 10));

        const pos = body.translation();
        expect(pos.x).toBe(5);
        expect(pos.y).toBe(10);

        world.free();
    });

    test("body removal via removeRigidBody", () => {
        const world = new RAPIER.World({x: 0, y: -9.81});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        world.removeRigidBody(body);
        world.step();

        world.free();
    });

    test("kinematic position-based body moves to target", () => {
        const world = new RAPIER.World({x: 0, y: -9.81});
        const body = world.createRigidBody(
            RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(0, 0),
        );
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        body.setNextKinematicTranslation({x: 5, y: 10});
        world.step();

        const pos = body.translation();
        expect(pos.x).toBeCloseTo(5);
        expect(pos.y).toBeCloseTo(10);

        world.free();
    });

    test("kinematic body is not affected by gravity", () => {
        const world = new RAPIER.World({x: 0, y: -9.81});
        const body = world.createRigidBody(
            RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(0, 5),
        );
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        world.step();
        expect(body.translation().y).toBeCloseTo(5);

        world.free();
    });
    test("setNextKinematicRotation works with scalar angle", () => {
        const world = new RAPIER.World({x: 0, y: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.kinematicPositionBased());
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        body.setNextKinematicRotation(Math.PI / 2);
        world.step();

        expect(body.rotation()).toBeCloseTo(Math.PI / 2, 3);

        world.free();
    });
});
