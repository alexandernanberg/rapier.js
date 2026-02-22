import RAPIER, {init} from "@alexandernanberg/rapier3d/compat";
import {describe, test, expect, beforeAll} from "vitest";

beforeAll(async () => {
    await init();
});

describe("RigidBody", () => {
    test("creates dynamic body", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        expect(body.isDynamic()).toBe(true);
        expect(body.isFixed()).toBe(false);
        expect(body.isKinematic()).toBe(false);
        world.free();
    });

    test("creates fixed body", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        expect(body.isFixed()).toBe(true);
        expect(body.isDynamic()).toBe(false);
        world.free();
    });

    test("creates kinematic position-based body", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.kinematicPositionBased());
        expect(body.isKinematic()).toBe(true);
        world.free();
    });

    test("creates kinematic velocity-based body", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.kinematicVelocityBased());
        expect(body.isKinematic()).toBe(true);
        world.free();
    });

    test("translation returns correct initial values", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(1, 2, 3));

        const pos = body.translation();
        expect(pos.x).toBe(1);
        expect(pos.y).toBe(2);
        expect(pos.z).toBe(3);

        world.free();
    });

    test("rotation returns identity quaternion by default", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());

        const rot = body.rotation();
        expect(rot.x).toBe(0);
        expect(rot.y).toBe(0);
        expect(rot.z).toBe(0);
        expect(rot.w).toBe(1);

        world.free();
    });

    test("setTranslation updates position", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());

        body.setTranslation({x: 5, y: 10, z: 15}, true);
        const pos = body.translation();
        expect(pos.x).toBe(5);
        expect(pos.y).toBe(10);
        expect(pos.z).toBe(15);

        world.free();
    });

    test("setRotation updates orientation", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());

        // 90 degrees around Y axis
        const q = {x: 0, y: 0.7071068, z: 0, w: 0.7071068};
        body.setRotation(q, true);

        const rot = body.rotation();
        expect(rot.x).toBeCloseTo(q.x, 4);
        expect(rot.y).toBeCloseTo(q.y, 4);
        expect(rot.z).toBeCloseTo(q.z, 4);
        expect(rot.w).toBeCloseTo(q.w, 4);

        world.free();
    });

    test("setLinvel and linvel work correctly", () => {
        const world = new RAPIER.World({x: 0, y: 0, z: 0}); // No gravity
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 0, 0));
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        body.setLinvel({x: 1, y: 0, z: 0}, true);

        const vel = body.linvel();
        expect(vel.x).toBe(1);
        expect(vel.y).toBe(0);
        expect(vel.z).toBe(0);

        // After stepping, the body should have moved in X
        world.step();
        expect(body.translation().x).toBeGreaterThan(0);

        world.free();
    });

    test("setAngvel and angvel work correctly", () => {
        const world = new RAPIER.World({x: 0, y: 0, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        body.setAngvel({x: 0, y: 1, z: 0}, true);

        const angvel = body.angvel();
        expect(angvel.y).toBeCloseTo(1, 4);

        world.free();
    });

    test("zero-allocation translation with target parameter", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(3, 7, 11));
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        // Step to populate the transform buffer
        world.step();

        const target = {x: 0, y: 0, z: 0};
        const result = body.translation(target);

        // Result should be the same object
        expect(result).toBe(target);
        // Values should be populated
        expect(target.x).toBeCloseTo(3, 1);
        // Y should have changed due to gravity
        expect(target.y).toBeLessThan(7);

        world.free();
    });

    test("zero-allocation rotation with target parameter", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        world.step();

        const target = {x: 0, y: 0, z: 0, w: 0};
        const result = body.rotation(target);

        expect(result).toBe(target);
        // Identity quaternion (ball has no torque)
        expect(target.w).toBeCloseTo(1, 4);

        world.free();
    });

    test("zero-allocation linvel with target parameter", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        world.step();

        const target = {x: 0, y: 0, z: 0};
        const result = body.linvel(target);

        expect(result).toBe(target);
        // Should have negative Y velocity from gravity
        expect(target.y).toBeLessThan(0);

        world.free();
    });

    test("zero-allocation angvel with target parameter", () => {
        const world = new RAPIER.World({x: 0, y: 0, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        body.setAngvel({x: 0, y: 2, z: 0}, true);
        world.step();

        const target = {x: 0, y: 0, z: 0};
        const result = body.angvel(target);

        expect(result).toBe(target);
        expect(target.y).toBeCloseTo(2, 1);

        world.free();
    });

    test("transform buffer reads from buffer after step", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 10, 0));
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        // After step, buffer should be populated
        world.step();
        const pos1 = body.translation();
        expect(pos1.y).toBeLessThan(10);

        // After another step, position should change further
        world.step();
        const pos2 = body.translation();
        expect(pos2.y).toBeLessThan(pos1.y);

        world.free();
    });

    test("translation reads from WASM when buffer not yet populated", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(
            RAPIER.RigidBodyDesc.dynamic().setTranslation(5, 10, 15),
        );

        // Before any step, translation should still return correct values
        const pos = body.translation();
        expect(pos.x).toBe(5);
        expect(pos.y).toBe(10);
        expect(pos.z).toBe(15);

        world.free();
    });

    test("body removal via removeRigidBody", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        world.removeRigidBody(body);

        // World should still step fine after removal
        world.step();

        world.free();
    });
});
