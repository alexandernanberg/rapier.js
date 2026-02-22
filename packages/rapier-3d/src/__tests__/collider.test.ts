import RAPIER, {init} from "@alexandernanberg/rapier3d/compat";
import {describe, test, expect, beforeAll} from "vitest";

beforeAll(async () => {
    await init();
});

describe("Collider", () => {
    test("creates ball collider", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        const collider = world.createCollider(RAPIER.ColliderDesc.ball(1.5), body);

        const shape = collider.shape;
        expect(shape.type).toBe(RAPIER.ShapeType.Ball);
        expect((shape as RAPIER.Ball).radius).toBe(1.5);

        world.free();
    });

    test("creates cuboid collider", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        const collider = world.createCollider(RAPIER.ColliderDesc.cuboid(1, 2, 3), body);

        const shape = collider.shape;
        expect(shape.type).toBe(RAPIER.ShapeType.Cuboid);
        const cuboid = shape as RAPIER.Cuboid;
        expect(cuboid.halfExtents.x).toBe(1);
        expect(cuboid.halfExtents.y).toBe(2);
        expect(cuboid.halfExtents.z).toBe(3);

        world.free();
    });

    test("creates capsule collider", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        const collider = world.createCollider(RAPIER.ColliderDesc.capsule(2, 0.5), body);

        const shape = collider.shape;
        expect(shape.type).toBe(RAPIER.ShapeType.Capsule);
        const capsule = shape as RAPIER.Capsule;
        expect(capsule.halfHeight).toBe(2);
        expect(capsule.radius).toBe(0.5);

        world.free();
    });

    test("collider attached to body moves with it", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 10, 0));
        const collider = world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        const initialColliderY = collider.translation().y;
        expect(initialColliderY).toBeCloseTo(10, 1);

        world.step();

        // Collider should have moved with the body
        const newColliderY = collider.translation().y;
        expect(newColliderY).toBeLessThan(initialColliderY);

        // Collider and body positions should match
        expect(newColliderY).toBeCloseTo(body.translation().y, 4);

        world.free();
    });

    test("collider translation and rotation", () => {
        const world = new RAPIER.World({x: 0, y: 0, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(5, 10, 15));
        const collider = world.createCollider(RAPIER.ColliderDesc.ball(1), body);

        const pos = collider.translation();
        expect(pos.x).toBeCloseTo(5);
        expect(pos.y).toBeCloseTo(10);
        expect(pos.z).toBeCloseTo(15);

        const rot = collider.rotation();
        expect(rot.x).toBeCloseTo(0);
        expect(rot.y).toBeCloseTo(0);
        expect(rot.z).toBeCloseTo(0);
        expect(rot.w).toBeCloseTo(1);

        world.free();
    });

    test("collider with density affects body mass", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        world.createCollider(RAPIER.ColliderDesc.ball(1).setDensity(5.0), body);

        expect(body.mass()).toBeGreaterThan(0);

        world.free();
    });

    test("collider with sensor does not generate contacts", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});

        // Floor
        const floor = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        world.createCollider(RAPIER.ColliderDesc.cuboid(10, 0.1, 10), floor);

        // Falling sensor ball
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 1, 0));
        world.createCollider(RAPIER.ColliderDesc.ball(0.5).setSensor(true), body);

        // Step multiple times - sensor should fall through the floor
        for (let i = 0; i < 120; i++) {
            world.step();
        }

        // Sensor body should have fallen below the floor (y < -0.1)
        expect(body.translation().y).toBeLessThan(-0.1);

        world.free();
    });
});
