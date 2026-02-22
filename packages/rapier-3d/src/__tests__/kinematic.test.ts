import RAPIER, {init} from "@alexandernanberg/rapier3d/compat";
import {describe, test, expect, beforeAll} from "vitest";

beforeAll(async () => {
    await init();
});

describe("Kinematic Bodies", () => {
    test("setNextKinematicTranslation before step works", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(
            RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(0, 0, 0),
        );
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        body.setNextKinematicTranslation({x: 1, y: 2, z: 3});
        world.step();

        const pos = body.translation();
        expect(pos.x).toBeCloseTo(1);
        expect(pos.y).toBeCloseTo(2);
        expect(pos.z).toBeCloseTo(3);

        world.free();
    });

    test("setNextKinematicTranslation after step works", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(
            RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(0, 0, 0),
        );
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        world.step();

        body.setNextKinematicTranslation({x: 5, y: 5, z: 5});
        world.step();

        const pos = body.translation();
        expect(pos.x).toBeCloseTo(5);
        expect(pos.y).toBeCloseTo(5);
        expect(pos.z).toBeCloseTo(5);

        world.free();
    });

    test("position-based kinematic body moves to target after step", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(
            RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(0, 0, 0),
        );
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        // Set target and step
        body.setNextKinematicTranslation({x: 10, y: 0, z: 0});
        world.step();

        // Should be at the target position
        expect(body.translation().x).toBeCloseTo(10);
        expect(body.translation().y).toBeCloseTo(0);

        world.free();
    });

    test("velocity-based kinematic body moves with setLinvel", () => {
        const world = new RAPIER.World({x: 0, y: 0, z: 0});
        const body = world.createRigidBody(
            RAPIER.RigidBodyDesc.kinematicVelocityBased().setTranslation(0, 0, 0),
        );
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        body.setLinvel({x: 10, y: 0, z: 0}, true);
        world.step();

        // Should have moved in X direction
        expect(body.translation().x).toBeGreaterThan(0);

        world.free();
    });

    test("kinematic body is not affected by gravity", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(
            RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(0, 5, 0),
        );
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        world.step();

        // Kinematic body should not fall
        expect(body.translation().y).toBeCloseTo(5);

        world.free();
    });

    test("kinematic body pushes dynamic body", () => {
        const world = new RAPIER.World({x: 0, y: 0, z: 0}); // No gravity

        // Create kinematic body on the left
        const kinematic = world.createRigidBody(
            RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(0, 0, 0),
        );
        world.createCollider(RAPIER.ColliderDesc.ball(1), kinematic);

        // Create dynamic body nearby
        const dynamic = world.createRigidBody(
            RAPIER.RigidBodyDesc.dynamic().setTranslation(2.5, 0, 0),
        );
        world.createCollider(RAPIER.ColliderDesc.ball(1), dynamic);

        const initialDynamicX = dynamic.translation().x;

        // Move kinematic body toward dynamic body
        for (let i = 0; i < 10; i++) {
            kinematic.setNextKinematicTranslation({
                x: (i + 1) * 0.5,
                y: 0,
                z: 0,
            });
            world.step();
        }

        // Dynamic body should have been pushed
        expect(dynamic.translation().x).toBeGreaterThan(initialDynamicX);

        world.free();
    });

    test("multi-frame kinematic movement cycle", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(
            RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(0, 0, 0),
        );
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        // Simulate a back-and-forth motion over multiple frames
        const positions: number[] = [];

        for (let i = 0; i < 20; i++) {
            const x = Math.sin(i * 0.3) * 5;
            body.setNextKinematicTranslation({x, y: 0, z: 0});
            world.step();
            positions.push(body.translation().x);
        }

        // Verify the body followed the sinusoidal path
        for (let i = 0; i < 20; i++) {
            const expectedX = Math.sin(i * 0.3) * 5;
            expect(positions[i]).toBeCloseTo(expectedX, 1);
        }

        world.free();
    });
    test("setNextKinematicRotation works", () => {
        const world = new RAPIER.World({x: 0, y: 0, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.kinematicPositionBased());
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        // 90 degrees around Y axis
        const q = {x: 0, y: 0.7071068, z: 0, w: 0.7071068};
        body.setNextKinematicRotation(q);
        world.step();

        const rot = body.rotation();
        expect(rot.x).toBeCloseTo(q.x, 3);
        expect(rot.y).toBeCloseTo(q.y, 3);
        expect(rot.z).toBeCloseTo(q.z, 3);
        expect(rot.w).toBeCloseTo(q.w, 3);

        world.free();
    });
});
