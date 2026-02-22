import RAPIER, {init} from "@alexandernanberg/rapier3d/compat";
import {describe, test, expect, beforeAll} from "vitest";

beforeAll(async () => {
    await init();
});

describe("World", () => {
    test("creates world with gravity", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        expect(world.gravity.x).toBe(0);
        expect(world.gravity.y).toBe(-9.81);
        expect(world.gravity.z).toBe(0);
        world.free();
    });

    test("dynamic body falls under gravity", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 10, 0);
        const body = world.createRigidBody(bodyDesc);
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        const initialY = body.translation().y;
        world.step();
        expect(body.translation().y).toBeLessThan(initialY);

        world.free();
    });

    test("fixed body does not move under gravity", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const bodyDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(0, 5, 0);
        const body = world.createRigidBody(bodyDesc);
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        world.step();
        expect(body.translation().y).toBe(5);

        world.free();
    });

    test("step with event queue collects collision events", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});

        // Create a floor
        const floorBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        world.createCollider(
            RAPIER.ColliderDesc.cuboid(10, 0.1, 10).setActiveEvents(
                RAPIER.ActiveEvents.COLLISION_EVENTS,
            ),
            floorBody,
        );

        // Create a falling ball very close to the floor
        const ballBody = world.createRigidBody(
            RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 0.6, 0),
        );
        world.createCollider(
            RAPIER.ColliderDesc.ball(0.5).setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
            ballBody,
        );

        const eventQueue = new RAPIER.EventQueue(true);
        const collisions: {h1: number; h2: number; started: boolean}[] = [];

        // Step until we get a collision
        for (let i = 0; i < 60; i++) {
            world.step(eventQueue);
            eventQueue.drainCollisionEvents((h1, h2, started) => {
                collisions.push({h1, h2, started});
            });
            if (collisions.length > 0) break;
        }

        expect(collisions.length).toBeGreaterThan(0);
        expect(collisions[0]!.started).toBe(true);

        eventQueue.free();
        world.free();
    });

    test("multiple steps advance simulation progressively", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(
            RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 100, 0),
        );
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        const positions: number[] = [];
        for (let i = 0; i < 10; i++) {
            world.step();
            positions.push(body.translation().y);
        }

        // Each position should be lower than the previous (falling)
        for (let i = 1; i < positions.length; i++) {
            expect(positions[i]).toBeLessThan(positions[i - 1]!);
        }

        world.free();
    });

    test("world.free() cleans up resources", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        // Should not throw
        world.free();
    });
});
