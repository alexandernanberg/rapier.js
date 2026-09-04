import RAPIER, {init} from "@alexandernanberg/rapier2d/compat";
import {describe, test, expect, beforeAll} from "vitest";

beforeAll(async () => {
    await init();
});

describe("World", () => {
    test("creates world with gravity", () => {
        const world = new RAPIER.World({x: 0, y: -9.81});
        expect(world.gravity.x).toBe(0);
        expect(world.gravity.y).toBe(-9.81);
        world.free();
    });

    test("dynamic body falls under gravity", () => {
        const world = new RAPIER.World({x: 0, y: -9.81});
        const bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 10);
        const body = world.createRigidBody(bodyDesc);
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        const initialY = body.translation().y;
        world.step();
        expect(body.translation().y).toBeLessThan(initialY);

        world.free();
    });

    test("fixed body does not move under gravity", () => {
        const world = new RAPIER.World({x: 0, y: -9.81});
        const bodyDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(0, 5);
        const body = world.createRigidBody(bodyDesc);
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        world.step();
        expect(body.translation().y).toBe(5);

        world.free();
    });

    test("step with event queue collects collision events", () => {
        const world = new RAPIER.World({x: 0, y: -9.81});

        // Create a floor
        const floorBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        world.createCollider(
            RAPIER.ColliderDesc.cuboid(10, 0.1).setActiveEvents(
                RAPIER.ActiveEvents.COLLISION_EVENTS,
            ),
            floorBody,
        );

        // Create a falling ball very close to the floor
        const ballBody = world.createRigidBody(
            RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 0.6),
        );
        world.createCollider(
            RAPIER.ColliderDesc.ball(0.5).setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
            ballBody,
        );

        const eventQueue = new RAPIER.EventQueue(true);
        const collisions: {h1: number; h2: number; started: boolean}[] = [];

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
        const world = new RAPIER.World({x: 0, y: -9.81});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 100));
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        const positions: number[] = [];
        for (let i = 0; i < 10; i++) {
            world.step();
            positions.push(body.translation().y);
        }

        for (let i = 1; i < positions.length; i++) {
            expect(positions[i]).toBeLessThan(positions[i - 1]!);
        }

        world.free();
    });

    test("world.free() cleans up resources", () => {
        const world = new RAPIER.World({x: 0, y: -9.81});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
        world.free();
    });

    test("reading a transform after world.free() throws instead of reading freed memory", () => {
        const world = new RAPIER.World({x: 0, y: -9.81});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(1, 2));
        const collider = world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
        world.step();

        // Populates the shared transform buffers.
        expect(body.translation().y).toBeCloseTo(2, 1);
        expect(collider.translation().y).toBeCloseTo(2, 1);

        world.free();

        // The buffers point into WASM memory that has just been freed, so reads
        // must not silently return whatever is left there.
        expect(() => body.translation()).toThrow();
        expect(() => collider.translation()).toThrow();
    });

    // A drain callback that throws used to abandon the rest of the step's events,
    // and — because the exception unwound straight out of the WASM frame — left
    // the queue's borrow flag set, so the next call to it panicked with
    // "recursive use of an object detected".
    test("a throwing drain callback still delivers every event and leaves the queue usable", () => {
        const world = new RAPIER.World({x: 0, y: -9.81});
        const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        world.createCollider(RAPIER.ColliderDesc.cuboid(20, 0.5), ground);

        // Four balls dropped from the same height land on the same step, so a
        // single drain has more than one event to deliver.
        for (let i = 0; i < 4; i++) {
            const body = world.createRigidBody(
                RAPIER.RigidBodyDesc.dynamic().setTranslation(i * 2 - 3, 1.2),
            );
            world.createCollider(
                RAPIER.ColliderDesc.ball(0.5).setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
                body,
            );
        }

        const eventQueue = new RAPIER.EventQueue(true);
        let delivered = 0;
        let caught: unknown = null;

        // Keep the busiest drain seen: the assertion is about one drain having
        // delivered every one of its events, not about the total across steps.
        for (let i = 0; i < 120 && delivered < 2; i++) {
            world.step(eventQueue);

            let count = 0;
            let error: unknown = null;
            try {
                eventQueue.drainCollisionEvents(() => {
                    count++;
                    throw new Error(`boom ${count}`);
                });
            } catch (thrown) {
                error = thrown;
            }

            if (count > delivered) {
                delivered = count;
                caught = error;
            }
        }

        // Every event reached the callback, not just the one that threw...
        expect(delivered).toBeGreaterThanOrEqual(2);
        // ...and the exception that surfaced is the first one.
        expect((caught as Error).message).toBe("boom 1");

        // The throw must not leave the queue borrowed on the WASM side.
        expect(() => eventQueue.drainCollisionEvents(() => {})).not.toThrow();

        eventQueue.free();
        world.free();
    });
});
