import RAPIER, {init} from "@alexandernanberg/rapier2d/compat";
import {beforeAll, describe, expect, test} from "vitest";

const GRAVITY = {x: 0, y: -9.81};

beforeAll(async () => {
    await init();
});

/**
 * Both drains move every pending event into a WASM-side buffer and let JS walk
 * it, instead of calling the handler once per event from Rust. Handles have to
 * survive that trip through two `f32` slots, and the views have to cope with a
 * handler that calls back into WASM.
 */
describe("event drain buffers", () => {
    function collidingScene(bodies: number) {
        const world = new RAPIER.World(GRAVITY);
        const queue = new RAPIER.EventQueue(true);

        const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        const groundCollider = world.createCollider(
            RAPIER.ColliderDesc.cuboid(50, 0.5).setActiveEvents(
                RAPIER.ActiveEvents.COLLISION_EVENTS,
            ),
            ground,
        );

        const colliders: number[] = [];
        for (let i = 0; i < bodies; i++) {
            const body = world.createRigidBody(
                RAPIER.RigidBodyDesc.dynamic().setTranslation(i * 2, 1.5),
            );
            colliders.push(
                world.createCollider(
                    RAPIER.ColliderDesc.ball(0.5).setActiveEvents(
                        RAPIER.ActiveEvents.COLLISION_EVENTS,
                    ),
                    body,
                ).handle,
            );
        }

        return {world, queue, groundCollider: groundCollider.handle, colliders};
    }

    test("collision handles survive the round trip through the buffer", () => {
        const {world, queue, groundCollider, colliders} = collidingScene(6);

        const started = new Set<number>();
        for (let i = 0; i < 60; i++) {
            world.step(queue);
            queue.drainCollisionEvents((h1, h2, isStart) => {
                // A handle packs an arena index and a generation into an f64; both
                // halves have to come back bit-exact, so the handle must resolve to
                // a live collider rather than to some near-miss.
                expect(world.getCollider(h1)).not.toBeNull();
                expect(world.getCollider(h2)).not.toBeNull();

                const other = h1 === groundCollider ? h2 : h1;
                if (isStart) started.add(other);
            });
        }

        // Every ball landed on the ground and reported it under its own handle.
        for (const handle of colliders) {
            expect(started.has(handle)).toBe(true);
        }

        world.free();
        queue.free();
    });

    test("a recycled collider slot keeps its generation through the buffer", () => {
        const world = new RAPIER.World(GRAVITY);
        const queue = new RAPIER.EventQueue(true);

        const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        world.createCollider(
            RAPIER.ColliderDesc.cuboid(50, 0.5).setActiveEvents(
                RAPIER.ActiveEvents.COLLISION_EVENTS,
            ),
            ground,
        );

        // Removing a collider and creating another recycles the arena slot with a
        // bumped generation, so the handle's high 32 bits stop being zero. That half
        // travels in its own `f32` slot; dropping it would yield a handle that still
        // resolves, just to the wrong incarnation.
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 1.5));
        const first = world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
        world.removeCollider(first, true);
        const recycled = world.createCollider(
            RAPIER.ColliderDesc.ball(0.5).setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
            body,
        );
        expect(recycled.handle).not.toBe(first.handle);

        let sawRecycled = false;
        for (let i = 0; i < 60; i++) {
            world.step(queue);
            queue.drainCollisionEvents((h1, h2) => {
                if (h1 === recycled.handle || h2 === recycled.handle) sawRecycled = true;
            });
        }
        expect(sawRecycled).toBe(true);

        world.free();
        queue.free();
    });

    test("a handler calling back into WASM still reads the rest of the events", () => {
        const {world, queue, colliders} = collidingScene(4);

        let seen = 0;
        for (let i = 0; i < 60; i++) {
            world.step(queue);
            queue.drainCollisionEvents((h1, h2) => {
                seen++;
                // Allocating in WASM can grow the linear memory, which detaches every
                // view onto it. The remaining events must still be readable.
                const body = world.createRigidBody(
                    RAPIER.RigidBodyDesc.fixed().setTranslation(0, -50),
                );
                world.createCollider(RAPIER.ColliderDesc.ball(0.1), body);

                expect(Number.isFinite(h1)).toBe(true);
                expect(Number.isFinite(h2)).toBe(true);
                expect(world.getCollider(h1)).not.toBeNull();
                expect(world.getCollider(h2)).not.toBeNull();
            });
        }

        expect(seen).toBeGreaterThanOrEqual(colliders.length);

        world.free();
        queue.free();
    });

    test("an empty drain calls the handler no times and leaves the queue usable", () => {
        const world = new RAPIER.World(GRAVITY);
        const queue = new RAPIER.EventQueue(true);

        let calls = 0;
        queue.drainCollisionEvents(() => calls++);
        queue.drainContactForceEvents(() => calls++);
        expect(calls).toBe(0);

        world.step(queue);
        queue.drainCollisionEvents(() => calls++);
        queue.drainContactForceEvents(() => calls++);
        expect(calls).toBe(0);

        world.free();
        queue.free();
    });

    test("contact force events carry both colliders and a unit max-force direction", () => {
        const world = new RAPIER.World(GRAVITY);
        const queue = new RAPIER.EventQueue(true);

        const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        const groundCollider = world.createCollider(RAPIER.ColliderDesc.cuboid(20, 0.5), ground);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 3));
        const ballCollider = world.createCollider(
            RAPIER.ColliderDesc.ball(0.5)
                .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
                .setContactForceEventThreshold(0.1),
            body,
        );

        let events = 0;
        for (let i = 0; i < 90; i++) {
            world.step(queue);
            queue.drainContactForceEvents((event) => {
                events++;

                const pair = [event.collider1(), event.collider2()].sort();
                expect(pair).toEqual([groundCollider.handle, ballCollider.handle].sort());

                expect(event.totalForceMagnitude()).toBeGreaterThan(0);
                expect(event.maxForceMagnitude()).toBeGreaterThan(0);

                const direction = event.maxForceDirection();
                expect(Math.hypot(direction.x, direction.y)).toBeCloseTo(1, 3);

                // The target overload writes into the caller's object.
                const target = {x: 0, y: 0};
                expect(event.maxForceDirection(target)).toBe(target);
                expect(target.y).toBeCloseTo(direction.y, 6);
            });
        }

        expect(events).toBeGreaterThan(0);

        world.free();
        queue.free();
    });

    test("clear() drops events that were never drained", () => {
        const {world, queue} = collidingScene(3);

        // `autoDrain` is on, so clear before the events of the last step are read.
        for (let i = 0; i < 60; i++) world.step(queue);
        queue.clear();

        let calls = 0;
        queue.drainCollisionEvents(() => calls++);
        expect(calls).toBe(0);

        world.free();
        queue.free();
    });
});
