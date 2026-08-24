import RAPIER, {init} from "@alexandernanberg/rapier2d/compat";
import {describe, test, expect, beforeAll} from "vitest";
import {_v} from "./_target";

const GRAVITY = {x: 0, y: -9.81};

beforeAll(async () => {
    await init();
});

function restingScene() {
    const world = new RAPIER.World(GRAVITY);
    const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    const groundCollider = world.createCollider(RAPIER.ColliderDesc.cuboid(10, 0.5), ground);
    const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 1));
    const bodyCollider = world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
    for (let i = 0; i < 30; i++) world.step();
    return {world, groundCollider, bodyCollider};
}

/**
 * Contact manifolds are handed over as a temporary raw pointer that is only
 * valid until the next step, and their getters take the same optional `target`
 * as the transform getters. Both properties are easy to regress and untested
 * everywhere else.
 */
describe("narrow phase contacts", () => {
    test("contactPairsWith finds the collider being rested upon", () => {
        const {world, groundCollider, bodyCollider} = restingScene();

        const touching: number[] = [];
        world.contactPairsWith(bodyCollider, (other) => touching.push(other.handle));
        expect(touching).toContain(groundCollider.handle);

        world.free();
    });

    test("contactPair yields a manifold with a unit normal and contact points", () => {
        const {world, groundCollider, bodyCollider} = restingScene();

        let manifolds = 0;
        world.contactPair(bodyCollider, groundCollider, (manifold) => {
            manifolds++;
            expect(manifold.numContacts()).toBeGreaterThan(0);

            const normal = manifold.normal(_v());
            expect(Math.hypot(normal.x, normal.y)).toBeCloseTo(1, 3);

            expect(manifold.localContactPoint1(0, _v())).not.toBeNull();
            expect(manifold.localContactPoint2(0, _v())).not.toBeNull();
        });
        expect(manifolds).toBeGreaterThan(0);

        world.free();
    });

    test("manifold getters write into a caller-supplied target", () => {
        const {world, groundCollider, bodyCollider} = restingScene();

        world.contactPair(bodyCollider, groundCollider, (manifold) => {
            const expected = manifold.normal(_v());

            const target = {x: 0, y: 0};
            expect(manifold.normal(target)).toBe(target);
            expect(target.x).toBeCloseTo(expected.x, 6);
            expect(target.y).toBeCloseTo(expected.y, 6);

            const point = {x: 0, y: 0};
            expect(manifold.localContactPoint1(0, point)).toBe(point);
        });

        world.free();
    });

    test("a sensor reports an intersection rather than a contact", () => {
        const world = new RAPIER.World({x: 0, y: 0});
        const a = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        const sensor = world.createCollider(RAPIER.ColliderDesc.ball(1).setSensor(true), a);
        const b = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0.2, 0));
        const solid = world.createCollider(RAPIER.ColliderDesc.ball(0.3), b);
        world.step();

        expect(sensor.isSensor()).toBe(true);
        expect(world.intersectionPair(sensor, solid)).toBe(true);

        const overlapping: number[] = [];
        world.intersectionPairsWith(sensor, (other) => overlapping.push(other.handle));
        expect(overlapping).toContain(solid.handle);

        world.free();
    });
});

describe("contact force events", () => {
    test("drainContactForceEvents reports the impact force", () => {
        const world = new RAPIER.World(GRAVITY);
        const queue = new RAPIER.EventQueue(true);

        const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        world.createCollider(RAPIER.ColliderDesc.cuboid(20, 0.5), ground);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 3));
        world.createCollider(
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
                expect(event.totalForceMagnitude()).toBeGreaterThan(0);

                const force = event.totalForce(_v());
                expect(Number.isFinite(force.x)).toBe(true);
                expect(Number.isFinite(force.y)).toBe(true);

                const direction = event.maxForceDirection(_v());
                expect(Math.hypot(direction.x, direction.y)).toBeCloseTo(1, 3);

                // These read through a shared scratch buffer, so the target
                // overload has to be filled in rather than ignored.
                const target = {x: 0, y: 0};
                expect(event.totalForce(target)).toBe(target);
                expect(target.x).toBeCloseTo(force.x, 6);
                expect(target.y).toBeCloseTo(force.y, 6);
            });
        }
        expect(events).toBeGreaterThan(0);

        queue.free();
        world.free();
    });
});

describe("physics hooks", () => {
    function hookScene() {
        const world = new RAPIER.World(GRAVITY);
        const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        world.createCollider(
            RAPIER.ColliderDesc.cuboid(20, 0.5).setActiveHooks(
                RAPIER.ActiveHooks.FILTER_CONTACT_PAIRS,
            ),
            ground,
        );
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 3));
        world.createCollider(
            RAPIER.ColliderDesc.ball(0.5).setActiveHooks(RAPIER.ActiveHooks.FILTER_CONTACT_PAIRS),
            body,
        );

        const hooks: RAPIER.PhysicsHooks = {
            // Returning null drops the contact entirely.
            filterContactPair: () => null,
            filterIntersectionPair: () => false,
        };

        return {world, body, hooks};
    }

    test("filterContactPair vetoes the contact when an event queue is passed", () => {
        const {world, body, hooks} = hookScene();
        const queue = new RAPIER.EventQueue(false);

        for (let i = 0; i < 60; i++) world.step(queue, hooks);

        // With the contact filtered out the body falls straight through.
        expect(body.translation(_v()).y).toBeLessThan(0);

        queue.free();
        world.free();
    });

    // The hook methods are optional, so a collider can carry the filter flag while
    // the hooks object only implements contact modification. A missing filter must
    // leave the pair alone rather than dropping it.
    test("a hooks object without filterContactPair leaves contacts alone", () => {
        const {world, body} = hookScene();
        const hooks: RAPIER.PhysicsHooks = {};

        for (let i = 0; i < 60; i++) world.step(undefined, hooks);

        expect(body.translation(_v()).y).toBeGreaterThan(0);

        world.free();
    });

    // `step()` used to forward hooks only on the `stepWithEvents` branch, so
    // stepping with hooks but no event queue silently ignored them.
    test("filterContactPair is honoured without an event queue", () => {
        const {world, body, hooks} = hookScene();

        for (let i = 0; i < 60; i++) world.step(undefined, hooks);

        expect(body.translation(_v()).y).toBeLessThan(0);

        world.free();
    });
});
