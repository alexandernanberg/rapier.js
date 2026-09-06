import RAPIER, {init} from "@alexandernanberg/rapier3d/compat";
import {describe, test, expect, beforeAll} from "vitest";

const GRAVITY = {x: 0, y: -9.81, z: 0};

beforeAll(async () => {
    await init();
});

function restingScene() {
    const world = new RAPIER.World(GRAVITY);
    const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    const groundCollider = world.createCollider(RAPIER.ColliderDesc.cuboid(10, 0.5, 10), ground);
    const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 1, 0));
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

            const n = manifold.normal();
            expect(Math.hypot(n.x, n.y, n.z)).toBeCloseTo(1, 3);

            expect(manifold.localContactPoint1(0)).not.toBeNull();
            expect(manifold.localContactPoint2(0)).not.toBeNull();
        });
        expect(manifolds).toBeGreaterThan(0);

        world.free();
    });

    test("manifold getters write into a caller-supplied target", () => {
        const {world, groundCollider, bodyCollider} = restingScene();

        world.contactPair(bodyCollider, groundCollider, (manifold) => {
            const expected = manifold.normal();

            const target = {x: 0, y: 0, z: 0};
            expect(manifold.normal(target)).toBe(target);
            expect(target.x).toBeCloseTo(expected.x, 6);
            expect(target.y).toBeCloseTo(expected.y, 6);
            expect(target.z).toBeCloseTo(expected.z, 6);

            const point = {x: 0, y: 0, z: 0};
            expect(manifold.localContactPoint1(0, point)).toBe(point);
        });

        world.free();
    });

    test("every manifold field reads back a sane resting contact", () => {
        const {world, groundCollider, bodyCollider} = restingScene();

        let seen = 0;
        world.contactPair(bodyCollider, groundCollider, (manifold) => {
            seen++;
            const n = manifold.numContacts();
            expect(n).toBeGreaterThan(0);
            for (let i = 0; i < n; i++) {
                // Resting: the surfaces touch, so the separation is about zero.
                expect(Math.abs(manifold.contactDist(i))).toBeLessThan(0.05);
                // Holding the ball up against gravity takes a positive normal impulse.
                expect(manifold.contactImpulse(i)).toBeGreaterThan(0);
                expect(Number.isFinite(manifold.contactTangentImpulseX(i))).toBe(true);
                expect(Number.isInteger(manifold.contactFid1(i))).toBe(true);
                expect(Number.isInteger(manifold.contactFid2(i))).toBe(true);
            }

            // Plain shapes: no sub-shapes, no hook has touched the manifold.
            expect(manifold.subshape1()).toBe(0);
            expect(manifold.subshape2()).toBe(0);
            expect(manifold.userData()).toBe(0);
            // Both colliders keep the default coefficients (friction 0.5, restitution 0).
            expect(manifold.friction()).toBeCloseTo(0.5, 5);
            expect(manifold.restitution()).toBeCloseTo(0, 5);

            // A ball resting on a horizontal plane: vertical normals on both sides.
            expect(Math.abs(manifold.normal().y)).toBeCloseTo(1, 3);
            expect(Math.abs(manifold.localNormal1().y)).toBeCloseTo(1, 3);
            expect(Math.abs(manifold.localNormal2().y)).toBeCloseTo(1, 3);

            // The solver acts midway between both surfaces, which meet where the
            // ball (radius 0.5, resting on the ground's top face at y = 0.5) touches.
            expect(manifold.numSolverContacts()).toBeGreaterThan(0);
            const point = manifold.solverContactPoint(0)!;
            expect(point.y).toBeCloseTo(0.5, 1);
            expect(Math.abs(manifold.solverContactDist(0))).toBeLessThan(0.05);
            expect(manifold.solverContactAnchor1(0)).not.toBeNull();
            expect(manifold.solverContactAnchor2(0)).not.toBeNull();
            const tangentVelocity = manifold.solverContactTangentVelocity(0)!;
            expect(Math.hypot(tangentVelocity.x, tangentVelocity.y, tangentVelocity.z)).toBeCloseTo(
                0,
                5,
            );
        });
        expect(seen).toBeGreaterThan(0);

        world.free();
    });

    test("an out-of-range contact index reads as null or zero", () => {
        const {world, groundCollider, bodyCollider} = restingScene();

        world.contactPair(bodyCollider, groundCollider, (manifold) => {
            const n = manifold.numContacts();
            expect(manifold.localContactPoint1(n)).toBeNull();
            expect(manifold.localContactPoint2(-1)).toBeNull();
            expect(manifold.contactDist(n)).toBe(0);
            expect(manifold.contactFid1(n)).toBe(0);
            expect(manifold.contactImpulse(-1)).toBe(0);
            expect(manifold.contactTangentImpulseX(n)).toBe(0);

            const m = manifold.numSolverContacts();
            expect(manifold.solverContactPoint(m)).toBeNull();
            expect(manifold.solverContactAnchor1(m)).toBeNull();
            expect(manifold.solverContactAnchor2(-1)).toBeNull();
            expect(manifold.solverContactTangentVelocity(m)).toBeNull();
            expect(manifold.solverContactDist(m)).toBe(0);

            // A target must be left alone on a miss.
            const target = {x: 7, y: 8, z: 9};
            expect(manifold.localContactPoint1(n, target)).toBeNull();
            expect(target).toEqual({x: 7, y: 8, z: 9});
        });

        world.free();
    });

    test("colliders that are not touching yield no manifold", () => {
        const world = new RAPIER.World(GRAVITY);
        const a = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        const ca = world.createCollider(RAPIER.ColliderDesc.ball(0.5), a);
        const b = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(10, 0, 0));
        const cb = world.createCollider(RAPIER.ColliderDesc.ball(0.5), b);
        world.step();

        let calls = 0;
        world.contactPair(ca, cb, () => calls++);
        expect(calls).toBe(0);

        world.free();
    });

    test("the flipped flag follows the order the colliders were asked for", () => {
        const {world, groundCollider, bodyCollider} = restingScene();

        const flips: boolean[] = [];
        world.contactPair(bodyCollider, groundCollider, (_, flipped) => flips.push(flipped));
        world.contactPair(groundCollider, bodyCollider, (_, flipped) => flips.push(flipped));
        expect(flips).toHaveLength(2);
        // The pair is stored once, so exactly one of the two orders is flipped.
        expect(flips[0]).not.toBe(flips[1]);

        world.free();
    });

    test("the manifold is a snapshot that survives a step from inside the callback", () => {
        const {world, groundCollider, bodyCollider} = restingScene();

        world.contactPair(bodyCollider, groundCollider, (manifold) => {
            const before = manifold.normal();
            const distBefore = manifold.contactDist(0);
            // The old protocol handed out pointers into the narrow phase, which a
            // step invalidated; the buffer read here is not affected by it.
            world.step();
            expect(manifold.normal()).toEqual(before);
            expect(manifold.contactDist(0)).toBe(distBefore);
        });

        world.free();
    });

    test("a sensor reports an intersection rather than a contact", () => {
        const world = new RAPIER.World({x: 0, y: 0, z: 0});
        const a = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        const sensor = world.createCollider(RAPIER.ColliderDesc.ball(1).setSensor(true), a);
        const b = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0.2, 0, 0));
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
        world.createCollider(RAPIER.ColliderDesc.cuboid(20, 0.5, 20), ground);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 3, 0));
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

                const force = event.totalForce();
                expect(Number.isFinite(force.x)).toBe(true);
                expect(Number.isFinite(force.y)).toBe(true);

                const direction = event.maxForceDirection();
                expect(Math.hypot(direction.x, direction.y, direction.z)).toBeCloseTo(1, 3);

                // These read through a shared scratch buffer, so the target
                // overload has to be filled in rather than ignored.
                const target = {x: 0, y: 0, z: 0};
                expect(event.totalForce(target)).toBe(target);
                expect(target.x).toBeCloseTo(force.x, 6);
                expect(target.y).toBeCloseTo(force.y, 6);
                expect(target.z).toBeCloseTo(force.z, 6);
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
            RAPIER.ColliderDesc.cuboid(20, 0.5, 20).setActiveHooks(
                RAPIER.ActiveHooks.FILTER_CONTACT_PAIRS,
            ),
            ground,
        );
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 3, 0));
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
        expect(body.translation().y).toBeLessThan(0);

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

        expect(body.translation().y).toBeGreaterThan(0);

        world.free();
    });

    // `step()` used to forward hooks only on the `stepWithEvents` branch, so
    // stepping with hooks but no event queue silently ignored them.
    test("filterContactPair is honoured without an event queue", () => {
        const {world, body, hooks} = hookScene();

        for (let i = 0; i < 60; i++) world.step(undefined, hooks);

        expect(body.translation().y).toBeLessThan(0);

        world.free();
    });
});
