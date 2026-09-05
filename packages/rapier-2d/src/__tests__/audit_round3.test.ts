import RAPIER, {init} from "@alexandernanberg/rapier2d/compat";
import {describe, test, expect, beforeAll} from "vitest";

beforeAll(async () => {
    await init();
});

const GRAVITY = {x: 0, y: -9.81};

describe("forEachActiveRigidBody", () => {
    // The active set is published as a snapshot before the walk, so a body an
    // earlier callback removed is still listed: it used to reach the callback
    // as `null`.
    test("skips bodies removed by an earlier callback", () => {
        const world = new RAPIER.World(GRAVITY);
        const bodies: RAPIER.RigidBody[] = [];
        for (let i = 0; i < 4; i++) {
            const body = world.createRigidBody(
                RAPIER.RigidBodyDesc.dynamic().setTranslation(i * 3, 5),
            );
            world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
            bodies.push(body);
        }
        world.step();

        const seen: number[] = [];
        world.forEachActiveRigidBody((body) => {
            expect(body).not.toBeNull();
            seen.push(body.handle);
            for (const other of bodies) {
                if (other !== body && other.isValid()) world.removeRigidBody(other);
            }
        });

        expect(seen).toHaveLength(1);
        expect(world.bodies.len()).toBe(1);
        world.free();
    });
});

describe("scalar setters", () => {
    test("RigidBody.setAdditionalMassProperties takes effect", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        world.createCollider(RAPIER.ColliderDesc.ball(0.5).setDensity(1), body);
        const before = body.mass();
        expect(before).toBeGreaterThan(0);

        body.setAdditionalMassProperties(10, {x: 0, y: 0}, 1, true);
        // Rapier folds additional mass properties in at the next step.
        world.step();
        expect(body.mass()).toBeCloseTo(before + 10, 4);
        world.free();
    });

    test("Collider.setMassProperties takes effect", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        const collider = world.createCollider(RAPIER.ColliderDesc.ball(0.5).setDensity(1), body);

        collider.setMassProperties(7, {x: 0, y: 0}, 1);
        expect(collider.mass()).toBeCloseTo(7, 4);
        world.step();
        expect(body.mass()).toBeCloseTo(7, 4);
        world.free();
    });

    test("ImpulseJoint frame setters round-trip", () => {
        const world = new RAPIER.World(GRAVITY);
        const body1 = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        const body2 = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(2, 0));
        const joint = world.createImpulseJoint(
            RAPIER.JointData.revolute({x: 0, y: 0}, {x: 0, y: 0}),
            body1,
            body2,
            true,
        );

        joint.setAnchor1({x: 1, y: 2});
        expect(joint.anchor1()).toEqual({x: 1, y: 2});

        joint.setAnchor2({x: -1, y: -2});
        expect(joint.anchor2()).toEqual({x: -1, y: -2});

        // 2D exposes no frame getter; the setters must at least accept an angle.
        joint.setFrameX1(0.5);
        joint.setFrameX2(-0.5);

        joint.setLocalFrame2({x: 4, y: 5}, -0.25);
        expect(joint.anchor2()).toEqual({x: 4, y: 5});

        joint.setLocalFrame1({x: 7, y: 8}, 1);
        expect(joint.anchor1()).toEqual({x: 7, y: 8});

        world.free();
    });
});

describe("contact force events", () => {
    test("started is set on the first event of a pair only", () => {
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

        const flags: boolean[] = [];
        for (let i = 0; i < 90; i++) {
            world.step(queue);
            queue.drainContactForceEvents((event) => {
                flags.push(event.started());
                // The slot before it is still the max force magnitude.
                expect(event.maxForceMagnitude()).toBeGreaterThan(0);
            });
        }

        expect(flags.length).toBeGreaterThan(1);
        expect(flags[0]).toBe(true);
        expect(flags[flags.length - 1]).toBe(false);

        world.free();
        queue.free();
    });
});

describe("propagateModifiedBodyPositionsToColliders", () => {
    test("updates the collider transform buffer instead of invalidating it", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 5));
        const collider = world.createCollider(
            RAPIER.ColliderDesc.ball(0.5).setTranslation(1, 0),
            body,
        );
        const fixed = world.createCollider(RAPIER.ColliderDesc.cuboid(1, 1).setTranslation(-8, 0));
        world.step();
        expect(world.colliders._bufferRef.ptr).not.toBe(0);

        body.setTranslation({x: 10, y: 5}, true);
        world.propagateModifiedBodyPositionsToColliders();

        expect(world.colliders._bufferRef.ptr).not.toBe(0);
        expect(collider.translation().x).toBeCloseTo(11, 5);
        expect(collider.translation().y).toBeCloseTo(5, 5);
        expect(fixed.translation().x).toBeCloseTo(-8, 5);

        world.step();
        expect(collider.translation().x).toBeCloseTo(11, 4);
        world.free();
    });

    test("is coherent before the first step too", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 5));
        const collider = world.createCollider(
            RAPIER.ColliderDesc.ball(0.5).setTranslation(1, 0),
            body,
        );
        body.setTranslation({x: 10, y: 5}, true);
        world.propagateModifiedBodyPositionsToColliders();
        expect(collider.translation().x).toBeCloseTo(11, 5);
        world.step();
        expect(collider.translation().x).toBeCloseTo(11, 4);
        world.free();
    });
});

describe("kinematic bodies", () => {
    test("setNextKinematicTranslation is reflected after the step for many bodies", () => {
        const world = new RAPIER.World(GRAVITY);
        const bodies = [];
        for (let i = 0; i < 100; i++) {
            const body = world.createRigidBody(
                RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(i, 0),
            );
            world.createCollider(RAPIER.ColliderDesc.ball(0.25), body);
            bodies.push(body);
        }
        const tiles = [];
        for (let i = 0; i < 300; i++) {
            tiles.push(
                world.createCollider(RAPIER.ColliderDesc.cuboid(0.5, 0.5).setTranslation(i, -5)),
            );
        }

        for (let step = 1; step <= 5; step++) {
            for (let i = 0; i < bodies.length; i++) {
                bodies[i].setNextKinematicTranslation({x: i, y: step * 0.1});
            }
            world.step();
            for (let i = 0; i < bodies.length; i++) {
                expect(bodies[i].translation().y).toBeCloseTo(step * 0.1, 5);
                expect(bodies[i].collider(0)!.translation().y).toBeCloseTo(step * 0.1, 5);
            }
        }
        expect(tiles[123].translation().x).toBeCloseTo(123, 5);
        world.free();
    });
});

describe("input validation", () => {
    test("a joint between a body and itself is rejected", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        const params = RAPIER.JointData.revolute({x: 0, y: 0}, {x: 0, y: 0});
        expect(() => world.createImpulseJoint(params, body, body, true)).toThrow();
        expect(() => world.createMultibodyJoint(params, body, body, true)).toThrow();
        expect(world.impulseJoints.len()).toBe(0);
        world.step();
        world.free();
    });

    test("zero VHACD parameters do not trap", () => {
        // An L-shaped polyline, which needs more than one convex part.
        const vertices = new Float32Array([0, 0, 2, 0, 2, 1, 1, 1, 1, 2, 0, 2]);
        const indices = new Uint32Array([0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 0]);
        expect(() =>
            RAPIER.ColliderDesc.convexDecomposition(vertices, indices, {
                resolution: 0,
                planeDownsampling: 0,
                convexHullDownsampling: 0,
            }),
        ).not.toThrow();
    });
});

describe("World", () => {
    test("free() is idempotent", () => {
        const world = new RAPIER.World(GRAVITY);
        world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        world.free();
        expect(() => world.free()).not.toThrow();
    });

    test("contactPairsWith honours the stop signal and a throwing callback", () => {
        const world = new RAPIER.World(GRAVITY);
        const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        const groundCollider = world.createCollider(RAPIER.ColliderDesc.cuboid(20, 0.5), ground);
        for (let i = 0; i < 3; i++) {
            const body = world.createRigidBody(
                RAPIER.RigidBodyDesc.dynamic().setTranslation(i * 3, 1),
            );
            world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
        }
        for (let i = 0; i < 30; i++) world.step();

        let all = 0;
        world.contactPairsWith(groundCollider, () => {
            all++;
        });
        expect(all).toBe(3);

        // World ignores the callback's return value (so a `Set.delete` or
        // `Array.push` result can never cut the walk short); the raw narrow
        // phase honours an explicit `false`.
        let notStopped = 0;
        world.contactPairsWith(groundCollider, () => {
            notStopped++;
            return false;
        });
        expect(notStopped).toBe(3);

        let stopped = 0;
        world.narrowPhase.contactPairsWith(groundCollider.handle, () => {
            stopped++;
            return false;
        });
        expect(stopped).toBe(1);

        let calls = 0;
        expect(() =>
            world.contactPairsWith(groundCollider, () => {
                calls++;
                throw new Error("boom");
            }),
        ).toThrow("boom");
        expect(calls).toBe(1);
        world.free();
    });
});
