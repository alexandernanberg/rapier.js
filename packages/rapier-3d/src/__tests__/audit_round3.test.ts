import RAPIER, {init} from "@alexandernanberg/rapier3d/compat";
import {describe, test, expect, beforeAll} from "vitest";

beforeAll(async () => {
    await init();
});

const GRAVITY = {x: 0, y: -9.81, z: 0};
const IDENTITY = {x: 0, y: 0, z: 0, w: 1};
const ZERO_Q = {x: 0, y: 0, z: 0, w: 0};
const QUARTER_TURN_Y = {x: 0, y: Math.SQRT1_2, z: 0, w: Math.SQRT1_2};

describe("forEachActiveRigidBody", () => {
    // The active set is published as a snapshot before the walk, so a body an
    // earlier callback removed is still listed: it used to reach the callback
    // as `null`.
    test("skips bodies removed by an earlier callback", () => {
        const world = new RAPIER.World(GRAVITY);
        const bodies: RAPIER.RigidBody[] = [];
        for (let i = 0; i < 4; i++) {
            const body = world.createRigidBody(
                RAPIER.RigidBodyDesc.dynamic().setTranslation(i * 3, 5, 0),
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

describe("setRotation", () => {
    test("a rejected (zero) quaternion still honours wakeUp", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(
            RAPIER.RigidBodyDesc.dynamic()
                .setTranslation(0, 5, 0)
                .setRotation(QUARTER_TURN_Y)
                .setSleeping(true),
        );
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
        expect(body.isSleeping()).toBe(true);

        body.setRotation(ZERO_Q, true);
        expect(body.isSleeping()).toBe(false);
        // The rotation itself is left alone.
        expect(body.rotation().y).toBeCloseTo(Math.SQRT1_2, 5);
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

        body.setAdditionalMassProperties(
            10,
            {x: 0, y: 0, z: 0},
            {x: 1, y: 1, z: 1},
            IDENTITY,
            true,
        );
        // Rapier folds additional mass properties in at the next step.
        world.step();
        expect(body.mass()).toBeCloseTo(before + 10, 4);
        world.free();
    });

    test("Collider.setMassProperties takes effect", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        const collider = world.createCollider(RAPIER.ColliderDesc.ball(0.5).setDensity(1), body);

        collider.setMassProperties(7, {x: 0, y: 0, z: 0}, {x: 1, y: 1, z: 1}, IDENTITY);
        expect(collider.mass()).toBeCloseTo(7, 4);
        world.step();
        expect(body.mass()).toBeCloseTo(7, 4);
        world.free();
    });

    test("ImpulseJoint frame setters round-trip", () => {
        const world = new RAPIER.World(GRAVITY);
        const body1 = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        const body2 = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(2, 0, 0));
        const joint = world.createImpulseJoint(
            RAPIER.JointData.spherical({x: 0, y: 0, z: 0}, {x: 0, y: 0, z: 0}),
            body1,
            body2,
            true,
        );

        joint.setAnchor1({x: 1, y: 2, z: 3});
        expect(joint.anchor1()).toEqual({x: 1, y: 2, z: 3});

        joint.setAnchor2({x: -1, y: -2, z: -3});
        expect(joint.anchor2()).toEqual({x: -1, y: -2, z: -3});

        joint.setFrameX1(QUARTER_TURN_Y);
        expect(joint.frameX1().y).toBeCloseTo(Math.SQRT1_2, 5);
        expect(joint.frameX1().w).toBeCloseTo(Math.SQRT1_2, 5);

        joint.setLocalFrame2({x: 4, y: 5, z: 6}, QUARTER_TURN_Y);
        expect(joint.anchor2()).toEqual({x: 4, y: 5, z: 6});
        expect(joint.frameX2().y).toBeCloseTo(Math.SQRT1_2, 5);

        // A rejected rotation keeps the current one but still moves the anchor.
        joint.setLocalFrame1({x: 7, y: 8, z: 9}, ZERO_Q);
        expect(joint.anchor1()).toEqual({x: 7, y: 8, z: 9});
        expect(joint.frameX1().y).toBeCloseTo(Math.SQRT1_2, 5);

        world.free();
    });

    test("Collider.setRotationWrtParent keeps the quaternion it was given", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        const collider = world.createCollider(
            RAPIER.ColliderDesc.cuboid(0.5, 0.5, 0.5).setTranslation(1, 0, 0),
            body,
        );

        // The same rotation as QUARTER_TURN_Y with the opposite sign; the
        // axis-angle round trip this used to go through flipped it back.
        collider.setRotationWrtParent({x: 0, y: -Math.SQRT1_2, z: 0, w: -Math.SQRT1_2});
        const rot = collider.rotationWrtParent()!;
        expect(rot.y).toBeCloseTo(-Math.SQRT1_2, 5);
        expect(rot.w).toBeCloseTo(-Math.SQRT1_2, 5);
        // The parent-relative translation is untouched.
        expect(collider.translationWrtParent()!.x).toBeCloseTo(1, 5);
        world.free();
    });
});

describe("contact force events", () => {
    test("started is set on the first event of a pair only", () => {
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
        // The ball rests on the ground afterwards, so the force stays above the
        // threshold and the sustained events report `false`.
        expect(flags[flags.length - 1]).toBe(false);

        world.free();
        queue.free();
    });
});

describe("propagateModifiedBodyPositionsToColliders", () => {
    test("updates the collider transform buffer instead of invalidating it", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 5, 0));
        const collider = world.createCollider(
            RAPIER.ColliderDesc.ball(0.5).setTranslation(1, 0, 0),
            body,
        );
        const fixed = world.createCollider(
            RAPIER.ColliderDesc.cuboid(1, 1, 1).setTranslation(-8, 0, 0),
        );
        world.step();
        expect(world.colliders._bufferRef.ptr).not.toBe(0);

        body.setTranslation({x: 10, y: 5, z: 0}, true);
        world.propagateModifiedBodyPositionsToColliders();

        // Still live, and describing the propagated pose.
        expect(world.colliders._bufferRef.ptr).not.toBe(0);
        expect(collider.translation().x).toBeCloseTo(11, 5);
        expect(collider.translation().y).toBeCloseTo(5, 5);
        expect(fixed.translation().x).toBeCloseTo(-8, 5);

        // The next step agrees.
        world.step();
        expect(collider.translation().x).toBeCloseTo(11, 4);
        world.free();
    });

    test("is coherent before the first step too", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 5, 0));
        const collider = world.createCollider(
            RAPIER.ColliderDesc.ball(0.5).setTranslation(1, 0, 0),
            body,
        );
        body.setTranslation({x: 10, y: 5, z: 0}, true);
        world.propagateModifiedBodyPositionsToColliders();
        expect(collider.translation().x).toBeCloseTo(11, 5);
        world.step();
        expect(collider.translation().x).toBeCloseTo(11, 4);
        world.free();
    });
});

describe("kinematic bodies", () => {
    // `setNextKinematic*` no longer marks the body pending: the post-step sync
    // must still refresh every driven body through the active set, including
    // when there are many of them among many standalone colliders.
    test("setNextKinematicTranslation is reflected after the step for many bodies", () => {
        const world = new RAPIER.World(GRAVITY);
        const bodies = [];
        for (let i = 0; i < 100; i++) {
            const body = world.createRigidBody(
                RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(i, 0, 0),
            );
            world.createCollider(RAPIER.ColliderDesc.ball(0.25), body);
            bodies.push(body);
        }
        const tiles = [];
        for (let i = 0; i < 300; i++) {
            tiles.push(
                world.createCollider(
                    RAPIER.ColliderDesc.cuboid(0.5, 0.5, 0.5).setTranslation(i, -5, 0),
                ),
            );
        }

        for (let step = 1; step <= 5; step++) {
            for (let i = 0; i < bodies.length; i++) {
                bodies[i].setNextKinematicTranslation({x: i, y: step * 0.1, z: 0});
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
        const params = RAPIER.JointData.spherical({x: 0, y: 0, z: 0}, {x: 0, y: 0, z: 0});
        expect(() => world.createImpulseJoint(params, body, body, true)).toThrow();
        expect(() => world.createMultibodyJoint(params, body, body, true)).toThrow();
        expect(world.impulseJoints.len()).toBe(0);
        world.step();
        world.free();
    });

    test("vehicle axis indices are range-checked", () => {
        const world = new RAPIER.World(GRAVITY);
        const chassis = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        world.createCollider(RAPIER.ColliderDesc.cuboid(1, 0.5, 2), chassis);
        const vehicle = world.createVehicleController(chassis);

        expect(() => (vehicle.indexUpAxis = 3)).toThrow();
        expect(() => (vehicle.indexForwardAxis = -1)).toThrow();
        vehicle.indexUpAxis = 2;
        vehicle.indexForwardAxis = 0;
        expect(vehicle.indexUpAxis).toBe(2);
        expect(vehicle.indexForwardAxis).toBe(0);

        world.removeVehicleController(vehicle);
        world.free();
    });

    test("zero VHACD parameters do not trap", () => {
        // A unit cube.
        const vertices = new Float32Array([
            -1, -1, -1, 1, -1, -1, 1, 1, -1, -1, 1, -1, -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, 1, 1,
        ]);
        const indices = new Uint32Array([
            0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 2, 3, 7, 2, 7, 6, 1, 2, 6, 1, 6,
            5, 0, 4, 7, 0, 7, 3,
        ]);
        expect(() =>
            RAPIER.ColliderDesc.convexDecomposition(vertices, indices, {
                resolution: 0,
                planeDownsampling: 0,
                convexHullDownsampling: 0,
            }),
        ).not.toThrow();
    });

    test("PidController ignores a zero target rotation", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(
            RAPIER.RigidBodyDesc.dynamic().setRotation(QUARTER_TURN_Y),
        );
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
        const pid = world.createPidController(
            10,
            0,
            0,
            RAPIER.PidAxesMask.AngX | RAPIER.PidAxesMask.AngY | RAPIER.PidAxesMask.AngZ,
        );

        pid.applyAngularCorrection(body, ZERO_Q, {x: 0, y: 0, z: 0});
        const angvel = body.angvel();
        expect(Math.hypot(angvel.x, angvel.y, angvel.z)).toBe(0);

        const correction = pid.angularCorrection(body, ZERO_Q, {x: 0, y: 0, z: 0});
        expect(correction).toEqual({x: 0, y: 0, z: 0});

        // A real target still steers.
        pid.applyAngularCorrection(body, IDENTITY, {x: 0, y: 0, z: 0});
        const steered = body.angvel();
        expect(Math.hypot(steered.x, steered.y, steered.z)).toBeGreaterThan(0);

        world.removePidController(pid);
        world.free();
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
        const groundCollider = world.createCollider(
            RAPIER.ColliderDesc.cuboid(20, 0.5, 20),
            ground,
        );
        for (let i = 0; i < 3; i++) {
            const body = world.createRigidBody(
                RAPIER.RigidBodyDesc.dynamic().setTranslation(i * 3, 1, 0),
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

        // A throwing callback is reported once and ends the walk early.
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

describe("physics hooks", () => {
    // The pair's handles reach the hook through the scratch buffer now; a
    // parentless collider must still show up as a `null` body, and the
    // handles must survive the trip exactly.
    test("filter hooks receive the pair's colliders and bodies", () => {
        const world = new RAPIER.World(GRAVITY);
        const ground = world.createCollider(
            RAPIER.ColliderDesc.cuboid(10, 0.5, 10).setActiveHooks(
                RAPIER.ActiveHooks.FILTER_CONTACT_PAIRS |
                    RAPIER.ActiveHooks.FILTER_INTERSECTION_PAIRS,
            ),
        );
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 1, 0));
        const boxCollider = world.createCollider(RAPIER.ColliderDesc.cuboid(0.5, 0.5, 0.5), body);
        const sensorBody = world.createRigidBody(
            RAPIER.RigidBodyDesc.dynamic().setTranslation(3, 0.4, 0),
        );
        const sensor = world.createCollider(
            RAPIER.ColliderDesc.ball(0.5).setSensor(true),
            sensorBody,
        );

        const contactPairs: [number, number, number | null, number | null][] = [];
        const intersectionPairs: [number, number, number | null, number | null][] = [];
        const hooks: RAPIER.PhysicsHooks = {
            filterContactPair: (c1, c2, b1, b2) => {
                contactPairs.push([c1, c2, b1, b2]);
                return RAPIER.SolverFlags.COMPUTE_IMPULSE;
            },
            filterIntersectionPair: (c1, c2, b1, b2) => {
                intersectionPairs.push([c1, c2, b1, b2]);
                return true;
            },
        };
        for (let i = 0; i < 30; i++) world.step(undefined, hooks);

        expect(contactPairs.length).toBeGreaterThan(0);
        for (const [c1, c2, b1, b2] of contactPairs) {
            expect([c1, c2].sort()).toEqual([ground.handle, boxCollider.handle].sort());
            // The parentless ground reports no body; the box reports its own.
            expect(c1 === ground.handle ? b1 : b2).toBeNull();
            expect(c1 === ground.handle ? b2 : b1).toBe(body.handle);
        }
        expect(intersectionPairs.length).toBeGreaterThan(0);
        for (const [c1, c2, b1, b2] of intersectionPairs) {
            expect([c1, c2].sort()).toEqual([ground.handle, sensor.handle].sort());
            expect(c1 === ground.handle ? b1 : b2).toBeNull();
            expect(c1 === ground.handle ? b2 : b1).toBe(sensorBody.handle);
        }
        // The box came to rest on the ground, so the pair was not filtered out.
        expect(body.translation().y).toBeCloseTo(1, 1);
        world.free();
    });
});
