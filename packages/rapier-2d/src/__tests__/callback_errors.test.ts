import RAPIER, {init} from "@alexandernanberg/rapier2d/compat";
import {describe, test, expect, beforeAll} from "vitest";

beforeAll(async () => {
    await init();
});

const GRAVITY = {x: 0, y: -9.81};

function stackedBoxes(world: RAPIER.World, count: number): RAPIER.Collider[] {
    const colliders: RAPIER.Collider[] = [];
    for (let i = 0; i < count; ++i) {
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, i));
        colliders.push(world.createCollider(RAPIER.ColliderDesc.cuboid(0.5, 0.5), body));
    }
    return colliders;
}

// A JS exception cannot cross the WASM boundary from inside a callback, so the
// Rust side used to treat a throwing predicate as "include this collider" and a
// throwing hit callback as "keep going", and drop the error. These pin the
// wrapped behaviour: the query stops and the error reaches the caller.
describe("exceptions thrown from query callbacks", () => {
    test("a throwing filter predicate propagates out of castRay", () => {
        const world = new RAPIER.World(GRAVITY);
        stackedBoxes(world, 3);
        world.step();

        const ray = new RAPIER.Ray({x: 0, y: 10}, {x: 0, y: -1});
        expect(() =>
            world.castRay(ray, 100, true, undefined, undefined, undefined, undefined, () => {
                throw new Error("boom");
            }),
        ).toThrow("boom");
        // The world is still usable afterwards.
        expect(world.castRay(ray, 100, true)).not.toBeNull();
        world.free();
    });

    test("a throwing hit callback stops intersectionsWithRay and propagates", () => {
        const world = new RAPIER.World(GRAVITY);
        stackedBoxes(world, 3);
        world.step();

        const ray = new RAPIER.Ray({x: 0, y: 10}, {x: 0, y: -1});
        let calls = 0;
        expect(() =>
            world.intersectionsWithRay(ray, 100, true, () => {
                calls += 1;
                throw new Error("boom");
            }),
        ).toThrow("boom");
        expect(calls).toBe(1);
        world.free();
    });

    test("a throwing callback propagates out of the shape and point queries", () => {
        const world = new RAPIER.World(GRAVITY);
        stackedBoxes(world, 3);
        world.step();

        const fail = () => {
            throw new Error("boom");
        };
        const shape = new RAPIER.Ball(0.1);
        expect(() => world.intersectionsWithShape({x: 0, y: 1}, 0, shape, fail)).toThrow("boom");
        expect(() => world.intersectionsWithPoint({x: 0, y: 1}, fail)).toThrow("boom");
        expect(() =>
            world.collidersWithAabbIntersectingAabb({x: 0, y: 1}, {x: 1, y: 1}, fail),
        ).toThrow("boom");
        expect(() =>
            world.projectPoint(
                {x: 0, y: 1},
                true,
                undefined,
                undefined,
                undefined,
                undefined,
                fail,
            ),
        ).toThrow("boom");
        expect(() =>
            world.castShape(
                {x: 0, y: 10},
                0,
                {x: 0, y: -1},
                shape,
                0,
                100,
                true,
                undefined,
                undefined,
                undefined,
                undefined,
                fail,
            ),
        ).toThrow("boom");
        // Everything above freed its temporaries and released its borrows.
        expect(
            world.castRay(new RAPIER.Ray({x: 0, y: 10}, {x: 0, y: -1}), 100, true),
        ).not.toBeNull();
        world.step();
        world.free();
    });

    test("mutating the world from inside a query callback is reported, not swallowed", () => {
        const world = new RAPIER.World(GRAVITY);
        const colliders = stackedBoxes(world, 3);
        world.step();

        // The query holds a borrow of the collider set; removing a collider from
        // inside the callback is rejected by wasm-bindgen. That rejection used
        // to be dropped, leaving the collider in place with no indication.
        expect(() =>
            world.intersectionsWithShape({x: 0, y: 1}, 0, new RAPIER.Cuboid(5, 5), (collider) => {
                world.removeCollider(collider, true);
                return true;
            }),
        ).toThrow();
        expect(world.colliders.len()).toBe(3);
        for (const collider of colliders) expect(collider.isValid()).toBe(true);
        // Not freed: wasm-bindgen reports the aliasing by unwinding straight
        // through the Rust frames, which leaves the borrows the rejected call
        // had already taken in place. That is the error the user now gets to see.
    });

    test("a throwing predicate propagates out of the character controller", () => {
        const world = new RAPIER.World(GRAVITY);
        stackedBoxes(world, 2);
        const character = world.createRigidBody(
            RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(0, 5),
        );
        const collider = world.createCollider(RAPIER.ColliderDesc.ball(0.5), character);
        world.step();
        const controller = world.createCharacterController(0.01);
        expect(() =>
            controller.computeColliderMovement(
                collider,
                {x: 0, y: -10},
                undefined,
                undefined,
                () => {
                    throw new Error("boom");
                },
            ),
        ).toThrow("boom");
        world.free();
    });

    test("a throwing callback propagates out of contactPairsWith", () => {
        const world = new RAPIER.World(GRAVITY);
        const colliders = stackedBoxes(world, 2);
        world.step();
        expect(() =>
            world.contactPairsWith(colliders[0], () => {
                throw new Error("boom");
            }),
        ).toThrow("boom");
        world.free();
    });
});

describe("exceptions thrown from physics hooks", () => {
    test("a throwing filterContactPair is reported by step, which stays usable", () => {
        const world = new RAPIER.World(GRAVITY);
        const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        world.createCollider(
            RAPIER.ColliderDesc.cuboid(10, 0.5).setActiveHooks(
                RAPIER.ActiveHooks.FILTER_CONTACT_PAIRS,
            ),
            ground,
        );
        const box = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 1));
        world.createCollider(RAPIER.ColliderDesc.cuboid(0.5, 0.5), box);

        let calls = 0;
        const hooks: RAPIER.PhysicsHooks = {
            filterContactPair: () => {
                calls += 1;
                throw new Error("hook boom");
            },
            filterIntersectionPair: () => true,
        };
        expect(() => world.step(undefined, hooks)).toThrow("hook boom");
        // The hook is not called again during the step that failed, and the
        // pair is treated as an ordinary contact rather than filtered out.
        expect(calls).toBe(1);
        expect(box.translation().y).toBeGreaterThan(0.9);

        // Reassigning the hook on the same object is picked up.
        hooks.filterContactPair = () => RAPIER.SolverFlags.COMPUTE_IMPULSE;
        for (let i = 0; i < 60; ++i) world.step(undefined, hooks);
        expect(box.translation().y).toBeCloseTo(1, 1);
        world.free();
    });

    test("a throwing modifySolverContacts is reported by step", () => {
        const world = new RAPIER.World(GRAVITY);
        const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        world.createCollider(
            RAPIER.ColliderDesc.cuboid(10, 0.5).setActiveHooks(
                RAPIER.ActiveHooks.MODIFY_SOLVER_CONTACTS,
            ),
            ground,
        );
        const box = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 1));
        world.createCollider(RAPIER.ColliderDesc.cuboid(0.5, 0.5), box);

        const hooks: RAPIER.PhysicsHooks = {
            filterContactPair: () => RAPIER.SolverFlags.COMPUTE_IMPULSE,
            filterIntersectionPair: () => true,
            modifySolverContacts: () => {
                throw new Error("modify boom");
            },
        };
        const events = new RAPIER.EventQueue(true);
        expect(() => world.step(events, hooks)).toThrow("modify boom");
        // Transforms were still synced for the step that ran.
        expect(box.translation().y).toBeLessThan(1);
        world.step(events);
        world.free();
    });
});

describe("contactPair re-entrancy", () => {
    test("a nested contactPair call does not free the outer manifold", () => {
        const world = new RAPIER.World(GRAVITY);
        const colliders = stackedBoxes(world, 3);
        for (let i = 0; i < 10; ++i) world.step();

        let outerNormalAfterInner: number | null = null;
        let inner = 0;
        world.contactPair(colliders[0], colliders[1], (manifold) => {
            world.contactPair(colliders[1], colliders[2], (innerManifold) => {
                inner += innerManifold.numContacts();
            });
            // The outer manifold must still be readable after the inner walk.
            outerNormalAfterInner = Math.abs(manifold.normal().y);
        });
        expect(inner).toBeGreaterThan(0);
        expect(outerNormalAfterInner).toBeCloseTo(1, 3);
        world.free();
    });
});
