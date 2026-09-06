import RAPIER, {init} from "@alexandernanberg/rapier2d/compat";
import {describe, test, expect, beforeAll} from "vitest";

beforeAll(async () => {
    await init();
});

const GRAVITY = {x: 0, y: -9.81};

function groundAndBox(world: RAPIER.World, hooks: RAPIER.ActiveHooks) {
    const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    world.createCollider(RAPIER.ColliderDesc.cuboid(10, 0.5).setActiveHooks(hooks), ground);
    const box = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 1));
    world.createCollider(RAPIER.ColliderDesc.cuboid(0.5, 0.5), box);
    return box;
}

describe("physics hooks", () => {
    // The wrapped hook closes over the hooks object it was built for. Switching
    // to an object that lacks that hook used to leave the previous object's
    // wrapper installed, so the old filter kept running.
    test("a hook absent from a new hooks object no longer runs", () => {
        const world = new RAPIER.World(GRAVITY);
        const box = groundAndBox(world, RAPIER.ActiveHooks.FILTER_CONTACT_PAIRS);

        let callsA = 0;
        const hooksA: RAPIER.PhysicsHooks = {
            // Filters every pair out: the box falls through the ground.
            filterContactPair: () => {
                callsA += 1;
                return null;
            },
            filterIntersectionPair: () => true,
        };
        world.step(undefined, hooksA);
        expect(callsA).toBeGreaterThan(0);
        const callsAfterSwitch = callsA;

        const hooksB: RAPIER.PhysicsHooks = {
            filterIntersectionPair: () => true,
        };
        for (let i = 0; i < 120; ++i) world.step(undefined, hooksB);

        expect(callsA).toBe(callsAfterSwitch);
        // Without a contact filter the box rests on the ground.
        expect(box.translation().y).toBeCloseTo(1, 1);
        world.free();
    });

    test("a modifySolverContacts absent from a new hooks object no longer runs", () => {
        const world = new RAPIER.World(GRAVITY);
        groundAndBox(world, RAPIER.ActiveHooks.MODIFY_SOLVER_CONTACTS);

        let calls = 0;
        const hooksA: RAPIER.PhysicsHooks = {
            filterContactPair: () => RAPIER.SolverFlags.COMPUTE_IMPULSE,
            filterIntersectionPair: () => true,
            modifySolverContacts: () => {
                calls += 1;
            },
        };
        for (let i = 0; i < 5; ++i) world.step(undefined, hooksA);
        expect(calls).toBeGreaterThan(0);
        const before = calls;

        const hooksB: RAPIER.PhysicsHooks = {
            filterContactPair: () => RAPIER.SolverFlags.COMPUTE_IMPULSE,
            filterIntersectionPair: () => true,
        };
        for (let i = 0; i < 5; ++i) world.step(undefined, hooksB);
        expect(calls).toBe(before);
        world.free();
    });
});

describe("coefficient combine rules", () => {
    // `ClampedSum` and `GeometricMean` exist in rapier but used to be folded
    // into `Max` on the way in.
    test("ClampedSum and GeometricMean round-trip", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        const collider = world.createCollider(
            RAPIER.ColliderDesc.ball(0.5)
                .setFrictionCombineRule(RAPIER.CoefficientCombineRule.GeometricMean)
                .setRestitutionCombineRule(RAPIER.CoefficientCombineRule.ClampedSum),
            body,
        );
        expect(collider.frictionCombineRule()).toBe(RAPIER.CoefficientCombineRule.GeometricMean);
        expect(collider.restitutionCombineRule()).toBe(RAPIER.CoefficientCombineRule.ClampedSum);

        collider.setFrictionCombineRule(RAPIER.CoefficientCombineRule.ClampedSum);
        collider.setRestitutionCombineRule(RAPIER.CoefficientCombineRule.GeometricMean);
        expect(collider.frictionCombineRule()).toBe(RAPIER.CoefficientCombineRule.ClampedSum);
        expect(collider.restitutionCombineRule()).toBe(RAPIER.CoefficientCombineRule.GeometricMean);
        world.free();
    });
});

// The `World` wrappers already guarded their callbacks; the same enumerations
// reached directly through `world.broadPhase` / `world.narrowPhase` handed the
// user's callback to WASM unguarded, where a throw was swallowed at the
// boundary and the walk went on.
describe("exceptions thrown from callbacks passed to the phases directly", () => {
    function overlappingBalls(world: RAPIER.World, count: number) {
        const colliders: RAPIER.Collider[] = [];
        for (let i = 0; i < count; ++i) {
            const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
            colliders.push(world.createCollider(RAPIER.ColliderDesc.ball(1), body));
        }
        world.step();
        return colliders;
    }

    test("BroadPhase.intersectionsWithPoint stops and propagates", () => {
        const world = new RAPIER.World(GRAVITY);
        overlappingBalls(world, 3);
        let calls = 0;
        expect(() =>
            world.broadPhase.intersectionsWithPoint(
                world.narrowPhase,
                world.bodies,
                world.colliders,
                {x: 0, y: 0},
                () => {
                    calls += 1;
                    throw new Error("point boom");
                },
            ),
        ).toThrow("point boom");
        expect(calls).toBe(1);
        world.free();
    });

    test("BroadPhase.intersectionsWithShape stops and propagates", () => {
        const world = new RAPIER.World(GRAVITY);
        overlappingBalls(world, 3);
        let calls = 0;
        expect(() =>
            world.broadPhase.intersectionsWithShape(
                world.narrowPhase,
                world.bodies,
                world.colliders,
                {x: 0, y: 0},
                0,
                new RAPIER.Ball(0.5),
                () => {
                    calls += 1;
                    throw new Error("shape boom");
                },
            ),
        ).toThrow("shape boom");
        expect(calls).toBe(1);
        world.free();
    });

    test("BroadPhase.collidersWithAabbIntersectingAabb stops and propagates", () => {
        const world = new RAPIER.World(GRAVITY);
        overlappingBalls(world, 3);
        let calls = 0;
        expect(() =>
            world.broadPhase.collidersWithAabbIntersectingAabb(
                world.narrowPhase,
                world.bodies,
                world.colliders,
                {x: 0, y: 0},
                {x: 1, y: 1},
                () => {
                    calls += 1;
                    throw new Error("aabb boom");
                },
            ),
        ).toThrow("aabb boom");
        expect(calls).toBe(1);
        world.free();
    });

    test("NarrowPhase.contactPairsWith stops and propagates", () => {
        const world = new RAPIER.World(GRAVITY);
        const colliders = overlappingBalls(world, 3);
        let calls = 0;
        expect(() =>
            world.narrowPhase.contactPairsWith(colliders[0].handle, () => {
                calls += 1;
                throw new Error("pair boom");
            }),
        ).toThrow("pair boom");
        expect(calls).toBe(1);
        // The walk is usable afterwards.
        let seen = 0;
        world.narrowPhase.contactPairsWith(colliders[0].handle, () => {
            seen += 1;
        });
        expect(seen).toBe(2);
        world.free();
    });

    test("a void callback's return value cannot end the walk", () => {
        const world = new RAPIER.World(GRAVITY);
        const colliders = overlappingBalls(world, 3);
        const handles = new Set<number>();
        world.contactPairsWith(colliders[0], (other) => {
            // `Set.add` returns the set, `Set.delete` a boolean: neither may stop it.
            handles.add(other.handle);
            handles.delete(-1);
        });
        expect(handles.size).toBe(2);
        world.free();
    });
});

describe("shape-specific collider getters", () => {
    test("return null for a collider of another shape", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        const ball = world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
        const box = world.createCollider(RAPIER.ColliderDesc.cuboid(1, 1), body);

        expect(ball.radius()).toBeCloseTo(0.5, 6);
        expect(ball.halfHeight()).toBeNull();
        expect(ball.roundRadius()).toBeNull();
        expect(ball.vertices()).toBeNull();
        expect(ball.heightfieldHeights()).toBeNull();
        expect(box.radius()).toBeNull();
        world.free();
    });
});

// The event queue now is the event handler: the step writes each event straight
// into the drain buffer. These pin the drain semantics the channel used to give.
describe("event queue without auto-drain", () => {
    function fallingBall() {
        const world = new RAPIER.World(GRAVITY);
        const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        world.createCollider(
            RAPIER.ColliderDesc.cuboid(10, 0.5).setActiveEvents(
                RAPIER.ActiveEvents.COLLISION_EVENTS,
            ),
            ground,
        );
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 1.2));
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
        return {world, body};
    }

    function drain(queue: RAPIER.EventQueue): boolean[] {
        const started: boolean[] = [];
        queue.drainCollisionEvents((_h1, _h2, s) => started.push(s));
        return started;
    }

    test("events accumulate across steps and a drain empties the queue", () => {
        const {world, body} = fallingBall();
        const queue = new RAPIER.EventQueue(false);

        for (let i = 0; i < 30; ++i) world.step(queue);
        expect(drain(queue)).toEqual([true]);
        // Drained: a second drain finds nothing, and so does one after quiet steps.
        expect(drain(queue)).toEqual([]);
        for (let i = 0; i < 5; ++i) world.step(queue);
        expect(drain(queue)).toEqual([]);

        // A new event after a drain starts a fresh batch rather than re-delivering
        // the old one.
        body.setTranslation({x: 0, y: 10}, true);
        world.step(queue);
        expect(drain(queue)).toEqual([false]);
        world.free();
    });

    test("a handler that grows WASM memory inside the drain sees the whole batch", () => {
        const {world} = fallingBall();
        const queue = new RAPIER.EventQueue(false);
        for (let i = 0; i < 30; ++i) world.step(queue);
        let seen = 0;
        queue.drainCollisionEvents(() => {
            seen += 1;
            // Grows the WASM heap and rewrites the event buffer while the walk
            // is still going.
            for (let i = 0; i < 200; ++i) {
                world.createCollider(RAPIER.ColliderDesc.ball(0.1).setTranslation(50 + i, 50));
            }
        });
        expect(seen).toBe(1);
        world.free();
    });
});
