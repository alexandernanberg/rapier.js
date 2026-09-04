import RAPIER, {init} from "@alexandernanberg/rapier2d/compat";
import {describe, test, expect, beforeAll} from "vitest";

const GRAVITY = {x: 0, y: -9.81};

beforeAll(async () => {
    await init();
});

/**
 * The contact-modification hook hands JS a context that points straight at the
 * solver contacts of the manifold being built, so it is only valid for the
 * duration of the call. These cover both directions: what the hook can read out
 * of a manifold, and what the solver does with the edits it writes back.
 */
describe("contact modification", () => {
    function groundAndBall(hooksActive = true) {
        const world = new RAPIER.World(GRAVITY);
        const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        let groundDesc = RAPIER.ColliderDesc.cuboid(20, 0.5);
        if (hooksActive)
            groundDesc = groundDesc.setActiveHooks(RAPIER.ActiveHooks.MODIFY_SOLVER_CONTACTS);
        const groundCollider = world.createCollider(groundDesc, ground);

        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 2));
        const ballCollider = world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        return {world, body, groundCollider, ballCollider};
    }

    test("the hook only runs for colliders that opted in", () => {
        const {world} = groundAndBall(false);
        let calls = 0;
        const hooks: RAPIER.PhysicsHooks = {
            modifySolverContacts() {
                calls += 1;
            },
        };

        for (let i = 0; i < 120; i++) world.step(undefined, hooks);

        expect(calls).toBe(0);
        world.free();
    });

    test("dropping the solver contacts disables the contact response", () => {
        const {world, body} = groundAndBall();
        let calls = 0;
        const hooks: RAPIER.PhysicsHooks = {
            modifySolverContacts(context) {
                calls += 1;
                context.clearSolverContacts();
            },
        };

        for (let i = 0; i < 120; i++) world.step(undefined, hooks);

        expect(calls).toBeGreaterThan(0);
        // Without a contact response the ball falls straight through the ground.
        expect(body.translation().y).toBeLessThan(-1);
        world.free();
    });

    test("the context reports the pair and its contacts while the hook runs", () => {
        const {world, body, groundCollider, ballCollider} = groundAndBall();
        let colliders: [number, number] | null = null;
        let numContacts = 0;
        let normalY = 0;
        let dist = Number.NaN;
        let point: RAPIER.Vector | null = null;

        const hooks: RAPIER.PhysicsHooks = {
            modifySolverContacts(context) {
                colliders = [context.collider1(), context.collider2()];
                numContacts = context.numSolverContacts();
                normalY = context.normal()!.y;
                dist = context.solverContactDist(0);
                point = context.solverContactPoint1(0);
            },
        };

        for (let i = 0; i < 120; i++) world.step(undefined, hooks);

        expect(colliders!.sort()).toEqual([groundCollider.handle, ballCollider.handle].sort());
        expect(numContacts).toBeGreaterThan(0);
        // The pair is (ground, ball) either way round, so the normal is vertical.
        expect(Math.abs(normalY)).toBeCloseTo(1, 1);
        expect(Math.abs(dist)).toBeLessThan(0.1);
        expect(point).not.toBeNull();
        expect(point!.y).toBeCloseTo(body.translation().y - 0.5, 1);

        world.free();
    });

    test("the context is inert outside of a hook call", () => {
        const context = new RAPIER.ContactModificationContext();

        expect(context.isActive()).toBe(false);
        expect(context.numSolverContacts()).toBe(0);
        expect(context.collider1()).toBeUndefined();
        expect(context.solverContactPoint1(0)).toBeNull();
        // Setters are no-ops rather than writes through a dangling pointer.
        context.setFriction(1);
        expect(context.friction()).toBe(0);

        context.free();
    });

    test("manifold user data set by the hook survives to the narrow-phase", () => {
        const {world, groundCollider, ballCollider} = groundAndBall();
        const hooks: RAPIER.PhysicsHooks = {
            modifySolverContacts(context) {
                context.setUserData(context.userData() + 1);
            },
        };

        for (let i = 0; i < 60; i++) world.step(undefined, hooks);

        let userData = 0;
        world.contactPair(groundCollider, ballCollider, (manifold) => {
            userData = manifold.userData();
        });

        expect(userData).toBeGreaterThan(0);
        world.free();
    });

    test("a tangent velocity makes the ground act like a conveyor belt", () => {
        const {world, body} = groundAndBall();
        const hooks: RAPIER.PhysicsHooks = {
            modifySolverContacts(context) {
                for (let i = 0; i < context.numSolverContacts(); i++) {
                    context.setSolverContactTangentVelocity(i, {x: 4, y: 0});
                }
            },
        };

        for (let i = 0; i < 180; i++) world.step(undefined, hooks);

        // The ball lands, is dragged along the belt, and never falls through.
        expect(body.translation().y).toBeGreaterThan(0);
        expect(Math.abs(body.translation().x)).toBeGreaterThan(0.5);
        world.free();
    });

    test("zeroing the friction lets a box keep sliding", () => {
        function slideDistance(friction: number | null): number {
            const world = new RAPIER.World({x: 0, y: -9.81});
            const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
            world.createCollider(
                RAPIER.ColliderDesc.cuboid(50, 0.5)
                    .setFriction(1)
                    .setActiveHooks(RAPIER.ActiveHooks.MODIFY_SOLVER_CONTACTS),
                ground,
            );
            const box = world.createRigidBody(
                RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 1).setLinvel(5, 0),
            );
            world.createCollider(RAPIER.ColliderDesc.cuboid(0.5, 0.5).setFriction(1), box);

            const hooks: RAPIER.PhysicsHooks =
                friction === null
                    ? {}
                    : {
                          modifySolverContacts(context) {
                              context.setFriction(friction);
                          },
                      };

            for (let i = 0; i < 120; i++) world.step(undefined, hooks);
            const x = box.translation().x;
            world.free();
            return x;
        }

        expect(slideDistance(0)).toBeGreaterThan(slideDistance(null) + 1);
    });
});
