import RAPIER, {init} from "@alexandernanberg/rapier2d/compat";
import {describe, test, expect, beforeAll} from "vitest";

beforeAll(async () => {
    await init();
});

const GRAVITY = {x: 0, y: -9.81};
const IDENTITY = 0;
const QUARTER_TURN = Math.PI / 2;

describe("setTransform", () => {
    // Rapier only wakes a body from inside its "this component changed" branch,
    // so the wake-up used to be dropped whenever the component it rode on was
    // unchanged.
    test("wakes a sleeping body when only the rotation changes", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(
            RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 5).setSleeping(true),
        );
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
        expect(body.isSleeping()).toBe(true);

        body.setTransform(body.translation(), QUARTER_TURN, true);
        expect(body.isSleeping()).toBe(false);
        expect(body.rotation()).toBeCloseTo(QUARTER_TURN, 5);
        world.free();
    });

    test("wakes a sleeping body when only the translation changes", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(
            RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 5).setSleeping(true),
        );
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
        expect(body.isSleeping()).toBe(true);

        body.setTransform({x: 1, y: 5}, IDENTITY, true);
        expect(body.isSleeping()).toBe(false);
        expect(body.translation().x).toBeCloseTo(1);
        world.free();
    });
});

describe("invalid input is rejected instead of trapping the module", () => {
    test("a joint attached to a removed body throws at creation", () => {
        const world = new RAPIER.World(GRAVITY);
        const a = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        const b = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), b);
        world.removeRigidBody(a);

        const params = RAPIER.JointData.fixed({x: 0, y: 0}, IDENTITY, {x: 0, y: 0}, IDENTITY);
        // Used to be accepted and then trap the module from inside the next step.
        expect(() => world.createImpulseJoint(params, a, b, true)).toThrow(/removed/);
        expect(() => world.createMultibodyJoint(params, a, b, true)).toThrow(/removed/);
        expect(world.impulseJoints.len()).toBe(0);
        expect(world.multibodyJoints.len()).toBe(0);

        world.step();
        expect(b.translation().y).toBeLessThan(0);
        world.free();
    });

    test("a collider descriptor with an unknown mass-properties mode throws", () => {
        const world = new RAPIER.World(GRAVITY);
        const desc = RAPIER.ColliderDesc.ball(0.5);
        (desc as unknown as {massPropsMode: number}).massPropsMode = 7;
        expect(() => world.createCollider(desc)).toThrow(/mass-properties/);
        expect(world.colliders.len()).toBe(0);
        world.step();
        world.free();
    });

    test("ragged voxel data throws", () => {
        const world = new RAPIER.World(GRAVITY);
        const size = {x: 1, y: 1};
        expect(() =>
            world.createCollider(RAPIER.ColliderDesc.voxels(new Int32Array([0, 0, 1]), size)),
        ).toThrow(/grid coordinates/);
        expect(() =>
            world.createCollider(RAPIER.ColliderDesc.voxels(new Float32Array([0, 0, 1]), size)),
        ).toThrow(/points/);
        // Well-formed data still works.
        const ok = world.createCollider(
            RAPIER.ColliderDesc.voxels(new Int32Array([0, 0, 1, 0]), size),
        );
        expect(ok.isValid()).toBe(true);
        world.free();
    });
});

describe("character controller impulses", () => {
    test("the body rapier pushes reports its new velocity right away, even when it is not the hit one", () => {
        const world = new RAPIER.World({x: 0, y: 0});
        const character = world.createRigidBody(RAPIER.RigidBodyDesc.kinematicPositionBased());
        const characterCollider = world.createCollider(
            RAPIER.ColliderDesc.cuboid(1, 0.5),
            character,
        );
        // Two boxes in front of the character, both within the controller's
        // contact prediction margin once it stops at the nearer one. The
        // shape-cast reports the nearer box, but rapier resolves the push with a
        // contact query over the character's whole neighbourhood, and which body
        // ends up receiving the impulse depends on its traversal order, not on
        // which one was hit.
        const boxes = [
            [1.65, 0.25],
            [1.69, -0.25],
        ].map(([x, y]) => {
            const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y));
            world.createCollider(RAPIER.ColliderDesc.cuboid(0.5, 0.25).setDensity(1), body);
            return body;
        });
        // Settle: the buffer is live from here on, which is the path under test.
        world.step();
        for (const box of boxes) expect(box.linvel().x).toBeCloseTo(0, 3);

        const controller = world.createCharacterController(0.01);
        controller.setApplyImpulsesToDynamicBodies(true);
        controller.setCharacterMass(50);
        controller.computeColliderMovement(characterCollider, {x: 1, y: 0});
        expect(controller.numComputedCollisions()).toBeGreaterThan(0);

        // Whichever body was pushed, the buffered velocity must agree with the
        // one WASM holds (`velocityAtPoint` at the center of mass is the linear
        // velocity, read through WASM).
        let pushed = 0;
        for (const box of boxes) {
            const fromWasm = box.velocityAtPoint(box.worldCom()).x;
            expect(box.linvel().x).toBeCloseTo(fromWasm, 4);
            if (fromWasm > 0) pushed += 1;
        }
        expect(pushed).toBeGreaterThan(0);
        world.free();
    });
});

describe("transform sync with few bodies and many colliders", () => {
    // The "most bodies moved, walk everything" shortcut used to be taken on the
    // body count alone, forcing a full collider pass every step; the decision now
    // accounts for the colliders, so these exercise the incremental path.
    test("standalone and JS-moved colliders stay correct", () => {
        const world = new RAPIER.World(GRAVITY);
        const standalone: RAPIER.Collider[] = [];
        for (let i = 0; i < 200; ++i) {
            standalone.push(
                world.createCollider(RAPIER.ColliderDesc.ball(0.1).setTranslation(i, 0)),
            );
        }
        const bodies: RAPIER.RigidBody[] = [];
        for (let i = 0; i < 4; ++i) {
            const body = world.createRigidBody(
                RAPIER.RigidBodyDesc.dynamic().setTranslation(i, 10),
            );
            world.createCollider(RAPIER.ColliderDesc.ball(0.1), body);
            bodies.push(body);
        }

        for (let step = 0; step < 3; ++step) world.step();
        for (let i = 0; i < 200; ++i) {
            expect(standalone[i].translation().x).toBeCloseTo(i, 5);
            expect(standalone[i].translation().y).toBeCloseTo(0, 5);
        }
        const fallen = bodies[0].translation().y;
        expect(fallen).toBeLessThan(10);
        for (const body of bodies) expect(body.translation().y).toBeCloseTo(fallen, 5);

        // Mutations from JS between steps land in the buffer too.
        standalone[7].setTranslation({x: 500, y: 1});
        bodies[2].setTranslation({x: 42, y: 42}, true);
        // Untracked collider setters must not disturb the pose bookkeeping.
        for (const collider of standalone) collider.setFriction(0.3);
        world.step();
        expect(standalone[7].translation()).toEqual({x: 500, y: 1});
        expect(standalone[8].translation().x).toBeCloseTo(8, 5);
        expect(bodies[2].translation().x).toBeCloseTo(42, 5);
        expect(bodies[2].translation().y).toBeLessThan(42);
        expect(bodies[2].collider(0)!.translation().x).toBeCloseTo(42, 5);
        expect(bodies[2].collider(0)!.translation().y).toBeCloseTo(bodies[2].translation().y, 5);
        world.free();
    });

    test("a world with colliders but no bodies keeps their transforms", () => {
        const world = new RAPIER.World(GRAVITY);
        const colliders: RAPIER.Collider[] = [];
        for (let i = 0; i < 50; ++i) {
            colliders.push(
                world.createCollider(RAPIER.ColliderDesc.cuboid(0.5, 0.5).setTranslation(0, i)),
            );
        }
        world.step();
        colliders[3].setTranslation({x: 1, y: 2});
        world.step();
        for (let i = 0; i < 50; ++i) {
            const expected = i === 3 ? {x: 1, y: 2} : {x: 0, y: i};
            expect(colliders[i].translation()).toEqual(expected);
        }
        world.free();
    });

    test("a body woken up by hand moves and reads back correctly", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(
            RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 5).setSleeping(true),
        );
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
        world.step();
        expect(body.translation().y).toBeCloseTo(5);
        body.wakeUp();
        expect(body.isSleeping()).toBe(false);
        world.step();
        expect(body.translation().y).toBeLessThan(5);
        expect(body.collider(0)!.translation().y).toBeCloseTo(body.translation().y, 5);
        world.free();
    });
});
