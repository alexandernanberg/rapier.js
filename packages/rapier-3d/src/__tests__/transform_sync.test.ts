import RAPIER, {init, scratch} from "@alexandernanberg/rapier3d/compat";
import {describe, test, expect, beforeAll} from "vitest";
import {_v} from "./_target";

beforeAll(async () => {
    await init();
});

// Handles are f64-encoded arena slots: low 32 bits index, high 32 generation.
const _handleBuf = new Float64Array(1);
const _handleView = new Uint32Array(_handleBuf.buffer);
function handleIndex(handle: number) {
    _handleBuf[0] = handle;
    return _handleView[0];
}

/** Reads a body's translation straight from WASM, bypassing the shared buffer. */
function rawBodyTranslation(world: RAPIER.World, body: RAPIER.RigidBody) {
    world.bodies.raw.rbTranslation(body.handle);
    const s = scratch();
    return {x: s[0], y: s[1], z: s[2]};
}

/** Reads a body's linear velocity straight from WASM. */
function rawBodyLinvel(world: RAPIER.World, body: RAPIER.RigidBody) {
    world.bodies.raw.rbLinvel(body.handle);
    const s = scratch();
    return {x: s[0], y: s[1], z: s[2]};
}

/** Reads a collider's world translation straight from WASM. */
function rawColliderTranslation(world: RAPIER.World, collider: RAPIER.Collider) {
    world.colliders.raw.coTranslation(collider.handle);
    const s = scratch();
    return {x: s[0], y: s[1], z: s[2]};
}

/** Asserts that every buffered transform in the world matches WASM exactly. */
function expectBufferMatchesWasm(world: RAPIER.World) {
    world.bodies.forEach((body) => {
        expect(body.translation(_v())).toEqual(rawBodyTranslation(world, body));
        expect(body.linvel(_v())).toEqual(rawBodyLinvel(world, body));
    });
    world.colliders.forEach((collider) => {
        expect(collider.translation(_v())).toEqual(rawColliderTranslation(world, collider));
    });
}

function createStack(world: RAPIER.World, count: number) {
    const floor = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    world.createCollider(RAPIER.ColliderDesc.cuboid(50, 0.1, 50), floor);

    const bodies: RAPIER.RigidBody[] = [];
    for (let i = 0; i < count; i++) {
        const body = world.createRigidBody(
            RAPIER.RigidBodyDesc.dynamic().setTranslation(i * 1.5 - count, 0.6, 0),
        );
        world.createCollider(RAPIER.ColliderDesc.cuboid(0.5, 0.5, 0.5), body);
        bodies.push(body);
    }
    return bodies;
}

function settle(world: RAPIER.World, steps = 600) {
    for (let i = 0; i < steps; i++) world.step();
}

/**
 * The transform buffers are refreshed incrementally: a step only rewrites the
 * slots of bodies the island manager reports as active, the bodies that were
 * active during the previous step (so a body that just fell asleep still gets
 * its final pose written), and anything JS created or mutated in between. These
 * tests pin the cases where a slot could otherwise be left stale.
 */
describe("incremental transform buffer sync", () => {
    test("a body that falls asleep leaves its final pose in the buffer", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const [body] = createStack(world, 1);

        settle(world);
        expect(body.isSleeping()).toBe(true);
        expectBufferMatchesWasm(world);

        world.free();
    });

    test("sleeping bodies stay in sync while other bodies keep moving", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const bodies = createStack(world, 8);

        settle(world);
        expect(bodies.every((b) => b.isSleeping())).toBe(true);

        // Wake a single body and drop it back down; every other slot must stay
        // valid while only that one island is being refreshed.
        bodies[0].setTranslation({x: bodies[0].translation(_v()).x, y: 8, z: 0}, true);
        for (let i = 0; i < 200; i++) {
            world.step();
            expectBufferMatchesWasm(world);
        }

        world.free();
    });

    test("moving a sleeping body without waking it refreshes body and collider", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const [body] = createStack(world, 1);
        const collider = body.collider(0);

        settle(world);
        expect(body.isSleeping()).toBe(true);

        // Whether rapier decides to wake the body is its business; either way the
        // buffered pose has to follow.
        body.setTranslation({x: 20, y: 3, z: -5}, false);
        world.step();

        expectBufferMatchesWasm(world);
        expect(collider.translation(_v()).x).toBeCloseTo(20, 5);
        expect(collider.translation(_v()).z).toBeCloseTo(-5, 5);

        world.free();
    });

    test("changing a sleeping body's velocity without waking it refreshes the buffer", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const [body] = createStack(world, 1);

        settle(world);
        expect(body.isSleeping()).toBe(true);

        body.setLinvel({x: 3, y: 0, z: 0}, false);
        world.step();

        expect(body.linvel(_v())).toEqual(rawBodyLinvel(world, body));

        world.free();
    });

    test("a collider repositioned on a sleeping body is refreshed", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const [body] = createStack(world, 1);
        const collider = body.collider(0);

        settle(world);
        expect(body.isSleeping()).toBe(true);

        collider.setTranslationWrtParent({x: 0, y: 4, z: 0});
        world.step();

        expectBufferMatchesWasm(world);

        world.free();
    });

    test("a body created next to sleeping ones is written into the buffer", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        createStack(world, 4);
        settle(world);

        const fresh = world.createRigidBody(
            RAPIER.RigidBodyDesc.dynamic().setTranslation(30, 5, 30),
        );
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), fresh);
        world.step();

        expectBufferMatchesWasm(world);

        world.free();
    });

    test("recycled arena indices do not resurrect a removed body's transform", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const bodies = createStack(world, 4);
        settle(world);

        world.removeRigidBody(bodies[2]);
        const fresh = world.createRigidBody(
            RAPIER.RigidBodyDesc.dynamic().setTranslation(-40, 2, 7),
        );
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), fresh);
        world.step();

        expect(fresh.translation(_v()).x).toBeCloseTo(-40, 5);
        expect(fresh.translation(_v()).z).toBeCloseTo(7, 5);
        expectBufferMatchesWasm(world);

        world.free();
    });

    test("a fixed body recycling an active body's slot is written", () => {
        // The refresh list is deduplicated by arena index, but an index outlives
        // the body that held it: the handle of a body that was active last step
        // must not shadow the new body that took over its index. It bites hardest
        // when the newcomer is one the island manager never reports, because then
        // nothing else would ever write its slot.
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const bodies = createStack(world, 6);

        // Keep them falling, so the body removed below is in the active set.
        for (const b of bodies) b.setTranslation({x: b.translation(_v()).x, y: 20, z: 0}, true);
        for (let i = 0; i < 5; i++) world.step();
        expect(bodies.some((b) => !b.isSleeping())).toBe(true);

        const victim = bodies[3];
        const victimIndex = handleIndex(victim.handle);
        world.removeRigidBody(victim);

        const fixed = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(17, 4, -6));
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), fixed);
        // The arena hands out the freed index again; without that the test would
        // not be exercising the collision it is meant to pin.
        expect(handleIndex(fixed.handle)).toBe(victimIndex);

        world.step();

        expect(fixed.translation(_v())).toEqual({x: 17, y: 4, z: -6});
        expectBufferMatchesWasm(world);

        world.free();
    });
});
