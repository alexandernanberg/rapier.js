import RAPIER, {init} from "@alexandernanberg/rapier2d/compat";
import {beforeAll, describe, expect, test} from "vitest";

const GRAVITY = {x: 0, y: -9.81};

beforeAll(async () => {
    await init();
});

/**
 * Active-body iteration publishes every handle into a WASM-side buffer and walks
 * it from JS, rather than calling the closure once per body from Rust. Handles
 * ride through two `f32` slots, so they have to come back bit-exact, and the walk
 * has to stay correct when the closure calls back into WASM.
 */
describe("active rigid-body iteration", () => {
    function fallingScene(bodies: number) {
        const world = new RAPIER.World(GRAVITY);
        const handles: number[] = [];

        for (let i = 0; i < bodies; i++) {
            const body = world.createRigidBody(
                RAPIER.RigidBodyDesc.dynamic().setTranslation(i * 2, 10),
            );
            world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
            handles.push(body.handle);
        }

        world.step();
        return {world, handles};
    }

    test("yields every awake body exactly once, under its own handle", () => {
        const {world, handles} = fallingScene(5);

        const seen: number[] = [];
        world.islands.forEachActiveRigidBodyHandle((handle) => seen.push(handle));

        expect(seen.length).toBe(handles.length);
        expect([...seen].sort()).toEqual([...handles].sort());
        // A handle packs an arena index and a generation into an f64; both halves
        // have to survive, so every one must resolve back to its body.
        for (const handle of seen) {
            expect(world.getRigidBody(handle)).not.toBeNull();
        }

        world.free();
    });

    test("a recycled arena slot keeps its generation through the buffer", () => {
        const world = new RAPIER.World(GRAVITY);

        // Freeing a body and creating another recycles the arena slot with a bumped
        // generation, so the handle's high 32 bits stop being zero. That half rides
        // in its own `f32` slot, and dropping it would still yield a handle that
        // resolves — only to the wrong incarnation — so compare the handles exactly.
        const first = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 10));
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), first);
        world.removeRigidBody(first);

        const recycled = world.createRigidBody(
            RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 10),
        );
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), recycled);
        expect(recycled.handle).not.toBe(first.handle);
        world.step();

        const seen: number[] = [];
        world.islands.forEachActiveRigidBodyHandle((handle) => seen.push(handle));
        expect(seen).toEqual([recycled.handle]);

        world.free();
    });

    test("an empty world yields nothing", () => {
        const world = new RAPIER.World(GRAVITY);
        world.step();

        let calls = 0;
        world.islands.forEachActiveRigidBodyHandle(() => calls++);
        expect(calls).toBe(0);

        world.free();
    });

    test("sleeping bodies drop out of the iteration", () => {
        const world = new RAPIER.World(GRAVITY);
        const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        world.createCollider(RAPIER.ColliderDesc.cuboid(20, 0.5), ground);

        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 1));
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        // Long enough for the ball to land and the island to fall asleep.
        for (let i = 0; i < 400; i++) world.step();
        expect(body.isSleeping()).toBe(true);

        let calls = 0;
        world.islands.forEachActiveRigidBodyHandle(() => calls++);
        expect(calls).toBe(0);

        // Waking it puts it back in the list.
        body.wakeUp();
        world.step();
        const awake: number[] = [];
        world.islands.forEachActiveRigidBodyHandle((handle) => awake.push(handle));
        expect(awake).toContain(body.handle);

        world.free();
    });

    test("a closure calling back into WASM still sees the remaining bodies", () => {
        const {world, handles} = fallingScene(6);

        const seen: number[] = [];
        world.islands.forEachActiveRigidBodyHandle((handle) => {
            seen.push(handle);
            // Allocating in WASM can grow the linear memory, which detaches every
            // view onto it; the rest of the walk must still read correctly.
            const extra = world.createRigidBody(
                RAPIER.RigidBodyDesc.fixed().setTranslation(0, -100),
            );
            world.createCollider(RAPIER.ColliderDesc.ball(0.1), extra);
            expect(world.getRigidBody(handle)).not.toBeNull();
        });

        expect([...seen].sort()).toEqual([...handles].sort());

        world.free();
    });

    test("World.forEachActiveRigidBody resolves each handle to its body", () => {
        const {world, handles} = fallingScene(4);

        const seen: number[] = [];
        world.forEachActiveRigidBody((body) => seen.push(body.handle));
        expect([...seen].sort()).toEqual([...handles].sort());

        world.free();
    });
});
