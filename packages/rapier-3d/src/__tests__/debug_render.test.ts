import RAPIER, {init} from "@alexandernanberg/rapier3d/compat";
import {beforeAll, describe, expect, test} from "vitest";

const GRAVITY = {x: 0, y: -9.81, z: 0};

beforeAll(async () => {
    await init();
});

function sceneWith(cuboids: number) {
    const world = new RAPIER.World(GRAVITY);
    for (let i = 0; i < cuboids; i++) {
        const body = world.createRigidBody(
            RAPIER.RigidBodyDesc.fixed().setTranslation(i * 3, 0, 0),
        );
        world.createCollider(RAPIER.ColliderDesc.cuboid(0.5, 0.5, 0.5), body);
    }
    world.step();
    return world;
}

/**
 * The debug renderer builds its lines inside WASM memory and JS reads them
 * through a view, rather than having Rust allocate a JS array and copy into it.
 * The buffer is rebuilt (and may be moved) by every render, so the view handling
 * — and the optional reusable target that copies out of it — is worth pinning
 * down.
 */
describe("debug render buffers", () => {
    test("produces lines with one RGBA color per vertex", () => {
        const world = sceneWith(2);
        const buffers = world.debugRender();

        expect(buffers.vertices.length).toBeGreaterThan(0);
        // Three floats per vertex, four color components per vertex.
        expect(buffers.vertices.length % 3).toBe(0);
        expect(buffers.colors.length).toBe((buffers.vertices.length / 3) * 4);

        world.free();
    });

    test("the copy matches the WASM-resident view it came from", () => {
        const world = sceneWith(2);
        const buffers = world.debugRender();

        // The pipeline's own properties are views straight into WASM memory, and
        // must describe exactly what was copied out.
        const vertices = world.debugRenderPipeline.vertices;
        const colors = world.debugRenderPipeline.colors;
        expect(Array.from(vertices)).toEqual(Array.from(buffers.vertices));
        expect(Array.from(colors)).toEqual(Array.from(buffers.colors));

        world.free();
    });

    test("without a target, each call allocates a fresh pair", () => {
        const world = sceneWith(2);

        const first = world.debugRender();
        const second = world.debugRender();
        expect(second).not.toBe(first);
        expect(second.vertices).not.toBe(first.vertices);
        // The scene did not move, so the contents still agree.
        expect(Array.from(second.vertices)).toEqual(Array.from(first.vertices));

        world.free();
    });

    test("a reused target keeps the same arrays across frames", () => {
        const world = sceneWith(2);

        const target = new RAPIER.DebugRenderBuffers();
        expect(world.debugRender(undefined, undefined, target)).toBe(target);

        const vertices = target.vertices;
        const colors = target.colors;
        const contents = Array.from(vertices);

        world.step();
        expect(world.debugRender(undefined, undefined, target)).toBe(target);

        // Same collider set, same line count: nothing was reallocated.
        expect(target.vertices).toBe(vertices);
        expect(target.colors).toBe(colors);
        expect(Array.from(target.vertices)).toEqual(contents);

        world.free();
    });

    test("a target grows for a bigger scene and reports the smaller length again", () => {
        const world = sceneWith(1);

        const target = new RAPIER.DebugRenderBuffers();
        world.debugRender(undefined, undefined, target);
        const small = target.vertices.length;
        expect(small).toBeGreaterThan(0);

        const extra = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(5, 0, 0));
        world.createCollider(RAPIER.ColliderDesc.cuboid(0.5, 0.5, 0.5), extra);
        world.step();

        world.debugRender(undefined, undefined, target);
        const big = target.vertices.length;
        expect(big).toBeGreaterThan(small);

        world.removeRigidBody(extra);
        world.step();

        world.debugRender(undefined, undefined, target);
        // The backing store stays big, but the exposed window shrinks back, so a
        // consumer never sees the previous frame's leftover lines.
        expect(target.vertices.length).toBe(small);
        expect(target.colors.length).toBe((small / 3) * 4);

        world.free();
    });

    test("an empty world renders nothing", () => {
        const world = new RAPIER.World(GRAVITY);

        const buffers = world.debugRender();
        expect(buffers.vertices.length).toBe(0);
        expect(buffers.colors.length).toBe(0);
        expect(world.debugRenderPipeline.vertices.length).toBe(0);
        expect(world.debugRenderPipeline.colors.length).toBe(0);

        world.free();
    });

    test("a detached view is re-attached rather than read as empty", () => {
        const world = sceneWith(2);
        const expected = Array.from(world.debugRender().vertices);

        // WASM `memory.grow()` detaches every view onto the linear memory, leaving
        // a zero-length one. It can't be triggered on demand, so install a detached
        // view to reproduce the exact state the getter has to handle.
        const detached = new Float32Array(16);
        (detached.buffer as ArrayBuffer & {transfer(): ArrayBuffer}).transfer();
        expect(detached.byteLength).toBe(0);
        world.debugRenderPipeline._vertexRef.buffer = detached;

        // The buffer contents survived the growth, so the view is simply rebuilt.
        expect(Array.from(world.debugRenderPipeline.vertices)).toEqual(expected);

        world.free();
    });
});
