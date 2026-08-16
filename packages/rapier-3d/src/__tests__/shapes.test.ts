import RAPIER, {init} from "@alexandernanberg/rapier3d/compat";
import {describe, test, expect, beforeAll} from "vitest";

const GRAVITY = {x: 0, y: -9.81, z: 0};

beforeAll(async () => {
    await init();
});

/**
 * Every shape family builds its raw counterpart differently — several take
 * typed arrays that have to be copied into WASM memory. The suite otherwise
 * only ever creates balls, cuboids and capsules.
 */
describe("shape families", () => {
    test("cylinder and cone colliders round-trip through the shape wrapper", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());

        const cylinder = world.createCollider(RAPIER.ColliderDesc.cylinder(1, 0.5), body);
        expect(cylinder.shape).toBeInstanceOf(RAPIER.Cylinder);

        const cone = world.createCollider(RAPIER.ColliderDesc.cone(1, 0.5), body);
        expect(cone.shape).toBeInstanceOf(RAPIER.Cone);

        world.step();
        world.free();
    });

    test("a convex hull collider round-trips through the shape wrapper", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());

        const desc = RAPIER.ColliderDesc.convexHull(
            new Float32Array([0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1]),
        );
        expect(desc).not.toBeNull();

        const collider = world.createCollider(desc!, body);
        expect(collider.shape).toBeInstanceOf(RAPIER.ConvexPolyhedron);

        world.step();
        world.free();
    });

    test("a trimesh collider round-trips through the shape wrapper", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());

        const collider = world.createCollider(
            RAPIER.ColliderDesc.trimesh(
                new Float32Array([0, 0, 0, 1, 0, 0, 0, 0, 1]),
                new Uint32Array([0, 1, 2]),
            ),
            body,
        );
        expect(collider.shape).toBeInstanceOf(RAPIER.TriMesh);

        world.step();
        world.free();
    });

    test("a heightfield collider round-trips through the shape wrapper", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());

        const collider = world.createCollider(
            RAPIER.ColliderDesc.heightfield(1, 1, new Float32Array([0, 0, 0, 0]), {
                x: 10,
                y: 1,
                z: 10,
            }),
            body,
        );
        expect(collider.shape).toBeInstanceOf(RAPIER.Heightfield);

        world.step();
        world.free();
    });

    test("a ball lands on a trimesh floor", () => {
        const world = new RAPIER.World(GRAVITY);
        const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        world.createCollider(
            RAPIER.ColliderDesc.trimesh(
                new Float32Array([-10, 0, -10, 10, 0, -10, 10, 0, 10, -10, 0, 10]),
                new Uint32Array([0, 1, 2, 0, 2, 3]),
            ),
            ground,
        );

        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 3, 0));
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        for (let i = 0; i < 120; i++) world.step();

        expect(body.translation().y).toBeGreaterThan(0);

        world.free();
    });

    test("mesh builders reject a ragged vertex or index array", () => {
        // A trailing partial vertex/triangle used to reach `Vector::from_slice`
        // through `chunks`, panicking — which surfaces in JS as a bare
        // `RuntimeError: unreachable`. The shape wrappers only reach WASM at
        // `intoRaw()`, so that is where the rejection is visible.
        const ragged = new Float32Array([0, 0, 0, 1, 0, 0, 0, 1, 0, 5]);
        const whole = new Float32Array([0, 0, 0, 1, 0, 0, 0, 1, 0]);
        const raggedIdx = new Uint32Array([0, 1, 2, 0]);
        const wholeIdx = new Uint32Array([0, 1, 2]);

        expect(() => new RAPIER.TriMesh(ragged, wholeIdx).intoRaw()).toThrow();
        expect(() => new RAPIER.TriMesh(whole, raggedIdx).intoRaw()).toThrow();
        expect(() => new RAPIER.ConvexPolyhedron(ragged, null).intoRaw()).toThrow();
        expect(() => new RAPIER.ConvexPolyhedron(ragged, wholeIdx).intoRaw()).toThrow();
        expect(RAPIER.ColliderDesc.convexDecomposition(ragged, wholeIdx)).toBeNull();

        // A well-formed mesh still builds, so the guard is not just rejecting
        // everything.
        const ok = new RAPIER.TriMesh(whole, wholeIdx).intoRaw();
        expect(ok).toBeDefined();
        ok.free();

        // The module is still usable afterwards.
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 5, 0));
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
        world.step();
        expect(body.translation().y).toBeLessThan(5);
        world.free();
    });

    test("mesh builders reject an index that points past the last vertex", () => {
        // Only `TriMesh` validates its indices inside parry. Everything else here
        // indexes the vertex buffer directly while building, so an out-of-range
        // index used to be an out-of-bounds panic inside WASM.
        const vertices = new Float32Array([0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1]);
        const outOfRange = new Uint32Array([0, 1, 9]);

        expect(() => new RAPIER.TriMesh(vertices, outOfRange).intoRaw()).toThrow();
        expect(() => new RAPIER.ConvexPolyhedron(vertices, outOfRange).intoRaw()).toThrow();
        expect(() =>
            new RAPIER.RoundConvexPolyhedron(vertices, outOfRange, 0.1).intoRaw(),
        ).toThrow();
        expect(() => new RAPIER.Polyline(vertices, new Uint32Array([0, 4])).intoRaw()).toThrow();
        expect(RAPIER.ColliderDesc.convexDecomposition(vertices, outOfRange)).toBeNull();

        // An empty index buffer means "line strip", and is still accepted.
        const strip = new RAPIER.Polyline(vertices).intoRaw();
        expect(strip).toBeDefined();
        strip.free();
    });

    test("heightfields reject a grid the heights don't fill", () => {
        // `(nrows + 1) * (ncols + 1)` heights are required; anything else used to
        // trip an `assert_eq!` inside parry's `Array2::new`, and a degenerate grid
        // an `assert!` inside `HeightField::with_flags`.
        const scale = {x: 10, y: 1, z: 10};

        expect(() =>
            new RAPIER.Heightfield(1, 1, new Float32Array([0, 0, 0]), scale).intoRaw(),
        ).toThrow();
        expect(() =>
            new RAPIER.Heightfield(2, 1, new Float32Array([0, 0, 0, 0]), scale).intoRaw(),
        ).toThrow();
        expect(() =>
            new RAPIER.Heightfield(0, 0, new Float32Array([0]), scale).intoRaw(),
        ).toThrow();
        expect(() =>
            new RAPIER.Heightfield(0xffffffff, 1, new Float32Array([0, 0, 0, 0]), scale).intoRaw(),
        ).toThrow();

        const ok = new RAPIER.Heightfield(1, 1, new Float32Array([0, 0, 0, 0]), scale).intoRaw();
        expect(ok).toBeDefined();
        ok.free();
    });

    test("compounds reject composite sub-shapes", () => {
        // Compounds, triangle meshes and polylines are all composite shapes, and
        // parry panics with "Nested composite shapes are not allowed." for each.
        const at = [{x: 0, y: 0, z: 0}];
        const facing = [{x: 0, y: 0, z: 0, w: 1}];
        const vertices = new Float32Array([0, 0, 0, 1, 0, 0, 0, 1, 0]);

        expect(
            () =>
                new RAPIER.Compound([new RAPIER.TriMesh(vertices, new Uint32Array([0, 1, 2]))], at, facing),
        ).toThrow();
        expect(() => new RAPIER.Compound([new RAPIER.Polyline(vertices)], at, facing)).toThrow();
        expect(
            () => new RAPIER.Compound([new RAPIER.Compound([new RAPIER.Ball(1)], at, facing)], at, facing),
        ).toThrow();

        const ok = new RAPIER.Compound([new RAPIER.Ball(1)], at, facing).intoRaw();
        expect(ok).toBeDefined();
        ok.free();
    });
});
