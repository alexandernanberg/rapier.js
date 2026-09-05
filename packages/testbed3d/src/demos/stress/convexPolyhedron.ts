import seedrandom from "seedrandom";
import type {Testbed} from "../../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // Create Ground.
    let groundSize = 200.1;
    let groundHeight = 0.1;
    let groundDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(0.0, -groundHeight, 0.0);
    let ground = world.createRigidBody(groundDesc);
    world.createCollider(RAPIER.ColliderDesc.cuboid(groundSize, groundHeight, groundSize), ground);

    // Five random round convex hulls, reused across every body: this is a
    // collision-detection stress test, not a geometry one, and sharing the five
    // shapes is also what lets the renderer instance them.
    let num = 8;
    let scale = 2.0;
    let rad = 1.0;
    let borderRad = 0.1;

    let shift = borderRad * 2.0 + scale;
    let centerx = shift * (num / 2);
    let centery = shift / 2.0;
    let centerz = shift * (num / 2);

    let offset = -num * shift * 0.5;
    let rng = seedrandom("convex-polyhedron-stress");
    let i, j, k;

    let shapes = [];

    for (i = 0; i < 5; ++i) {
        let points = new Float32Array(10 * 3);

        for (j = 0; j < points.length; ++j) {
            points[j] = rng() * scale;
        }

        shapes.push(RAPIER.ColliderDesc.roundConvexHull(points, borderRad));
    }

    for (j = 0; j < 47; ++j) {
        for (i = 0; i < num; ++i) {
            for (k = 0; k < num; ++k) {
                let x = i * shift - centerx + offset;
                let y = j * shift + centery + 3.0;
                let z = k * shift - centerz + offset;

                let colliderDesc = shapes[(i + k) % 5];

                if (colliderDesc === null) {
                    continue;
                }

                let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z);
                let body = world.createRigidBody(bodyDesc);
                world.createCollider(colliderDesc, body);
            }
        }

        offset -= 0.05 * rad * (num - 1.0);
    }

    testbed.setWorld(world);

    let cameraPosition = {
        eye: {x: 100.0, y: 100.0, z: 100.0},
        target: {x: 0.0, y: 0.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
