import type * as RAPIER_NS from "@alexandernanberg/rapier3d";
import seedrandom from "seedrandom";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof RAPIER_NS;

function generateTriMesh(nsubdivs: number, wx: number, wy: number, wz: number) {
    const vertices = [];
    const indices = [];

    const elementWidth = 1.0 / nsubdivs;
    const rng = seedrandom("trimesh");

    let i, j;
    for (i = 0; i <= nsubdivs; ++i) {
        for (j = 0; j <= nsubdivs; ++j) {
            const x = (j * elementWidth - 0.5) * wx;
            const y = rng() * wy;
            const z = (i * elementWidth - 0.5) * wz;

            vertices.push(x, y, z);
        }
    }

    for (i = 0; i < nsubdivs; ++i) {
        for (j = 0; j < nsubdivs; ++j) {
            const i1 = (i + 0) * (nsubdivs + 1) + (j + 0);
            const i2 = (i + 0) * (nsubdivs + 1) + (j + 1);
            const i3 = (i + 1) * (nsubdivs + 1) + (j + 0);
            const i4 = (i + 1) * (nsubdivs + 1) + (j + 1);

            indices.push(i1, i3, i2);
            indices.push(i3, i4, i2);
        }
    }

    return {
        vertices: new Float32Array(vertices),
        indices: new Uint32Array(indices),
    };
}

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    const gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    const world = new RAPIER.World(gravity);

    // Create Ground.
    let bodyDesc = RAPIER.RigidBodyDesc.fixed();
    let body = world.createRigidBody(bodyDesc);
    const trimesh = generateTriMesh(20, 40.0, 4.0, 40.0);
    const colliderDesc = RAPIER.ColliderDesc.trimesh(trimesh.vertices, trimesh.indices);
    world.createCollider(colliderDesc, body);

    /*
     * Create the polyhedra
     */
    const num = 5;
    const scale = 2.0;
    const border_rad = 0.1;

    const shift = border_rad * 2.0 + scale;
    const centerx = shift * (num / 2);
    const centery = shift / 2.0;
    const centerz = shift * (num / 2);

    const rng = seedrandom("convexPolyhedron");
    let i, j, k, l;

    for (j = 0; j < 15; ++j) {
        for (i = 0; i < num; ++i) {
            for (k = 0; k < num; ++k) {
                const x = i * shift - centerx;
                const y = j * shift + centery + 3.0;
                const z = k * shift - centerz;

                const vertices = [];
                for (l = 0; l < 10; ++l) {
                    vertices.push(rng() * scale, rng() * scale, rng() * scale);
                }
                const v = new Float32Array(vertices);

                // Build the rigid body.
                bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z);
                body = world.createRigidBody(bodyDesc);
                // `roundConvexHull` returns null if the random point set is degenerate.
                const hullDesc = RAPIER.ColliderDesc.roundConvexHull(v, border_rad);
                if (hullDesc !== null) {
                    world.createCollider(hullDesc, body);
                }
            }
        }
    }

    testbed.setWorld(world);

    const cameraPosition = {
        eye: {
            x: -88.48024008669711,
            y: 46.911325612198354,
            z: 83.56055570254844,
        },
        target: {x: 0.0, y: 0.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
