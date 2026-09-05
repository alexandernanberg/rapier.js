import type RAPIER from "@alexandernanberg/rapier3d";
import type {Testbed} from "../../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

export const GROUND_SIZE = {x: 200.0, y: 1.0, z: 200.0};
export const NSUBDIVS = 20;

/**
 * A ridged bowl: sinusoidal in the middle, walled at the border so nothing falls
 * off the edge. Column-major, like the matrix rapier expects.
 */
export function generateHeightfield() {
    let heights = [];
    let i, j;

    for (j = 0; j <= NSUBDIVS; ++j) {
        for (i = 0; i <= NSUBDIVS; ++i) {
            if (i == 0 || i == NSUBDIVS || j == 0 || j == NSUBDIVS) {
                heights.push(10.0);
            } else {
                let x = (i * GROUND_SIZE.x) / NSUBDIVS;
                let z = (j * GROUND_SIZE.z) / NSUBDIVS;
                heights.push(Math.sin(x) + Math.cos(z));
            }
        }
    }

    return new Float32Array(heights);
}

/** The alternating cubes and balls both this and the `trimesh` stress test drop. */
export function createFallingShapes(RAPIER: RAPIER_API, world: RAPIER.World) {
    let num = 8;
    let rad = 1.0;

    let shift = rad * 2.0 + rad;
    let centerx = shift * (num / 2);
    let centery = shift / 2.0;
    let centerz = shift * (num / 2);
    let i, j, k;

    for (j = 0; j < 47; ++j) {
        for (i = 0; i < num; ++i) {
            for (k = 0; k < num; ++k) {
                let x = i * shift - centerx;
                let y = j * shift + centery + 3.0;
                let z = k * shift - centerz;

                let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z);
                let body = world.createRigidBody(bodyDesc);
                let colliderDesc =
                    j % 2 == 0
                        ? RAPIER.ColliderDesc.cuboid(rad, rad, rad)
                        : RAPIER.ColliderDesc.ball(rad);
                world.createCollider(colliderDesc, body);
            }
        }
    }
}

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    let ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    world.createCollider(
        RAPIER.ColliderDesc.heightfield(
            NSUBDIVS,
            NSUBDIVS,
            generateHeightfield(),
            new RAPIER.Vector3(GROUND_SIZE.x, GROUND_SIZE.y, GROUND_SIZE.z),
        ),
        ground,
    );

    createFallingShapes(RAPIER, world);

    testbed.setWorld(world);

    let cameraPosition = {
        eye: {x: 100.0, y: 100.0, z: 100.0},
        target: {x: 0.0, y: 0.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
