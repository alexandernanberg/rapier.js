import type * as RAPIER_NS from "@alexandernanberg/rapier3d";
import seedrandom from "seedrandom";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof RAPIER_NS;

function generateHeightfield(nsubdivs: number) {
    const heights = [];

    const rng = seedrandom("heightfield");

    let i, j;
    for (i = 0; i <= nsubdivs; ++i) {
        for (j = 0; j <= nsubdivs; ++j) {
            heights.push(rng());
        }
    }

    return new Float32Array(heights);
}

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    const gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    const world = new RAPIER.World(gravity);

    // Create Ground.
    const nsubdivs = 20;
    const scale = new RAPIER.Vector3(70.0, 4.0, 70.0);
    const groundBodyDesc = RAPIER.RigidBodyDesc.fixed();
    const groundBody = world.createRigidBody(groundBodyDesc);
    const heights = generateHeightfield(nsubdivs);
    const groundColliderDesc = RAPIER.ColliderDesc.heightfield(nsubdivs, nsubdivs, heights, scale);
    world.createCollider(groundColliderDesc, groundBody);

    // Dynamic cubes.
    const num = 4;
    const numy = 10;
    const rad = 1.0;

    const shift = rad * 2.0 + rad;
    const centery = shift / 2.0;

    let offset = -num * (rad * 2.0 + rad) * 0.5;
    let i, j, k;

    for (j = 0; j < numy; ++j) {
        for (i = 0; i < num; ++i) {
            for (k = 0; k < num; ++k) {
                const x = i * shift + offset;
                const y = j * shift + centery + 3.0;
                const z = k * shift + offset;

                // Create dynamic cube.
                const bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z);
                const body = world.createRigidBody(bodyDesc);
                let colliderDesc;

                switch (j % 5) {
                    case 0:
                        colliderDesc = RAPIER.ColliderDesc.cuboid(rad, rad, rad);
                        break;
                    case 1:
                        colliderDesc = RAPIER.ColliderDesc.ball(rad);
                        break;
                    case 2:
                        colliderDesc = RAPIER.ColliderDesc.roundCylinder(rad, rad, rad / 10.0);
                        break;
                    case 3:
                        colliderDesc = RAPIER.ColliderDesc.cone(rad, rad);
                        break;
                    // `j % 5` is always 0..4 — using `default` for the last case
                    // lets the compiler see `colliderDesc` is always assigned.
                    default:
                        colliderDesc = RAPIER.ColliderDesc.cuboid(rad / 2.0, rad / 2.0, rad / 2.0);
                        world.createCollider(colliderDesc, body);
                        colliderDesc = RAPIER.ColliderDesc.cuboid(
                            rad / 2.0,
                            rad,
                            rad / 2.0,
                        ).setTranslation(rad, 0.0, 0.0);
                        world.createCollider(colliderDesc, body);
                        colliderDesc = RAPIER.ColliderDesc.cuboid(
                            rad / 2.0,
                            rad,
                            rad / 2.0,
                        ).setTranslation(-rad, 0.0, 0.0);
                        break;
                }

                world.createCollider(colliderDesc, body);
            }
        }

        offset -= 0.05 * rad * (num - 1.0);
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
