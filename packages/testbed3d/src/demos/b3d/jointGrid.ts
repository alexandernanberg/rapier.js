import type RAPIER from "@alexandernanberg/rapier3d";
import type {Testbed} from "../../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

/**
 * Port of box3d's `joint_grid` benchmark (`CreateJointGrid`,
 * `box3d/shared/benchmarks.c`). Release settings: a grid of spheres wired
 * together with spherical joints; the `i == 0` column is static. Sleeping
 * disabled.
 */
export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    // box3d's `b3DefaultWorldDef`: gravity (0, -10, 0). box3d steps at dt = 1/60
    // with 4 solver substeps, matching rapier's defaults.
    let gravity = new RAPIER.Vector3(0.0, -10.0, 0.0);
    let world = new RAPIER.World(gravity);

    // Upstream uses a 100x100 grid (10k jointed balls); 55 keeps it around 3k.
    let n = 55;
    let bodies: Array<RAPIER.RigidBody> = [];
    let index = 0;
    let i, k;

    for (k = 0; k < n; ++k) {
        for (i = 0; i < n; ++i) {
            let bodyDesc = (
                i == 0
                    ? RAPIER.RigidBodyDesc.fixed()
                    : RAPIER.RigidBodyDesc.dynamic().setCanSleep(false)
            ).setTranslation(k, -i, 0.0);
            let body = world.createRigidBody(bodyDesc);
            world.createCollider(RAPIER.ColliderDesc.ball(0.4), body);

            // Spherical joint to the body above (previous i).
            if (i > 0) {
                world.createImpulseJoint(
                    RAPIER.JointData.spherical(
                        new RAPIER.Vector3(0.0, -0.5, 0.0),
                        new RAPIER.Vector3(0.0, 0.5, 0.0),
                    ),
                    bodies[index - 1],
                    body,
                    true,
                );
            }

            // Spherical joint to the body in the previous column.
            if (k > 0) {
                world.createImpulseJoint(
                    RAPIER.JointData.spherical(
                        new RAPIER.Vector3(0.5, 0.0, 0.0),
                        new RAPIER.Vector3(-0.5, 0.0, 0.0),
                    ),
                    bodies[index - n],
                    body,
                    true,
                );
            }

            bodies.push(body);
            index += 1;
        }
    }

    testbed.setWorld(world);

    let cameraPosition = {
        eye: {x: 27.0, y: -14.0, z: 50.0},
        target: {x: 27.0, y: -28.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
