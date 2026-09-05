import seedrandom from "seedrandom";
import type {Testbed} from "../../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

const _translation = {x: 0.0, y: 0.0, z: 0.0};
const _linvel = {x: 0.0, y: 0.0, z: 0.0};

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, 0.0, 0.0);
    let world = new RAPIER.World(gravity);

    // A cloud of kinematic balls flying through each other and bouncing off the
    // walls of an invisible box. Kinematic bodies don't respond to contacts, so
    // this measures the broad-phase against a set that never settles. Upstream
    // uses 30 per side (27k balls).
    let num = 15;
    let rad = 1.0;
    let shift = rad * 6.0 + 1.0;
    let centerx = (shift * num) / 2.0;
    let centery = (shift * num) / 2.0;
    let centerz = (shift * num) / 2.0;
    let bound = (shift * num) / 2.0;

    let rng = seedrandom("many-kinematics");
    let i, j, k;

    for (i = 0; i < num; ++i) {
        for (j = 0; j < num; ++j) {
            for (k = 0; k < num; ++k) {
                let x = i * shift - centerx;
                let y = j * shift - centery;
                let z = k * shift - centerz;

                let bodyDesc = RAPIER.RigidBodyDesc.kinematicVelocityBased()
                    .setTranslation(x, y, z)
                    .setLinvel((rng() - 0.5) * 30.0, (rng() - 0.5) * 30.0, (rng() - 0.5) * 30.0);
                let body = world.createRigidBody(bodyDesc);
                world.createCollider(RAPIER.ColliderDesc.ball(rad), body);
            }
        }
    }

    testbed.setWorld(world);

    testbed.setpostTimestepAction(() => {
        // Bounce every ball back once it leaves the box it started in.
        testbed.world.forEachRigidBody((body) => {
            body.translation(_translation);
            body.linvel(_linvel);
            let bounced = false;

            if (
                (_linvel.x > 0.0 && _translation.x > bound) ||
                (_linvel.x < 0.0 && _translation.x < -bound)
            ) {
                _linvel.x = -_linvel.x;
                bounced = true;
            }
            if (
                (_linvel.y > 0.0 && _translation.y > bound) ||
                (_linvel.y < 0.0 && _translation.y < -bound)
            ) {
                _linvel.y = -_linvel.y;
                bounced = true;
            }
            if (
                (_linvel.z > 0.0 && _translation.z > bound) ||
                (_linvel.z < 0.0 && _translation.z < -bound)
            ) {
                _linvel.z = -_linvel.z;
                bounced = true;
            }

            if (bounced) {
                body.setLinvel(_linvel, false);
            }
        });
    });

    let cameraPosition = {
        eye: {x: 100.0, y: 100.0, z: 100.0},
        target: {x: 0.0, y: 0.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
