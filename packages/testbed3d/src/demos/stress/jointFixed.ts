import type RAPIER from "@alexandernanberg/rapier3d";
import type {Testbed} from "../../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

const IDENTITY = {x: 0.0, y: 0.0, z: 0.0, w: 1.0};

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // Many small independent grids of fixed joints rather than one big one, so
    // the solver sees a large number of separate islands. Upstream builds 10x10
    // grids of them (12.5k balls); 5x5 keeps it around 3k.
    let rad = 0.4;
    let num = 5;
    let shift = 1.0;
    let i, j, k, l, m;

    let bodies: Array<RAPIER.RigidBody> = [];

    for (m = 0; m < 5; ++m) {
        let z = m * shift * (num + 2.0);

        for (l = 0; l < 5; ++l) {
            let y = l * shift * 3.0;

            for (j = 0; j < 5; ++j) {
                let x = j * shift * num * 2.0;

                for (k = 0; k < num; ++k) {
                    for (i = 0; i < num; ++i) {
                        // NOTE: the num - 2 test is to avoid two consecutive fixed
                        // bodies, which upstream keeps for parity with physx.
                        let fixed = i == 0 && ((k % 4 == 0 && k != num - 2) || k == num - 1);
                        let bodyDesc = (
                            fixed ? RAPIER.RigidBodyDesc.fixed() : RAPIER.RigidBodyDesc.dynamic()
                        ).setTranslation(x + k * shift, y, z + i * shift);
                        let child = world.createRigidBody(bodyDesc);
                        world.createCollider(RAPIER.ColliderDesc.ball(rad), child);

                        // Vertical joint.
                        if (i > 0) {
                            let parent = bodies[bodies.length - 1];
                            world.createImpulseJoint(
                                RAPIER.JointData.fixed(
                                    new RAPIER.Vector3(0.0, 0.0, 0.0),
                                    IDENTITY,
                                    new RAPIER.Vector3(0.0, 0.0, -shift),
                                    IDENTITY,
                                ),
                                parent,
                                child,
                                true,
                            );
                        }

                        // Horizontal joint.
                        if (k > 0) {
                            let parent = bodies[bodies.length - num];
                            world.createImpulseJoint(
                                RAPIER.JointData.fixed(
                                    new RAPIER.Vector3(0.0, 0.0, 0.0),
                                    IDENTITY,
                                    new RAPIER.Vector3(-shift, 0.0, 0.0),
                                    IDENTITY,
                                ),
                                parent,
                                child,
                                true,
                            );
                        }

                        bodies.push(child);
                    }
                }
            }
        }
    }

    testbed.setWorld(world);

    let cameraPosition = {
        eye: {x: -38.0, y: 14.0, z: 108.0},
        target: {x: 46.0, y: 12.0, z: 23.0},
    };
    testbed.lookAt(cameraPosition);
}
