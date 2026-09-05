import type {Testbed} from "../../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // Many short chains of limited prismatic joints hanging off fixed anchors.
    // Upstream builds 8x8x50 of them (19.2k boxes); 4x4x25 keeps it around 2.4k.
    let rad = 0.4;
    let num = 5;
    let shift = 1.0;
    let i, j, l, m;

    for (m = 0; m < 4; ++m) {
        let z = m * shift * (num + 2.0);

        for (l = 0; l < 4; ++l) {
            let y = l * shift * num * 2.0;

            for (j = 0; j < 25; ++j) {
                let x = j * shift * 4.0;

                let anchorDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(x, y, z);
                let currParent = world.createRigidBody(anchorDesc);
                world.createCollider(RAPIER.ColliderDesc.cuboid(rad, rad, rad), currParent);

                for (i = 0; i < num; ++i) {
                    let childz = z + (i + 1) * shift;
                    let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, childz);
                    let currChild = world.createRigidBody(bodyDesc);
                    world.createCollider(
                        RAPIER.ColliderDesc.cuboid(rad, rad, rad).setDensity(1.0),
                        currChild,
                    );

                    let axis =
                        i % 2 == 0
                            ? new RAPIER.Vector3(Math.SQRT1_2, Math.SQRT1_2, 0.0)
                            : new RAPIER.Vector3(-Math.SQRT1_2, Math.SQRT1_2, 0.0);

                    let prism = RAPIER.JointData.prismatic(
                        new RAPIER.Vector3(0.0, 0.0, 0.0),
                        new RAPIER.Vector3(0.0, 0.0, -shift),
                        axis,
                    );
                    prism.limitsEnabled = true;
                    prism.limits = [-2.0, 0.0];
                    world.createImpulseJoint(prism, currParent, currChild, true);

                    currParent = currChild;
                }
            }
        }
    }

    testbed.setWorld(world);

    let cameraPosition = {
        eye: {x: 131.0, y: 40.0, z: 62.0},
        target: {x: 50.0, y: 4.0, z: -3.0},
    };
    testbed.lookAt(cameraPosition);
}
