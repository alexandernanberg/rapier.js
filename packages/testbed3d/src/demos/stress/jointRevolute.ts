import type RAPIER from "@alexandernanberg/rapier3d";
import type {Testbed} from "../../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // Long chains of four-body revolute loops. Upstream builds 4x50 of them
    // (8.2k boxes); 2x25 keeps it around 2k.
    let rad = 0.4;
    let num = 10;
    let shift = 2.0;
    let i, j, k, l;

    for (l = 0; l < 2; ++l) {
        let y = l * shift * num * 3.0;

        for (j = 0; j < 25; ++j) {
            let x = j * shift * 4.0;

            let anchorDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(x, y, 0.0);
            let currParent = world.createRigidBody(anchorDesc);
            world.createCollider(RAPIER.ColliderDesc.cuboid(rad, rad, rad), currParent);

            for (i = 0; i < num; ++i) {
                // Create four bodies.
                let z = i * shift * 2.0 + shift;
                let positions = [
                    new RAPIER.Vector3(x, y, z),
                    new RAPIER.Vector3(x + shift, y, z),
                    new RAPIER.Vector3(x + shift, y, z + shift),
                    new RAPIER.Vector3(x, y, z + shift),
                ];
                let handles: Array<RAPIER.RigidBody> = [];

                for (k = 0; k < 4; ++k) {
                    let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
                        positions[k].x,
                        positions[k].y,
                        positions[k].z,
                    );
                    let body = world.createRigidBody(bodyDesc);
                    world.createCollider(
                        RAPIER.ColliderDesc.cuboid(rad, rad, rad).setDensity(1.0),
                        body,
                    );
                    handles.push(body);
                }

                // Setup four joints closing the loop back onto the parent.
                let o = new RAPIER.Vector3(0.0, 0.0, 0.0);
                let xAxis = new RAPIER.Vector3(1.0, 0.0, 0.0);
                let zAxis = new RAPIER.Vector3(0.0, 0.0, 1.0);

                world.createImpulseJoint(
                    RAPIER.JointData.revolute(o, new RAPIER.Vector3(0.0, 0.0, -shift), zAxis),
                    currParent,
                    handles[0],
                    true,
                );
                world.createImpulseJoint(
                    RAPIER.JointData.revolute(o, new RAPIER.Vector3(-shift, 0.0, 0.0), xAxis),
                    handles[0],
                    handles[1],
                    true,
                );
                world.createImpulseJoint(
                    RAPIER.JointData.revolute(o, new RAPIER.Vector3(0.0, 0.0, -shift), zAxis),
                    handles[1],
                    handles[2],
                    true,
                );
                world.createImpulseJoint(
                    RAPIER.JointData.revolute(o, new RAPIER.Vector3(shift, 0.0, 0.0), xAxis),
                    handles[2],
                    handles[3],
                    true,
                );

                currParent = handles[3];
            }
        }
    }

    testbed.setWorld(world);

    let cameraPosition = {
        eye: {x: 239.0, y: 41.0, z: 114.0},
        target: {x: 67.0, y: 41.0, z: -58.0},
    };
    testbed.lookAt(cameraPosition);
}
