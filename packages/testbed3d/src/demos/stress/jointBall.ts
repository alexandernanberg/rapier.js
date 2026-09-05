import type RAPIER from "@alexandernanberg/rapier3d";
import type {Testbed} from "../../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // A sheet of balls wired to their neighbors by spherical joints, pinned along
    // one edge. Upstream uses 100 per side (10k balls); 55 keeps it around 3k.
    let rad = 0.4;
    let num = 55;
    let shift = 1.0;
    let i, k;

    let bodies: Array<RAPIER.RigidBody> = [];

    for (k = 0; k < num; ++k) {
        for (i = 0; i < num; ++i) {
            let fixed = i == 0 && (k % 4 == 0 || k == num - 1);
            let bodyDesc = (
                fixed ? RAPIER.RigidBodyDesc.fixed() : RAPIER.RigidBodyDesc.dynamic()
            ).setTranslation(k * shift, 0.0, i * shift);
            let child = world.createRigidBody(bodyDesc);
            world.createCollider(RAPIER.ColliderDesc.ball(rad), child);

            // Vertical joint.
            if (i > 0) {
                let parent = bodies[bodies.length - 1];
                world.createImpulseJoint(
                    RAPIER.JointData.spherical(
                        new RAPIER.Vector3(0.0, 0.0, 0.0),
                        new RAPIER.Vector3(0.0, 0.0, -shift),
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
                    RAPIER.JointData.spherical(
                        new RAPIER.Vector3(0.0, 0.0, 0.0),
                        new RAPIER.Vector3(-shift, 0.0, 0.0),
                    ),
                    parent,
                    child,
                    true,
                );
            }

            bodies.push(child);
        }
    }

    testbed.setWorld(world);

    let cameraPosition = {
        eye: {x: -60.0, y: -26.0, z: 95.0},
        target: {x: 30.0, y: -21.0, z: 16.0},
    };
    testbed.lookAt(cameraPosition);
}
