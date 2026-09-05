import type {Testbed} from "../../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // An 8x8 grid of hanging ropes, 60 capsule segments each (3840 dynamic
    // bodies, one spherical joint per segment), swinging from an initial sideways
    // kick. Nearly contact-free: a joint-solver stress test.
    let segments = 60;
    let segLen = 1.0;
    let i, k, s;

    for (i = 0; i < 8; ++i) {
        for (k = 0; k < 8; ++k) {
            let topx = i * 4.0;
            let topz = k * 4.0;

            let parent = world.createRigidBody(
                RAPIER.RigidBodyDesc.fixed().setTranslation(topx, 0.0, topz),
            );

            for (s = 0; s < segments; ++s) {
                let y = -(s + 0.5) * segLen;
                let bodyDesc = RAPIER.RigidBodyDesc.dynamic()
                    .setTranslation(topx, y, topz)
                    .setLinvel(2.0, 0.0, 0.0);
                let body = world.createRigidBody(bodyDesc);
                world.createCollider(RAPIER.ColliderDesc.capsule(0.35, 0.1), body);

                // The first segment hangs from the fixed anchor's origin; every
                // other one from the bottom of the segment above it.
                let anchor1 =
                    s == 0
                        ? new RAPIER.Vector3(0.0, 0.0, 0.0)
                        : new RAPIER.Vector3(0.0, -segLen / 2.0, 0.0);
                let joint = world.createImpulseJoint(
                    RAPIER.JointData.spherical(anchor1, new RAPIER.Vector3(0.0, segLen / 2.0, 0.0)),
                    parent,
                    body,
                    true,
                );
                joint.setContactsEnabled(false);

                parent = body;
            }
        }
    }

    testbed.setWorld(world);

    let cameraPosition = {
        eye: {x: -45.0, y: -10.0, z: -45.0},
        target: {x: 14.0, y: -30.0, z: 14.0},
    };
    testbed.lookAt(cameraPosition);
}
