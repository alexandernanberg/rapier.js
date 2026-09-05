import type {Testbed} from "../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    let radius = 0.5;
    let length = 10.0 * radius;
    let n = 5;
    let i;

    // Perfectly elastic balls hanging from spherical joints, side by side. The
    // last one starts with a velocity, and the momentum travels through the row.
    for (i = 0; i < n; ++i) {
        let ballPos = new RAPIER.Vector3(i * 2.02 * radius, 0.0, 0.0);
        let attach = new RAPIER.Vector3(0.0, length, 0.0);

        let anchorDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(
            ballPos.x + attach.x,
            ballPos.y + attach.y,
            ballPos.z + attach.z,
        );
        let anchor = world.createRigidBody(anchorDesc);

        let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
            ballPos.x,
            ballPos.y,
            ballPos.z,
        );

        if (i >= n - 1) {
            bodyDesc.setLinvel(7.0, 0.0, 0.0);
        }

        let body = world.createRigidBody(bodyDesc);
        let colliderDesc = RAPIER.ColliderDesc.ball(radius).setRestitution(1.0);
        world.createCollider(colliderDesc, body);

        let joint = RAPIER.JointData.spherical(new RAPIER.Vector3(0.0, 0.0, 0.0), attach);
        world.createImpulseJoint(joint, anchor, body, true);
    }

    testbed.setWorld(world);

    let cameraPosition = {
        eye: {x: 10.0, y: 10.0, z: 10.0},
        target: {x: 0.0, y: 0.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
