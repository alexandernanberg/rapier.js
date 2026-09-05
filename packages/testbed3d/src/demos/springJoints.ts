import type {Testbed} from "../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // Fixed ground to attach one end of the joints to.
    let ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());

    // Spring joints with a variety of spring parameters: the damping ratio goes
    // from 0 (undamped) on the left to 2 (overdamped) on the right, so the middle
    // one is critically damped.
    let num = 30;
    let radius = 0.5;
    // Mass of a ball collider of that radius, at the default density of 1.
    let mass = (4.0 / 3.0) * Math.PI * radius * radius * radius;
    let stiffness = 1.0e3;
    let criticalDamping = 2.0 * Math.sqrt(stiffness * mass);
    let i;

    for (i = 0; i <= num; ++i) {
        let xPos = -6.0 + 1.5 * i;
        let ballPos = new RAPIER.Vector3(xPos, 4.5, 0.0);

        let bodyDesc = RAPIER.RigidBodyDesc.dynamic()
            .setTranslation(ballPos.x, ballPos.y, ballPos.z)
            .setCanSleep(false);
        let body = world.createRigidBody(bodyDesc);
        world.createCollider(RAPIER.ColliderDesc.ball(radius), body);

        let dampingRatio = i / (num / 2.0);
        let damping = dampingRatio * criticalDamping;
        let joint = RAPIER.JointData.spring(
            0.0,
            stiffness,
            damping,
            new RAPIER.Vector3(ballPos.x, ballPos.y - 3.0, ballPos.z),
            new RAPIER.Vector3(0.0, 0.0, 0.0),
        );
        world.createImpulseJoint(joint, ground, body, true);

        // Box that will fall on top of the springed balls, which makes the
        // simulation funnier to watch.
        let boxDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
            ballPos.x,
            ballPos.y + 5.0,
            ballPos.z,
        );
        let box = world.createRigidBody(boxDesc);
        let boxColliderDesc = RAPIER.ColliderDesc.cuboid(radius, radius, radius).setDensity(100.0);
        world.createCollider(boxColliderDesc, box);
    }

    testbed.setWorld(world);

    let cameraPosition = {
        eye: {x: 15.0, y: 5.0, z: 42.0},
        target: {x: 13.0, y: 1.0, z: 1.0},
    };
    testbed.lookAt(cameraPosition);
}
