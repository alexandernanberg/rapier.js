import type RAPIER from "@alexandernanberg/rapier3d";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    // No gravity: the only thing moving these rectangles is their joint motor.
    let gravity = new RAPIER.Vector3(0.0, 0.0, 0.0);
    let world = new RAPIER.World(gravity);

    // Fixed ground to attach one end of the joints to.
    let ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    let zAxis = new RAPIER.Vector3(0.0, 0.0, 1.0);
    let num;

    // A row of rectangles, each on a motor driven towards a different angle.
    for (num = 0; num < 9; ++num) {
        let xPos = -6.0 + 1.5 * num;
        let bodyDesc = RAPIER.RigidBodyDesc.dynamic()
            .setTranslation(xPos, 2.0, 0.0)
            .setCanSleep(false);
        let body = world.createRigidBody(bodyDesc);
        world.createCollider(RAPIER.ColliderDesc.cuboid(0.1, 0.5, 0.1), body);

        let targetAngle = -Math.PI + (Math.PI / 4.0) * num;
        let jointData = RAPIER.JointData.revolute(
            new RAPIER.Vector3(xPos, 1.5, 0.0),
            new RAPIER.Vector3(0.0, -0.5, 0.0),
            zAxis,
        );
        let joint = world.createImpulseJoint(
            jointData,
            ground,
            body,
            true,
        ) as RAPIER.RevoluteImpulseJoint;
        joint.configureMotorPosition(targetAngle, 1000.0, 150.0);
    }

    // A second row, on velocity motors this time. Each one spins until it reaches
    // its own upper limit, so the row ends up fanned out.
    for (num = 0; num < 8; ++num) {
        let xPos = -6.0 + 1.5 * num;
        let bodyDesc = RAPIER.RigidBodyDesc.dynamic()
            .setTranslation(xPos, 4.5, 0.0)
            // A half turn around the `z` axis.
            .setRotation(new RAPIER.Quaternion(0.0, 0.0, 1.0, 0.0))
            .setCanSleep(false);
        let body = world.createRigidBody(bodyDesc);
        world.createCollider(RAPIER.ColliderDesc.cuboid(0.1, 0.5, 0.1), body);

        let maxAngleLimit = -Math.PI + (Math.PI / 4.0) * num;
        let jointData = RAPIER.JointData.revolute(
            new RAPIER.Vector3(xPos, 5.0, 0.0),
            new RAPIER.Vector3(0.0, -0.5, 0.0),
            zAxis,
        );
        let joint = world.createImpulseJoint(
            jointData,
            ground,
            body,
            true,
        ) as RAPIER.RevoluteImpulseJoint;
        joint.configureMotorVelocity(1.5, 30.0);
        joint.setMotorMaxForce(100.0);
        joint.setLimits(-Math.PI, maxAngleLimit);
    }

    testbed.setWorld(world);

    let cameraPosition = {
        eye: {x: 3.0, y: 3.0, z: 20.0},
        target: {x: 3.0, y: 3.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
