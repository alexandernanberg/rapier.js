import type RAPIER from "@alexandernanberg/rapier3d";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

function generateHeightfield(nsubdivs: number, scale: {x: number; z: number}) {
    let heights = [];
    let i, j;

    // Column-major, like the matrix rapier expects.
    for (j = 0; j <= nsubdivs; ++j) {
        for (i = 0; i <= nsubdivs; ++i) {
            heights.push(
                -Math.cos((i * scale.x) / nsubdivs / 2.0) -
                    Math.cos((j * scale.z) / nsubdivs / 2.0),
            );
        }
    }

    return new Float32Array(heights);
}

/**
 * A car simulated with joints rather than with the ray-cast vehicle controller:
 * each wheel is a real rigid-body, held by a suspension joint to the chassis and
 * spun by a motor on its axle.
 *
 * Strongly inspired from https://github.com/h3r2tic/cornell-mcray/blob/main/src/car.rs
 * (MIT/Apache license).
 */
export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // Ground.
    let nsubdivs = 100;
    let groundScale = new RAPIER.Vector3(60.0, 0.4, 60.0);
    world.createCollider(
        RAPIER.ColliderDesc.heightfield(
            nsubdivs,
            nsubdivs,
            generateHeightfield(nsubdivs, groundScale),
            groundScale,
        )
            .setTranslation(-7.0, 0.0, 0.0)
            .setFriction(1.0),
    );

    // The car parts are all members of one group, and interact with every group but
    // that one, so they collide with the world and never with each other.
    const CAR_GROUP = 0x0001;
    let carGroups = ((CAR_GROUP << 16) | (~CAR_GROUP & 0xffff)) >>> 0;

    let wheelParams = [
        new RAPIER.Vector3(0.6874, 0.2783, -0.7802),
        new RAPIER.Vector3(-0.6874, 0.2783, -0.7802),
        new RAPIER.Vector3(0.64, 0.2783, 1.0254),
        new RAPIER.Vector3(-0.64, 0.2783, 1.0254),
    ];

    let suspensionHeight = 0.12;
    let maxSteeringAngle = (35.0 * Math.PI) / 180.0;
    let driveStrength = 1.0;
    let wheelRadius = 0.28;
    let carPosition = new RAPIER.Vector3(0.0, wheelRadius + suspensionHeight, 0.0);
    let bodyPositionInCarSpace = new RAPIER.Vector3(0.0, 0.4739, 0.0);

    let chassisDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
        carPosition.x + bodyPositionInCarSpace.x,
        carPosition.y + bodyPositionInCarSpace.y,
        carPosition.z + bodyPositionInCarSpace.z,
    );
    let chassis = world.createRigidBody(chassisDesc);
    world.createCollider(
        RAPIER.ColliderDesc.cuboid(0.65, 0.3, 0.9).setDensity(100.0).setCollisionGroups(carGroups),
        chassis,
    );

    let steeringJoints: Array<RAPIER.ImpulseJointHandle> = [];
    let motorJoints: Array<RAPIER.ImpulseJointHandle> = [];

    wheelParams.forEach((wheelPosInCarSpace, wheelId) => {
        let isFront = wheelId >= 2;
        let wheelCenter = new RAPIER.Vector3(
            carPosition.x + wheelPosInCarSpace.x,
            carPosition.y + wheelPosInCarSpace.y,
            carPosition.z + wheelPosInCarSpace.z,
        );

        // The axle carries the steering angle and the suspension travel; the wheel
        // itself only spins around it. It has no collider, so it is given the mass
        // properties of a ball of the wheel’s size instead.
        let axleMass = 100.0 * (4.0 / 3.0) * Math.PI * wheelRadius ** 3;
        let axleInertia = 0.4 * axleMass * wheelRadius ** 2;
        let axleDesc = RAPIER.RigidBodyDesc.dynamic()
            .setTranslation(wheelCenter.x, wheelCenter.y, wheelCenter.z)
            .setAdditionalMassProperties(
                axleMass,
                new RAPIER.Vector3(0.0, 0.0, 0.0),
                new RAPIER.Vector3(axleInertia, axleInertia, axleInertia),
                new RAPIER.Quaternion(0.0, 0.0, 0.0, 1.0),
            );
        let axle = world.createRigidBody(axleDesc);

        // Simulating the wheel as a ball is interesting as it is mathematically
        // simpler than a cylinder, and cheaper for collision-detection.
        let wheelDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
            wheelCenter.x,
            wheelCenter.y,
            wheelCenter.z,
        );
        let wheel = world.createRigidBody(wheelDesc);
        world.createCollider(
            RAPIER.ColliderDesc.ball(wheelRadius)
                .setDensity(100.0)
                .setCollisionGroups(carGroups)
                .setFriction(1.0),
            wheel,
        );

        // A cylinder that only exists so the testbed draws something wheel-shaped.
        // It collides with nothing and weighs nothing.
        world.createCollider(
            RAPIER.ColliderDesc.cylinder(wheelRadius / 2.0, wheelRadius)
                .setRotation(new RAPIER.Quaternion(0.0, 0.0, Math.SQRT1_2, Math.SQRT1_2))
                .setSensor(true)
                .setDensity(0.0)
                .setCollisionGroups(0),
            wheel,
        );

        // Suspension between the chassis and the axle: everything is locked except
        // the vertical travel, plus the steering rotation on the front wheels.
        let lockedAxes =
            RAPIER.JointAxesMask.LinX |
            RAPIER.JointAxesMask.LinZ |
            RAPIER.JointAxesMask.AngX |
            RAPIER.JointAxesMask.AngZ;

        if (!isFront) {
            lockedAxes |= RAPIER.JointAxesMask.AngY;
        }

        let suspensionData = RAPIER.JointData.generic(
            new RAPIER.Vector3(
                wheelPosInCarSpace.x - bodyPositionInCarSpace.x,
                wheelPosInCarSpace.y - bodyPositionInCarSpace.y,
                wheelPosInCarSpace.z - bodyPositionInCarSpace.z,
            ),
            new RAPIER.Vector3(0.0, 0.0, 0.0),
            new RAPIER.Vector3(1.0, 0.0, 0.0),
            lockedAxes,
        );
        let suspension = world.createImpulseJoint(
            suspensionData,
            chassis,
            axle,
            true,
        ) as RAPIER.GenericImpulseJoint;
        suspension.setLimits(RAPIER.JointAxis.LinY, 0.0, suspensionHeight);
        suspension.configureMotorPosition(RAPIER.JointAxis.LinY, 0.0, 1.0e4, 1.0e3);

        if (isFront) {
            suspension.setLimits(RAPIER.JointAxis.AngY, -maxSteeringAngle, maxSteeringAngle);
        }

        // Joint between the axle and the wheel.
        let wheelJointData = RAPIER.JointData.revolute(
            new RAPIER.Vector3(0.0, 0.0, 0.0),
            new RAPIER.Vector3(0.0, 0.0, 0.0),
            new RAPIER.Vector3(1.0, 0.0, 0.0),
        );
        let wheelJoint = world.createImpulseJoint(wheelJointData, axle, wheel, true);

        if (isFront) {
            steeringJoints.push(suspension.handle);
            motorJoints.push(wheelJoint.handle);
        }
    });

    testbed.setWorld(world);

    let thrust = 0.0;
    let steering = 0.0;
    let boost = 1.0;

    testbed.setpreTimestepAction(() => {
        let shouldWakeUp = thrust != 0.0 || steering != 0.0;

        steeringJoints.forEach((handle) => {
            let joint = testbed.world.getImpulseJoint(handle) as RAPIER.GenericImpulseJoint | null;

            if (joint !== null) {
                if (shouldWakeUp) {
                    joint.body1().wakeUp();
                    joint.body2().wakeUp();
                }

                joint.configureMotorPosition(
                    RAPIER.JointAxis.AngY,
                    maxSteeringAngle * steering,
                    1.0e4,
                    1.0e3,
                );
            }
        });

        // Pseudo-differential adjusting the speed of the two engines depending on
        // the steering arc. Higher values result in more drifty behavior.
        let differentialStrength = 0.5;
        let sidewaysShift = Math.sin(maxSteeringAngle * steering) * differentialStrength;
        let speedDiff =
            sidewaysShift > 0.0
                ? Math.hypot(1.0, sidewaysShift)
                : 1.0 / Math.hypot(1.0, sidewaysShift);
        let motorSpeeds = [1.0 / speedDiff, speedDiff];

        motorJoints.forEach((handle, i) => {
            let joint = testbed.world.getImpulseJoint(handle) as RAPIER.RevoluteImpulseJoint | null;

            if (joint !== null) {
                if (shouldWakeUp) {
                    joint.body1().wakeUp();
                    joint.body2().wakeUp();
                }

                joint.configureMotorVelocity(-30.0 * thrust * motorSpeeds[i] * boost, 1.0e2);
            }
        });
    });

    document.onkeydown = function (event: KeyboardEvent) {
        if (event.key == "ArrowUp") thrust = -driveStrength;
        if (event.key == "ArrowDown") thrust = driveStrength;
        if (event.key == "ArrowLeft") steering = 1.0;
        if (event.key == "ArrowRight") steering = -1.0;
        if (event.key == "Shift") boost = 1.5;
    };

    document.onkeyup = function (event: KeyboardEvent) {
        if (event.key == "ArrowUp" || event.key == "ArrowDown") thrust = 0.0;
        if (event.key == "ArrowLeft" || event.key == "ArrowRight") steering = 0.0;
        if (event.key == "Shift") boost = 1.0;
    };

    let cameraPosition = {
        eye: {x: 10.0, y: 10.0, z: 10.0},
        target: {x: 0.0, y: 0.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
