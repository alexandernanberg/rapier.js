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

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // Create Ground.
    let groundSize = 5.0;
    let groundHeight = 0.1;
    let groundDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(0.0, -groundHeight, 0.0);
    let ground = world.createRigidBody(groundDesc);
    world.createCollider(RAPIER.ColliderDesc.cuboid(groundSize, groundHeight, groundSize), ground);

    // Vehicle we will control manually. Its wheels aren't colliders: the controller
    // ray-casts along each suspension to find the ground.
    let hw = 0.3;
    let hh = 0.15;
    let chassisDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(0.0, 1.0, 0.0);
    let chassis = world.createRigidBody(chassisDesc);
    world.createCollider(RAPIER.ColliderDesc.cuboid(hw * 2.0, hh, hw).setDensity(100.0), chassis);

    let vehicle = world.createVehicleController(chassis);
    let wheelPositions = [
        new RAPIER.Vector3(hw * 1.5, -hh, hw),
        new RAPIER.Vector3(hw * 1.5, -hh, -hw),
        new RAPIER.Vector3(-hw * 1.5, -hh, hw),
        new RAPIER.Vector3(-hw * 1.5, -hh, -hw),
    ];
    let suspensionDirection = new RAPIER.Vector3(0.0, -1.0, 0.0);
    let axle = new RAPIER.Vector3(0.0, 0.0, 1.0);

    wheelPositions.forEach((position, i) => {
        vehicle.addWheel(position, suspensionDirection, axle, hh, hh / 4.0);
        vehicle.setWheelSuspensionStiffness(i, 100.0);
        vehicle.setWheelSuspensionRelaxation(i, 10.0);
    });

    // Create the cubes to run over.
    let num = 8;
    let rad = 0.1;
    let shift = rad * 2.0;
    let centerx = shift * (num / 2);
    let centery = rad;
    let i, k;

    for (k = 0; k < 4; ++k) {
        for (i = 0; i < num; ++i) {
            let x = i * shift - centerx;
            let z = k * shift + centerx;

            let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, centery, z);
            let body = world.createRigidBody(bodyDesc);
            world.createCollider(RAPIER.ColliderDesc.cuboid(rad, rad, rad), body);
        }
    }

    // A slope we can climb…
    let slopeAngle = 0.2;
    let slopeSize = 2.0;
    world.createCollider(
        RAPIER.ColliderDesc.cuboid(slopeSize, groundHeight, groundSize)
            .setTranslation(groundSize + slopeSize, -groundHeight + 0.4, 0.0)
            .setRotation(
                new RAPIER.Quaternion(
                    0.0,
                    0.0,
                    Math.sin(slopeAngle / 2.0),
                    Math.cos(slopeAngle / 2.0),
                ),
            ),
    );

    // …and one we can’t.
    let impossibleSlopeAngle = 0.9;
    let impossibleSlopeSize = 2.0;
    world.createCollider(
        RAPIER.ColliderDesc.cuboid(slopeSize, groundHeight, groundSize)
            .setTranslation(
                groundSize + slopeSize * 2.0 + impossibleSlopeSize - 0.9,
                -groundHeight + 2.3,
                0.0,
            )
            .setRotation(
                new RAPIER.Quaternion(
                    0.0,
                    0.0,
                    Math.sin(impossibleSlopeAngle / 2.0),
                    Math.cos(impossibleSlopeAngle / 2.0),
                ),
            ),
    );

    // More complex ground.
    let nsubdivs = 20;
    let heightfieldScale = new RAPIER.Vector3(10.0, 0.4, 10.0);
    world.createCollider(
        RAPIER.ColliderDesc.heightfield(
            nsubdivs,
            nsubdivs,
            generateHeightfield(nsubdivs, heightfieldScale),
            heightfieldScale,
        ).setTranslation(-7.0, 0.0, 0.0),
    );

    testbed.setWorld(world);

    let engineForce = 0.0;
    let steeringAngle = 0.0;

    testbed.setpreTimestepAction(() => {
        // The front wheels are the ones being driven and steered.
        vehicle.setWheelEngineForce(0, engineForce);
        vehicle.setWheelSteering(0, steeringAngle);
        vehicle.setWheelEngineForce(1, engineForce);
        vehicle.setWheelSteering(1, steeringAngle);

        // The suspension ray-casts only care about the ground: the chassis is
        // dynamic, so excluding dynamic bodies excludes the vehicle too.
        vehicle.updateVehicle(testbed.world.timestep, RAPIER.QueryFilterFlags.EXCLUDE_DYNAMIC);
    });

    document.onkeydown = function (event: KeyboardEvent) {
        if (event.key == "ArrowUp") engineForce = 30.0;
        if (event.key == "ArrowDown") engineForce = -30.0;
        if (event.key == "ArrowLeft") steeringAngle = 0.7;
        if (event.key == "ArrowRight") steeringAngle = -0.7;
    };

    document.onkeyup = function (event: KeyboardEvent) {
        if (event.key == "ArrowUp" || event.key == "ArrowDown") engineForce = 0.0;
        if (event.key == "ArrowLeft" || event.key == "ArrowRight") steeringAngle = 0.0;
    };

    let cameraPosition = {
        eye: {x: 10.0, y: 10.0, z: 10.0},
        target: {x: 0.0, y: 0.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
