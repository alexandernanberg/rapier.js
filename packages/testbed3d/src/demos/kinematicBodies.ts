import type {Testbed} from "../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // Create Ground.
    let groundDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(0.0, -0.5, 0.0);
    let groundBody = world.createRigidBody(groundDesc);
    let groundColliderDesc = RAPIER.ColliderDesc.cuboid(30.0, 0.5, 30.0);
    world.createCollider(groundColliderDesc, groundBody);

    // Platform parameters.
    let platformHx = 4.0;
    let platformHy = 0.2;
    let platformHz = 4.0;
    let boxRad = 0.5;

    // --- Position-based kinematic platform (left side, oscillates on X) ---
    let posPlatformDesc = RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(
        -10.0,
        2.0,
        0.0,
    );
    let posPlatform = world.createRigidBody(posPlatformDesc);
    let posPlatformColliderDesc = RAPIER.ColliderDesc.cuboid(platformHx, platformHy, platformHz);
    world.createCollider(posPlatformColliderDesc, posPlatform);

    // Stack of dynamic boxes on position-based platform.
    for (let j = 0; j < 3; j++) {
        for (let ix = 0; ix < 3; ix++) {
            for (let iz = 0; iz < 3; iz++) {
                let x = -10.0 + (ix - 1) * (boxRad * 2.2);
                let y = 2.0 + platformHy + boxRad + j * (boxRad * 2.05);
                let z = (iz - 1) * (boxRad * 2.2);
                let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z);
                let body = world.createRigidBody(bodyDesc);
                let colliderDesc = RAPIER.ColliderDesc.cuboid(boxRad, boxRad, boxRad);
                world.createCollider(colliderDesc, body);
            }
        }
    }

    // --- Velocity-based kinematic platform (right side, oscillates on X) ---
    let velPlatformDesc = RAPIER.RigidBodyDesc.kinematicVelocityBased().setTranslation(
        10.0,
        2.0,
        0.0,
    );
    let velPlatform = world.createRigidBody(velPlatformDesc);
    let velPlatformColliderDesc = RAPIER.ColliderDesc.cuboid(platformHx, platformHy, platformHz);
    world.createCollider(velPlatformColliderDesc, velPlatform);

    // Stack of dynamic boxes on velocity-based platform.
    for (let j = 0; j < 3; j++) {
        for (let ix = 0; ix < 3; ix++) {
            for (let iz = 0; iz < 3; iz++) {
                let x = 10.0 + (ix - 1) * (boxRad * 2.2);
                let y = 2.0 + platformHy + boxRad + j * (boxRad * 2.05);
                let z = (iz - 1) * (boxRad * 2.2);
                let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z);
                let body = world.createRigidBody(bodyDesc);
                let colliderDesc = RAPIER.ColliderDesc.cuboid(boxRad, boxRad, boxRad);
                world.createCollider(colliderDesc, body);
            }
        }
    }

    // --- Vertical kinematic platform (center, oscillates on Y) ---
    let vertPlatformDesc = RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(
        0.0,
        2.0,
        0.0,
    );
    let vertPlatform = world.createRigidBody(vertPlatformDesc);
    let vertPlatformColliderDesc = RAPIER.ColliderDesc.cuboid(
        platformHx * 0.6,
        platformHy,
        platformHz * 0.6,
    );
    world.createCollider(vertPlatformColliderDesc, vertPlatform);

    // Stack of dynamic boxes on vertical platform.
    for (let j = 0; j < 3; j++) {
        for (let ix = 0; ix < 2; ix++) {
            for (let iz = 0; iz < 2; iz++) {
                let x = (ix - 0.5) * (boxRad * 2.2);
                let y = 2.0 + platformHy + boxRad + j * (boxRad * 2.05);
                let z = (iz - 0.5) * (boxRad * 2.2);
                let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z);
                let body = world.createRigidBody(bodyDesc);
                let colliderDesc = RAPIER.ColliderDesc.cuboid(boxRad, boxRad, boxRad);
                world.createCollider(colliderDesc, body);
            }
        }
    }

    // Capture handles instead of body references so callbacks survive snapshot restore.
    let posPlatformHandle = posPlatform.handle;
    let velPlatformHandle = velPlatform.handle;
    let vertPlatformHandle = vertPlatform.handle;

    // Animation state.
    let time = 0.0;
    let horizontalAmplitude = 5.0;
    let verticalAmplitude = 4.0;
    let horizontalSpeed = 1.5;
    let verticalSpeed = 1.0;
    let velPlatformSpeed = 6.0;

    let updatePlatforms = () => {
        time += 1 / 60;

        let posPlatformBody = testbed.world.getRigidBody(posPlatformHandle);
        let velPlatformBody = testbed.world.getRigidBody(velPlatformHandle);
        let vertPlatformBody = testbed.world.getRigidBody(vertPlatformHandle);

        // Position-based platform: oscillates on X axis.
        let posX = -10.0 + Math.sin(time * horizontalSpeed) * horizontalAmplitude;
        posPlatformBody.setNextKinematicTranslation({x: posX, y: 2.0, z: 0.0});

        // Velocity-based platform: oscillates on X axis using velocity.
        let velX = Math.cos(time * horizontalSpeed) * velPlatformSpeed;
        velPlatformBody.setLinvel({x: velX, y: 0.0, z: 0.0}, true);

        // Vertical platform: oscillates up and down.
        let vertY = 2.0 + Math.sin(time * verticalSpeed) * verticalAmplitude;
        vertPlatformBody.setNextKinematicTranslation({x: 0.0, y: vertY, z: 0.0});
    };

    testbed.setWorld(world);
    testbed.setpreTimestepAction(updatePlatforms);
    let cameraPosition = {
        eye: {x: -40.0, y: 30.0, z: 40.0},
        target: {x: 0.0, y: 4.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
