import type * as RAPIER_NS from "@alexandernanberg/rapier3d";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof RAPIER_NS;

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    const gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    const world = new RAPIER.World(gravity);

    // Create Ground.
    const groundDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(0.0, -0.5, 0.0);
    const groundBody = world.createRigidBody(groundDesc);
    const groundColliderDesc = RAPIER.ColliderDesc.cuboid(30.0, 0.5, 30.0);
    world.createCollider(groundColliderDesc, groundBody);

    // Platform parameters.
    const platformHx = 4.0;
    const platformHy = 0.2;
    const platformHz = 4.0;
    const boxRad = 0.5;

    // --- Position-based kinematic platform (left side, oscillates on X) ---
    const posPlatformDesc = RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(
        -10.0,
        2.0,
        0.0,
    );
    const posPlatform = world.createRigidBody(posPlatformDesc);
    const posPlatformColliderDesc = RAPIER.ColliderDesc.cuboid(platformHx, platformHy, platformHz);
    world.createCollider(posPlatformColliderDesc, posPlatform);

    // Stack of dynamic boxes on position-based platform.
    for (let j = 0; j < 3; j++) {
        for (let ix = 0; ix < 3; ix++) {
            for (let iz = 0; iz < 3; iz++) {
                const x = -10.0 + (ix - 1) * (boxRad * 2.2);
                const y = 2.0 + platformHy + boxRad + j * (boxRad * 2.05);
                const z = (iz - 1) * (boxRad * 2.2);
                const bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z);
                const body = world.createRigidBody(bodyDesc);
                const colliderDesc = RAPIER.ColliderDesc.cuboid(boxRad, boxRad, boxRad);
                world.createCollider(colliderDesc, body);
            }
        }
    }

    // --- Velocity-based kinematic platform (right side, oscillates on X) ---
    const velPlatformDesc = RAPIER.RigidBodyDesc.kinematicVelocityBased().setTranslation(
        10.0,
        2.0,
        0.0,
    );
    const velPlatform = world.createRigidBody(velPlatformDesc);
    const velPlatformColliderDesc = RAPIER.ColliderDesc.cuboid(platformHx, platformHy, platformHz);
    world.createCollider(velPlatformColliderDesc, velPlatform);

    // Stack of dynamic boxes on velocity-based platform.
    for (let j = 0; j < 3; j++) {
        for (let ix = 0; ix < 3; ix++) {
            for (let iz = 0; iz < 3; iz++) {
                const x = 10.0 + (ix - 1) * (boxRad * 2.2);
                const y = 2.0 + platformHy + boxRad + j * (boxRad * 2.05);
                const z = (iz - 1) * (boxRad * 2.2);
                const bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z);
                const body = world.createRigidBody(bodyDesc);
                const colliderDesc = RAPIER.ColliderDesc.cuboid(boxRad, boxRad, boxRad);
                world.createCollider(colliderDesc, body);
            }
        }
    }

    // --- Vertical kinematic platform (center, oscillates on Y) ---
    const vertPlatformDesc = RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(
        0.0,
        2.0,
        0.0,
    );
    const vertPlatform = world.createRigidBody(vertPlatformDesc);
    const vertPlatformColliderDesc = RAPIER.ColliderDesc.cuboid(
        platformHx * 0.6,
        platformHy,
        platformHz * 0.6,
    );
    world.createCollider(vertPlatformColliderDesc, vertPlatform);

    // Stack of dynamic boxes on vertical platform.
    for (let j = 0; j < 3; j++) {
        for (let ix = 0; ix < 2; ix++) {
            for (let iz = 0; iz < 2; iz++) {
                const x = (ix - 0.5) * (boxRad * 2.2);
                const y = 2.0 + platformHy + boxRad + j * (boxRad * 2.05);
                const z = (iz - 0.5) * (boxRad * 2.2);
                const bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z);
                const body = world.createRigidBody(bodyDesc);
                const colliderDesc = RAPIER.ColliderDesc.cuboid(boxRad, boxRad, boxRad);
                world.createCollider(colliderDesc, body);
            }
        }
    }

    // Capture handles instead of body references so callbacks survive snapshot restore.
    const posPlatformHandle = posPlatform.handle;
    const velPlatformHandle = velPlatform.handle;
    const vertPlatformHandle = vertPlatform.handle;

    // Animation state.
    let time = 0.0;
    const horizontalAmplitude = 5.0;
    const verticalAmplitude = 4.0;
    const horizontalSpeed = 1.5;
    const verticalSpeed = 1.0;
    const velPlatformSpeed = 6.0;

    const updatePlatforms = () => {
        time += 1 / 60;

        const posPlatformBody = testbed.world.getRigidBody(posPlatformHandle);
        const velPlatformBody = testbed.world.getRigidBody(velPlatformHandle);
        const vertPlatformBody = testbed.world.getRigidBody(vertPlatformHandle);

        // The handles go stale if the world is swapped out from under us.
        if (posPlatformBody === null || velPlatformBody === null || vertPlatformBody === null) {
            return;
        }

        // Position-based platform: oscillates on X axis.
        const posX = -10.0 + Math.sin(time * horizontalSpeed) * horizontalAmplitude;
        posPlatformBody.setNextKinematicTranslation({x: posX, y: 2.0, z: 0.0});

        // Velocity-based platform: oscillates on X axis using velocity.
        const velX = Math.cos(time * horizontalSpeed) * velPlatformSpeed;
        velPlatformBody.setLinvel({x: velX, y: 0.0, z: 0.0}, true);

        // Vertical platform: oscillates up and down.
        const vertY = 2.0 + Math.sin(time * verticalSpeed) * verticalAmplitude;
        vertPlatformBody.setNextKinematicTranslation({x: 0.0, y: vertY, z: 0.0});
    };

    testbed.setWorld(world);
    testbed.setpreTimestepAction(updatePlatforms);
    const cameraPosition = {
        eye: {x: -40.0, y: 30.0, z: 40.0},
        target: {x: 0.0, y: 4.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
