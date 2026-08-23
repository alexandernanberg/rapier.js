import type * as RAPIER_NS from "@alexandernanberg/rapier3d";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof RAPIER_NS;

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    const gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    const world = new RAPIER.World(gravity);

    // Create Ground.
    const groundBodyDesc = RAPIER.RigidBodyDesc.fixed();
    const groundBody = world.createRigidBody(groundBodyDesc);
    const groundColliderDesc = RAPIER.ColliderDesc.cuboid(15.0, 0.1, 15.0);
    world.createCollider(groundColliderDesc, groundBody);

    // Dynamic cubes.
    const rad = 0.5;
    const num = 5;
    let i, j, k;
    const shift = rad * 2.5;
    const center = num * rad;
    const height = 5.0;

    for (i = 0; i < num; ++i) {
        for (j = i; j < num; ++j) {
            for (k = i; k < num; ++k) {
                const x = (i * shift) / 2.0 + (k - i) * shift - center;
                const y = (i * shift) / 2.0 + height;
                const z = (i * shift) / 2.0 + (j - i) * shift - center;

                // Create dynamic cube.
                const bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z);
                const body = world.createRigidBody(bodyDesc);
                const colliderDesc = RAPIER.ColliderDesc.cuboid(rad, rad / 2.0, rad);
                world.createCollider(colliderDesc, body);
            }
        }
    }

    // Character.
    const characterDesc = RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(
        -10.0,
        4.0,
        -10.0,
    );
    const character = world.createRigidBody(characterDesc);
    const characterColliderDesc = RAPIER.ColliderDesc.cylinder(1.2, 0.6);
    const characterCollider = world.createCollider(characterColliderDesc, character);

    const characterController = world.createCharacterController(0.1);
    characterController.enableAutostep(0.7, 0.3, true);
    characterController.enableSnapToGround(0.7);

    // Capture handles instead of body references so callbacks survive snapshot restore.
    const characterHandle = character.handle;
    const characterColliderHandle = characterCollider.handle;

    const speed = 0.2;
    const movementDirection = {x: 0.0, y: -speed, z: 0.0};

    const updateCharacter = () => {
        const charBody = testbed.world.getRigidBody(characterHandle);
        const charCollider = testbed.world.getCollider(characterColliderHandle);

        // The handles go stale if the world is swapped out from under us.
        if (charBody === null || charCollider === null) {
            return;
        }

        characterController.computeColliderMovement(charCollider, movementDirection);

        const movement = characterController.computedMovement();
        const newPos = charBody.translation();
        newPos.x += movement.x;
        newPos.y += movement.y;
        newPos.z += movement.z;
        charBody.setNextKinematicTranslation(newPos);
    };

    testbed.setWorld(world);
    testbed.setpreTimestepAction(updateCharacter);

    document.addEventListener(
        "keydown",
        (event) => {
            if (event.key == "ArrowUp") movementDirection.x = speed;
            if (event.key == "ArrowDown") movementDirection.x = -speed;
            if (event.key == "ArrowLeft") movementDirection.z = -speed;
            if (event.key == "ArrowRight") movementDirection.z = speed;
            if (event.key == " ") movementDirection.y = speed;
        },
        {signal: testbed.demoSignal},
    );

    document.addEventListener(
        "keyup",
        (event) => {
            if (event.key == "ArrowUp") movementDirection.x = 0.0;
            if (event.key == "ArrowDown") movementDirection.x = 0.0;
            if (event.key == "ArrowLeft") movementDirection.z = 0.0;
            if (event.key == "ArrowRight") movementDirection.z = 0.0;
            if (event.key == " ") movementDirection.y = -speed; // Gravity
        },
        {signal: testbed.demoSignal},
    );

    const cameraPosition = {
        eye: {x: -40.0, y: 19.730000000000008, z: 0.0},
        target: {x: 0.0, y: -0.4126, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
