import type * as RAPIER_NS from "@alexandernanberg/rapier2d";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof RAPIER_NS;

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    const gravity = new RAPIER.Vector2(0.0, -9.81);
    const world = new RAPIER.World(gravity);

    // Create Ground.
    const groundBodyDesc = RAPIER.RigidBodyDesc.fixed();
    const groundBody = world.createRigidBody(groundBodyDesc);
    const groundColliderDesc = RAPIER.ColliderDesc.cuboid(15.0, 0.1);
    world.createCollider(groundColliderDesc, groundBody);

    // Dynamic cubes.
    const rad = 0.5;
    const num = 5;
    let i, _j, k;
    const shift = rad * 2.5;
    const center = num * rad;
    const height = 5.0;

    for (i = 0; i < num; ++i) {
        for (k = i; k < num; ++k) {
            const x = (i * shift) / 2.0 + (k - i) * shift - center;
            const y = (i * shift) / 2.0 + height;

            // Create dynamic cube.
            const bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y);
            const body = world.createRigidBody(bodyDesc);
            const colliderDesc = RAPIER.ColliderDesc.cuboid(rad, rad / 2.0);
            world.createCollider(colliderDesc, body);
        }
    }

    // Character.
    const characterDesc = RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(-10.0, 4.0);
    const character = world.createRigidBody(characterDesc);
    const characterColliderDesc = RAPIER.ColliderDesc.cuboid(0.6, 1.2);
    const characterCollider = world.createCollider(characterColliderDesc, character);

    const characterController = world.createCharacterController(0.1);
    characterController.enableAutostep(0.7, 0.3, true);
    characterController.enableSnapToGround(0.7);

    // Capture handles instead of body references so callbacks survive snapshot restore.
    const characterHandle = character.handle;
    const characterColliderHandle = characterCollider.handle;

    const speed = 0.2;
    const movementDirection = {x: 0.0, y: -speed};

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
        charBody.setNextKinematicTranslation(newPos);
    };

    testbed.setWorld(world);
    testbed.setpreTimestepAction(updateCharacter);

    document.addEventListener(
        "keydown",
        (event) => {
            if (event.key == "ArrowLeft") movementDirection.x = -speed;
            if (event.key == "ArrowRight") movementDirection.x = speed;
            if (event.key == " ") movementDirection.y = speed;
        },
        {signal: testbed.demoSignal},
    );

    document.addEventListener(
        "keyup",
        (event) => {
            if (event.key == "ArrowLeft") movementDirection.x = 0.0;
            if (event.key == "ArrowRight") movementDirection.x = 0.0;
            if (event.key == " ") movementDirection.y = -speed; // Gravity
        },
        {signal: testbed.demoSignal},
    );

    testbed.lookAt({
        target: {x: 0.0, y: -1.0},
        zoom: 50.0,
    });
}
