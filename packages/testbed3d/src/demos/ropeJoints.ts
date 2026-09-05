import type {Testbed} from "../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // A small platform with a lip on each side, so the tethered ball has somewhere
    // to be dragged around before it falls off.
    let groundSize = 0.75;
    let groundHeight = 0.1;

    let groundDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(0.0, -groundHeight, 0.0);
    let ground = world.createRigidBody(groundDesc);
    world.createCollider(RAPIER.ColliderDesc.cuboid(groundSize, groundHeight, groundSize), ground);

    let walls: Array<[number, number, number, number, number, number]> = [
        [-groundSize - groundHeight, groundHeight, 0.0, groundHeight, groundHeight, groundSize],
        [groundSize + groundHeight, groundHeight, 0.0, groundHeight, groundHeight, groundSize],
        [0.0, groundHeight, -groundSize - groundHeight, groundSize, groundHeight, groundHeight],
        [0.0, groundHeight, groundSize + groundHeight, groundSize, groundHeight, groundHeight],
    ];

    walls.forEach(([x, y, z, hx, hy, hz]) => {
        let wallDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(x, y, z);
        let wall = world.createRigidBody(wallDesc);
        world.createCollider(RAPIER.ColliderDesc.cuboid(hx, hy, hz), wall);
    });

    // Character we will control manually with the arrow keys.
    let characterDesc = RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(0.0, 0.3, 0.0);
    let character = world.createRigidBody(characterDesc);
    let characterCollider = world.createCollider(
        RAPIER.ColliderDesc.cuboid(0.15, 0.3, 0.15),
        character,
    );

    // Tethered ball: the rope joint lets it move freely until it is 2 units away
    // from the character, and then keeps it there.
    let rad = 0.04;
    let ballDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(1.0, 1.0, 0.0);
    let ball = world.createRigidBody(ballDesc);
    world.createCollider(RAPIER.ColliderDesc.ball(rad), ball);

    let joint = RAPIER.JointData.rope(
        2.0,
        new RAPIER.Vector3(0.0, 0.0, 0.0),
        new RAPIER.Vector3(0.0, 0.0, 0.0),
    );
    world.createImpulseJoint(joint, character, ball, true);

    let characterController = world.createCharacterController(0.01);
    characterController.enableSnapToGround(0.2);

    // Capture handles instead of body references so callbacks survive snapshot restore.
    let characterHandle = character.handle;
    let characterColliderHandle = characterCollider.handle;

    let speed = 0.01;
    let movementDirection = {x: 0.0, y: -speed, z: 0.0};

    let updateCharacter = () => {
        let charBody = testbed.world.getRigidBody(characterHandle);
        let charCollider = testbed.world.getCollider(characterColliderHandle);

        // The handles go stale if the world is swapped out from under us.
        if (charBody === null || charCollider === null) {
            return;
        }

        characterController.computeColliderMovement(charCollider, movementDirection);

        let movement = characterController.computedMovement();
        let newPos = charBody.translation();
        newPos.x += movement.x;
        newPos.y += movement.y;
        newPos.z += movement.z;
        charBody.setNextKinematicTranslation(newPos);
    };

    testbed.setWorld(world);
    testbed.setpreTimestepAction(updateCharacter);

    document.onkeydown = function (event: KeyboardEvent) {
        if (event.key == "ArrowUp") movementDirection.x = speed;
        if (event.key == "ArrowDown") movementDirection.x = -speed;
        if (event.key == "ArrowLeft") movementDirection.z = -speed;
        if (event.key == "ArrowRight") movementDirection.z = speed;
    };

    document.onkeyup = function (event: KeyboardEvent) {
        if (event.key == "ArrowUp" || event.key == "ArrowDown") movementDirection.x = 0.0;
        if (event.key == "ArrowLeft" || event.key == "ArrowRight") movementDirection.z = 0.0;
    };

    let cameraPosition = {
        eye: {x: 4.0, y: 4.0, z: 4.0},
        target: {x: 0.0, y: 0.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
