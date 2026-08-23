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
    const characterDesc = RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(-10.0, 4.0)
        .setGravityScale(10.0)
        .setSoftCcdPrediction(10.0);
    const character = world.createRigidBody(characterDesc);
    const characterColliderDesc = RAPIER.ColliderDesc.cuboid(0.6, 1.2);
    world.createCollider(characterColliderDesc, character);

    const pidController = world.createPidController(60.0, 0.0, 1.0, RAPIER.PidAxesMask.AllAng);

    // Capture handle instead of body reference so callback survives snapshot restore.
    const characterHandle = character.handle;

    const speed = 0.2;
    const movementDirection = {x: 0.0, y: 0.0};
    const targetVelocity = {x: 0.0, y: 0.0};
    const _targetRotation = 0.0;

    const updateCharacter = () => {
        const charBody = testbed.world.getRigidBody(characterHandle);

        // The handle goes stale if the world is swapped out from under us.
        if (charBody === null) {
            return;
        }

        if (movementDirection.x == 0.0 && movementDirection.y == 0.0) {
            // Only adjust the rotation, but let translation.
            pidController.setAxes(RAPIER.PidAxesMask.AllAng);
        } else if (movementDirection.y == 0.0) {
            // Don't control the linear Y axis so the player can fall down due to gravity.
            pidController.setAxes(RAPIER.PidAxesMask.AllAng | RAPIER.PidAxesMask.LinX);
        } else {
            pidController.setAxes(RAPIER.PidAxesMask.All);
        }

        const targetPoint = charBody.translation();
        targetPoint.x += movementDirection.x;
        targetPoint.y += movementDirection.y;

        pidController.applyLinearCorrection(charBody, targetPoint, targetVelocity);
        pidController.applyAngularCorrection(charBody, 0.0, 0.0);
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
            if (event.key == " ") movementDirection.y = 0.0;
        },
        {signal: testbed.demoSignal},
    );

    testbed.lookAt({
        target: {x: 0.0, y: -1.0},
        zoom: 50.0,
    });
}
