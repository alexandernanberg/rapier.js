import type {Testbed} from "../Testbed";
import {ACCENT_COLOR} from "../Graphics";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

/** Palette slot the boxes go back to once they leave the sensor. */
const IDLE_COLOR = 1;

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // Create Ground.
    let groundSize = 10.1;
    let groundHeight = 0.1;
    let groundDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(0.0, -groundHeight, 0.0);
    let ground = world.createRigidBody(groundDesc);
    let groundColliderDesc = RAPIER.ColliderDesc.cuboid(groundSize, groundHeight, groundSize);
    let groundHandle = world.createCollider(groundColliderDesc, ground).handle;

    // A grid of boxes for the sensor to sweep through.
    let num = 10;
    let rad = 0.2;
    let shift = rad * 2.0;
    let centerx = (shift * num) / 2.0;
    let centerz = (shift * num) / 2.0;
    let i, k;

    let boxes = [];

    for (i = 0; i < num; ++i) {
        for (k = 0; k < num; ++k) {
            let x = i * shift - centerx;
            let z = k * shift - centerz;

            let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, 3.0, z);
            let body = world.createRigidBody(bodyDesc);
            let collider = world.createCollider(RAPIER.ColliderDesc.cuboid(rad, rad, rad), body);
            boxes.push(collider.handle);
        }
    }

    // A cube with a ball-shaped sensor attached to it. The cube is solid so the
    // boxes below can bounce off it; the sensor only reports intersections.
    let sensorBodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(0.0, 5.0, 0.0);
    let sensorBody = world.createRigidBody(sensorBodyDesc);
    world.createCollider(RAPIER.ColliderDesc.cuboid(rad, rad, rad), sensorBody);

    // Density 0 so the sensor doesn't contribute to the rigid-body mass.
    let sensorColliderDesc = RAPIER.ColliderDesc.ball(rad * 5.0)
        .setDensity(0.0)
        .setSensor(true)
        .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS);
    let sensorCollider = world.createCollider(sensorColliderDesc, sensorBody);
    let sensorHandle = sensorCollider.handle;

    testbed.setWorld(world);

    // `setWorld` has just drawn every collider with the palette rotation, so give
    // the boxes a color of their own to make the intersection flag readable.
    boxes.forEach((handle) => {
        let collider = world.getCollider(handle);

        if (collider !== null) {
            testbed.graphics.setColliderColor(RAPIER, world, collider, IDLE_COLOR);
        }
    });

    testbed.setpostTimestepAction((graphics) => {
        testbed.events.drainCollisionEvents((handle1, handle2, started) => {
            // One of the two colliders is the sensor; the other one is the box to
            // recolor. Contacts between two boxes never reach here: only the sensor
            // has `COLLISION_EVENTS` enabled.
            let other = handle1 == sensorHandle ? handle2 : handle1;

            if (other == groundHandle) {
                return;
            }

            let collider = testbed.world.getCollider(other);

            if (collider !== null) {
                graphics.setColliderColor(
                    RAPIER,
                    testbed.world,
                    collider,
                    started ? ACCENT_COLOR : IDLE_COLOR,
                );
            }
        });
    });

    let cameraPosition = {
        eye: {x: 6.0, y: 4.0, z: 6.0},
        target: {x: 0.0, y: 1.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
