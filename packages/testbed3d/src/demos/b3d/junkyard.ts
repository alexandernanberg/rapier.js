import type {Testbed} from "../../Testbed";
import {createCylinder, createRock} from "./util";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

/**
 * Port of box3d's `junkyard` benchmark (`CreateJunkyard` + `StepJunkyard`,
 * `box3d/shared/benchmarks.c`). Release settings: a walled arena filled with a
 * stack of convex "rocks", stirred by an orbiting kinematic cylinder.
 */
export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    // box3d's `b3DefaultWorldDef`: gravity (0, -10, 0). box3d steps at dt = 1/60
    // with 4 solver substeps, matching rapier's defaults.
    let gravity = new RAPIER.Vector3(0.0, -10.0, 0.0);
    let world = new RAPIER.World(gravity);

    // Ground + walls: one fixed body at y = -1 with 5 box colliders.
    let groundDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(0.0, -1.0, 0.0);
    let ground = world.createRigidBody(groundDesc);
    world.createCollider(RAPIER.ColliderDesc.cuboid(120.0, 1.0, 120.0), ground);

    let walls: Array<[number, number, number, number, number, number]> = [
        [1.0, 8.0, 50.0, -50.0, 8.0, 0.0],
        [1.0, 8.0, 50.0, 50.0, 8.0, 0.0],
        [50.0, 8.0, 1.0, 0.0, 8.0, -50.0],
        [50.0, 8.0, 1.0, 0.0, 8.0, 50.0],
    ];

    walls.forEach(([hx, hy, hz, x, y, z]) => {
        world.createCollider(
            RAPIER.ColliderDesc.cuboid(hx, hy, hz).setTranslation(x, y, z),
            ground,
        );
    });

    // Rocks. box3d shares a single hull across all of them; the same points here
    // means the renderer sees one geometry and instances the whole pile into a
    // single draw call. Upstream stacks 24 layers of 21x21 (10,584 rocks); 12
    // layers of 15x15 keeps it around 2.7k.
    let rock = createRock(1.5);
    let layers = 12;
    let side = 15;
    let height = 24.0;
    let x, y, z;

    for (y = 0; y < layers; ++y) {
        for (x = 0; x < side; ++x) {
            for (z = 0; z < side; ++z) {
                let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
                    -40.0 + 4.0 * x,
                    4.0 * y + height + 1.0,
                    -40.0 + 4.0 * z,
                );
                let body = world.createRigidBody(bodyDesc);
                let colliderDesc = RAPIER.ColliderDesc.convexHull(rock);

                if (colliderDesc !== null) {
                    world.createCollider(colliderDesc, body);
                }
            }
        }
    }

    // Orbiting kinematic pusher.
    let radius = 35.0;
    let pusherDesc = RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(radius, 0.0, 0.0);
    let pusher = world.createRigidBody(pusherDesc);
    let pusherHull = RAPIER.ColliderDesc.convexHull(createCylinder(24.0, 4.0, 0.0, 16));

    if (pusherHull !== null) {
        world.createCollider(pusherHull, pusher);
    }

    testbed.setWorld(world);

    let pusherHandle = pusher.handle;
    let degrees = 0.0;
    let timeStep = 1.0 / 60.0;
    let omega = -6.0;
    let target = {x: radius, y: 0.0, z: 0.0};

    testbed.setpreTimestepAction(() => {
        let body = testbed.world.getRigidBody(pusherHandle);

        if (body === null) {
            return;
        }

        degrees += omega * timeStep;
        let rad = (degrees * Math.PI) / 180.0;
        target.x = radius * Math.cos(rad);
        target.z = radius * Math.sin(rad);
        body.setNextKinematicTranslation(target);
    });

    let cameraPosition = {
        eye: {x: 0.0, y: 90.0, z: 125.0},
        target: {x: 0.0, y: 0.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
