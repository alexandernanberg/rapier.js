import seedrandom from "seedrandom";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // Create the ground.
    let groundDesc = RAPIER.RigidBodyDesc.fixed();
    let ground = world.createRigidBody(groundDesc);
    world.createCollider(RAPIER.ColliderDesc.cuboid(30.0, 0.1, 30.0), ground);

    const identity = {x: 0.0, y: 0.0, z: 0.0, w: 1.0};

    // A dumbbell: a bar with a ball at each end.
    const dumbbell = () =>
        RAPIER.ColliderDesc.compound(
            [new RAPIER.Cuboid(1.2, 0.15, 0.15), new RAPIER.Ball(0.6), new RAPIER.Ball(0.6)],
            [
                {x: 0.0, y: 0.0, z: 0.0},
                {x: -1.4, y: 0.0, z: 0.0},
                {x: 1.4, y: 0.0, z: 0.0},
            ],
            [identity, identity, identity],
        );

    // A chair-like shape: a seat with a backrest.
    const chair = () =>
        RAPIER.ColliderDesc.compound(
            [new RAPIER.Cuboid(1.0, 0.15, 1.0), new RAPIER.Cuboid(1.0, 1.0, 0.15)],
            [
                {x: 0.0, y: 0.0, z: 0.0},
                {x: 0.0, y: 1.0, z: -0.85},
            ],
            [identity, identity],
        );

    let rng = seedrandom("compoundShapes");

    for (let j = 0; j < 8; ++j) {
        for (let i = 0; i < 4; ++i) {
            for (let k = 0; k < 4; ++k) {
                let x = i * 4.0 - 6.0;
                let y = j * 3.0 + 3.0;
                let z = k * 4.0 - 6.0;

                let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z);
                let body = world.createRigidBody(bodyDesc);
                world.createCollider(rng() < 0.5 ? dumbbell() : chair(), body);
            }
        }
    }

    testbed.setWorld(world);

    testbed.lookAt({
        eye: {x: -30.0, y: 30.0, z: 30.0},
        target: {x: 0.0, y: 5.0, z: 0.0},
    });
}
