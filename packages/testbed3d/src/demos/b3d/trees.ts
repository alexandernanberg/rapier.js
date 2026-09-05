import type {Testbed} from "../../Testbed";
import {createCylinder, createWaveMesh} from "./util";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

const _worldCom = {x: 0.0, y: 0.0, z: 0.0};

/**
 * Port of box3d's `trees100` benchmark (`CreateTrees(1)`,
 * `box3d/shared/benchmarks.c`): 50 "tree" bodies, each a stack of 22 convex
 * cylinders, spun onto a sinusoidal wave-mesh ground.
 *
 * Upstream also registers `trees50` and `trees25`, the same scene over a ground
 * tessellated 2x and 4x finer (up to ~960k triangles). They're left out here:
 * the extra triangles are all in the ground mesh, and the browser spends its
 * time uploading them rather than simulating.
 */
export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    // box3d's `b3DefaultWorldDef`: gravity (0, -10, 0). box3d steps at dt = 1/60
    // with 4 solver substeps, matching rapier's defaults.
    let gravity = new RAPIER.Vector3(0.0, -10.0, 0.0);
    let world = new RAPIER.World(gravity);

    let scale = 1;
    let xCount = scale * 150;
    let zCount = scale * 200;
    let cellWidth = 1.0 / scale;

    let mesh = createWaveMesh(xCount, zCount, cellWidth, 0.4, 0.05, 0.1);
    let ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    world.createCollider(RAPIER.ColliderDesc.trimesh(mesh.vertices, mesh.indices), ground);

    // The 22 stacked cylinders shared by every tree body. Building the point
    // clouds once lets all 50 trees reuse them, and lets the renderer instance
    // the repeated hulls.
    let hullCount = 22;
    let hulls = [];
    let y = 1.0;
    let r = 0.75;
    let l = 1.5;
    let i;

    for (i = 0; i < hullCount; ++i) {
        hulls.push(createCylinder(l + 2.0 * r, r, y - r, 6));
        y += l + 2.0 * r;
        r *= 0.95;
    }

    let bodyCount = 50;
    let angularVelocity = -0.5;
    let z = -70.0;

    for (i = 0; i < bodyCount; ++i) {
        let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(0.0, 1.0, z);
        let body = world.createRigidBody(bodyDesc);

        hulls.forEach((points) => {
            let colliderDesc = RAPIER.ColliderDesc.convexHull(points);

            if (colliderDesc !== null) {
                world.createCollider(colliderDesc.setDensity(1.0).setFriction(0.9), body);
            }
        });

        // Spun about `z`, with the linear velocity that puts the rotation center
        // at the body's origin rather than at its center of mass.
        let velocityScale = 0.5 + (0.5 * i) / bodyCount;
        let omegaZ = velocityScale * angularVelocity;
        body.worldCom(_worldCom);

        body.setAngvel(new RAPIER.Vector3(0.0, 0.0, omegaZ), true);
        body.setLinvel(
            // omega x (com - position), with omega along `z`.
            new RAPIER.Vector3(-omegaZ * (_worldCom.y - 1.0), omegaZ * _worldCom.x, 0.0),
            true,
        );

        z += 3.0;
        angularVelocity = -angularVelocity;
    }

    testbed.setWorld(world);

    let cameraPosition = {
        eye: {x: 0.0, y: 30.0, z: 140.0},
        target: {x: 0.0, y: 15.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
