import {Quaternion, Vector3} from "three";
import type {Testbed} from "../../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

const _zAxis = new Vector3(0.0, 0.0, 1.0);

/**
 * Port of box3d's `washer` benchmark (`CreateWasher`,
 * `box3d/shared/benchmarks.c`). Release settings: a spinning kinematic "washer"
 * (a ring built from ~40 convex hulls) tumbling a grid of small cubes.
 */
export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    // box3d's `b3DefaultWorldDef`: gravity (0, -10, 0). box3d steps at dt = 1/60
    // with 4 solver substeps, matching rapier's defaults.
    let gravity = new RAPIER.Vector3(0.0, -10.0, 0.0);
    let world = new RAPIER.World(gravity);

    // Create Ground.
    let groundDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(0.0, -1.0, 0.0);
    let ground = world.createRigidBody(groundDesc);
    world.createCollider(RAPIER.ColliderDesc.cuboid(60.0, 1.0, 60.0), ground);

    // Spinning kinematic washer body (velocity-based: constant angular plus a
    // tiny linear velocity, matching box3d's kinematic branch).
    let motorSpeed = 25.0;
    let washerDesc = RAPIER.RigidBodyDesc.kinematicVelocityBased()
        .setTranslation(0.0, 21.0, 0.0)
        .setAngvel(new RAPIER.Vector3(0.0, 0.0, (Math.PI / 180.0) * motorSpeed))
        .setLinvel(0.001, -0.002, 0.0);
    let washer = world.createRigidBody(washerDesc);

    let r0 = 14.0;
    let r1 = 16.0;
    let r2 = 18.0;
    let negD = -10.0;
    let posD = 10.0;

    let angle = Math.PI / 18.0;
    let q = new Quaternion().setFromAxisAngle(_zAxis, angle);
    let qo = new Quaternion().setFromAxisAngle(_zAxis, 0.1 * angle);
    let qoInv = qo.clone().invert();
    let u1 = new Vector3(1.0, 0.0, 0.0);
    let i;

    // Each segment of the ring is a box-ish hull spanning two radii and the
    // washer's depth; every ninth one also gets an inner paddle.
    let hull = (rInner: number, rOuter: number, a1: Vector3, a2: Vector3) => {
        let points = new Float32Array(8 * 3);
        let radii = [rInner, rOuter];
        let axes = [a1, a2];
        let n = 0;

        [negD, posD].forEach((d) => {
            axes.forEach((axis) => {
                radii.forEach((r) => {
                    points[n++] = r * axis.x;
                    points[n++] = r * axis.y;
                    points[n++] = d + r * axis.z;
                });
            });
        });

        return RAPIER.ColliderDesc.convexHull(points);
    };

    for (i = 0; i < 36; ++i) {
        let u2 = i == 35 ? new Vector3(1.0, 0.0, 0.0) : u1.clone().applyQuaternion(q);

        let a1 = u1.clone().applyQuaternion(qoInv);
        let a2 = u2.clone().applyQuaternion(qo);
        let segment = hull(r1, r2, a1, a2);

        if (segment !== null) {
            world.createCollider(segment, washer);
        }

        if (i % 9 == 0) {
            let paddle = hull(r0, r1, u1, u2);

            if (paddle !== null) {
                world.createCollider(paddle, washer);
            }
        }

        u1 = u2;
    }

    // Grid of small cubes. Upstream drops 20x20x20 of them (8000 cubes); 15 keeps
    // it around 3.4k.
    let gridCount = 15;
    let a = 0.2;
    let j, k;

    let x = -2.0 * a * gridCount;
    for (i = 0; i < gridCount; ++i) {
        let y = -2.0 * a * gridCount + 21.0;

        for (j = 0; j < gridCount; ++j) {
            let z = -2.0 * a * gridCount;

            for (k = 0; k < gridCount; ++k) {
                let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z);
                let body = world.createRigidBody(bodyDesc);
                world.createCollider(RAPIER.ColliderDesc.cuboid(a, a, a).setDensity(1000.0), body);
                z += 4.0 * a;
            }

            y += 4.0 * a;
        }

        x += 4.0 * a;
    }

    testbed.setWorld(world);

    let cameraPosition = {
        eye: {x: 60.0, y: 35.0, z: 60.0},
        target: {x: 0.0, y: 15.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
