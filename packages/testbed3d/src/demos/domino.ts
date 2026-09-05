import {Quaternion, Vector3} from "three";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

const _yAxis = new Vector3(0.0, 1.0, 0.0);
const _tiltAxis = new Vector3();
const _rot = new Quaternion();
const _tiltRot = new Quaternion();

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // Create Ground.
    let groundSize = 200.1;
    let groundHeight = 0.1;
    let groundDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(0.0, -groundHeight, 0.0);
    let ground = world.createRigidBody(groundDesc);
    let groundColliderDesc = RAPIER.ColliderDesc.cuboid(groundSize, groundHeight, groundSize);
    world.createCollider(groundColliderDesc, ground);

    // A spiral of dominoes: each one is placed a fixed arc-length after the
    // previous one, on a circle whose radius grows a little on every turn. The
    // domino that closes a turn is tilted so it topples into the next one, and
    // the five dominoes after it are skipped to leave room for it to fall.
    let num = 4000;
    let width = 1.0;
    let thickness = 0.1;

    let currAngle = 0.0;
    let currRad = 10.0;
    let skip = 0;
    let i;

    for (i = 0; i < num; ++i) {
        let perimeter = 2.0 * Math.PI * currRad;
        let spacing = thickness * 4.0;
        let prevAngle = currAngle;
        currAngle += (2.0 * Math.PI * spacing) / perimeter;
        let x = Math.sin(currAngle);
        let z = Math.cos(currAngle);

        let twoPi = 2.0 * Math.PI;
        let nudged = currAngle % twoPi < prevAngle % twoPi;
        let tilt = nudged || i == num - 1 ? 0.2 : 0.0;

        if (skip == 0) {
            _rot.setFromAxisAngle(_yAxis, currAngle);
            _tiltAxis.set(0.0, 0.0, 1.0).applyQuaternion(_rot);
            _tiltRot.setFromAxisAngle(_tiltAxis, tilt).multiply(_rot);

            let bodyDesc = RAPIER.RigidBodyDesc.dynamic()
                .setTranslation(x * currRad, width * 2.0 + groundHeight, z * currRad)
                .setRotation(_tiltRot);
            let body = world.createRigidBody(bodyDesc);
            let colliderDesc = RAPIER.ColliderDesc.cuboid(thickness, width * 2.0, width);
            world.createCollider(colliderDesc, body);
        } else {
            skip -= 1;
        }

        if (nudged) {
            skip = 5;
        }

        currRad += 1.5 / perimeter;
    }

    testbed.setWorld(world);

    let cameraPosition = {
        eye: {x: 100.0, y: 100.0, z: 100.0},
        target: {x: 0.0, y: 0.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
