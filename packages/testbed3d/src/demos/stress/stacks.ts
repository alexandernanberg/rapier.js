import type RAPIER from "@alexandernanberg/rapier3d";
import {Quaternion, Vector3} from "three";
import type {Testbed} from "../../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

const _yAxis = new Vector3(0.0, 1.0, 0.0);
const _rotation = new Quaternion();
const _offset = new Vector3();

function createPyramid(
    RAPIER: RAPIER_API,
    world: RAPIER.World,
    offset: RAPIER.Vector3,
    stackHeight: number,
    hext: number,
) {
    let shift = hext * 2.0;
    let i, j, k;

    for (i = 0; i < stackHeight; ++i) {
        for (j = i; j < stackHeight; ++j) {
            for (k = i; k < stackHeight; ++k) {
                let x = (i * shift) / 2.0 + (k - i) * shift + offset.x - stackHeight * hext;
                let y = i * shift + offset.y;
                let z = (i * shift) / 2.0 + (j - i) * shift + offset.z - stackHeight * hext;

                let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z);
                let body = world.createRigidBody(bodyDesc);
                world.createCollider(RAPIER.ColliderDesc.cuboid(hext, hext, hext), body);
            }
        }
    }
}

function createWall(
    RAPIER: RAPIER_API,
    world: RAPIER.World,
    offset: RAPIER.Vector3,
    stackHeight: number,
    hext: number,
) {
    let shift = hext * 2.0;
    let i, j;

    for (i = 0; i < stackHeight; ++i) {
        for (j = i; j < stackHeight; ++j) {
            let y = i * shift + offset.y;
            let z = (i * shift) / 2.0 + (j - i) * shift + offset.z - stackHeight * hext;

            let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(offset.x, y, z);
            let body = world.createRigidBody(bodyDesc);
            world.createCollider(RAPIER.ColliderDesc.cuboid(hext, hext, hext), body);
        }
    }
}

function createTowerCircle(
    RAPIER: RAPIER_API,
    world: RAPIER.World,
    offset: RAPIER.Vector3,
    stackHeight: number,
    nsubdivs: number,
    hext: number,
) {
    let angStep = (Math.PI * 2.0) / nsubdivs;
    let radius = (1.3 * nsubdivs * hext) / Math.PI;
    let shift = hext * 2.0;
    let i, j;

    for (i = 0; i < stackHeight; ++i) {
        for (j = 0; j < nsubdivs; ++j) {
            // Each brick is placed radius away from the tower axis, then the whole
            // ring is turned by its own angle, so consecutive layers interlock.
            _rotation.setFromAxisAngle(_yAxis, (i / 2.0 + j) * angStep);
            _offset.set(0.0, i * shift, radius).applyQuaternion(_rotation);

            let bodyDesc = RAPIER.RigidBodyDesc.dynamic()
                .setTranslation(offset.x + _offset.x, offset.y + _offset.y, offset.z + _offset.z)
                .setRotation(_rotation);
            let body = world.createRigidBody(bodyDesc);
            world.createCollider(RAPIER.ColliderDesc.cuboid(hext, hext, hext), body);
        }
    }
}

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // Create Ground.
    let groundSize = 200.0;
    let groundHeight = 0.1;
    let groundDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(0.0, -groundHeight, 0.0);
    let ground = world.createRigidBody(groundDesc);
    world.createCollider(RAPIER.ColliderDesc.cuboid(groundSize, groundHeight, groundSize), ground);

    // Four pyramids, three walls and a tower, all dropped from 50 units up so
    // they land and collapse into each other.
    let cubeSize = 1.0;
    let bottomy = cubeSize * 50.0;

    createPyramid(RAPIER, world, new RAPIER.Vector3(-110.0, bottomy, 0.0), 12, cubeSize);
    createPyramid(RAPIER, world, new RAPIER.Vector3(-80.0, bottomy, 0.0), 12, cubeSize);
    createPyramid(RAPIER, world, new RAPIER.Vector3(-50.0, bottomy, 0.0), 12, cubeSize);
    createPyramid(RAPIER, world, new RAPIER.Vector3(-20.0, bottomy, 0.0), 12, cubeSize);
    createWall(RAPIER, world, new RAPIER.Vector3(-2.0, bottomy, 0.0), 12, cubeSize);
    createWall(RAPIER, world, new RAPIER.Vector3(4.0, bottomy, 0.0), 12, cubeSize);
    createWall(RAPIER, world, new RAPIER.Vector3(10.0, bottomy, 0.0), 12, cubeSize);
    createTowerCircle(RAPIER, world, new RAPIER.Vector3(25.0, bottomy, 0.0), 8, 24, cubeSize);

    testbed.setWorld(world);

    let cameraPosition = {
        eye: {x: 100.0, y: 100.0, z: 100.0},
        target: {x: 0.0, y: 0.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
