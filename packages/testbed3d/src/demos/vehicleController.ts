import * as THREE from "three";
import type {Graphics} from "../Graphics";
import type {Testbed} from "../Testbed";
import {VehicleController} from "../VehicleController";
import {
    spawnVehicle,
    VEHICLE_PRESET_NAMES,
    VEHICLE_PRESETS,
    type VehiclePresetName,
} from "../vehiclePresets";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

// Steering sign. +1 means A/left steers the car left, which is verified by
// the simVehicle steering-direction test; this is not machine dependent.
const STEER_SIGN = 1;

// Scratch objects reused every frame to avoid per-frame allocations.
const _carPos = new THREE.Vector3();
const _forward = new THREE.Vector3();
const _desiredEye = new THREE.Vector3();
const _camTarget = new THREE.Vector3();
const _dirWs = new THREE.Vector3();
const _wheelPos = new THREE.Vector3();
const _chassisQuat = new THREE.Quaternion();
const _qSteer = new THREE.Quaternion();
const _qSpin = new THREE.Quaternion();
const _wheelQuat = new THREE.Quaternion();
const _axisUp = new THREE.Vector3(0, 1, 0);
const _axisAxle = new THREE.Vector3(1, 0, 0);
// Aligns a cylinder (whose length runs along Y) onto the wheel's axle (X).
const _qAlign = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), -Math.PI / 2);

function createHud(): HTMLDivElement {
    document.getElementById("vehicle-hud")?.remove();

    const hud = document.createElement("div");
    hud.id = "vehicle-hud";
    hud.className = "demo-overlay";
    hud.style.cssText = [
        "position:absolute",
        "left:12px",
        "bottom:12px",
        "padding:10px 14px",
        "font:14px/1.4 ui-monospace,Menlo,Consolas,monospace",
        "color:#fff",
        "background:rgba(0,0,0,0.45)",
        "border-radius:8px",
        "pointer-events:none",
        "white-space:pre",
        "z-index:10",
    ].join(";");
    document.body.appendChild(hud);
    return hud;
}

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    const gfx = testbed.graphics;
    const gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    const world = new RAPIER.World(gravity);

    // --- Ground -----------------------------------------------------------
    const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    world.createCollider(
        RAPIER.ColliderDesc.cuboid(120.0, 0.5, 120.0).setTranslation(0, -0.5, 0).setFriction(1.2),
        ground,
    );

    // --- Ramps to jump off ------------------------------------------------
    const rampBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    const makeRamp = (x: number, z: number, angle: number) => {
        const rot = new RAPIER.Quaternion(Math.sin(angle / 2), 0, 0, Math.cos(angle / 2));
        world.createCollider(
            RAPIER.ColliderDesc.cuboid(4.0, 0.25, 5.0)
                .setTranslation(x, Math.sin(angle) * 5.0 - 0.2, z)
                .setRotation(rot)
                .setFriction(1.0),
            rampBody,
        );
    };
    makeRamp(0, 20, -0.32);
    makeRamp(-12, 35, -0.4);

    // --- A few static blocks + a wall of crates to knock around -----------
    for (let i = 0; i < 5; i++) {
        const crate = world.createRigidBody(
            RAPIER.RigidBodyDesc.dynamic().setTranslation(8 + (i % 2), 0.5 + Math.floor(i / 2), 28),
        );
        world.createCollider(RAPIER.ColliderDesc.cuboid(0.5, 0.5, 0.5).setDensity(20), crate);
    }
    for (let i = 0; i < 8; i++) {
        const cone = world.createRigidBody(
            RAPIER.RigidBodyDesc.dynamic().setTranslation(-6 + i * 1.5, 0.4, 10),
        );
        world.createCollider(RAPIER.ColliderDesc.cone(0.4, 0.35).setDensity(8), cone);
    }

    // --- Vehicle (built from a preset; swap cars with the number keys) -----
    const spawn = {x: 0, y: 1.5, z: -20};
    let currentPreset: VehiclePresetName = "skyline";
    let vehicle = spawnVehicle(RAPIER, world, currentPreset, spawn);

    // --- Wheel visuals (raycast wheels have no colliders of their own) ----
    // Remove a stale wheel group if this demo is being re-loaded (the testbed
    // only auto-cleans collider meshes, not custom objects we add ourselves).
    gfx.scene.getObjectByName("vehicle-wheels")?.removeFromParent();
    const wheelGroup = new THREE.Group();
    wheelGroup.name = "vehicle-wheels";
    wheelGroup.userData.demoObject = true; // let Graphics.reset() clean it up
    gfx.scene.add(wheelGroup);

    const wheelMaterial = new THREE.MeshPhongMaterial({color: 0x111111, flatShading: true});
    const wheelMeshes: THREE.Mesh[] = [];

    const buildWheels = () => {
        for (const mesh of wheelMeshes) {
            mesh.geometry.dispose();
            wheelGroup.remove(mesh);
        }
        wheelMeshes.length = 0;

        const r = vehicle.options.wheelRadius;
        const geometry = new THREE.CylinderGeometry(r, r, 0.3, 20);
        for (let i = 0; i < vehicle.wheelCount; i++) {
            const mesh = new THREE.Mesh(geometry, wheelMaterial);
            wheelGroup.add(mesh);
            wheelMeshes.push(mesh);
        }
    };
    buildWheels();

    const switchPreset = (name: VehiclePresetName) => {
        // Tear down the current car (graphics first, while its colliders exist).
        gfx.removeRigidBody(vehicle.chassis);
        world.removeVehicleController(vehicle.controller);
        world.removeRigidBody(vehicle.chassis);

        // Spawn the new one and register its colliders with the renderer.
        currentPreset = name;
        vehicle = spawnVehicle(RAPIER, world, name, spawn);
        for (let i = 0; i < vehicle.chassis.numColliders(); i++) {
            gfx.addCollider(RAPIER, world, vehicle.chassis.collider(i));
        }
        buildWheels();
    };

    let hud!: HTMLDivElement;

    // --- Input ------------------------------------------------------------
    const keys = {forward: false, back: false, left: false, right: false, handbrake: false};

    const respawn = () => {
        vehicle.chassis.setTranslation(new RAPIER.Vector3(spawn.x, spawn.y, spawn.z), true);
        vehicle.chassis.setRotation(new RAPIER.Quaternion(0, 0, 0, 1), true);
        vehicle.chassis.setLinvel(new RAPIER.Vector3(0, 0, 0), true);
        vehicle.chassis.setAngvel(new RAPIER.Vector3(0, 0, 0), true);
    };

    // NOTE: these are attached *after* `testbed.setWorld()` below, because
    // setWorld clears `document.onkeydown` / `onkeyup` to drop the previous
    // demo's bindings. Attaching them here would simply be wiped.
    const onKeyDown = (event: KeyboardEvent) => {
        switch (event.key.toLowerCase()) {
            case "arrowup":
            case "w":
                keys.forward = true;
                break;
            case "arrowdown":
            case "s":
                keys.back = true;
                break;
            case "arrowleft":
            case "a":
                keys.left = true;
                break;
            case "arrowright":
            case "d":
                keys.right = true;
                break;
            case " ":
                keys.handbrake = true;
                break;
            case "r":
                respawn();
                break;
            default: {
                // Number keys 1..N switch between vehicle presets.
                const index = Number.parseInt(event.key, 10) - 1;
                if (index >= 0 && index < VEHICLE_PRESET_NAMES.length) {
                    switchPreset(VEHICLE_PRESET_NAMES[index]);
                }
            }
        }
    };
    const onKeyUp = (event: KeyboardEvent) => {
        switch (event.key.toLowerCase()) {
            case "arrowup":
            case "w":
                keys.forward = false;
                break;
            case "arrowdown":
            case "s":
                keys.back = false;
                break;
            case "arrowleft":
            case "a":
                keys.left = false;
                break;
            case "arrowright":
            case "d":
                keys.right = false;
                break;
            case " ":
                keys.handbrake = false;
                break;
        }
    };

    const menu = VEHICLE_PRESET_NAMES.map((n, i) => `${i + 1} ${VEHICLE_PRESETS[n].label}`).join(
        "   ",
    );

    // Wheel and chassis pose at the last two physics steps, so the renderer can
    // interpolate them the way it already interpolates the colliders. Driving
    // meshes straight from the fixed-step callback makes them stutter against
    // the smoothly interpolated body.
    const wheelPose = [0, 1, 2, 3].map(() => ({
        prevPos: new THREE.Vector3(),
        pos: new THREE.Vector3(),
        prevQuat: new THREE.Quaternion(),
        quat: new THREE.Quaternion(),
    }));
    const carPose = {
        prevPos: new THREE.Vector3(),
        pos: new THREE.Vector3(),
        prevQuat: new THREE.Quaternion(),
        quat: new THREE.Quaternion(),
    };

    const capturePose = () => {
        const chassis = vehicle.chassis;
        const rot = chassis.rotation();
        _chassisQuat.set(rot.x, rot.y, rot.z, rot.w);
        const t = chassis.translation();
        carPose.prevPos.copy(carPose.pos);
        carPose.prevQuat.copy(carPose.quat);
        carPose.pos.set(t.x, t.y, t.z);
        carPose.quat.copy(_chassisQuat);

        _dirWs.set(0, -1, 0).applyQuaternion(_chassisQuat);
        for (let i = 0; i < wheelPose.length; i++) {
            const pose = wheelPose[i];
            pose.prevPos.copy(pose.pos);
            pose.prevQuat.copy(pose.quat);

            const hardPoint = vehicle.controller.wheelHardPoint(i);
            if (!hardPoint) continue;
            const suspension =
                vehicle.controller.wheelSuspensionLength(i) ?? vehicle.options.suspensionRestLength;
            const steer = vehicle.controller.wheelSteering(i) ?? 0;
            const spin = vehicle.controller.wheelRotation(i) ?? 0;

            pose.pos.set(hardPoint.x, hardPoint.y, hardPoint.z).addScaledVector(_dirWs, suspension);
            _qSteer.setFromAxisAngle(_axisUp, steer);
            _qSpin.setFromAxisAngle(_axisAxle, spin);
            pose.quat.copy(_chassisQuat).multiply(_qSteer).multiply(_qSpin).multiply(_qAlign);
        }
    };
    capturePose();
    capturePose();

    // `restoreSnapshot` frees the world and swaps in a deserialised copy, which
    // invalidates the chassis and the raw vehicle controller hanging off it.
    // Rebuild against the new world when that happens.
    let activeWorld = world;
    const rebindAfterSnapshotRestore = () => {
        if (testbed.world === activeWorld) return;
        const chassis = testbed.world.getRigidBody(vehicle.chassis.handle);
        if (!chassis) return;
        activeWorld = testbed.world;
        vehicle = new VehicleController(activeWorld, chassis, {
            ...VEHICLE_PRESETS[currentPreset].controller,
        });
        capturePose();
        capturePose();
    };

    // Fixed-step callback: input and physics only.
    const update = () => {
        rebindAfterSnapshotRestore();
        // Map keyboard to driver input.
        vehicle.input.accelerate = keys.forward ? 1 : 0;
        vehicle.input.brake = keys.back ? 1 : 0;
        vehicle.input.steer = STEER_SIGN * ((keys.left ? 1 : 0) - (keys.right ? 1 : 0));
        vehicle.input.handbrake = keys.handbrake;

        vehicle.update(activeWorld.timestep);
        capturePose();
    };

    // Once per rendered frame, interpolated by `alpha`.
    const render = (graphics: Graphics, alpha: number) => {
        for (let i = 0; i < wheelMeshes.length; i++) {
            const pose = wheelPose[i];
            wheelMeshes[i].position.lerpVectors(pose.prevPos, pose.pos, alpha);
            wheelMeshes[i].quaternion.copy(pose.prevQuat).slerp(pose.quat, alpha);
        }

        // Chase camera, from the interpolated chassis pose.
        _carPos.lerpVectors(carPose.prevPos, carPose.pos, alpha);
        _chassisQuat.copy(carPose.prevQuat).slerp(carPose.quat, alpha);
        _forward.set(0, 0, 1).applyQuaternion(_chassisQuat);
        _forward.y = 0;
        if (_forward.lengthSq() > 1e-4) _forward.normalize();

        _desiredEye
            .copy(_carPos)
            .addScaledVector(_forward, -9)
            .setY(_carPos.y + 4.5);
        graphics.camera.position.lerp(_desiredEye, 0.12);
        _camTarget.copy(_carPos).setY(_carPos.y + 1.0);
        graphics.controls.target.lerp(_camTarget, 0.2);

        // HUD.
        const speed = vehicle.currentSpeed();
        const kmh = Math.abs(speed) * 3.6;
        const gear = speed > 0.5 ? "D" : speed < -0.5 ? "R" : "N";
        hud.textContent =
            `${VEHICLE_PRESETS[currentPreset].label}\n` +
            `${kmh.toFixed(0).padStart(3)} km/h   [${gear}]${keys.handbrake ? "  HANDBRAKE" : ""}\n` +
            `W/↑ accel · S/↓ brake-reverse · A/D steer · Space handbrake · R reset\n` +
            `cars:  ${menu}`;
    };

    testbed.setWorld(world);
    hud = createHud(); // after setWorld, which clears the previous demo's overlays
    testbed.setpreTimestepAction(update);
    testbed.setRenderAction(render);
    testbed.useChaseCamera();

    // Only now, since setWorld() clears the previous demo's key bindings.
    document.onkeydown = onKeyDown;
    document.onkeyup = onKeyUp;

    testbed.lookAt({
        eye: {x: 0, y: 6, z: -32},
        target: {x: 0, y: 1, z: -20},
    });
}
