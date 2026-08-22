import * as THREE from "three";
import type {Graphics} from "../Graphics";
import type {DifferentialType} from "../sim/drivetrain";
import type {Testbed} from "../Testbed";
import {ChaseCamera} from "../ChaseCamera";
import {ROAD_CAR, ROAD_CAR_CHASSIS} from "../sim/roadCarSetup";
import {SimVehicleController} from "../sim/SimVehicleController";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

// Steering sign. +1 means A/left steers the car left, which is verified by
// the simVehicle steering-direction test; this is not machine dependent.
const STEER_SIGN = 1;

const MASS = ROAD_CAR_CHASSIS.mass;
// Narrower than the track (2 * 0.78 + tyre width) so the wheels sit proud of
// the bodywork and you can actually watch them steer and spin.
const BODY = {
    w: ROAD_CAR_CHASSIS.size.x,
    h: ROAD_CAR_CHASSIS.size.y,
    d: ROAD_CAR_CHASSIS.size.z,
};

// Scratch objects reused every frame.
const _carPos = new THREE.Vector3();
const _forward = new THREE.Vector3();
const _desiredEye = new THREE.Vector3();
const _camTarget = new THREE.Vector3();
const _chassisQuat = new THREE.Quaternion();
const _qSteer = new THREE.Quaternion();
const _qSpin = new THREE.Quaternion();
const _wheelQuat = new THREE.Quaternion();
const _axisUp = new THREE.Vector3(0, 1, 0);
const _axisAxle = new THREE.Vector3(1, 0, 0);
const _qAlign = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), -Math.PI / 2);

function createHud(): HTMLDivElement {
    document.getElementById("sim-vehicle-hud")?.remove();
    const hud = document.createElement("div");
    hud.id = "sim-vehicle-hud";
    hud.className = "demo-overlay";
    hud.style.cssText = [
        "position:absolute",
        "left:12px",
        "bottom:12px",
        "padding:10px 14px",
        "font:13px/1.45 ui-monospace,Menlo,Consolas,monospace",
        "color:#fff",
        "background:rgba(0,0,0,0.5)",
        "border-radius:8px",
        "pointer-events:none",
        "white-space:pre",
        "z-index:10",
    ].join(";");
    document.body.appendChild(hud);
    return hud;
}

/** A little text bar, e.g. for the rev counter. */
function bar(fraction: number, width = 20): string {
    const filled = Math.max(0, Math.min(width, Math.round(fraction * width)));
    return "█".repeat(filled) + "░".repeat(width - filled);
}

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    const gfx = testbed.graphics;
    const world = new RAPIER.World(new RAPIER.Vector3(0, -9.81, 0));

    // --- Ground, with a low-grip patch to feel the differential ------------
    const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    world.createCollider(
        RAPIER.ColliderDesc.cuboid(400, 0.5, 400).setTranslation(0, -0.5, 0).setFriction(1.0),
        ground,
    );
    // An icy strip running down the left of the straight (+X is the driver's
    // left here): put two wheels on it and an open diff will simply spin them.
    world.createCollider(
        RAPIER.ColliderDesc.cuboid(2.2, 0.5, 60).setTranslation(3.0, -0.49, 40).setFriction(0.1),
        ground,
    );

    // --- A ramp and some cones -------------------------------------------
    const rampRot = new RAPIER.Quaternion(Math.sin(-0.16), 0, 0, Math.cos(-0.16));
    world.createCollider(
        RAPIER.ColliderDesc.cuboid(5, 0.25, 6)
            .setTranslation(14, 1.2, 30)
            .setRotation(rampRot)
            .setFriction(1.0),
        ground,
    );
    for (let i = 0; i < 10; i++) {
        const cone = world.createRigidBody(
            RAPIER.RigidBodyDesc.dynamic().setTranslation(-10 + i * 2.2, 0.4, 14),
        );
        world.createCollider(RAPIER.ColliderDesc.cone(0.4, 0.35).setDensity(6), cone);
    }

    // --- The car ----------------------------------------------------------
    const spawn = {x: 0, y: 0.9, z: -25};
    const {w, h, d} = BODY;
    let chassis = world.createRigidBody(
        RAPIER.RigidBodyDesc.dynamic()
            .setTranslation(spawn.x, spawn.y, spawn.z)
            .setAdditionalMassProperties(
                MASS,
                {x: 0, y: -ROAD_CAR_CHASSIS.comDrop, z: 0},
                {
                    x: (MASS / 12) * (h * h + d * d),
                    y: (MASS / 12) * (w * w + d * d),
                    z: (MASS / 12) * (w * w + h * h),
                },
                {x: 0, y: 0, z: 0, w: 1},
            )
            .setCanSleep(false)
            .setCcdEnabled(true),
    );
    world.createCollider(RAPIER.ColliderDesc.cuboid(w / 2, h / 2, d / 2).setFriction(0.4), chassis);
    world.createCollider(
        RAPIER.ColliderDesc.cuboid(w / 2 - 0.22, h / 2, d / 4)
            .setTranslation(0, h, -0.3)
            .setFriction(0.4),
        chassis,
    );

    const setup = ROAD_CAR;
    let car = new SimVehicleController(world, chassis, setup);
    const chassisHandle = chassis.handle;

    // --- Wheel visuals ----------------------------------------------------
    gfx.scene.getObjectByName("sim-vehicle-wheels")?.removeFromParent();
    const wheelGroup = new THREE.Group();
    wheelGroup.name = "sim-vehicle-wheels";
    wheelGroup.userData.demoObject = true; // let Graphics.reset() clean it up
    gfx.scene.add(wheelGroup);

    const markerMaterial = new THREE.MeshPhongMaterial({color: 0xd8d8d8});
    const wheelMeshes = car.wheels.map((wheel) => {
        const r = car.axleOf(wheel).wheelRadius;
        const mesh = new THREE.Mesh(
            new THREE.CylinderGeometry(r, r, 0.28, 22),
            new THREE.MeshPhongMaterial({color: 0x101010, flatShading: true}),
        );
        // A stripe across each face. A bare cylinder gives no clue which way it
        // is turning (and strobes badly at speed), so this makes wheelspin,
        // lock-up and direction of travel immediately readable.
        for (const side of [-1, 1]) {
            const stripe = new THREE.Mesh(
                new THREE.BoxGeometry(r * 1.5, 0.02, r * 0.22),
                markerMaterial,
            );
            stripe.position.set(0, side * 0.145, 0);
            mesh.add(stripe);
        }
        wheelGroup.add(mesh);
        return mesh;
    });

    const chaseCamera = new ChaseCamera();
    let hud!: HTMLDivElement;

    // --- Input ------------------------------------------------------------
    const keys = {up: false, down: false, left: false, right: false, space: false};
    const diffTypes: DifferentialType[] = ["open", "lsd", "locked"];
    let diffIndex = 1;

    const respawn = () => {
        chassis.setTranslation(new RAPIER.Vector3(spawn.x, spawn.y, spawn.z), true);
        chassis.setRotation(new RAPIER.Quaternion(0, 0, 0, 1), true);
        chassis.setLinvel(new RAPIER.Vector3(0, 0, 0), true);
        chassis.setAngvel(new RAPIER.Vector3(0, 0, 0), true);
        for (const wheel of car.wheels) wheel.omega = 0;
        chaseCamera.reset();
        car.gearState.gear = 1;
        car.gearState.shiftCooldown = 0;
    };

    // NOTE: these are attached *after* `testbed.setWorld()` below, because
    // setWorld clears `document.onkeydown` / `onkeyup` to drop the previous
    // demo's bindings. Attaching them here would simply be wiped.
    const onKeyDown = (event: KeyboardEvent) => {
        switch (event.key.toLowerCase()) {
            case "arrowup":
            case "w":
                keys.up = true;
                break;
            case "arrowdown":
            case "s":
                keys.down = true;
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
                keys.space = true;
                break;
            case "r":
                respawn();
                break;
            case "e":
                // Cycle the differential to feel it on the ice strip.
                diffIndex = (diffIndex + 1) % diffTypes.length;
                car.differential.type = diffTypes[diffIndex];
                break;
            case "q":
                // Toggle the anti-roll bars to feel the balance change.
                car.front.antiRollStiffness = car.front.antiRollStiffness > 0 ? 0 : 16000;
                car.rear.antiRollStiffness = car.rear.antiRollStiffness > 0 ? 0 : 11000;
                break;
            case "f":
                // Toggle downforce.
                car.aero.downforceCoefficient = car.aero.downforceCoefficient > 0 ? 0 : 1.6;
                break;
            case "t":
                // Traction control off lets a keyboard throttle light up the
                // rears; on, it keeps wheelspin near the tyre's peak.
                car.options.tractionControl = car.options.tractionControl > 0 ? 0 : 0.25;
                break;
        }
    };
    const onKeyUp = (event: KeyboardEvent) => {
        switch (event.key.toLowerCase()) {
            case "arrowup":
            case "w":
                keys.up = false;
                break;
            case "arrowdown":
            case "s":
                keys.down = false;
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
                keys.space = false;
                break;
        }
    };

    // Pose of each wheel (and the chassis) at the last two physics steps, so
    // the renderer can interpolate between them exactly as it does for the
    // colliders. Without this the body slides smoothly while the wheels and
    // camera jump, because the physics step and the frame do not line up.
    const wheelPose = car.wheels.map(() => ({
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
        const rot = chassis.rotation();
        _chassisQuat.set(rot.x, rot.y, rot.z, rot.w);
        const t = chassis.translation();

        carPose.prevPos.copy(carPose.pos);
        carPose.prevQuat.copy(carPose.quat);
        carPose.pos.set(t.x, t.y, t.z);
        carPose.quat.copy(_chassisQuat);

        for (let i = 0; i < wheelPose.length; i++) {
            const wheel = car.wheels[i];
            const pose = wheelPose[i];
            pose.prevPos.copy(pose.pos);
            pose.prevQuat.copy(pose.quat);
            pose.pos.set(wheel.centre.x, wheel.centre.y, wheel.centre.z);
            _qSteer.setFromAxisAngle(_axisUp, wheel.steer);
            _qSpin.setFromAxisAngle(_axisAxle, wheel.rotation);
            pose.quat.copy(_chassisQuat).multiply(_qSteer).multiply(_qSpin).multiply(_qAlign);
        }
    };
    capturePose();
    capturePose();

    // `restoreSnapshot` frees the whole world and swaps in a deserialised one,
    // which invalidates the chassis and everything the controller holds. The
    // other demos deal with this by looking bodies up by handle each step; a
    // controller owns state, so instead rebuild it against the new world.
    let activeWorld = world;
    const rebindAfterSnapshotRestore = () => {
        if (testbed.world === activeWorld) return;
        const restored = testbed.world.getRigidBody(chassisHandle);
        if (!restored) return;
        activeWorld = testbed.world;
        chassis = restored;
        car = new SimVehicleController(activeWorld, chassis, setup);
        capturePose();
        capturePose();
    };

    // Runs inside the fixed-step loop: input and physics only.
    const update = () => {
        rebindAfterSnapshotRestore();
        car.input.throttle = keys.up ? 1 : 0;
        car.input.brake = keys.down ? 1 : 0;
        car.input.steer = STEER_SIGN * ((keys.left ? 1 : 0) - (keys.right ? 1 : 0));
        car.input.handbrake = keys.space;

        car.update(activeWorld.timestep);
        capturePose();
    };

    // Runs once per rendered frame, interpolated by `alpha`.
    const render = (graphics: Graphics, alpha: number) => {
        for (let i = 0; i < wheelMeshes.length; i++) {
            const pose = wheelPose[i];
            wheelMeshes[i].position.lerpVectors(pose.prevPos, pose.pos, alpha);
            wheelMeshes[i].quaternion.copy(pose.prevQuat).slerp(pose.quat, alpha);
        }

        // Chase camera, from the interpolated chassis pose.
        _carPos.lerpVectors(carPose.prevPos, carPose.pos, alpha);
        _chassisQuat.copy(carPose.prevQuat).slerp(carPose.quat, alpha);
        chaseCamera.update(graphics.camera, graphics.controls, _carPos, _chassisQuat);

        // --- Telemetry: this is the point of the whole exercise ------------
        const speed = car.forwardSpeed();
        const kmh = Math.abs(speed) * 3.6;
        const gear = car.gearState.gear < 0 ? "R" : car.gearState.gear;
        const revFrac = car.rpm / car.engine.redlineRpm;

        const wheelRows = car.wheels
            .map((wheel) => {
                const name = ["FL", "FR", "RL", "RR"][wheel.index];
                const state = !wheel.inContact
                    ? "AIR "
                    : wheel.slip > 0.35
                      ? "SLIP"
                      : wheel.slip > 0.16
                        ? "edge"
                        : "grip";
                return (
                    `  ${name} ${state}` +
                    ` load ${wheel.load.toFixed(0).padStart(5)}N` +
                    ` slip ${wheel.slip.toFixed(2)}` +
                    ` sr ${wheel.slipRatio.toFixed(2).padStart(6)}` +
                    ` sa ${((wheel.slipAngle * 180) / Math.PI).toFixed(1).padStart(6)}°` +
                    ` μ ${wheel.surfaceFriction.toFixed(2)}`
                );
            })
            .join("\n");

        hud.textContent =
            `${kmh.toFixed(0).padStart(3)} km/h   gear ${gear}   ${car.rpm.toFixed(0).padStart(4)} rpm\n` +
            `  [${bar(revFrac)}]${car.rpm >= car.engine.redlineRpm - 100 ? " REDLINE" : ""}\n` +
            `${wheelRows}\n` +
            `diff ${car.differential.type.toUpperCase()}   ` +
            `ARB ${car.front.antiRollStiffness > 0 ? "on" : "off"}   ` +
            `downforce ${car.aero.downforceCoefficient > 0 ? "on" : "off"}   ` +
            `TC ${car.options.tractionControl > 0 ? "on" : "off"}\n` +
            `W/S throttle-brake · A/D steer · Space handbrake · R reset\n` +
            `E cycle diff · Q toggle ARB · F toggle downforce · T traction control ` +
            `· ice strip on the left`;
    };

    testbed.setWorld(world);
    hud = createHud(); // after setWorld, which clears the previous demo's overlays
    testbed.setpreTimestepAction(update);
    testbed.setRenderAction(render);
    testbed.useChaseCamera();

    // Only now, since setWorld() clears the previous demo's key bindings.
    document.onkeydown = onKeyDown;
    document.onkeyup = onKeyUp;

    testbed.lookAt({eye: {x: 0, y: 6, z: -36}, target: {x: 0, y: 1, z: -25}});
}
