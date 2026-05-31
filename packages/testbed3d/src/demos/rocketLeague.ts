import type RAPIER from "@alexandernanberg/rapier3d";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

// ---------------------------------------------------------------------------
// Rocket League-style ball physics prototype.
//
// This demo reproduces the single most important "custom physics" trick from
// Jared Cone's GDC 2018 talk "It IS Rocket Science! The Physics of 'Rocket
// League' Detailed": the car does NOT rely on the solver's contact resolution
// to decide how the ball flies off a hit. Instead, on touch, a *scripted*
// impulse is applied to the ball, directed roughly from the car toward the
// ball and scaled by the closing speed (capped, with a baseline punch). The
// regular collider-vs-collider resolution still runs underneath; this just
// adds the extra "pop" that makes hits feel punchy and predictable.
//
// Numbers here are in the testbed's metric-ish scale, not Rocket League's
// "uu" units. The mechanism is what's faithful, not the constants.
// ---------------------------------------------------------------------------

// Car driving feel.
const DRIVE_SPEED = 18.0; // target ground speed (m/s) under throttle
const BOOST_SPEED = 30.0; // target ground speed while boosting
const TURN_RATE = 2.6; // yaw angular velocity (rad/s) under full steer
const JUMP_SPEED = 9.0; // upward velocity added on jump

// Psyonix-style hit impulse tuning.
const HIT_MARGIN = 0.2; // contact slop added to (ballR + 0) test
const HIT_GAIN = 0.75; // fraction of closing speed converted to added speed
const HIT_BASELINE = 3.0; // minimum added speed so light touches still register
const HIT_MAX_ADDED = 28.0; // cap on added speed from a single hit
const HIT_UP_BIAS = 0.12; // small upward bias so touches tend to pop the ball up
const HIT_COOLDOWN = 6; // steps to wait before the same touch can re-fire

const BALL_RADIUS = 1.0;

type Vec = {x: number; y: number; z: number};

function cross(a: Vec, b: Vec): Vec {
    return {
        x: a.y * b.z - a.z * b.y,
        y: a.z * b.x - a.x * b.z,
        z: a.x * b.y - a.y * b.x,
    };
}

// Rotate vector v by quaternion q (x, y, z, w).
function quatRotate(q: RAPIER.Rotation, v: Vec): Vec {
    let u = {x: q.x, y: q.y, z: q.z};
    let t = cross(u, v);
    t = {x: 2 * t.x, y: 2 * t.y, z: 2 * t.z};
    let ct = cross(u, t);
    return {
        x: v.x + q.w * t.x + ct.x,
        y: v.y + q.w * t.y + ct.y,
        z: v.z + q.w * t.z + ct.z,
    };
}

// Rotate vector v by the inverse (conjugate) of quaternion q.
function quatInvRotate(q: RAPIER.Rotation, v: Vec): Vec {
    return quatRotate({x: -q.x, y: -q.y, z: -q.z, w: q.w} as RAPIER.Rotation, v);
}

function length(v: Vec): number {
    return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

function normalize(v: Vec): Vec {
    let l = length(v);
    if (l < 1e-6) return {x: 0, y: 1, z: 0};
    return {x: v.x / l, y: v.y / l, z: v.z / l};
}

function clamp(v: number, lo: number, hi: number): number {
    return Math.max(lo, Math.min(hi, v));
}

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -13.0, 0.0); // RL gravity is stronger than Earth.
    let world = new RAPIER.World(gravity);

    // --- Arena: floor + four walls (fixed bodies). --------------------------
    let arenaHX = 26.0;
    let arenaHZ = 18.0;
    let wallH = 8.0;
    let wallT = 0.5;

    let floorBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    world.createCollider(
        RAPIER.ColliderDesc.cuboid(arenaHX, 0.5, arenaHZ)
            .setTranslation(0.0, -0.5, 0.0)
            .setFriction(0.7),
        floorBody,
    );

    let makeWall = (hx: number, hz: number, x: number, z: number) => {
        let b = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        world.createCollider(
            RAPIER.ColliderDesc.cuboid(hx, wallH, hz)
                .setTranslation(x, wallH, z)
                .setRestitution(0.5),
            b,
        );
    };
    makeWall(wallT, arenaHZ, arenaHX, 0.0); // +X
    makeWall(wallT, arenaHZ, -arenaHX, 0.0); // -X
    makeWall(arenaHX, wallT, 0.0, arenaHZ); // +Z
    makeWall(arenaHX, wallT, 0.0, -arenaHZ); // -Z

    // --- Ball: light, bouncy, with air drag. --------------------------------
    let ballBody = world.createRigidBody(
        RAPIER.RigidBodyDesc.dynamic()
            .setTranslation(0.0, 4.0, 0.0)
            .setLinearDamping(0.35)
            .setAngularDamping(0.5),
    );
    world.createCollider(
        RAPIER.ColliderDesc.ball(BALL_RADIUS).setDensity(0.18).setRestitution(0.6).setFriction(0.4),
        ballBody,
    );

    // --- Car: a simple cuboid hitbox (just like RL). ------------------------
    // Local axes: +X = forward (length 1.8), Y = height (0.6), Z = width (0.9).
    let carStart = {x: -10.0, y: 0.35, z: 0.0};
    let carBody = world.createRigidBody(
        RAPIER.RigidBodyDesc.dynamic()
            .setTranslation(carStart.x, carStart.y, carStart.z)
            .setLinearDamping(0.2),
    );
    // Keep the car upright: only allow yaw (Y) rotation.
    carBody.setEnabledRotations(false, true, false, true);
    world.createCollider(
        RAPIER.ColliderDesc.cuboid(0.9, 0.3, 0.45).setDensity(2.0).setFriction(0.6),
        carBody,
    );
    // Car cuboid half-extents, reused in the contact test below.
    let carHE = {x: 0.9, y: 0.3, z: 0.45};

    // Capture handles (not body refs) so the callback survives snapshot restore.
    let ballHandle = ballBody.handle;
    let carHandle = carBody.handle;

    // --- Input state. -------------------------------------------------------
    let input = {throttle: 0, steer: 0, boost: false, jump: false};
    let hitCooldown = 0;

    let resetBall = () => {
        let ball = testbed.world.getRigidBody(ballHandle);
        ball.setTranslation({x: 0, y: 4, z: 0}, true);
        ball.setLinvel({x: 0, y: 0, z: 0}, true);
        ball.setAngvel({x: 0, y: 0, z: 0}, true);
    };

    let update = () => {
        let car = testbed.world.getRigidBody(carHandle);
        let ball = testbed.world.getRigidBody(ballHandle);

        let carPos = car.translation();
        let carRot = car.rotation();
        let forward = quatRotate(carRot, {x: 1, y: 0, z: 0});

        // --- Drive the car. Set planar velocity along facing, keep gravity Y.
        let carVel = car.linvel();
        let target = input.boost ? BOOST_SPEED : DRIVE_SPEED;
        let speed = input.throttle * target;
        car.setLinvel({x: forward.x * speed, y: carVel.y, z: forward.z * speed}, true);

        // Steering: yaw rate, a touch tighter when on the throttle.
        let turn = input.steer * TURN_RATE * (input.throttle !== 0 ? 1.0 : 0.5);
        car.setAngvel({x: 0, y: turn, z: 0}, true);

        // Jump: one-shot upward velocity when grounded-ish.
        if (input.jump && carPos.y < 0.6) {
            car.setLinvel({x: carVel.x, y: JUMP_SPEED, z: carVel.z}, true);
        }
        input.jump = false;

        // --- Psyonix-style ball hit. ---------------------------------------
        if (hitCooldown > 0) hitCooldown -= 1;

        let ballPos = ball.translation();
        // Closest point on the car's oriented box to the ball center.
        let rel = {x: ballPos.x - carPos.x, y: ballPos.y - carPos.y, z: ballPos.z - carPos.z};
        let localRel = quatInvRotate(carRot, rel);
        let localClosest = {
            x: clamp(localRel.x, -carHE.x, carHE.x),
            y: clamp(localRel.y, -carHE.y, carHE.y),
            z: clamp(localRel.z, -carHE.z, carHE.z),
        };
        let worldClosest = quatRotate(carRot, localClosest);
        worldClosest = {
            x: carPos.x + worldClosest.x,
            y: carPos.y + worldClosest.y,
            z: carPos.z + worldClosest.z,
        };
        let toBall = {
            x: ballPos.x - worldClosest.x,
            y: ballPos.y - worldClosest.y,
            z: ballPos.z - worldClosest.z,
        };
        let dist = length(toBall);

        if (dist <= BALL_RADIUS + HIT_MARGIN && hitCooldown === 0) {
            // Direction from car center to ball, with a small upward bias.
            let dir = normalize({x: rel.x, y: rel.y + HIT_UP_BIAS * 2.0, z: rel.z});

            // Closing speed of car relative to ball along the hit direction.
            let ballVel = ball.linvel();
            let relVel = {
                x: carVel.x - ballVel.x,
                y: carVel.y - ballVel.y,
                z: carVel.z - ballVel.z,
            };
            let closing = relVel.x * dir.x + relVel.y * dir.y + relVel.z * dir.z;
            if (closing < 0) closing = 0;

            // Curve: baseline punch + gain on closing speed, capped.
            let addedSpeed = clamp(HIT_BASELINE + closing * HIT_GAIN, 0, HIT_MAX_ADDED);
            let mass = ball.mass();
            let j = mass * addedSpeed;
            ball.applyImpulse({x: dir.x * j, y: dir.y * j, z: dir.z * j}, true);

            hitCooldown = HIT_COOLDOWN;
        }
    };

    testbed.setWorld(world);
    testbed.setpreTimestepAction(update);

    document.onkeydown = function (event: KeyboardEvent) {
        switch (event.key) {
            case "ArrowUp":
            case "w":
                input.throttle = 1;
                break;
            case "ArrowDown":
            case "s":
                input.throttle = -1;
                break;
            case "ArrowLeft":
            case "a":
                input.steer = 1;
                break;
            case "ArrowRight":
            case "d":
                input.steer = -1;
                break;
            case "Shift":
                input.boost = true;
                break;
            case " ":
                input.jump = true;
                break;
            case "r":
                resetBall();
                break;
        }
    };

    document.onkeyup = function (event: KeyboardEvent) {
        switch (event.key) {
            case "ArrowUp":
            case "w":
            case "ArrowDown":
            case "s":
                input.throttle = 0;
                break;
            case "ArrowLeft":
            case "a":
            case "ArrowRight":
            case "d":
                input.steer = 0;
                break;
            case "Shift":
                input.boost = false;
                break;
        }
    };

    let cameraPosition = {
        eye: {x: 0.0, y: 28.0, z: -42.0},
        target: {x: 0.0, y: 0.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
