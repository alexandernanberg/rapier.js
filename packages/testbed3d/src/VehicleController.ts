import type * as RAPIER from "@alexandernanberg/rapier3d";

/**
 * A small, GTA-style arcade-sim vehicle controller built on top of Rapier's
 * {@link RAPIER.DynamicRayCastVehicleController} primitive.
 *
 * The raw Rapier controller gives us raycast suspension, per-wheel engine
 * force, brake, steering and tire friction. This class wraps it with the
 * higher-level "driving feel" you expect from an arcade racer:
 *
 *  - an engine model whose force tapers off as you approach top speed,
 *  - a brake / reverse pedal that brakes while moving forward and only shifts
 *    into reverse once you've (nearly) stopped,
 *  - speed-sensitive steering that tightens up at low speed and calms down at
 *    speed, with smooth turn-in and self-centering,
 *  - a handbrake that locks the rear axle and drops its lateral grip so the
 *    car breaks traction and drifts,
 *  - configurable drivetrain (FWD / RWD / AWD).
 *
 * It is intentionally framework-agnostic (it only depends on Rapier types) so
 * it can be lifted out of the testbed and reused, or promoted into the library
 * later. Rendering reads the wheel state straight from {@link controller}.
 */
export type Drivetrain = "fwd" | "rwd" | "awd";

export interface VehicleControllerOptions {
    // --- Wheel layout (relative to the chassis origin / center of mass) ---
    /** Half the distance between the left and right wheels (along local X). */
    halfTrack?: number;
    /** Distance from the chassis origin to the front axle (along local +Z). */
    wheelBaseFront?: number;
    /** Distance from the chassis origin to the rear axle (along local -Z). */
    wheelBaseRear?: number;
    /** Vertical offset of the suspension connection point below the origin. */
    connectionHeight?: number;
    /** Wheel radius. */
    wheelRadius?: number;

    // --- Suspension (spring-damper) ---
    suspensionRestLength?: number;
    suspensionStiffness?: number;
    /** Damping while the spring is being compressed (relative coefficient). */
    suspensionCompression?: number;
    /** Damping while the spring is extending back out (relative coefficient). */
    suspensionRelaxation?: number;
    maxSuspensionForce?: number;
    maxSuspensionTravel?: number;

    // --- Tires ---
    /** Forward traction. Higher = grippier, but too high can flip the car. */
    frictionSlip?: number;
    /** Lateral grip. Higher = the car resists sliding sideways. */
    sideFrictionStiffness?: number;

    // --- Engine / drivetrain ---
    drivetrain?: Drivetrain;
    /** Total forward force at full throttle (N), split across driven wheels. */
    maxEngineForce?: number;
    /** Total reverse force (N). */
    maxReverseForce?: number;
    /** Forward speed (m/s) at which engine force fades to zero. */
    topSpeed?: number;

    // --- Brakes ---
    /** Per-wheel brake torque when the brake pedal is fully pressed. */
    brakeForce?: number;
    /** Per-wheel brake torque on the rear axle when the handbrake is engaged. */
    handbrakeForce?: number;
    /** Rear lateral grip while the handbrake is engaged (lower = more drift). */
    handbrakeSideFriction?: number;
    /** Light brake applied while coasting, to mimic engine/rolling drag. */
    engineBraking?: number;

    // --- Steering ---
    /** Max steering angle (radians) at a standstill. */
    maxSteerAngle?: number;
    /** Max steering angle (radians) once `steerSpeed` is reached. */
    minSteerAngle?: number;
    /** Forward speed (m/s) at which steering is reduced to `minSteerAngle`. */
    steerSpeed?: number;
    /** How quickly the wheels turn towards the target angle (rad/s). */
    steerRate?: number;
    /** How quickly the wheels self-center when there is no input (rad/s). */
    steerReturnRate?: number;
}

export interface VehicleInput {
    /** Throttle, `0..1`. */
    accelerate: number;
    /** Brake / reverse, `0..1`. */
    brake: number;
    /** Steering, `-1` (right) .. `+1` (left). */
    steer: number;
    /** Handbrake. */
    handbrake: boolean;
}

interface Wheel {
    steers: boolean;
    powered: boolean;
    brakes: boolean;
    handbrakes: boolean;
}

const DEFAULTS: Required<VehicleControllerOptions> = {
    halfTrack: 0.85,
    wheelBaseFront: 1.35,
    wheelBaseRear: 1.35,
    connectionHeight: 0.25,
    wheelRadius: 0.4,

    suspensionRestLength: 0.32,
    suspensionStiffness: 24.0,
    suspensionCompression: 0.82,
    suspensionRelaxation: 0.88,
    maxSuspensionForce: 30000.0,
    maxSuspensionTravel: 0.5,

    frictionSlip: 4.0,
    sideFrictionStiffness: 1.0,

    drivetrain: "rwd",
    maxEngineForce: 8000.0,
    maxReverseForce: 4000.0,
    topSpeed: 40.0,

    brakeForce: 12000.0,
    handbrakeForce: 8000.0,
    handbrakeSideFriction: 0.5,
    engineBraking: 120.0,

    maxSteerAngle: 0.6,
    minSteerAngle: 0.12,
    steerSpeed: 30.0,
    steerRate: 4.0,
    steerReturnRate: 6.0,
};

function clamp01(x: number): number {
    return x < 0 ? 0 : x > 1 ? 1 : x;
}

function lerp(a: number, b: number, t: number): number {
    return a + (b - a) * t;
}

/** Move `current` towards `target` by at most `maxDelta`. */
function approach(current: number, target: number, maxDelta: number): number {
    if (current < target) return Math.min(current + maxDelta, target);
    if (current > target) return Math.max(current - maxDelta, target);
    return target;
}

export class VehicleController {
    /** The underlying Rapier raycast vehicle controller (used for rendering). */
    readonly controller: RAPIER.DynamicRayCastVehicleController;
    /** The chassis rigid-body whose velocity the controller drives. */
    readonly chassis: RAPIER.RigidBody;

    readonly options: Required<VehicleControllerOptions>;
    private readonly wheels: Wheel[] = [];
    private readonly poweredWheelCount: number;

    /** Current (smoothed) steering angle in radians. */
    private steerAngle = 0;

    /** Driver input for the next `update()`. Mutate in place or replace. */
    input: VehicleInput = {accelerate: 0, brake: 0, steer: 0, handbrake: false};

    constructor(
        world: RAPIER.World,
        chassis: RAPIER.RigidBody,
        options: VehicleControllerOptions = {},
    ) {
        this.chassis = chassis;
        this.options = {...DEFAULTS, ...options};
        this.controller = world.createVehicleController(chassis);

        // Local axes: Y is up, Z is forward (matches the wheel layout below).
        this.controller.indexUpAxis = 1;
        this.controller.setIndexForwardAxis = 2;

        const o = this.options;
        const suspensionDir = {x: 0.0, y: -1.0, z: 0.0};
        const axle = {x: -1.0, y: 0.0, z: 0.0};

        const isFront = [true, true, false, false];
        // Front-left, front-right, rear-left, rear-right.
        const connections = [
            {x: -o.halfTrack, y: -o.connectionHeight, z: o.wheelBaseFront},
            {x: o.halfTrack, y: -o.connectionHeight, z: o.wheelBaseFront},
            {x: -o.halfTrack, y: -o.connectionHeight, z: -o.wheelBaseRear},
            {x: o.halfTrack, y: -o.connectionHeight, z: -o.wheelBaseRear},
        ];

        connections.forEach((connection, i) => {
            this.controller.addWheel(
                connection,
                suspensionDir,
                axle,
                o.suspensionRestLength,
                o.wheelRadius,
            );

            this.controller.setWheelSuspensionStiffness(i, o.suspensionStiffness);
            this.controller.setWheelSuspensionCompression(i, o.suspensionCompression);
            this.controller.setWheelSuspensionRelaxation(i, o.suspensionRelaxation);
            this.controller.setWheelMaxSuspensionForce(i, o.maxSuspensionForce);
            this.controller.setWheelMaxSuspensionTravel(i, o.maxSuspensionTravel);
            this.controller.setWheelFrictionSlip(i, o.frictionSlip);
            this.controller.setWheelSideFrictionStiffness(i, o.sideFrictionStiffness);

            const front = isFront[i];
            const powered = o.drivetrain === "awd" || (o.drivetrain === "fwd") === front;

            this.wheels.push({
                steers: front,
                powered,
                brakes: true,
                handbrakes: !front,
            });
        });

        this.poweredWheelCount = Math.max(1, this.wheels.filter((w) => w.powered).length);
    }

    /** Number of wheels (always 4 for the default layout). */
    get wheelCount(): number {
        return this.wheels.length;
    }

    /** Signed forward speed of the vehicle in m/s (negative = reversing). */
    currentSpeed(): number {
        return this.controller.currentVehicleSpeed();
    }

    /**
     * Apply the current {@link input}, then advance the raycast vehicle.
     *
     * This must be called once per physics step, *before* `world.step()`, as it
     * writes directly into the chassis rigid-body's velocity.
     */
    update(dt: number) {
        const o = this.options;
        const c = this.controller;
        const speed = c.currentVehicleSpeed();

        // --- Steering: tighter at low speed, calmer at speed ----------------
        const speedFrac = clamp01(Math.abs(speed) / o.steerSpeed);
        const maxSteer = lerp(o.maxSteerAngle, o.minSteerAngle, speedFrac);
        const targetSteer = clampSteer(this.input.steer) * maxSteer;
        const turnRate = (this.input.steer === 0 ? o.steerReturnRate : o.steerRate) * dt;
        this.steerAngle = approach(this.steerAngle, targetSteer, turnRate);

        // --- Engine / brake / reverse pedal --------------------------------
        const accel = clamp01(this.input.accelerate);
        const reverse = clamp01(this.input.brake);

        let engineForce = 0;
        // Coast braking: a touch of drag so the car slows when you let off.
        let brake = accel === 0 && reverse === 0 ? o.engineBraking : 0;

        if (accel > 0) {
            // Tractive force fades as we approach top speed.
            const fade = clamp01(1 - Math.max(0, speed) / o.topSpeed);
            engineForce += accel * o.maxEngineForce * fade;
        }

        if (reverse > 0) {
            if (speed > 0.5) {
                // Still rolling forward: the pedal is the brake.
                brake = reverse * o.brakeForce;
            } else {
                // Stopped or already reversing: shift into reverse.
                const fade = clamp01(1 - Math.max(0, -speed) / (o.topSpeed * 0.5));
                engineForce -= reverse * o.maxReverseForce * fade;
            }
        }

        const perWheelEngine = engineForce / this.poweredWheelCount;
        const handbrake = this.input.handbrake;

        for (let i = 0; i < this.wheels.length; i++) {
            const w = this.wheels[i];

            c.setWheelSteering(i, w.steers ? this.steerAngle : 0);

            if (handbrake && w.handbrakes) {
                // Lock the rear axle and let it slide: instant drift.
                c.setWheelEngineForce(i, 0);
                c.setWheelBrake(i, o.handbrakeForce);
                c.setWheelSideFrictionStiffness(i, o.handbrakeSideFriction);
            } else {
                c.setWheelEngineForce(i, w.powered ? perWheelEngine : 0);
                c.setWheelBrake(i, w.brakes ? brake : 0);
                c.setWheelSideFrictionStiffness(i, o.sideFrictionStiffness);
            }
        }

        c.updateVehicle(dt);
    }
}

function clampSteer(x: number): number {
    return x < -1 ? -1 : x > 1 ? 1 : x;
}
