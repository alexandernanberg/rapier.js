import type * as RAPIER from "@alexandernanberg/rapier3d";
import {
    DEFAULT_DIFFERENTIAL,
    DEFAULT_ENGINE,
    DEFAULT_GEARBOX,
    type DifferentialOptions,
    type EngineOptions,
    type GearboxOptions,
    type GearState,
    engineBrakingTorque,
    engineDriveTorque,
    gearRatioOf,
    rawRpmFromWheels,
    revLimiterFactor,
    rpmFromWheels,
    splitDifferential,
    updateGearbox,
} from "./drivetrain";
import {
    computeSlip,
    DEFAULT_TYRE,
    relaxationFactor,
    type TyreModelOptions,
    tyreForces,
} from "./tyreModel";

/**
 * A simulation-grade raycast vehicle built directly on Rapier primitives.
 *
 * Where {@link ../VehicleController} drives Rapier's built-in
 * `DynamicRayCastVehicleController`, this one owns the whole model: it
 * ray-casts its own wheels, integrates its own suspension, runs a real
 * slip-curve tyre, and applies the resulting forces to the chassis itself.
 *
 * That buys the things the built-in controller structurally cannot express:
 *
 *  - **Slip-curve tyres** — grip peaks and then falls away, so there is a limit
 *    to find and a slide to catch (see {@link ./tyreModel}).
 *  - **Wheel inertia** — each wheel has its own angular velocity, so you get
 *    genuine wheelspin off the line and locked wheels under heavy braking.
 *  - **A drivetrain** — torque curve, gearbox and an open / LSD / locked
 *    differential, instead of a single "engine force" number.
 *  - **Anti-roll bars** — the primary tool for tuning understeer/oversteer
 *    balance, working through load transfer and tyre load sensitivity.
 *  - **Aerodynamics** — drag that sets top speed, and downforce that genuinely
 *    adds grip as a function of speed.
 *
 * It is deliberately framework-agnostic: plain vectors in, impulses out.
 */

interface Vec3 {
    x: number;
    y: number;
    z: number;
}

interface Quat {
    x: number;
    y: number;
    z: number;
    w: number;
}

// --- Minimal vector maths (kept local so this file has no render dependency) --

function rotate(q: Quat, v: Vec3, out: Vec3): Vec3 {
    const tx = 2 * (q.y * v.z - q.z * v.y);
    const ty = 2 * (q.z * v.x - q.x * v.z);
    const tz = 2 * (q.x * v.y - q.y * v.x);
    out.x = v.x + q.w * tx + (q.y * tz - q.z * ty);
    out.y = v.y + q.w * ty + (q.z * tx - q.x * tz);
    out.z = v.z + q.w * tz + (q.x * ty - q.y * tx);
    return out;
}

function cross(a: Vec3, b: Vec3, out: Vec3): Vec3 {
    const x = a.y * b.z - a.z * b.y;
    const y = a.z * b.x - a.x * b.z;
    const z = a.x * b.y - a.y * b.x;
    out.x = x;
    out.y = y;
    out.z = z;
    return out;
}

function dot(a: Vec3, b: Vec3): number {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

function normalize(v: Vec3): Vec3 {
    const len = Math.hypot(v.x, v.y, v.z);
    if (len > 1e-9) {
        v.x /= len;
        v.y /= len;
        v.z /= len;
    }
    return v;
}

/** Remove the component of `v` along `n` (project onto the contact plane). */
function projectOnPlane(v: Vec3, n: Vec3, out: Vec3): Vec3 {
    const d = dot(v, n);
    out.x = v.x - n.x * d;
    out.y = v.y - n.y * d;
    out.z = v.z - n.z * d;
    return out;
}

export type SimDrivetrain = "fwd" | "rwd" | "awd";

export interface AxleOptions {
    /** Distance from the chassis origin to this axle along local Z. */
    offset?: number;
    /** Half the track width at this axle. */
    halfTrack?: number;
    /** Spring rate (N/m). */
    springRate?: number;
    /** Damper rate while compressing (N per m/s). */
    bumpDamping?: number;
    /** Damper rate while extending (N per m/s). */
    reboundDamping?: number;
    /** Natural (unloaded) suspension length. */
    restLength?: number;
    /** Maximum travel either side of rest. */
    maxTravel?: number;
    /** Anti-roll bar rate (N per m of left/right travel difference). */
    antiRollStiffness?: number;
    /** Wheel radius. */
    wheelRadius?: number;
    /** Wheel rotational inertia (kg·m²). */
    wheelInertia?: number;
    /** Maximum brake torque (Nm) at this axle. */
    brakeTorque?: number;
}

export interface AeroOptions {
    /** Drag force per (m/s)²: `F = dragCoefficient * v²`. */
    dragCoefficient?: number;
    /** Downforce per (m/s)²: `F = downforceCoefficient * v²`. */
    downforceCoefficient?: number;
    /** Centre of pressure along local Z (positive = forward). */
    centreOfPressure?: number;
    /**
     * Rolling resistance, as force per m/s: `F = -rollingResistance * v`.
     *
     * Linear in speed rather than quadratic, so unlike drag it still bites at
     * walking pace. It is what makes a car coast down to a stop naturally
     * instead of gliding on forever.
     */
    rollingResistance?: number;
}

export interface SimVehicleOptions {
    drivetrain?: SimDrivetrain;
    /** Vertical offset of the suspension mounting point from the origin. */
    connectionHeight?: number;
    front?: AxleOptions;
    rear?: AxleOptions;
    tyre?: TyreModelOptions;
    engine?: EngineOptions;
    gearbox?: GearboxOptions;
    differential?: DifferentialOptions;
    aero?: AeroOptions;
    /**
     * Traction control: the slip ratio the driven wheels are allowed to reach
     * before drive torque is cut back. `0` disables it.
     *
     * A keyboard throttle is all-or-nothing, and a powerful rear-driven car
     * asks for far more torque than the tyres can carry, so without this it
     * simply lights up the rears and spins. Real cars solve it the same way.
     */
    tractionControl?: number;
    /** Fraction of brake torque sent to the front axle. */
    brakeBias?: number;
    /** Handbrake torque (Nm) applied to the rear axle. */
    handbrakeTorque?: number;
    /** Steering angle (radians) at a standstill. */
    maxSteerAngle?: number;
    /** Steering angle (radians) at `steerSpeed` and above. */
    minSteerAngle?: number;
    steerSpeed?: number;
    steerRate?: number;
    steerReturnRate?: number;
    /** Wheel-dynamics substeps per physics step (higher = more stable). */
    wheelSubsteps?: number;
    /**
     * Collider friction that corresponds to the tyre's nominal grip. A surface
     * with exactly this friction gives `tyre.peakFriction`; half of it gives
     * half the grip.
     */
    referenceSurfaceFriction?: number;
}

export interface SimVehicleInput {
    /** Throttle, `0..1`. */
    throttle: number;
    /** Brake / reverse, `0..1`. */
    brake: number;
    /** Steering, `-1` (right) .. `+1` (left). */
    steer: number;
    handbrake: boolean;
}

const DEFAULT_FRONT: Required<AxleOptions> = {
    offset: 1.35,
    halfTrack: 0.78,
    springRate: 42000,
    bumpDamping: 3800,
    reboundDamping: 5200,
    restLength: 0.35,
    maxTravel: 0.16,
    antiRollStiffness: 16000,
    wheelRadius: 0.33,
    wheelInertia: 1.4,
    brakeTorque: 2600,
};

const DEFAULT_REAR: Required<AxleOptions> = {
    ...DEFAULT_FRONT,
    offset: -1.35,
    springRate: 40000,
    antiRollStiffness: 11000,
    brakeTorque: 1800,
};

/** The scalar (non-nested) part of the configuration, fully resolved. */
export type SimVehicleScalars = Required<
    Omit<
        SimVehicleOptions,
        "front" | "rear" | "tyre" | "engine" | "gearbox" | "differential" | "aero"
    >
>;

export const DEFAULT_SIM_VEHICLE: SimVehicleScalars = {
    drivetrain: "rwd",
    connectionHeight: 0.1,
    // Off by default: this is a simulation, and traction control is a driver
    // aid. The demo turns it on because a keyboard throttle cannot modulate.
    tractionControl: 0,
    brakeBias: 0.62,
    handbrakeTorque: 3000,
    maxSteerAngle: 0.55,
    minSteerAngle: 0.1,
    steerSpeed: 32,
    steerRate: 3.2,
    steerReturnRate: 5.0,
    wheelSubsteps: 8,
    referenceSurfaceFriction: 1.0,
};

export const DEFAULT_AERO: Required<AeroOptions> = {
    dragCoefficient: 0.9,
    downforceCoefficient: 1.6,
    centreOfPressure: -0.2,
    // Roughly 30x the drag coefficient, so the two are comparable around
    // 30 m/s and rolling resistance dominates below that.
    rollingResistance: 12,
};

/** Live state of one wheel — useful for rendering and for tests. */
export interface WheelState {
    /** Index: 0 = front-left, 1 = front-right, 2 = rear-left, 3 = rear-right. */
    index: number;
    isFront: boolean;
    isLeft: boolean;
    /** Angular velocity (rad/s). */
    omega: number;
    /** Accumulated spin angle (radians) for rendering. */
    rotation: number;
    /** Steering angle (radians). */
    steer: number;
    /** Vertical load (N) carried by this wheel. */
    load: number;
    /** How far the spring is compressed from rest (m). */
    compression: number;
    /** Current suspension length (m). */
    suspensionLength: number;
    inContact: boolean;
    /** Friction of the surface under this wheel (1 = the tyre's nominal grip). */
    surfaceFriction: number;
    /** Combined normalised slip — past ~0.2 the tyre is sliding. */
    slip: number;
    slipRatio: number;
    slipAngle: number;
    /** Contact-patch velocity along the wheel's heading (m/s). */
    vLong: number;
    /** Contact-patch velocity across the wheel (m/s). */
    vLat: number;
    /** Longitudinal tyre force (N). */
    fx: number;
    /** Lateral tyre force (N). */
    fy: number;
    /** World-space contact point. */
    contactPoint: Vec3;
    /** World-space contact normal (chassis up while airborne). */
    normalWs: Vec3;
    /** World-space wheel centre (for rendering). */
    centre: Vec3;
}

export class SimVehicleController {
    readonly chassis: RAPIER.RigidBody;
    readonly wheels: WheelState[] = [];
    readonly options: SimVehicleScalars;

    readonly front: Required<AxleOptions>;
    readonly rear: Required<AxleOptions>;
    readonly tyre: Required<TyreModelOptions>;
    readonly engine: Required<EngineOptions>;
    readonly gearbox: Required<GearboxOptions>;
    readonly differential: Required<DifferentialOptions>;
    readonly aero: Required<AeroOptions>;

    input: SimVehicleInput = {throttle: 0, brake: 0, steer: 0, handbrake: false};

    /** Current gear and shift cooldown. */
    gearState: GearState = {gear: 1, shiftCooldown: 0};
    /** Current engine speed (rpm). */
    rpm = 0;

    private world: RAPIER.World;
    private steerAngle = 0;
    /** Engine-braking torque handed to each driven wheel this step (Nm). */
    private engineBrakePerWheel = 0;
    /** Engine/gearbox inertia reflected onto each driven wheel (kg·m²). */
    private drivelineInertiaPerWheel = 0;
    /** Chassis mass (kg), refreshed each step. */
    private chassisMass = 1;
    /** How many wheels are touching the ground this step. */
    private contactCount = 4;

    // Scratch vectors, reused every step.
    private _q: Quat = {x: 0, y: 0, z: 0, w: 1};
    private _pos: Vec3 = {x: 0, y: 0, z: 0};
    private _com: Vec3 = {x: 0, y: 0, z: 0};
    private _linvel: Vec3 = {x: 0, y: 0, z: 0};
    private _angvel: Vec3 = {x: 0, y: 0, z: 0};
    private _up: Vec3 = {x: 0, y: 0, z: 0};
    private _down: Vec3 = {x: 0, y: 0, z: 0};
    private _fwd: Vec3 = {x: 0, y: 0, z: 0};
    private _tmp: Vec3 = {x: 0, y: 0, z: 0};
    private _tmp2: Vec3 = {x: 0, y: 0, z: 0};
    private _hard: Vec3 = {x: 0, y: 0, z: 0};
    private _vAt: Vec3 = {x: 0, y: 0, z: 0};
    private _wheelFwd: Vec3 = {x: 0, y: 0, z: 0};
    private _wheelLat: Vec3 = {x: 0, y: 0, z: 0};
    private _impulse: Vec3 = {x: 0, y: 0, z: 0};
    private _ray = {origin: {x: 0, y: 0, z: 0}, dir: {x: 0, y: 0, z: 0}};
    // Reused ray-cast result. `castRayAndGetNormal` only writes fields into the
    // target, so a plain object of the right shape avoids allocating one
    // intersection plus a normal vector per wheel per step.
    private _hit = {
        collider: null,
        timeOfImpact: 0,
        normal: {x: 0, y: 1, z: 0},
        featureType: 0,
        featureId: undefined,
    } as unknown as RAPIER.RayColliderIntersection;
    /** Per-wheel tyre options, rewritten each wheel so grip can track surface. */
    private _tyreScratch!: Required<TyreModelOptions>;
    // Reused each substep: the tyre model runs 4 wheels x wheelSubsteps times a
    // step, so returning fresh objects there is most of the garbage this
    // controller makes.
    private _slipTarget = {slipRatio: 0, slipAngle: 0};
    private _slipState = {slipRatio: 0, slipAngle: 0};
    // Per-step scratch. Rebuilding these each step is small but it is the bulk
    // of what this controller puts on the heap at 60 Hz.
    private _loads = [0, 0, 0, 0];
    private _torques = [0, 0, 0, 0];
    private _driven: WheelState[] = [];
    private _drivenFor: SimDrivetrain | null = null;
    private _axleSplit = {left: 0, right: 0};
    private _forceTarget = {fx: 0, fy: 0, slip: 0, friction: 0};

    constructor(world: RAPIER.World, chassis: RAPIER.RigidBody, options: SimVehicleOptions = {}) {
        this.world = world;
        this.chassis = chassis;
        this.options = {...DEFAULT_SIM_VEHICLE, ...options};
        this.front = {...DEFAULT_FRONT, ...options.front};
        this.rear = {...DEFAULT_REAR, ...options.rear};
        this.tyre = {...DEFAULT_TYRE, ...options.tyre};
        this.engine = {...DEFAULT_ENGINE, ...options.engine};
        this.gearbox = {...DEFAULT_GEARBOX, ...options.gearbox};
        this.differential = {...DEFAULT_DIFFERENTIAL, ...options.differential};
        this.aero = {...DEFAULT_AERO, ...options.aero};
        this._tyreScratch = {...this.tyre};

        for (let i = 0; i < 4; i++) {
            const isFront = i < 2;
            this.wheels.push({
                index: i,
                isFront,
                isLeft: i % 2 === 0,
                omega: 0,
                rotation: 0,
                steer: 0,
                load: 0,
                compression: 0,
                suspensionLength: isFront ? this.front.restLength : this.rear.restLength,
                inContact: false,
                surfaceFriction: 1,
                slip: 0,
                slipRatio: 0,
                slipAngle: 0,
                vLong: 0,
                vLat: 0,
                fx: 0,
                fy: 0,
                contactPoint: {x: 0, y: 0, z: 0},
                normalWs: {x: 0, y: 1, z: 0},
                centre: {x: 0, y: 0, z: 0},
            });
        }
    }

    /** Axle configuration for a wheel. */
    axleOf(wheel: WheelState): Required<AxleOptions> {
        return wheel.isFront ? this.front : this.rear;
    }

    /** Is this wheel driven, given the drivetrain layout? */
    isDriven(wheel: WheelState): boolean {
        const d = this.options.drivetrain;
        return d === "awd" || (d === "fwd") === wheel.isFront;
    }

    /** Signed forward speed (m/s); negative means reversing. */
    forwardSpeed(): number {
        const rot = this.chassis.rotation();
        this._q.x = rot.x;
        this._q.y = rot.y;
        this._q.z = rot.z;
        this._q.w = rot.w;
        rotate(this._q, {x: 0, y: 0, z: 1}, this._fwd);
        const v = this.chassis.linvel();
        return dot(this._fwd, v as Vec3);
    }

    /** Velocity of the point `p` on the chassis, written into `out`. */
    private velocityAt(p: Vec3, out: Vec3): Vec3 {
        this._tmp.x = p.x - this._com.x;
        this._tmp.y = p.y - this._com.y;
        this._tmp.z = p.z - this._com.z;
        cross(this._angvel, this._tmp, out);
        out.x += this._linvel.x;
        out.y += this._linvel.y;
        out.z += this._linvel.z;
        return out;
    }

    private applyImpulseAt(dir: Vec3, magnitude: number, point: Vec3) {
        if (!Number.isFinite(magnitude) || magnitude === 0) return;
        this._impulse.x = dir.x * magnitude;
        this._impulse.y = dir.y * magnitude;
        this._impulse.z = dir.z * magnitude;
        this.chassis.applyImpulseAtPoint(this._impulse, point, true);
    }

    /**
     * Advance the vehicle by `dt`. Call once per physics step, *before*
     * `world.step()`.
     */
    update(dt: number) {
        const o = this.options;
        const chassis = this.chassis;

        // --- Read the chassis state once (minimise WASM boundary crossings) --
        // These getters all take a target, so read straight into the scratch
        // objects rather than allocating five vectors every step.
        chassis.rotation(this._q);
        chassis.translation(this._pos);
        chassis.worldCom(this._com);
        chassis.linvel(this._linvel);
        chassis.angvel(this._angvel);

        rotate(this._q, {x: 0, y: 1, z: 0}, this._up);
        this._down.x = -this._up.x;
        this._down.y = -this._up.y;
        this._down.z = -this._up.z;
        rotate(this._q, {x: 0, y: 0, z: 1}, this._fwd);

        const speed = dot(this._fwd, this._linvel);

        // --- Steering: speed sensitive, eased, with a touch of Ackermann -----
        const steerInput = Math.max(-1, Math.min(1, this.input.steer));
        const speedFrac = Math.min(1, Math.abs(speed) / o.steerSpeed);
        const maxSteer = o.maxSteerAngle + (o.minSteerAngle - o.maxSteerAngle) * speedFrac;
        const target = steerInput * maxSteer;
        const rate = (steerInput === 0 ? o.steerReturnRate : o.steerRate) * dt;
        if (this.steerAngle < target) this.steerAngle = Math.min(this.steerAngle + rate, target);
        else if (this.steerAngle > target)
            this.steerAngle = Math.max(this.steerAngle - rate, target);

        // --- Suspension: ray-cast each wheel and size the spring -------------
        // Loads are gathered first so the anti-roll bars can redistribute them
        // before the tyres see them.
        this.chassisMass = chassis.mass();
        const loads = this._loads;
        this.contactCount = 0;
        for (const wheel of this.wheels) {
            this.castWheel(wheel);
            loads[wheel.index] = this.suspensionForce(wheel, dt);
            if (wheel.inContact) this.contactCount++;
        }

        // --- Anti-roll bars: transfer load across each axle -------------------
        // A stiffer bar on one axle makes that axle's outside tyre carry more
        // load; because grip is load-sensitive, that axle loses relative grip.
        // This is the classic balance knob, and it only works because the tyre
        // model is load-sensitive.
        this.applyAntiRollBar(0, 1, this.front.antiRollStiffness, loads);
        this.applyAntiRollBar(2, 3, this.rear.antiRollStiffness, loads);

        for (const wheel of this.wheels) {
            wheel.load = Math.max(0, loads[wheel.index]);
            if (wheel.inContact && wheel.load > 0) {
                // Suspension pushes the chassis along the contact normal.
                this.applyImpulseAt(wheel.normalWs, wheel.load * dt, wheel.contactPoint);
            }
        }

        // --- Drivetrain: engine -> gearbox -> differential -> wheels ---------
        const driveTorques = this.computeDriveTorques(dt, speed);

        // --- Tyres and wheel dynamics ----------------------------------------
        for (const wheel of this.wheels) {
            this.updateWheel(wheel, driveTorques[wheel.index], dt);
        }

        // --- Aerodynamics -----------------------------------------------------
        this.applyAero(dt);
    }

    /** Ray-cast one wheel and record its contact. */
    private castWheel(wheel: WheelState) {
        const axle = this.axleOf(wheel);
        const o = this.options;

        // Suspension mounting point in world space.
        // Forward is +Z and up is +Y in a right-handed frame, which puts the
        // driver's left at +X.
        const lx = (wheel.isLeft ? 1 : -1) * axle.halfTrack;
        const ly = -o.connectionHeight;
        const lz = axle.offset;
        this._tmp2.x = lx;
        this._tmp2.y = ly;
        this._tmp2.z = lz;
        rotate(this._q, this._tmp2, this._hard);
        this._hard.x += this._pos.x;
        this._hard.y += this._pos.y;
        this._hard.z += this._pos.z;

        const maxLength = axle.restLength + axle.maxTravel;
        const rayLength = maxLength + axle.wheelRadius;

        this._ray.origin.x = this._hard.x;
        this._ray.origin.y = this._hard.y;
        this._ray.origin.z = this._hard.z;
        this._ray.dir.x = this._down.x;
        this._ray.dir.y = this._down.y;
        this._ray.dir.z = this._down.z;

        const hit = this.world.castRayAndGetNormal(
            this._ray as unknown as RAPIER.Ray,
            rayLength,
            true,
            undefined,
            undefined,
            undefined,
            this.chassis,
            undefined,
            this._hit,
        );

        if (!hit) {
            wheel.inContact = false;
            wheel.surfaceFriction = 1;
            wheel.suspensionLength = maxLength;
            wheel.compression = 0;
            wheel.slip = 0;
            wheel.slipRatio = 0;
            wheel.slipAngle = 0;
            wheel.fx = 0;
            wheel.fy = 0;
            // Wheel centre hangs at full droop.
            wheel.centre.x = this._hard.x + this._down.x * maxLength;
            wheel.centre.y = this._hard.y + this._down.y * maxLength;
            wheel.centre.z = this._hard.z + this._down.z * maxLength;
            wheel.normalWs.x = this._up.x;
            wheel.normalWs.y = this._up.y;
            wheel.normalWs.z = this._up.z;
            return;
        }

        const suspensionLength = Math.max(
            axle.restLength - axle.maxTravel,
            hit.timeOfImpact - axle.wheelRadius,
        );
        // Let the surface matter: ice, gravel and tarmac are just colliders
        // with different friction, and the tyre should feel the difference.
        wheel.surfaceFriction = hit.collider.friction() / this.options.referenceSurfaceFriction;
        wheel.inContact = true;
        wheel.suspensionLength = suspensionLength;
        wheel.compression = axle.restLength - suspensionLength;
        wheel.normalWs.x = hit.normal.x;
        wheel.normalWs.y = hit.normal.y;
        wheel.normalWs.z = hit.normal.z;

        wheel.centre.x = this._hard.x + this._down.x * suspensionLength;
        wheel.centre.y = this._hard.y + this._down.y * suspensionLength;
        wheel.centre.z = this._hard.z + this._down.z * suspensionLength;
        wheel.contactPoint.x = this._hard.x + this._down.x * (suspensionLength + axle.wheelRadius);
        wheel.contactPoint.y = this._hard.y + this._down.y * (suspensionLength + axle.wheelRadius);
        wheel.contactPoint.z = this._hard.z + this._down.z * (suspensionLength + axle.wheelRadius);
    }

    /** Spring + damper force (N) for one wheel. */
    private suspensionForce(wheel: WheelState, _dt: number): number {
        if (!wheel.inContact) return 0;
        const axle = this.axleOf(wheel);

        // Compression speed, positive while the spring is being squashed.
        this.velocityAt(wheel.contactPoint, this._vAt);
        const compressionSpeed = -dot(this._vAt, wheel.normalWs);

        const spring = axle.springRate * wheel.compression;
        const damping =
            compressionSpeed > 0
                ? axle.bumpDamping * compressionSpeed
                : axle.reboundDamping * compressionSpeed;

        return Math.max(0, spring + damping);
    }

    /**
     * Anti-roll bar for one axle: move load from the less compressed wheel to
     * the more compressed one, leaving the total unchanged.
     */
    private applyAntiRollBar(left: number, right: number, stiffness: number, loads: number[]) {
        if (stiffness <= 0) return;
        const wl = this.wheels[left];
        const wr = this.wheels[right];
        if (!wl.inContact && !wr.inContact) return;

        const transfer = stiffness * (wl.compression - wr.compression);
        loads[left] = Math.max(0, loads[left] + transfer);
        loads[right] = Math.max(0, loads[right] - transfer);
    }

    /** Engine → gearbox → differential, returning per-wheel drive torque. */
    private computeDriveTorques(dt: number, speed: number): number[] {
        const torques = this._torques;
        torques[0] = 0;
        torques[1] = 0;
        torques[2] = 0;
        torques[3] = 0;

        // The driven set only changes if the drivetrain does, so cache it
        // rather than filtering a fresh array every step.
        if (this._drivenFor !== this.options.drivetrain) {
            this._driven.length = 0;
            for (const wheel of this.wheels) if (this.isDriven(wheel)) this._driven.push(wheel);
            this._drivenFor = this.options.drivetrain;
        }
        const driven = this._driven;
        if (driven.length === 0) return torques;

        const throttle = Math.max(0, Math.min(1, this.input.throttle));
        const reverseRequested = this.input.brake > 0 && speed < 0.5;

        // Pick a direction of travel: reverse gear once stopped, else forward.
        if (reverseRequested) this.gearState.gear = -1;
        else if (this.gearState.gear < 0 && (throttle > 0 || speed > 0.5)) this.gearState.gear = 1;

        let omegaSum = 0;
        for (const wheel of driven) omegaSum += wheel.omega;
        const avgOmega = omegaSum / driven.length;
        let ratio = gearRatioOf(this.gearState, this.gearbox);
        this.rpm = rpmFromWheels(avgOmega, ratio || 1, this.gearbox, this.engine);
        // Unclamped, so the limiter can see that the wheels have outrun the
        // engine even though `rpm` reads pegged at the redline.
        const rawRpm = rawRpmFromWheels(avgOmega, ratio || 1, this.gearbox);

        if (this.gearState.gear > 0) {
            updateGearbox(this.gearState, this.rpm, dt, this.gearbox);
            ratio = gearRatioOf(this.gearState, this.gearbox);
        }

        // No drive while the clutch is "open" during a shift.
        const effectiveThrottle = reverseRequested
            ? Math.max(throttle, this.input.brake)
            : throttle;

        // The clutch is eased out and back in across the shift, as a raised
        // cosine that starts at full drive, dips to nothing half way, and
        // returns to full. Switching it off and on outright puts a step change
        // in the acceleration at each end of every gear change, which is felt
        // as a lurch.
        let clutch = 1;
        if (this.gearbox.shiftTime > 0 && this.gearState.shiftCooldown > 0) {
            const progress = Math.max(
                0,
                Math.min(1, 1 - this.gearState.shiftCooldown / this.gearbox.shiftTime),
            );
            clutch = 0.5 + 0.5 * Math.cos(2 * Math.PI * progress);
        }

        const crankTorque =
            engineDriveTorque(this.rpm, effectiveThrottle, this.engine) *
            clutch *
            revLimiterFactor(rawRpm, this.engine);
        const wheelTorque = crankTorque * ratio * this.gearbox.finalDrive * this.gearbox.efficiency;

        // Engine braking is a *retarding* torque, so it is handed to the wheels
        // as brake torque. Applied as negative drive it would spin the wheels
        // backwards and quietly reverse a parked car down the road.
        const gearing = Math.abs(ratio) * this.gearbox.finalDrive * this.gearbox.efficiency;
        this.engineBrakePerWheel =
            (engineBrakingTorque(this.rpm, effectiveThrottle, this.engine) * gearing * clutch) /
            driven.length;

        // Inertia of the engine and gearbox as felt at a driven wheel, which
        // goes as the square of the gearing and so dwarfs the wheel's own in
        // the lower gears. It is what stops the throttle snapping the wheels
        // into a spin, and it falls away with the clutch during a shift.
        this.drivelineInertiaPerWheel =
            (this.engine.inertia * gearing * gearing * clutch) / driven.length;

        // Split front/rear, then across each axle through its differential.
        const d = this.options.drivetrain;
        const frontShare = d === "awd" ? this.differential.frontTorqueSplit : d === "fwd" ? 1 : 0;

        if (frontShare > 0) {
            const split = splitDifferential(
                wheelTorque * frontShare,
                this.wheels[0].omega,
                this.wheels[1].omega,
                this.differential,
                this._axleSplit,
            );
            torques[0] = split.left;
            torques[1] = split.right;
        }
        if (frontShare < 1) {
            const split = splitDifferential(
                wheelTorque * (1 - frontShare),
                this.wheels[2].omega,
                this.wheels[3].omega,
                this.differential,
                this._axleSplit,
            );
            torques[2] = split.left;
            torques[3] = split.right;
        }
        // Traction control, applied per wheel after the differential so a
        // spinning wheel is reined in without starving the one still gripping.
        // Uses last step's slip, which is a frame old but perfectly stable.
        const tc = this.options.tractionControl;
        if (tc > 0) {
            for (const wheel of this.wheels) {
                if (torques[wheel.index] === 0) continue;
                const slip = Math.abs(wheel.slipRatio);
                if (slip > tc) torques[wheel.index] *= tc / slip;
            }
        }

        return torques;
    }

    /** Tyre forces and wheel spin for one wheel. */
    private updateWheel(wheel: WheelState, driveTorque: number, dt: number) {
        const axle = this.axleOf(wheel);
        const o = this.options;

        // Steering angle, with a little Ackermann so the inside wheel turns more.
        let steer = 0;
        if (wheel.isFront) {
            const inside =
                (this.steerAngle > 0 && wheel.isLeft) || (this.steerAngle < 0 && !wheel.isLeft);
            steer = this.steerAngle * (inside ? 1.12 : 0.9);
        }
        wheel.steer = steer;

        // Brake torque for this wheel.
        const bias = wheel.isFront ? o.brakeBias : 1 - o.brakeBias;
        // The brake pedal doubles as reverse. Once reverse is actually engaged
        // it must stop applying the friction brakes too, or the brakes simply
        // hold the car against its own reverse drive and it never moves.
        const braking = this.input.brake > 0 && this.gearState.gear >= 0;
        let brakeTorque = braking ? this.input.brake * axle.brakeTorque * bias * 2 : 0;
        if (this.input.handbrake && !wheel.isFront) {
            brakeTorque = Math.max(brakeTorque, o.handbrakeTorque);
        }
        // Driven wheels also feel the engine holding them back.
        if (this.isDriven(wheel)) brakeTorque += this.engineBrakePerWheel;

        if (!wheel.inContact) {
            // Free-spinning wheel in the air: only drive and brake act on it.
            const net = driveTorque - Math.sign(wheel.omega) * brakeTorque;
            wheel.omega += (net / axle.wheelInertia) * dt;
            wheel.rotation += wheel.omega * dt;
            wheel.fx = 0;
            wheel.fy = 0;
            wheel.slip = 0;
            return;
        }

        // Wheel heading and lateral axis, projected onto the contact plane.
        // Derive the heading exactly the way the renderer does — rotate the
        // chassis-local forward about local up by the steering angle, then take
        // it to world space. Building it from the chassis basis by hand is how
        // the physics ended up steering the mirror image of the visible wheels.
        this._tmp2.x = Math.sin(steer);
        this._tmp2.y = 0;
        this._tmp2.z = Math.cos(steer);
        rotate(this._q, this._tmp2, this._tmp);
        projectOnPlane(this._tmp, wheel.normalWs, this._wheelFwd);
        normalize(this._wheelFwd);
        cross(wheel.normalWs, this._wheelFwd, this._wheelLat);
        normalize(this._wheelLat);

        // Contact patch velocity (chassis velocity is frozen for the substeps).
        this.velocityAt(wheel.contactPoint, this._vAt);
        const vLong = dot(this._vAt, this._wheelFwd);
        const vLat = dot(this._vAt, this._wheelLat);
        wheel.vLong = vLong;
        wheel.vLat = vLat;

        // Integrate the wheel against the tyre in substeps: the tyre is stiff,
        // and this is what keeps wheelspin and lock-up stable at 60 Hz.
        const sub = Math.max(1, o.wheelSubsteps);
        const dts = dt / sub;
        let sumFx = 0;
        let sumFy = 0;
        let last = this._forceTarget;
        const slipState = this._slipState;

        // Scale the tyre's grip by whatever surface it is standing on.
        this._tyreScratch.peakFriction = this.tyre.peakFriction * wheel.surfaceFriction;

        // Tyre relaxation on the *lateral* channel only: the carcass takes some
        // rolling distance to build cornering force, which is what gives the
        // car its transient response and stops the very stiff curve ringing at
        // low speed.
        //
        // The longitudinal channel is deliberately left instantaneous. Lagging
        // it too puts a delay inside the wheel-spin feedback loop (torque ->
        // spin -> slip -> force -> spin), which oscillates and can drive a
        // spinning wheel backwards.
        const relax = relaxationFactor(vLong, dts, this._tyreScratch);

        // The wheel speed at which the tyre is rolling true (zero slip ratio).
        const omegaRoll = vLong / axle.wheelRadius;
        const inertia =
            axle.wheelInertia + (this.isDriven(wheel) ? this.drivelineInertiaPerWheel : 0);
        const radius = axle.wheelRadius;

        // Reduced mass of this contact: the wheel's rotational inertia and the
        // chassis mass it carries, seen from the contact patch. The impulse
        // needed to *exactly* cancel a sliding velocity is `mEff * vSlip`, and
        // anything beyond that reverses the slide instead of stopping it.
        const shareMass = this.chassisMass / Math.max(1, this.contactCount);
        const mEffLong = 1 / (1 / shareMass + (radius * radius) / inertia);

        for (let s = 0; s < sub; s++) {
            const target = computeSlip(
                vLong,
                vLat,
                wheel.omega,
                axle.wheelRadius,
                this._tyreScratch,
                this._slipTarget,
            );
            wheel.slipRatio = target.slipRatio;
            wheel.slipAngle += (target.slipAngle - wheel.slipAngle) * relax;
            slipState.slipRatio = wheel.slipRatio;
            slipState.slipAngle = wheel.slipAngle;
            last = tyreForces(slipState, wheel.load, this._tyreScratch, this._forceTarget);

            // Cap the force at the impulse that would just cancel the slide.
            // Without this the tyre hands over far more impulse than the slip
            // is worth (~40x at a crawl), overshoots, reverses the slip, and
            // settles into a step-by-step limit cycle whose rectified average
            // walks a parked car down the road.
            const slipVelocity = wheel.omega * radius - vLong;
            // Longitudinal: per substep, because the projection below collapses
            // the slip within the first one, so the cap self-limits.
            const maxFx = (mEffLong * Math.abs(slipVelocity)) / dts;
            // Lateral: over the *whole* step. The chassis velocity is frozen
            // for the substeps, so vLat never shrinks; capping per substep would
            // license `wheelSubsteps` times the impulse needed to cancel the
            // slide, overshoot it, and chatter the cornering force between
            // positive and negative every step.
            const maxFy = (shareMass * Math.abs(vLat)) / dt;
            const fx = Math.max(-maxFx, Math.min(maxFx, last.fx));
            const fy = Math.max(-maxFy, Math.min(maxFy, last.fy));

            sumFx += fx;
            sumFy += fy;

            // Drive torque spins the wheel up.
            wheel.omega += (driveTorque / inertia) * dts;

            // The tyre reaction always pulls the wheel back towards rolling
            // true. It is applied as a *projection* rather than a raw impulse:
            // the curve is so stiff at low speed that an explicit step would
            // shoot straight past the rolling point and flip the slip's sign
            // every substep, leaving a violent oscillation whose average force
            // is zero -- tyres that quietly do nothing. Clamping at the rolling
            // point keeps it stable while still allowing genuine wheelspin,
            // because drive torque can hold omega beyond it.
            const reactionDelta = (-fx * radius * dts) / inertia;
            const toRoll = omegaRoll - wheel.omega;
            if (reactionDelta * toRoll > 0) {
                wheel.omega +=
                    Math.sign(toRoll) * Math.min(Math.abs(reactionDelta), Math.abs(toRoll));
            } else {
                wheel.omega += reactionDelta;
            }

            // Brakes cannot drive the wheel backwards — clamp to a stop.
            if (brakeTorque > 0) {
                const dOmega = (brakeTorque / inertia) * dts;
                if (Math.abs(wheel.omega) <= dOmega) wheel.omega = 0;
                else wheel.omega -= Math.sign(wheel.omega) * dOmega;
            }
        }

        wheel.rotation += wheel.omega * dt;
        wheel.slip = last.slip;
        wheel.fx = sumFx / sub;
        wheel.fy = sumFy / sub;

        this.applyImpulseAt(this._wheelFwd, wheel.fx * dt, wheel.contactPoint);
        this.applyImpulseAt(this._wheelLat, wheel.fy * dt, wheel.contactPoint);
    }

    /** Aerodynamic drag (sets top speed) and downforce (adds grip with speed). */
    private applyAero(dt: number) {
        const speedSq = dot(this._linvel, this._linvel);
        if (speedSq < 1e-6) return;
        const speed = Math.sqrt(speedSq);

        // Drag (quadratic) and rolling resistance (linear) both oppose the
        // velocity vector. Rolling resistance only applies while the wheels are
        // actually on something.
        this._tmp.x = -this._linvel.x / speed;
        this._tmp.y = -this._linvel.y / speed;
        this._tmp.z = -this._linvel.z / speed;
        const rolling =
            this.contactCount > 0
                ? (this.aero.rollingResistance * speed * this.contactCount) / this.wheels.length
                : 0;
        this.applyImpulseAt(
            this._tmp,
            (this.aero.dragCoefficient * speedSq + rolling) * dt,
            this._com,
        );

        // Downforce presses the car onto the road at the centre of pressure.
        if (this.aero.downforceCoefficient > 0) {
            this._tmp2.x = 0;
            this._tmp2.y = 0;
            this._tmp2.z = this.aero.centreOfPressure;
            rotate(this._q, this._tmp2, this._tmp);
            this._tmp.x += this._pos.x;
            this._tmp.y += this._pos.y;
            this._tmp.z += this._pos.z;
            this.applyImpulseAt(
                this._down,
                this.aero.downforceCoefficient * speedSq * dt,
                this._tmp,
            );
        }
    }
}
