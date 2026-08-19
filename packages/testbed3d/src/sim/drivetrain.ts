/**
 * Engine, gearbox and differential.
 *
 * The built-in controller takes a single "engine force" per wheel, so there is
 * nowhere to express the things that actually shape a car's character: a torque
 * curve, gearing, and how the driven wheels are tied together. This module
 * models them as plain functions over numbers.
 */

export type DifferentialType = "open" | "lsd" | "locked";

export interface EngineOptions {
    /** Torque (Nm) at each point of `torqueCurveRpm`, at full throttle. */
    torqueCurve?: number[];
    /** RPM samples matching `torqueCurve`. */
    torqueCurveRpm?: number[];
    idleRpm?: number;
    redlineRpm?: number;
    /** Engine braking torque (Nm) at zero throttle, scaled by rpm. */
    engineBrakeTorque?: number;
    /** RPM band below the redline over which the rev limiter cuts fuel. */
    revLimiterBand?: number;
    /**
     * Rotational inertia (kg·m²) of the engine, flywheel and gearbox, reflected
     * onto the driven wheels through the square of the gearing.
     *
     * Defaults to `0`. Textbook models add it so the wheels do not spin up
     * instantly, but this controller already limits that through the contact
     * impulse cap and the rolling projection, and measuring it here showed a
     * realistic 0.22 made things worse on every count: the wheel resists the
     * tyre's own correction, so the frame-to-frame swing in cornering force
     * grew from 8% of peak to 77%, wheelspin fell from 0.52 to 0.06 slip, and
     * the driven wheels would no longer lock under braking. Raise it only if
     * you want a heavier drivetrain and accept that trade.
     */
    inertia?: number;
}

export interface GearboxOptions {
    /** Forward gear ratios, first gear first. */
    gearRatios?: number[];
    reverseRatio?: number;
    finalDrive?: number;
    /** Driveline efficiency, `0..1`. */
    efficiency?: number;
    /** Upshift when engine rpm exceeds this. */
    upshiftRpm?: number;
    /** Downshift when engine rpm falls below this. */
    downshiftRpm?: number;
    /** Minimum time (s) between shifts. */
    shiftTime?: number;
}

export interface DifferentialOptions {
    type?: DifferentialType;
    /** Torque (Nm) transferred per rad/s of wheel-speed difference. */
    lockingStiffness?: number;
    /** Always-on locking torque (Nm) — a clutch-pack LSD's preload. */
    preload?: number;
    /** Largest share of the axle torque the diff may shuffle across, `0..1`. */
    maxBias?: number;
    /** For AWD: fraction of drive torque sent to the front axle. */
    frontTorqueSplit?: number;
}

export const DEFAULT_ENGINE: Required<EngineOptions> = {
    //                     idle          peak                    redline
    torqueCurveRpm: [800, 1500, 2500, 3500, 4500, 5500, 6500, 7500],
    torqueCurve: [180, 280, 360, 400, 405, 380, 330, 260],
    idleRpm: 800,
    redlineRpm: 7500,
    engineBrakeTorque: 45,
    revLimiterBand: 250,
    inertia: 0,
};

export const DEFAULT_GEARBOX: Required<GearboxOptions> = {
    gearRatios: [3.4, 2.1, 1.5, 1.15, 0.92, 0.76],
    reverseRatio: 3.2,
    finalDrive: 3.7,
    efficiency: 0.9,
    upshiftRpm: 6800,
    downshiftRpm: 2600,
    // Also the time the clutch takes to feed the torque back in, so a long
    // value is felt as a soft spot rather than a clean, quick change.
    shiftTime: 0.25,
};

export const DEFAULT_DIFFERENTIAL: Required<DifferentialOptions> = {
    type: "lsd",
    lockingStiffness: 45,
    preload: 30,
    maxBias: 0.65,
    frontTorqueSplit: 0.4,
};

/** Full-throttle engine torque (Nm) at `rpm`, linearly interpolated. */
export function engineTorqueAt(rpm: number, o: Required<EngineOptions>): number {
    const rpms = o.torqueCurveRpm;
    const torques = o.torqueCurve;
    if (rpm <= rpms[0]) return torques[0];
    if (rpm >= rpms[rpms.length - 1]) return torques[torques.length - 1];

    for (let i = 1; i < rpms.length; i++) {
        if (rpm <= rpms[i]) {
            const t = (rpm - rpms[i - 1]) / (rpms[i] - rpms[i - 1]);
            return torques[i - 1] + (torques[i] - torques[i - 1]) * t;
        }
    }
    return torques[torques.length - 1];
}

/** Crankshaft torque driving the wheels at this throttle. Never negative. */
export function engineDriveTorque(
    rpm: number,
    throttle: number,
    o: Required<EngineOptions>,
): number {
    return engineTorqueAt(rpm, o) * Math.max(0, Math.min(1, throttle));
}

/**
 * The torque the engine *absorbs* off the throttle, as a positive magnitude.
 *
 * This has to be applied as a retarding torque, the same way a brake is, and
 * never as negative drive: a standing car at idle would otherwise be handed a
 * constant backwards torque and quietly drive itself away in reverse.
 */
export function engineBrakingTorque(
    rpm: number,
    throttle: number,
    o: Required<EngineOptions>,
): number {
    const closed = 1 - Math.max(0, Math.min(1, throttle));
    return o.engineBrakeTorque * closed * (rpm / o.redlineRpm);
}

/** Engine rpm implied by the driven-wheel speed through the current gearing. */
export function rpmFromWheels(
    wheelOmega: number,
    gearRatio: number,
    o: Required<GearboxOptions>,
    engine: Required<EngineOptions>,
): number {
    return Math.min(
        engine.redlineRpm,
        Math.max(engine.idleRpm, rawRpmFromWheels(wheelOmega, gearRatio, o)),
    );
}

/** Engine speed the gearing implies, *un*clamped — may exceed the redline. */
export function rawRpmFromWheels(
    wheelOmega: number,
    gearRatio: number,
    o: Required<GearboxOptions>,
): number {
    return (Math.abs(wheelOmega) * Math.abs(gearRatio) * o.finalDrive * 60) / (2 * Math.PI);
}

/**
 * Rev limiter: the fraction of engine torque still delivered at `rawRpm`.
 *
 * Without one the torque curve simply reports its redline value forever, so
 * anything that lets the wheels outrun the engine — a driven wheel on ice, or
 * reverse, which has only one gear and so never upshifts — accelerates without
 * limit. Fuel is cut over a narrow band rather than at a hard edge so the
 * engine sits on the limiter instead of chattering on and off it.
 */
export function revLimiterFactor(rawRpm: number, o: Required<EngineOptions>): number {
    const band = Math.max(1, o.revLimiterBand);
    return Math.max(0, Math.min(1, (o.redlineRpm - rawRpm) / band));
}

export interface GearState {
    /** `-1` = reverse, `0` = neutral, `1..n` = forward gears. */
    gear: number;
    /** Seconds remaining before another shift is allowed. */
    shiftCooldown: number;
}

/** The ratio multiplying engine torque on its way to the wheels. */
export function gearRatioOf(state: GearState, o: Required<GearboxOptions>): number {
    if (state.gear === 0) return 0;
    if (state.gear < 0) return -o.reverseRatio;
    return o.gearRatios[Math.min(state.gear, o.gearRatios.length) - 1];
}

/**
 * Automatic gear selection. Mutates and returns `state`.
 *
 * Deliberately simple: shift up near the redline, down when bogging, with a
 * cooldown so it cannot hunt between gears.
 */
export function updateGearbox(
    state: GearState,
    rpm: number,
    dt: number,
    o: Required<GearboxOptions>,
): GearState {
    state.shiftCooldown = Math.max(0, state.shiftCooldown - dt);
    if (state.shiftCooldown > 0 || state.gear <= 0) return state;

    if (rpm >= o.upshiftRpm && state.gear < o.gearRatios.length) {
        state.gear += 1;
        state.shiftCooldown = o.shiftTime;
    } else if (rpm <= o.downshiftRpm && state.gear > 1) {
        state.gear -= 1;
        state.shiftCooldown = o.shiftTime;
    }
    return state;
}

export interface AxleTorques {
    left: number;
    right: number;
}

/**
 * Split axle torque across two wheels through a differential.
 *
 * - **open**: an even split. If one wheel is in the air or on ice it simply
 *   spins and the other gets nothing useful — the classic one-wheel-peel.
 * - **lsd**: biases torque towards the *slower* wheel, limited by `maxBias`,
 *   so a spinning inside wheel still lets the loaded outside wheel pull.
 * - **locked**: aggressively equalises wheel speeds; maximum traction, but it
 *   fights you in slow corners.
 */
export function splitDifferential(
    axleTorque: number,
    omegaLeft: number,
    omegaRight: number,
    o: Required<DifferentialOptions>,
    out?: AxleTorques,
): AxleTorques {
    const result: AxleTorques = out ?? {left: 0, right: 0};
    const even = axleTorque / 2;
    if (o.type === "open") {
        result.left = even;
        result.right = even;
        return result;
    }

    const stiffness = o.type === "locked" ? o.lockingStiffness * 8 : o.lockingStiffness;
    const preload = o.type === "locked" ? o.preload * 3 : o.preload;

    const delta = omegaLeft - omegaRight;
    // Torque taken off the faster wheel and handed to the slower one.
    //
    // The preload is ramped through a small deadband rather than switched on
    // `sign(delta)`: a hard step would hand the wheels equal-and-opposite
    // torque the instant their speeds differ by a floating-point hair, which
    // shows up as a phantom yaw/roll on a car standing still.
    const DEADBAND = 0.5; // rad/s
    const ramp = Math.max(-1, Math.min(1, delta / DEADBAND));
    const transferRaw = stiffness * delta + preload * ramp;

    const limit = Math.abs(axleTorque) * o.maxBias + (o.type === "locked" ? o.preload * 3 : 0);
    const transfer = Math.max(-limit, Math.min(limit, transferRaw));

    result.left = even - transfer;
    result.right = even + transfer;
    return result;
}
