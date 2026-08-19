import {describe, expect, test} from "vitest";
import {
    DEFAULT_DIFFERENTIAL,
    DEFAULT_ENGINE,
    DEFAULT_GEARBOX,
    engineTorqueAt,
    gearRatioOf,
    netEngineTorque,
    splitDifferential,
    updateGearbox,
} from "../sim/drivetrain";
import {
    computeSlip,
    DEFAULT_TYRE,
    frictionAtLoad,
    magicFormula,
    peakSlip,
    tyreForces,
} from "../sim/tyreModel";

// ============================================================================
// The tyre curve — the thing the built-in controller structurally lacks.
// ============================================================================

describe("tyre curve", () => {
    test("grip rises, peaks, then falls away to a sliding plateau", () => {
        const peak = peakSlip();
        expect(peak).toBeGreaterThan(0.05);
        expect(peak).toBeLessThan(0.4);

        // Peak is normalised to 1.
        expect(magicFormula(peak)).toBeCloseTo(1, 2);

        // Rising below the peak...
        expect(magicFormula(peak * 0.3)).toBeLessThan(magicFormula(peak * 0.6));
        expect(magicFormula(peak * 0.6)).toBeLessThan(magicFormula(peak));

        // ...and falling away beyond it. This is what a driver feels as the
        // limit letting go, and what the built-in hard clamp cannot produce.
        expect(magicFormula(peak * 3)).toBeLessThan(magicFormula(peak));
    });

    test("the deep-slide plateau keeps enough grip to be catchable", () => {
        const deep = magicFormula(5);
        expect(deep).toBeGreaterThan(0.5); // not a complete loss of grip
        expect(deep).toBeLessThan(0.95); // but clearly less than the peak
    });

    test("grip is roughly linear well below the peak", () => {
        // Deep in the linear region (slip << 1/B) doubling slip doubles grip.
        const a = magicFormula(0.001);
        const b = magicFormula(0.002);
        expect(b / a).toBeGreaterThan(1.85);
        expect(b / a).toBeLessThan(2.05);
    });

    test("the peak sits at a realistic slip ratio and slip angle", () => {
        const peak = peakSlip();
        // Pure longitudinal: sigma = k / (1 + k).
        const peakSlipRatio = peak / (1 - peak);
        expect(peakSlipRatio).toBeGreaterThan(0.08);
        expect(peakSlipRatio).toBeLessThan(0.3);

        // Pure lateral: sigma = tan(alpha).
        const peakSlipAngleDeg = (Math.atan(peak) * 180) / Math.PI;
        expect(peakSlipAngleDeg).toBeGreaterThan(4);
        expect(peakSlipAngleDeg).toBeLessThan(14);
    });

    test("friction falls as vertical load rises (load sensitivity)", () => {
        const o = DEFAULT_TYRE;
        const light = frictionAtLoad(o.nominalLoad * 0.5, o);
        const nominal = frictionAtLoad(o.nominalLoad, o);
        const heavy = frictionAtLoad(o.nominalLoad * 1.5, o);

        expect(nominal).toBeCloseTo(o.peakFriction, 5);
        expect(light).toBeGreaterThan(nominal);
        expect(heavy).toBeLessThan(nominal);

        // Doubling the load gives less than double the grip -- the reason load
        // transfer (and therefore an anti-roll bar) changes the balance.
        const gripAtNominal = nominal * o.nominalLoad;
        const gripAtDouble = frictionAtLoad(o.nominalLoad * 2, o) * o.nominalLoad * 2;
        expect(gripAtDouble).toBeLessThan(gripAtNominal * 2);
    });
});

describe("slip", () => {
    test("a freely rolling wheel has (almost) no slip", () => {
        const radius = 0.33;
        const v = 20;
        const slip = computeSlip(v, 0, v / radius, radius);
        expect(slip.slipRatio).toBeCloseTo(0, 6);
        expect(slip.slipAngle).toBeCloseTo(0, 6);
    });

    test("spinning faster than the road gives positive slip ratio", () => {
        const radius = 0.33;
        const slip = computeSlip(10, 0, 20 / radius, radius);
        expect(slip.slipRatio).toBeGreaterThan(0);
    });

    test("a locked wheel while moving gives slip ratio -1", () => {
        const slip = computeSlip(20, 0, 0, 0.33);
        expect(slip.slipRatio).toBeCloseTo(-1, 5);
    });

    test("sliding sideways gives a slip angle", () => {
        const slip = computeSlip(20, 20, 20 / 0.33, 0.33);
        expect(slip.slipAngle).toBeCloseTo(Math.PI / 4, 2);
    });

    test("slip stays finite at a standstill", () => {
        const slip = computeSlip(0, 0, 0, 0.33);
        expect(Number.isFinite(slip.slipRatio)).toBe(true);
        expect(Number.isFinite(slip.slipAngle)).toBe(true);
    });
});

describe("combined slip forces", () => {
    const load = 4000;

    test("lateral force opposes the direction of sliding", () => {
        const right = tyreForces({slipRatio: 0, slipAngle: 0.1}, load);
        const left = tyreForces({slipRatio: 0, slipAngle: -0.1}, load);
        expect(right.fy).toBeLessThan(0);
        expect(left.fy).toBeGreaterThan(0);
        expect(right.fy).toBeCloseTo(-left.fy, 6);
    });

    test("driving slip pushes the car forwards, braking slip backwards", () => {
        expect(tyreForces({slipRatio: 0.1, slipAngle: 0}, load).fx).toBeGreaterThan(0);
        expect(tyreForces({slipRatio: -0.1, slipAngle: 0}, load).fx).toBeLessThan(0);
    });

    test("total force never exceeds the friction circle", () => {
        const mu = frictionAtLoad(load, DEFAULT_TYRE);
        for (const slipRatio of [-1, -0.2, 0, 0.2, 1, 5]) {
            for (const slipAngle of [-0.8, -0.2, 0, 0.2, 0.8]) {
                const f = tyreForces({slipRatio, slipAngle}, load);
                const magnitude = Math.hypot(f.fx, f.fy);
                expect(magnitude).toBeLessThanOrEqual(mu * load * 1.0001);
            }
        }
    });

    test("power genuinely costs cornering grip (the friction circle)", () => {
        const pureCorner = tyreForces({slipRatio: 0, slipAngle: 0.15}, load);
        const corneringWhileDriving = tyreForces({slipRatio: 0.3, slipAngle: 0.15}, load);

        // Same steering, but now also putting power down: lateral grip drops.
        expect(Math.abs(corneringWhileDriving.fy)).toBeLessThan(Math.abs(pureCorner.fy));
        // ...and the wheel is doing longitudinal work instead.
        expect(corneringWhileDriving.fx).toBeGreaterThan(0);
    });

    test("force scales with load", () => {
        const light = tyreForces({slipRatio: 0, slipAngle: 0.1}, 2000);
        const heavy = tyreForces({slipRatio: 0, slipAngle: 0.1}, 6000);
        expect(Math.abs(heavy.fy)).toBeGreaterThan(Math.abs(light.fy));
    });

    test("an unloaded wheel makes no force", () => {
        const f = tyreForces({slipRatio: 0.5, slipAngle: 0.5}, 0);
        expect(f.fx).toBe(0);
        expect(f.fy).toBe(0);
    });
});

// ============================================================================
// Drivetrain: torque curve, gearbox, differential.
// ============================================================================

describe("engine and gearbox", () => {
    test("the torque curve peaks in the mid-range, not at the redline", () => {
        const o = DEFAULT_ENGINE;
        const peakRpm = o.torqueCurveRpm[o.torqueCurve.indexOf(Math.max(...o.torqueCurve))];
        expect(peakRpm).toBeGreaterThan(o.idleRpm);
        expect(peakRpm).toBeLessThan(o.redlineRpm);

        expect(engineTorqueAt(peakRpm, o)).toBeGreaterThan(engineTorqueAt(o.idleRpm, o));
        expect(engineTorqueAt(peakRpm, o)).toBeGreaterThan(engineTorqueAt(o.redlineRpm, o));
    });

    test("torque interpolates between curve points and clamps outside", () => {
        const o = DEFAULT_ENGINE;
        const mid = engineTorqueAt(2000, o);
        expect(mid).toBeGreaterThan(Math.min(280, 360));
        expect(mid).toBeLessThan(Math.max(280, 360));
        expect(engineTorqueAt(0, o)).toBe(o.torqueCurve[0]);
        expect(engineTorqueAt(99999, o)).toBe(o.torqueCurve[o.torqueCurve.length - 1]);
    });

    test("lifting off produces engine braking, not zero torque", () => {
        const o = DEFAULT_ENGINE;
        expect(netEngineTorque(4000, 1, o)).toBeGreaterThan(0);
        expect(netEngineTorque(4000, 0, o)).toBeLessThan(0);
    });

    test("the gearbox shifts up near the redline and back down when bogging", () => {
        const o = DEFAULT_GEARBOX;
        const state = {gear: 1, shiftCooldown: 0};

        updateGearbox(state, o.upshiftRpm + 100, 1 / 60, o);
        expect(state.gear).toBe(2);

        // A cooldown prevents hunting between gears.
        updateGearbox(state, o.upshiftRpm + 100, 1 / 60, o);
        expect(state.gear).toBe(2);

        state.shiftCooldown = 0;
        updateGearbox(state, o.downshiftRpm - 100, 1 / 60, o);
        expect(state.gear).toBe(1);

        // Never below first.
        state.shiftCooldown = 0;
        updateGearbox(state, 0, 1 / 60, o);
        expect(state.gear).toBe(1);
    });

    test("lower gears multiply torque more than higher ones", () => {
        const o = DEFAULT_GEARBOX;
        const first = gearRatioOf({gear: 1, shiftCooldown: 0}, o);
        const top = gearRatioOf({gear: o.gearRatios.length, shiftCooldown: 0}, o);
        expect(first).toBeGreaterThan(top);
    });
});

describe("differential", () => {
    const axleTorque = 1000;

    test("an open diff splits torque evenly regardless of wheel speeds", () => {
        const o = {...DEFAULT_DIFFERENTIAL, type: "open" as const};
        const even = splitDifferential(axleTorque, 10, 10, o);
        const spinning = splitDifferential(axleTorque, 100, 10, o);
        expect(even.left).toBeCloseTo(500, 6);
        expect(spinning.left).toBeCloseTo(500, 6);
        expect(spinning.right).toBeCloseTo(500, 6);
    });

    test("an LSD biases torque to the slower (gripping) wheel", () => {
        const o = {...DEFAULT_DIFFERENTIAL, type: "lsd" as const};
        // Left wheel spinning up on ice, right wheel gripping.
        const split = splitDifferential(axleTorque, 60, 10, o);
        expect(split.right).toBeGreaterThan(split.left);
        // Total torque is conserved.
        expect(split.left + split.right).toBeCloseTo(axleTorque, 6);
    });

    test("the LSD's bias is bounded by maxBias", () => {
        const o = {...DEFAULT_DIFFERENTIAL, type: "lsd" as const};
        const split = splitDifferential(axleTorque, 5000, 0, o);
        const bias = Math.abs(split.right - split.left) / 2;
        expect(bias).toBeLessThanOrEqual(Math.abs(axleTorque) * o.maxBias + 1e-6);
    });

    test("a locked diff fights speed differences harder than an LSD", () => {
        const lsd = splitDifferential(axleTorque, 40, 10, {
            ...DEFAULT_DIFFERENTIAL,
            type: "lsd",
        });
        const locked = splitDifferential(axleTorque, 40, 10, {
            ...DEFAULT_DIFFERENTIAL,
            type: "locked",
        });
        expect(locked.right - locked.left).toBeGreaterThan(lsd.right - lsd.left);
    });

    test("equal wheel speeds mean no bias at all", () => {
        const split = splitDifferential(axleTorque, 25, 25, DEFAULT_DIFFERENTIAL);
        expect(split.left).toBeCloseTo(split.right, 6);
    });
});
