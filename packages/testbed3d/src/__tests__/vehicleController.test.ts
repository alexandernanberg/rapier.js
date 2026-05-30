import type {
    RigidBody as DefaultRigidBody,
    World as DefaultWorld,
} from "@alexandernanberg/rapier3d";
import RAPIER, {init} from "@alexandernanberg/rapier3d/compat";
import {beforeAll, describe, expect, test} from "vitest";
import {
    computeDriveCommand,
    DEFAULT_VEHICLE_OPTIONS,
    VehicleController,
    type VehicleControllerOptions,
    type VehicleInput,
} from "../VehicleController";
import {
    spawnVehicle,
    VEHICLE_PRESET_NAMES,
    VEHICLE_PRESETS,
    type VehiclePresetName,
} from "../vehiclePresets";

// spawnVehicle is typed against the default build; we run on the (equivalent)
// compat build, so bridge the nominal type mismatch once, here.
type RapierApi = typeof import("@alexandernanberg/rapier3d");
function spawnPreset(world: RAPIER.World, name: VehiclePresetName): VehicleController {
    return spawnVehicle(RAPIER as unknown as RapierApi, world as unknown as DefaultWorld, name);
}

beforeAll(async () => {
    await init();
});

// --- Small quaternion helpers (kept local so the test has no THREE dep) -----

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

/** Rotate vector `v` by quaternion `q`. */
function rotate(q: Quat, v: Vec3): Vec3 {
    const tx = 2 * (q.y * v.z - q.z * v.y);
    const ty = 2 * (q.z * v.x - q.x * v.z);
    const tz = 2 * (q.x * v.y - q.y * v.x);
    return {
        x: v.x + q.w * tx + (q.y * tz - q.z * ty),
        y: v.y + q.w * ty + (q.z * tx - q.x * tz),
        z: v.z + q.w * tz + (q.x * ty - q.y * tx),
    };
}

/** Rotate by the inverse of `q`, i.e. transform a world vector into body space. */
function rotateInverse(q: Quat, v: Vec3): Vec3 {
    return rotate({x: -q.x, y: -q.y, z: -q.z, w: q.w}, v);
}

/** Heading (yaw) of the chassis: angle of its forward (+Z) axis in the XZ plane. */
function heading(q: Quat): number {
    const fwd = rotate(q, {x: 0, y: 0, z: 1});
    return Math.atan2(fwd.x, fwd.z);
}

/** How "upright" the chassis is: 1 = level, 0 = on its side, <0 = flipped. */
function uprightness(q: Quat): number {
    return rotate(q, {x: 0, y: 1, z: 0}).y;
}

// --- Scene helpers ----------------------------------------------------------

function createGround(world: RAPIER.World) {
    const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    world.createCollider(
        RAPIER.ColliderDesc.cuboid(500, 0.5, 500).setTranslation(0, -0.5, 0).setFriction(1.5),
        ground,
    );
}

const CHASSIS = {width: 1.8, height: 0.7, length: 3.6};

function createCar(world: RAPIER.World, options: VehicleControllerOptions = {}) {
    const mass = 1000;
    const {width: w, height: h, length: d} = CHASSIS;
    const chassisDesc = RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(0, 1.2, 0)
        // Low centre of mass: the anti-rollover trick.
        .setAdditionalMassProperties(
            mass,
            {x: 0, y: -0.25, z: 0},
            {
                x: (mass / 12) * (h * h + d * d),
                y: (mass / 12) * (w * w + d * d),
                z: (mass / 12) * (w * w + h * h),
            },
            {x: 0, y: 0, z: 0, w: 1},
        )
        .setLinearDamping(0.1)
        .setAngularDamping(0.3)
        .setCanSleep(false);
    const chassis = world.createRigidBody(chassisDesc);
    world.createCollider(RAPIER.ColliderDesc.cuboid(w / 2, h / 2, d / 2).setFriction(0.6), chassis);

    // The `compat` build we run on and the default build VehicleController is
    // typed against share an identical runtime API, but are distinct TS types.
    return new VehicleController(
        world as unknown as DefaultWorld,
        chassis as unknown as DefaultRigidBody,
        options,
    );
}

/** Advance the simulation, holding `input` for `steps` frames. */
function drive(
    world: RAPIER.World,
    vehicle: VehicleController,
    input: Partial<VehicleInput>,
    steps: number,
    onStep?: () => void,
) {
    const dt = world.timestep;
    // Reset every field each step so a previous phase's throttle/steer doesn't
    // leak into this one.
    const full: VehicleInput = {accelerate: 0, brake: 0, steer: 0, handbrake: false, ...input};
    for (let i = 0; i < steps; i++) {
        Object.assign(vehicle.input, full);
        vehicle.update(dt);
        world.step();
        onStep?.();
    }
}

/** Drop the car and let the suspension settle before a maneuver. */
function settle(world: RAPIER.World, vehicle: VehicleController) {
    drive(world, vehicle, {accelerate: 0, brake: 0, steer: 0, handbrake: false}, 90);
}

// ============================================================================
// Unit tests: the pure arcade "feel" model (no physics).
// ============================================================================

describe("computeDriveCommand (driving model)", () => {
    const o = DEFAULT_VEHICLE_OPTIONS;
    const dt = 1 / 60;
    const neutral: VehicleInput = {accelerate: 0, brake: 0, steer: 0, handbrake: false};

    test("full throttle from rest produces (near) full engine force", () => {
        const cmd = computeDriveCommand({...neutral, accelerate: 1}, 0, 0, dt, o, 2);
        expect(cmd.engineForce).toBeCloseTo(o.maxEngineForce, 5);
        expect(cmd.perWheelEngineForce).toBeCloseTo(o.maxEngineForce / 2, 5);
        expect(cmd.brake).toBe(0);
    });

    test("engine force fades to zero as speed approaches top speed", () => {
        const half = computeDriveCommand({...neutral, accelerate: 1}, o.topSpeed / 2, 0, dt, o, 2);
        const top = computeDriveCommand({...neutral, accelerate: 1}, o.topSpeed, 0, dt, o, 2);
        expect(half.engineForce).toBeCloseTo(o.maxEngineForce / 2, 5);
        expect(top.engineForce).toBeCloseTo(0, 5);
        // Monotonically decreasing with speed.
        expect(half.engineForce).toBeLessThan(o.maxEngineForce);
        expect(top.engineForce).toBeLessThan(half.engineForce);
    });

    test("brake pedal brakes (does not reverse) while rolling forward", () => {
        const cmd = computeDriveCommand({...neutral, brake: 1}, 10, 0, dt, o, 2);
        expect(cmd.brake).toBeCloseTo(o.brakeForce, 5);
        expect(cmd.engineForce).toBe(0);
    });

    test("brake pedal reverses once (nearly) stopped", () => {
        const stopped = computeDriveCommand({...neutral, brake: 1}, 0, 0, dt, o, 2);
        expect(stopped.engineForce).toBeCloseTo(-o.maxReverseForce, 5);
        expect(stopped.brake).toBe(0);

        // The 0.5 m/s threshold: just above brakes, just below reverses.
        expect(computeDriveCommand({...neutral, brake: 1}, 0.6, 0, dt, o, 2).engineForce).toBe(0);
        expect(
            computeDriveCommand({...neutral, brake: 1}, 0.4, 0, dt, o, 2).engineForce,
        ).toBeLessThan(0);
    });

    test("reverse force fades towards the reverse top speed", () => {
        const cmd = computeDriveCommand({...neutral, brake: 1}, -o.topSpeed * 0.5, 0, dt, o, 2);
        expect(cmd.engineForce).toBeCloseTo(0, 5);
    });

    test("coasting applies a light engine-braking drag", () => {
        const cmd = computeDriveCommand(neutral, 15, 0, dt, o, 2);
        expect(cmd.engineForce).toBe(0);
        expect(cmd.brake).toBeCloseTo(o.engineBraking, 5);
    });

    test("steering is tighter at low speed than at high speed", () => {
        // Large dt so the wheel reaches its target angle in a single call.
        const slow = computeDriveCommand({...neutral, steer: 1}, 0, 0, 1, o, 2);
        const fast = computeDriveCommand({...neutral, steer: 1}, o.steerSpeed, 0, 1, o, 2);
        expect(slow.steerAngle).toBeCloseTo(o.maxSteerAngle, 5);
        expect(fast.steerAngle).toBeCloseTo(o.minSteerAngle, 5);
        expect(fast.steerAngle).toBeLessThan(slow.steerAngle);
    });

    test("steering input is clamped to [-1, 1]", () => {
        expect(computeDriveCommand({...neutral, steer: 5}, 0, 0, 1, o, 2).steerAngle).toBeCloseTo(
            o.maxSteerAngle,
            5,
        );
        expect(computeDriveCommand({...neutral, steer: -5}, 0, 0, 1, o, 2).steerAngle).toBeCloseTo(
            -o.maxSteerAngle,
            5,
        );
    });

    test("steering eases towards the target instead of snapping", () => {
        // One small step should only move part of the way to the target.
        const step = computeDriveCommand({...neutral, steer: 1}, 0, 0, 1 / 60, o, 2);
        expect(step.steerAngle).toBeGreaterThan(0);
        expect(step.steerAngle).toBeLessThan(o.maxSteerAngle);
        expect(step.steerAngle).toBeCloseTo(o.steerRate / 60, 5);
    });

    test("steering self-centers when there is no input", () => {
        const recentered = computeDriveCommand(neutral, 0, 0.6, 1 / 60, o, 2);
        expect(recentered.steerAngle).toBeLessThan(0.6);
        expect(recentered.steerAngle).toBeCloseTo(0.6 - o.steerReturnRate / 60, 5);
    });

    test("opposite steering inputs give opposite, symmetric angles", () => {
        const left = computeDriveCommand({...neutral, steer: 1}, 5, 0, 1, o, 2).steerAngle;
        const right = computeDriveCommand({...neutral, steer: -1}, 5, 0, 1, o, 2).steerAngle;
        expect(left).toBeCloseTo(-right, 5);
        expect(left).toBeGreaterThan(0);
    });
});

// ============================================================================
// Integration tests: emergent behaviour in the real Rapier simulation.
// ============================================================================

describe("VehicleController (simulated behaviour)", () => {
    test("the car settles on its wheels with all of them in contact", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        createGround(world);
        const car = createCar(world);
        settle(world, car);

        const y = car.chassis.translation().y;
        expect(y).toBeGreaterThan(0.4);
        expect(y).toBeLessThan(1.2);
        for (let i = 0; i < car.wheelCount; i++) {
            expect(car.controller.wheelIsInContact(i)).toBe(true);
        }
        expect(uprightness(car.chassis.rotation())).toBeGreaterThan(0.99);

        world.free();
    });

    test("throttle accelerates the car forward", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        createGround(world);
        const car = createCar(world);
        settle(world, car);

        expect(car.currentSpeed()).toBeLessThan(0.5);
        drive(world, car, {accelerate: 1}, 120);

        expect(car.currentSpeed()).toBeGreaterThan(5);
        // Moved in +Z, the local forward direction.
        expect(car.chassis.translation().z).toBeGreaterThan(2);

        world.free();
    });

    test("top speed is bounded by the engine fade", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        createGround(world);
        const car = createCar(world, {drivetrain: "awd"});
        settle(world, car);

        drive(world, car, {accelerate: 1}, 1200); // ~20s of full throttle
        const speed = car.currentSpeed();

        expect(speed).toBeGreaterThan(10); // it really does build speed
        expect(speed).toBeLessThan(car.options.topSpeed * 1.05); // but never runs away

        world.free();
    });

    test("releasing the throttle lets the car coast to a stop", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        createGround(world);
        const car = createCar(world);
        settle(world, car);

        drive(world, car, {accelerate: 1}, 180);
        const fast = car.currentSpeed();
        drive(world, car, {accelerate: 0, brake: 0}, 600);
        const coasted = car.currentSpeed();

        expect(coasted).toBeLessThan(fast);
        expect(coasted).toBeLessThan(2);

        world.free();
    });

    test("the brake pedal slows a forward-moving car, then drives it in reverse", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        createGround(world);
        const car = createCar(world);
        settle(world, car);

        drive(world, car, {accelerate: 1}, 150);
        const fast = car.currentSpeed();
        expect(fast).toBeGreaterThan(5);

        // A short brake burst slows it down without snapping into reverse.
        drive(world, car, {brake: 1}, 40);
        const braked = car.currentSpeed();
        expect(braked).toBeLessThan(fast);
        expect(braked).toBeGreaterThan(-2);

        // Keep holding the pedal: it comes to rest and then reverses.
        drive(world, car, {brake: 1}, 180);
        expect(car.currentSpeed()).toBeLessThan(-1);

        world.free();
    });

    test("steering turns the car, and left/right are symmetric", () => {
        const turnYaw = (steer: number) => {
            const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
            createGround(world);
            const car = createCar(world);
            settle(world, car);
            drive(world, car, {accelerate: 1}, 60); // get rolling
            const before = heading(car.chassis.rotation());
            // Kept short so the yaw stays well clear of +/-PI (no wrap ambiguity).
            drive(world, car, {accelerate: 1, steer}, 60);
            const after = heading(car.chassis.rotation());
            world.free();
            let delta = after - before;
            while (delta > Math.PI) delta -= 2 * Math.PI;
            while (delta < -Math.PI) delta += 2 * Math.PI;
            return delta;
        };

        const left = turnYaw(1);
        const right = turnYaw(-1);

        expect(Math.abs(left)).toBeGreaterThan(0.2); // it actually turns
        expect(Math.sign(left)).toBe(-Math.sign(right)); // opposite directions
        expect(Math.abs(left)).toBeCloseTo(Math.abs(right), 1); // symmetric
    });

    test("stays upright through a hard corner (low center of mass)", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        createGround(world);
        const car = createCar(world);
        settle(world, car);

        drive(world, car, {accelerate: 1}, 150); // build speed
        let minUpright = 1;
        drive(world, car, {accelerate: 1, steer: 1}, 240, () => {
            minUpright = Math.min(minUpright, uprightness(car.chassis.rotation()));
        });

        // A grippy arcade car corners hard without tipping over.
        expect(minUpright).toBeGreaterThan(0.5);

        world.free();
    });

    test("the handbrake breaks rear traction and makes the car slide", () => {
        // Peak sideways slip (lateral speed vs forward speed) during a hard
        // turn, with and without the handbrake.
        const peakSlip = (handbrake: boolean) => {
            const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
            createGround(world);
            const car = createCar(world, {drivetrain: "awd"});
            settle(world, car);
            drive(world, car, {accelerate: 1}, 200); // get up to speed

            let slip = 0;
            drive(world, car, {accelerate: handbrake ? 0 : 1, steer: 1, handbrake}, 90, () => {
                const q = car.chassis.rotation();
                const vWorld = car.chassis.linvel();
                const vLocal = rotateInverse(q, {x: vWorld.x, y: 0, z: vWorld.z});
                const forward = Math.abs(vLocal.z);
                if (forward > 1) {
                    slip = Math.max(slip, Math.abs(vLocal.x) / forward);
                }
            });
            world.free();
            return slip;
        };

        const gripSlip = peakSlip(false);
        const driftSlip = peakSlip(true);

        // The handbrake run slides noticeably more sideways than the grippy run.
        expect(driftSlip).toBeGreaterThan(gripSlip * 1.3);
    });
});

// ============================================================================
// Vehicle presets: different cars should actually drive differently.
// ============================================================================

describe("vehicle presets", () => {
    /** Forward speed reached after a fixed full-throttle burst from rest. */
    function launchSpeed(name: VehiclePresetName, steps: number): number {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        createGround(world);
        const car = spawnPreset(world, name);
        settle(world, car);
        drive(world, car, {accelerate: 1}, steps);
        const speed = car.currentSpeed();
        world.free();
        return speed;
    }

    /** Near-terminal forward speed after a long full-throttle run. */
    function flatOutSpeed(name: VehiclePresetName): number {
        return launchSpeed(name, 1500);
    }

    /** Peak sideways slip in a hard corner taken with the handbrake. */
    function handbrakeSlip(name: VehiclePresetName): number {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        createGround(world);
        const car = spawnPreset(world, name);
        settle(world, car);
        drive(world, car, {accelerate: 1}, 200);

        let slip = 0;
        drive(world, car, {steer: 1, handbrake: true}, 90, () => {
            const v = car.chassis.linvel();
            const local = rotateInverse(car.chassis.rotation(), {x: v.x, y: 0, z: v.z});
            if (Math.abs(local.z) > 1) slip = Math.max(slip, Math.abs(local.x) / Math.abs(local.z));
        });
        world.free();
        return slip;
    }

    test("the presets encode distinct drivetrains", () => {
        expect(VEHICLE_PRESETS.skyline.controller.drivetrain).toBe("awd");
        expect(VEHICLE_PRESETS.supra.controller.drivetrain).toBe("rwd");
        expect(VEHICLE_PRESETS.golf.controller.drivetrain).toBe("fwd");
    });

    test.each(VEHICLE_PRESET_NAMES)("preset '%s' spawns a drivable, upright car", (name) => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        createGround(world);
        const car = spawnPreset(world, name);
        settle(world, car);

        expect(car.wheelCount).toBe(4);
        for (let i = 0; i < car.wheelCount; i++) {
            expect(car.controller.wheelIsInContact(i)).toBe(true);
        }
        expect(uprightness(car.chassis.rotation())).toBeGreaterThan(0.95);

        drive(world, car, {accelerate: 1}, 120);
        expect(car.currentSpeed()).toBeGreaterThan(4);

        world.free();
    });

    test("a higher-spec car reaches a higher top speed than a humble one", () => {
        // The Supra (topSpeed 50) clearly out-runs the little Miata (topSpeed 38).
        expect(flatOutSpeed("supra")).toBeGreaterThan(flatOutSpeed("miata") + 3);
        expect(flatOutSpeed("skyline")).toBeGreaterThan(flatOutSpeed("miata") + 3);
    });

    test("the more powerful car out-accelerates the light, low-power one", () => {
        expect(launchSpeed("supra", 120)).toBeGreaterThan(launchSpeed("miata", 120));
    });

    test("the drift-tuned RWD coupe slides far more on the handbrake than the grippy AWD car", () => {
        const supra = handbrakeSlip("supra");
        const skyline = handbrakeSlip("skyline");
        expect(supra).toBeGreaterThan(1.0); // it really breaks traction
        expect(supra).toBeGreaterThan(skyline * 1.5); // and much more than the AWD car
    });

    test("the cars are tuned with their own suspension and wheels", () => {
        // Per-car data really differs...
        expect(VEHICLE_PRESETS.miata.controller.wheelRadius).not.toBe(
            VEHICLE_PRESETS.skyline.controller.wheelRadius,
        );
        expect(VEHICLE_PRESETS.evo.controller.suspensionStiffness).not.toBe(
            VEHICLE_PRESETS.supra.controller.suspensionStiffness,
        );

        // ...and it shows up as a different ride height: the tall rally car
        // settles higher than the low roadster.
        const rideHeight = (name: VehiclePresetName) => {
            const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
            createGround(world);
            const car = spawnPreset(world, name);
            settle(world, car);
            const y = car.chassis.translation().y;
            world.free();
            return y;
        };
        expect(rideHeight("evo")).toBeGreaterThan(rideHeight("miata") + 0.05);
    });
});

// ============================================================================
// Drivetrain dynamics: the combined-slip (friction-circle) model.
// ============================================================================

describe("drivetrain dynamics", () => {
    /** Speed reached shortly after launch, at a given power and grip budget. */
    function launchAtPower(drivetrain: "fwd" | "rwd" | "awd", power: number, grip: number): number {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        createGround(world);
        const car = createCar(world, {drivetrain, maxEngineForce: power, gripCoefficient: grip});
        settle(world, car);
        drive(world, car, {accelerate: 1}, 60);
        const speed = car.currentSpeed();
        world.free();
        return speed;
    }

    /** Net heading change through a gentle, power-on corner. */
    function cornerYaw(drivetrain: "fwd" | "rwd" | "awd", grip: number): number {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        createGround(world);
        const car = createCar(world, {drivetrain, gripCoefficient: grip});
        settle(world, car);
        drive(world, car, {accelerate: 1}, 90);
        const before = heading(car.chassis.rotation());
        drive(world, car, {accelerate: 0.7, steer: 0.45}, 120);
        let delta = heading(car.chassis.rotation()) - before;
        while (delta > Math.PI) delta -= 2 * Math.PI;
        while (delta < -Math.PI) delta += 2 * Math.PI;
        world.free();
        return delta;
    }

    /** Side-grip left on a driven (rear) wheel after holding `throttle`. */
    function rearSideGrip(grip: number, throttle: number): number {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        createGround(world);
        const car = createCar(world, {drivetrain: "rwd", gripCoefficient: grip});
        settle(world, car);
        drive(world, car, {accelerate: throttle}, 30);
        const grip2 = car.controller.wheelSideFrictionStiffness(2) ?? 0;
        world.free();
        return grip2;
    }

    test("at high power an AWD car lays down more than FWD (traction limited)", () => {
        // Front wheels unload under hard acceleration, so a powerful FWD car
        // spins up and launches worse than the same power through all four.
        expect(launchAtPower("awd", 16000, 1.2)).toBeGreaterThan(
            launchAtPower("fwd", 16000, 1.2) + 2,
        );

        // With the model disabled, where the power is delivered no longer matters.
        const fwdOff = launchAtPower("fwd", 16000, 0);
        const awdOff = launchAtPower("awd", 16000, 0);
        expect(Math.abs(awdOff - fwdOff)).toBeLessThan(1);
    });

    test("the friction circle makes a FWD car push wide (understeer) under power", () => {
        const gapOn = cornerYaw("rwd", 1.2) - cornerYaw("fwd", 1.2);
        const gapOff = cornerYaw("rwd", 0) - cornerYaw("fwd", 0);
        // Turning the model on opens an understeer gap: FWD turns in less than RWD.
        expect(gapOn).toBeGreaterThan(gapOff + 0.08);
    });

    test("a driven wheel sheds lateral grip under throttle (friction circle)", () => {
        const idle = rearSideGrip(1.2, 0);
        const power = rearSideGrip(1.2, 1);
        expect(power).toBeLessThan(idle * 0.8); // grip is spent driving

        // Disabled: full lateral grip regardless of throttle.
        expect(rearSideGrip(0, 1)).toBeCloseTo(rearSideGrip(0, 0), 5);
    });
});
