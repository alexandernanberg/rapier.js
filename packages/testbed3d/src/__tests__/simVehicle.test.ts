import type {
    RigidBody as DefaultRigidBody,
    World as DefaultWorld,
} from "@alexandernanberg/rapier3d";
import RAPIER, {init} from "@alexandernanberg/rapier3d/compat";
import {beforeAll, describe, expect, test} from "vitest";
import {
    SimVehicleController,
    type SimVehicleInput,
    type SimVehicleOptions,
} from "../sim/SimVehicleController";

beforeAll(async () => {
    await init();
});

// --- Helpers ---------------------------------------------------------------

interface Quat {
    x: number;
    y: number;
    z: number;
    w: number;
}

function rotate(q: Quat, v: {x: number; y: number; z: number}) {
    const tx = 2 * (q.y * v.z - q.z * v.y);
    const ty = 2 * (q.z * v.x - q.x * v.z);
    const tz = 2 * (q.x * v.y - q.y * v.x);
    return {
        x: v.x + q.w * tx + (q.y * tz - q.z * ty),
        y: v.y + q.w * ty + (q.z * tx - q.x * tz),
        z: v.z + q.w * tz + (q.x * ty - q.y * tx),
    };
}

/** Body roll: how far the chassis' right axis tilts out of horizontal. */
function bodyRoll(q: Quat): number {
    return Math.asin(Math.max(-1, Math.min(1, rotate(q, {x: 1, y: 0, z: 0}).y)));
}

const MASS = 1400;
const BODY = {w: 1.8, h: 0.6, d: 4.2};

// Generous by default: a flat-out run covers kilometres, and a car that drops
// off the end of the plane produces very convincing nonsense.
function createGround(world: RAPIER.World, friction = 1.0, halfSize = 5000) {
    const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    world.createCollider(
        RAPIER.ColliderDesc.cuboid(halfSize, 0.5, halfSize)
            .setTranslation(0, -0.5, 0)
            .setFriction(friction),
        ground,
    );
}

/** Ground whose left half is ice and right half is tarmac. */
function createSplitGround(world: RAPIER.World) {
    const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    world.createCollider(
        RAPIER.ColliderDesc.cuboid(2, 0.5, 500).setTranslation(-2, -0.5, 0).setFriction(0.12),
        ground,
    );
    world.createCollider(
        RAPIER.ColliderDesc.cuboid(2, 0.5, 500).setTranslation(2, -0.5, 0).setFriction(1.0),
        ground,
    );
}

function createCar(world: RAPIER.World, options: SimVehicleOptions = {}) {
    const {w, h, d} = BODY;
    const chassis = world.createRigidBody(
        RAPIER.RigidBodyDesc.dynamic()
            .setTranslation(0, 0.8, 0)
            .setAdditionalMassProperties(
                MASS,
                {x: 0, y: -0.15, z: 0},
                {
                    x: (MASS / 12) * (h * h + d * d),
                    y: (MASS / 12) * (w * w + d * d),
                    z: (MASS / 12) * (w * w + h * h),
                },
                {x: 0, y: 0, z: 0, w: 1},
            )
            .setCanSleep(false),
    );
    world.createCollider(RAPIER.ColliderDesc.cuboid(w / 2, h / 2, d / 2).setFriction(0.4), chassis);

    // The compat build we run on and the default build the controller is typed
    // against share a runtime API but are distinct TS types.
    return new SimVehicleController(
        world as unknown as DefaultWorld,
        chassis as unknown as DefaultRigidBody,
        options,
    );
}

function drive(
    world: RAPIER.World,
    car: SimVehicleController,
    input: Partial<SimVehicleInput>,
    steps: number,
    onStep?: () => void,
) {
    const dt = world.timestep;
    const full: SimVehicleInput = {throttle: 0, brake: 0, steer: 0, handbrake: false, ...input};
    for (let i = 0; i < steps; i++) {
        Object.assign(car.input, full);
        car.update(dt);
        world.step();
        onStep?.();
    }
}

function settle(world: RAPIER.World, car: SimVehicleController) {
    drive(world, car, {}, 200);
}

const totalLoad = (car: SimVehicleController) => car.wheels.reduce((sum, w) => sum + w.load, 0);

// ============================================================================
// Suspension — the model has to hold the car up correctly before anything else.
// ============================================================================

describe("suspension", () => {
    test("the car settles level, on all four wheels", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        createGround(world);
        const car = createCar(world);
        settle(world, car);

        for (const wheel of car.wheels) expect(wheel.inContact).toBe(true);
        expect(Math.abs(bodyRoll(car.chassis.rotation()))).toBeLessThan(0.02); // ~1 degree
        const y = car.chassis.translation().y;
        expect(y).toBeGreaterThan(0.4);
        expect(y).toBeLessThan(1.0);

        world.free();
    });

    test("the wheel loads add up to the car's weight", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        createGround(world);
        const car = createCar(world);
        settle(world, car);

        const weight = MASS * 9.81;
        expect(totalLoad(car)).toBeGreaterThan(weight * 0.95);
        expect(totalLoad(car)).toBeLessThan(weight * 1.05);

        world.free();
    });

    test("left and right carry the same load at rest", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        createGround(world);
        const car = createCar(world);
        settle(world, car);

        const [fl, fr, rl, rr] = car.wheels.map((w) => w.load);
        expect(Math.abs(fl - fr)).toBeLessThan(0.05 * fl);
        expect(Math.abs(rl - rr)).toBeLessThan(0.05 * rl);

        world.free();
    });

    test("with no input the car stays put", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        createGround(world);
        const car = createCar(world);
        settle(world, car);

        const start = car.chassis.translation().z;
        drive(world, car, {}, 1200); // 20 seconds of being left alone
        const drift = car.chassis.translation().z - start;

        // Two failure modes this guards against, both of which produced a car
        // that quietly drove itself backwards down the road:
        //   * engine braking applied as *negative drive* rather than as a
        //     retarding torque, which spun the wheels the wrong way;
        //   * the tyre handing over far more impulse than the slip was worth,
        //     overshooting into a step-by-step limit cycle whose rectified
        //     average was a steady creep.
        expect(Math.abs(drift)).toBeLessThan(0.1);
        expect(Math.abs(car.forwardSpeed())).toBeLessThan(0.05);

        world.free();
    });

    test("braking transfers load onto the front axle", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        createGround(world);
        const car = createCar(world);
        settle(world, car);

        const restFront = car.wheels[0].load + car.wheels[1].load;
        drive(world, car, {throttle: 1}, 240);
        drive(world, car, {brake: 1}, 30);

        const front = car.wheels[0].load + car.wheels[1].load;
        const rear = car.wheels[2].load + car.wheels[3].load;
        expect(front).toBeGreaterThan(restFront * 1.3); // nose dives
        expect(front).toBeGreaterThan(rear * 1.5); // and unloads the rear

        world.free();
    });
});

// ============================================================================
// Wheel dynamics — wheelspin and lock-up, which need real wheel inertia.
// ============================================================================

describe("wheel dynamics", () => {
    test("a hard launch spins the driven wheels up past road speed", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        createGround(world);
        const car = createCar(world, {drivetrain: "rwd"});
        settle(world, car);

        let peakSlipRatio = 0;
        drive(world, car, {throttle: 1}, 40, () => {
            peakSlipRatio = Math.max(peakSlipRatio, car.wheels[2].slipRatio);
        });

        // Slip ratio > 0 means the tyre is turning faster than the road is
        // passing underneath it: genuine wheelspin, only possible because each
        // wheel carries its own angular velocity.
        expect(peakSlipRatio).toBeGreaterThan(0.3);

        world.free();
    });

    test("undriven wheels just roll along", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        createGround(world);
        const car = createCar(world, {drivetrain: "rwd"});
        settle(world, car);
        drive(world, car, {throttle: 0.4}, 240);

        // A free-rolling front wheel should sit near zero slip.
        expect(Math.abs(car.wheels[0].slipRatio)).toBeLessThan(0.1);

        world.free();
    });

    test("hard braking locks the wheels while the car is still moving", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        createGround(world);
        const car = createCar(world);
        settle(world, car);
        drive(world, car, {throttle: 1}, 300);

        expect(car.forwardSpeed()).toBeGreaterThan(10);
        drive(world, car, {brake: 1}, 40);

        // Still travelling, but the wheels have stopped turning.
        expect(car.forwardSpeed()).toBeGreaterThan(5);
        for (const wheel of car.wheels) expect(Math.abs(wheel.omega)).toBeLessThan(1);
        // A locked wheel is fully sliding: slip ratio heads for -1.
        expect(car.wheels[0].slipRatio).toBeLessThan(-0.8);

        world.free();
    });

    test("the handbrake locks only the rear axle", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        createGround(world);
        const car = createCar(world);
        settle(world, car);
        drive(world, car, {throttle: 1}, 300);
        drive(world, car, {handbrake: true}, 25);

        expect(Math.abs(car.wheels[2].omega)).toBeLessThan(1); // rear locked
        expect(Math.abs(car.wheels[0].omega)).toBeGreaterThan(5); // front still rolling

        world.free();
    });
});

// ============================================================================
// Drivetrain — gearbox and differential behaviour in the real simulation.
// ============================================================================

describe("drivetrain", () => {
    test("the car accelerates and works up through the gears", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        createGround(world);
        const car = createCar(world);
        settle(world, car);

        expect(car.gearState.gear).toBe(1);
        drive(world, car, {throttle: 1}, 600);

        expect(car.forwardSpeed()).toBeGreaterThan(20);
        expect(car.gearState.gear).toBeGreaterThan(1);
        expect(car.rpm).toBeGreaterThan(1000);

        world.free();
    });

    test("drag gives a finite top speed", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        createGround(world);
        const car = createCar(world, {aero: {dragCoefficient: 2.5}});
        settle(world, car);

        // Sample flat out at three points and watch the gains shrink: that is
        // what approaching an asymptote looks like, as drag (which grows with
        // v^2) catches up with the drive force.
        drive(world, car, {throttle: 1}, 600);
        const a = car.forwardSpeed();
        drive(world, car, {throttle: 1}, 600);
        const b = car.forwardSpeed();
        drive(world, car, {throttle: 1}, 600);
        const c = car.forwardSpeed();

        expect(b).toBeGreaterThan(a); // still accelerating early on
        expect(c - b).toBeLessThan(b - a); // but by less and less
        expect(c - b).toBeLessThan(1.0); // and it has all but converged
        expect(c).toBeLessThan(120);

        world.free();
    });

    test("less drag means a higher top speed", () => {
        const topSpeed = (dragCoefficient: number) => {
            const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
            createGround(world);
            const car = createCar(world, {aero: {dragCoefficient}});
            settle(world, car);
            drive(world, car, {throttle: 1}, 1800);
            const speed = car.forwardSpeed();
            world.free();
            return speed;
        };

        expect(topSpeed(1.2)).toBeGreaterThan(topSpeed(4.0) + 5);
    });

    test("on split traction an LSD gets the power down and an open diff does not", () => {
        // One driven wheel on ice, the other on tarmac.
        const launch = (type: "open" | "lsd" | "locked") => {
            const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
            createSplitGround(world);
            const car = createCar(world, {drivetrain: "rwd", differential: {type}});
            settle(world, car);
            const startZ = car.chassis.translation().z;
            drive(world, car, {throttle: 1}, 180);
            const result = {
                distance: car.chassis.translation().z - startZ,
                iceOmega: car.wheels[2].omega,
                gripOmega: car.wheels[3].omega,
            };
            world.free();
            return result;
        };

        const open = launch("open");
        const lsd = launch("lsd");

        // The surface really is split.
        expect(open.distance).toBeGreaterThan(0);

        // Open diff: the icy wheel spins away while the gripping one barely
        // turns -- the classic one-wheel-peel.
        expect(open.iceOmega).toBeGreaterThan(open.gripOmega * 2);

        // LSD: the two wheels turn at very nearly the same speed instead.
        expect(Math.abs(lsd.iceOmega - lsd.gripOmega)).toBeLessThan(
            Math.abs(open.iceOmega - open.gripOmega) * 0.25,
        );
        // ...and that traction turns into real distance. The margin is modest
        // (~13%) because the contact impulse cap limits how much the gripping
        // wheel can transmit per step, but it is a consistent difference.
        expect(lsd.distance).toBeGreaterThan(open.distance * 1.05);
    });

    test("the surface under a wheel changes its grip", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        createSplitGround(world);
        const car = createCar(world);
        settle(world, car);

        // Left wheels are on ice, right wheels on tarmac.
        expect(car.wheels[0].surfaceFriction).toBeLessThan(0.3);
        expect(car.wheels[1].surfaceFriction).toBeCloseTo(1.0, 1);

        world.free();
    });
});

// ============================================================================
// Chassis setup — anti-roll bars and aerodynamics.
// ============================================================================

describe("setup", () => {
    const maxRollThrough = (front: number, rear: number) => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        createGround(world);
        const car = createCar(world, {
            front: {antiRollStiffness: front},
            rear: {antiRollStiffness: rear},
        });
        settle(world, car);
        drive(world, car, {throttle: 1}, 120);
        let peak = 0;
        drive(world, car, {throttle: 0.5, steer: 1}, 100, () => {
            peak = Math.max(peak, Math.abs(bodyRoll(car.chassis.rotation())));
        });
        world.free();
        return peak;
    };

    test("stiffer anti-roll bars reduce body roll", () => {
        const soft = maxRollThrough(0, 0);
        const medium = maxRollThrough(16000, 16000);
        const stiff = maxRollThrough(60000, 60000);

        expect(soft).toBeGreaterThan(medium);
        expect(medium).toBeGreaterThan(stiff);
        // A car with no bars at all leans appreciably.
        expect(soft).toBeGreaterThan(0.05); // ~3 degrees
    });

    test("anti-roll bar balance sets understeer vs oversteer", () => {
        const yawRate = (front: number, rear: number) => {
            const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
            createGround(world);
            const car = createCar(world, {
                front: {antiRollStiffness: front},
                rear: {antiRollStiffness: rear},
            });
            settle(world, car);
            drive(world, car, {throttle: 1}, 120);
            let sum = 0;
            let n = 0;
            drive(world, car, {throttle: 0.5, steer: 1}, 120, () => {
                sum += Math.abs(car.chassis.angvel().y);
                n++;
            });
            world.free();
            return sum / n;
        };

        // Stiffening an axle makes *that* axle lose grip first, because its
        // outside tyre takes more load and grip is load-sensitive. A stiff rear
        // bar therefore rotates the car more than a stiff front bar.
        const stiffFront = yawRate(45000, 8000);
        const stiffRear = yawRate(8000, 45000);

        expect(stiffRear).toBeGreaterThan(stiffFront * 1.5);
    });

    test("downforce presses the car onto the road as speed rises", () => {
        const loadAtSpeed = (downforceCoefficient: number) => {
            const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
            createGround(world);
            const car = createCar(world, {aero: {downforceCoefficient}});
            settle(world, car);
            const rest = totalLoad(car);
            drive(world, car, {throttle: 1}, 700);
            const fast = totalLoad(car);
            const speed = car.forwardSpeed();
            world.free();
            return {rest, fast, speed};
        };

        const none = loadAtSpeed(0);
        const some = loadAtSpeed(1.6);
        const lots = loadAtSpeed(4);

        expect(none.speed).toBeGreaterThan(20); // we really are at speed
        // With no wings, load at speed is just the car's weight.
        expect(none.fast).toBeCloseTo(none.rest, -2);
        // With wings, the tyres carry appreciably more -- which is extra grip.
        expect(some.fast).toBeGreaterThan(some.rest * 1.15);
        expect(lots.fast).toBeGreaterThan(some.fast);
    });
});
