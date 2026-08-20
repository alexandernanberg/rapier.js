import type {SimVehicleOptions} from "./SimVehicleController";

/**
 * A heavy, soft, slidey road car.
 *
 * The aim is weight and body movement rather than lap time: the car leans,
 * dives and squats visibly, the grip lets go early and progressively, and the
 * steering takes its time. It is kept here rather than inside the demo so the
 * handling can be measured in tests.
 */
export const ROAD_CAR_CHASSIS = {
    mass: 1850,
    /** Full body size along local X / Y / Z. */
    size: {x: 1.62, y: 0.6, z: 4.3},
    /**
     * How far the centre of mass sits below the chassis origin.
     *
     * A car tips when the lateral force it can make exceeds roughly
     * halfTrack / centreOfMassHeight. Because this setup runs a low peak
     * friction the mass can sit higher than a grippy car's could while still
     * sliding before it tips -- and a higher mass is what gives the lean.
     */
    comDrop: 0.16,
};

// Tuned for a heavy, soft, slidey car rather than a racer: a lot of visible
// body movement, grip that lets go early and progressively, and steering
// that takes its time. Every number here is a demo choice; the controller's
// own defaults are deliberately more neutral.
export const ROAD_CAR: SimVehicleOptions = {
    drivetrain: "rwd",
    differential: {type: "lsd"},
    // Soft springs with long travel and light anti-roll bars. Roll *angle*
    // comes from roll stiffness while load transfer comes from the centre
    // of mass, so softening these buys visible lean, dive and squat without
    // moving the car any closer to tipping over.
    front: {
        springRate: 30000,
        bumpDamping: 2700,
        reboundDamping: 3600,
        restLength: 0.42,
        maxTravel: 0.26,
        antiRollStiffness: 4500,
        brakeTorque: 2400,
    },
    rear: {
        springRate: 27000,
        bumpDamping: 2500,
        reboundDamping: 3300,
        restLength: 0.42,
        maxTravel: 0.26,
        antiRollStiffness: 2800,
        brakeTorque: 1700,
    },
    // Low peak grip, and a much gentler curve: B is what sets how quickly
    // grip builds with slip, so dropping it from 40 widens the slip range
    // either side of the peak. The limit arrives softly and the slide that
    // follows is long and controllable rather than a sudden let-go.
    tyre: {peakFriction: 1.05, curve: {B: 16, C: 1.5, E: 0.92}, relaxationLength: 0.55},
    // Slow, deliberate steering: less lock, and the wheels take longer to
    // get there and to come back.
    maxSteerAngle: 0.5,
    minSteerAngle: 0.085,
    steerSpeed: 26,
    steerRate: 2.1,
    steerReturnRate: 3.4,
    // Barely any downforce -- this is a road car, not a racer -- and enough
    // rolling resistance that it slows down when you lift.
    aero: {dragCoefficient: 0.85, downforceCoefficient: 0.35, rollingResistance: 14},
    // A keyboard throttle is all-or-nothing, and this car asks for more
    // torque than the rear tyres can carry, so without help it just spins.
    // Left fairly permissive so it still steps out. Press T to switch off.
    tractionControl: 0.35,
};
