import type * as RAPIER from "@alexandernanberg/rapier3d";
import {VehicleController, type VehicleControllerOptions} from "./VehicleController";

/**
 * Vehicle presets.
 *
 * Each preset bundles the two halves that make up a "car": the chassis
 * rigid-body (mass, size, centre of mass) and the {@link VehicleController}
 * tuning (drivetrain, power, grip, steering, ...). {@link spawnVehicle} builds
 * both from a preset so you can drop in a whole car by name.
 *
 * NOTE: these are rough *arcade* approximations chosen so the cars feel
 * distinct to drive (an AWD car that grips, an RWD car that steps out, a light
 * RWD roadster, an FWD hatch, ...). The names are informal references for
 * flavour only — the numbers are not official specifications and the project is
 * not affiliated with any manufacturer.
 */
export interface VehiclePreset {
    /** Human-readable name for HUDs / menus. */
    label: string;
    /** Chassis rigid-body parameters. */
    chassis: {
        /** Total mass (kg). */
        mass: number;
        /** Full size of the body box along local X / Y / Z (m). */
        size: {x: number; y: number; z: number};
        /** Centre-of-mass offset below the origin (m, positive = lower). */
        comDrop: number;
    };
    /** Driving / handling tuning passed to the {@link VehicleController}. */
    controller: VehicleControllerOptions;
}

export const VEHICLE_PRESETS = {
    // AWD grand-tourer: heavy, planted, very hard to unstick. Think GT-R.
    skyline: {
        label: "Skyline (AWD)",
        chassis: {mass: 1540, size: {x: 1.8, y: 0.7, z: 4.5}, comDrop: 0.28},
        controller: {
            drivetrain: "awd",
            maxEngineForce: 11000,
            maxReverseForce: 5000,
            topSpeed: 47,
            frictionSlip: 4.5,
            sideFrictionStiffness: 1.2,
            handbrakeForce: 9000,
            handbrakeSideFriction: 0.6,
            maxSteerAngle: 0.55,
            // Firm, planted suspension on big wheels.
            suspensionStiffness: 30,
            suspensionRestLength: 0.3,
            wheelRadius: 0.34,
        },
    },
    // RWD coupe: lots of power at the rear, lower lateral grip — tail-happy.
    supra: {
        label: "Supra (RWD)",
        chassis: {mass: 1500, size: {x: 1.81, y: 0.7, z: 4.5}, comDrop: 0.25},
        controller: {
            drivetrain: "rwd",
            maxEngineForce: 11500,
            maxReverseForce: 5000,
            topSpeed: 50,
            frictionSlip: 4.0,
            sideFrictionStiffness: 0.85,
            handbrakeForce: 8000,
            handbrakeSideFriction: 0.3,
            maxSteerAngle: 0.6,
            // Stiff sports setup, sits low.
            suspensionStiffness: 30,
            suspensionRestLength: 0.3,
            wheelRadius: 0.34,
        },
    },
    // AWD rally car: lighter, razor-sharp turn-in, immense grip.
    evo: {
        label: "Evo (AWD)",
        chassis: {mass: 1400, size: {x: 1.77, y: 0.72, z: 4.5}, comDrop: 0.27},
        controller: {
            drivetrain: "awd",
            maxEngineForce: 10000,
            maxReverseForce: 5000,
            topSpeed: 44,
            frictionSlip: 4.8,
            sideFrictionStiffness: 1.35,
            handbrakeForce: 9000,
            handbrakeSideFriction: 0.5,
            maxSteerAngle: 0.66,
            steerRate: 5.0,
            // Softer, longer-travel rally suspension that rides higher.
            suspensionStiffness: 22,
            suspensionRestLength: 0.36,
            maxSuspensionTravel: 0.6,
            wheelRadius: 0.34,
        },
    },
    // Light RWD roadster: low power but nimble and playful.
    miata: {
        label: "Miata (RWD)",
        chassis: {mass: 1050, size: {x: 1.68, y: 0.6, z: 3.9}, comDrop: 0.22},
        controller: {
            drivetrain: "rwd",
            maxEngineForce: 6000,
            maxReverseForce: 3000,
            topSpeed: 38,
            frictionSlip: 4.0,
            sideFrictionStiffness: 0.95,
            handbrakeForce: 6000,
            handbrakeSideFriction: 0.35,
            maxSteerAngle: 0.62,
            // Low and small-wheeled roadster.
            suspensionStiffness: 26,
            suspensionRestLength: 0.26,
            wheelRadius: 0.3,
        },
    },
    // FWD hot hatch: practical, a touch of understeer, won't easily oversteer.
    golf: {
        label: "Golf GTI (FWD)",
        chassis: {mass: 1300, size: {x: 1.79, y: 0.78, z: 4.3}, comDrop: 0.24},
        controller: {
            drivetrain: "fwd",
            maxEngineForce: 7500,
            maxReverseForce: 3500,
            topSpeed: 40,
            frictionSlip: 4.2,
            sideFrictionStiffness: 1.1,
            handbrakeForce: 7000,
            handbrakeSideFriction: 0.5,
            maxSteerAngle: 0.6,
            // Comfortable hatch ride height.
            suspensionStiffness: 24,
            suspensionRestLength: 0.3,
            wheelRadius: 0.32,
        },
    },
} satisfies Record<string, VehiclePreset>;

export type VehiclePresetName = keyof typeof VEHICLE_PRESETS;

/** Preset names in a stable display order (e.g. for menus / number keys). */
export const VEHICLE_PRESET_NAMES = Object.keys(VEHICLE_PRESETS) as VehiclePresetName[];

type RapierApi = typeof import("@alexandernanberg/rapier3d");

/** Inertia tensor of a solid box of `mass` and full sizes `size`. */
function boxInertia(mass: number, size: {x: number; y: number; z: number}) {
    const {x: w, y: h, z: d} = size;
    return {
        x: (mass / 12) * (h * h + d * d),
        y: (mass / 12) * (w * w + d * d),
        z: (mass / 12) * (w * w + h * h),
    };
}

/**
 * Build a complete vehicle (chassis rigid-body + colliders + controller) from a
 * preset and add it to `world`.
 *
 * The wheel layout is derived from the chassis size unless the preset overrides
 * it. A low centre of mass is baked in for stability.
 */
export function spawnVehicle(
    RAPIER: RapierApi,
    world: RAPIER.World,
    name: VehiclePresetName,
    position: {x: number; y: number; z: number} = {x: 0, y: 1.5, z: 0},
): VehicleController {
    const preset = VEHICLE_PRESETS[name];
    const {mass, size, comDrop} = preset.chassis;

    const chassisDesc = RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(position.x, position.y, position.z)
        .setAdditionalMassProperties(mass, {x: 0, y: -comDrop, z: 0}, boxInertia(mass, size), {
            x: 0,
            y: 0,
            z: 0,
            w: 1,
        })
        .setLinearDamping(0.1)
        .setAngularDamping(0.3)
        .setCanSleep(false)
        .setCcdEnabled(true);
    const chassis = world.createRigidBody(chassisDesc);

    // Main body + a cosmetic cabin on top (both rendered, both excluded from the
    // wheel ray-casts because they belong to the chassis rigid-body).
    world.createCollider(
        RAPIER.ColliderDesc.cuboid(size.x / 2, size.y / 2, size.z / 2).setFriction(0.6),
        chassis,
    );
    world.createCollider(
        RAPIER.ColliderDesc.cuboid(size.x / 2 - 0.2, size.y / 2, size.z / 4)
            .setTranslation(0, size.y, -0.2)
            .setFriction(0.6),
        chassis,
    );

    // Wheel layout scaled to the body, overridable per preset.
    const layout: VehicleControllerOptions = {
        halfTrack: size.x / 2,
        wheelBaseFront: size.z * 0.33,
        wheelBaseRear: size.z * 0.33,
    };

    return new VehicleController(world, chassis, {...layout, ...preset.controller});
}
