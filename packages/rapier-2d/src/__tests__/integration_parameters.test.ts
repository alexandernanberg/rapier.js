import RAPIER, {init} from "@alexandernanberg/rapier2d/compat";
import {describe, test, expect, beforeAll} from "vitest";

const GRAVITY = {x: 0, y: -9.81};

beforeAll(async () => {
    await init();
});

/**
 * The solver tuning knobs are plain pass-throughs to the Rust
 * `IntegrationParameters`, so these check that each one round-trips and that the
 * world still steps with a non-default value set.
 */
describe("integration parameters", () => {
    test("numeric knobs round-trip", () => {
        const world = new RAPIER.World(GRAVITY);
        const params = world.integrationParameters;

        const knobs = [
            "dt",
            "contactNaturalFrequency",
            "contactDampingRatio",
            "staticContactNaturalFrequency",
            "staticContactDampingRatio",
            "warmstartCoefficient",
            "minCcdDt",
            "lengthUnit",
            "normalizedAllowedLinearError",
            "normalizedMaxCorrectiveVelocity",
            "normalizedPredictionDistance",
            "normalizedMaxLinearVelocity",
            "normalizedContactRecycleDistance",
        ] as const;

        for (const knob of knobs) {
            params[knob] = 0.25;
            expect(params[knob], knob).toBeCloseTo(0.25, 6);
        }

        const counts = [
            "numSolverIterations",
            "numInternalPgsIterations",
            "numInternalStabilizationIterations",
            "maxCcdSubsteps",
        ] as const;

        for (const knob of counts) {
            params[knob] = 3;
            expect(params[knob], knob).toBe(3);
        }

        world.free();
    });

    test("boolean knobs round-trip", () => {
        const world = new RAPIER.World(GRAVITY);
        const params = world.integrationParameters;

        const flags = [
            "warmstartJoints",
            "contactClustering",
            "contactRecycling",
            "frictionInBiasPass",
        ] as const;

        for (const flag of flags) {
            const initial = params[flag];
            params[flag] = !initial;
            expect(params[flag], flag).toBe(!initial);
            params[flag] = initial;
            expect(params[flag], flag).toBe(initial);
        }

        world.free();
    });

    test("contact softness is readable and writable", () => {
        const world = new RAPIER.World(GRAVITY);
        const params = world.integrationParameters;

        // `contact_natural_frequency` is the upstream-compatible alias.
        params.contact_natural_frequency = 40;
        expect(params.contactNaturalFrequency).toBeCloseTo(40, 6);

        params.contactNaturalFrequency = 20;
        params.contactDampingRatio = 4;
        expect(params.contactNaturalFrequency).toBeCloseTo(20, 6);
        expect(params.contactDampingRatio).toBeCloseTo(4, 6);
        // The ERP is derived from the softness coefficients and the timestep.
        expect(params.contact_erp).toBeGreaterThan(0);

        world.free();
    });

    test("a box still comes to rest with non-default tuning", () => {
        const world = new RAPIER.World(GRAVITY);
        world.integrationParameters.warmstartCoefficient = 0.5;
        world.integrationParameters.contactClustering = false;
        world.integrationParameters.contactRecycling = false;
        world.integrationParameters.numInternalStabilizationIterations = 4;

        const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        world.createCollider(RAPIER.ColliderDesc.cuboid(10, 0.5), ground);

        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 3));
        world.createCollider(RAPIER.ColliderDesc.cuboid(0.5, 0.5), body);

        for (let i = 0; i < 120; i++) world.step();

        // Resting on top of the ground: 0.5 (ground half-height) + 0.5 (box half-height).
        expect(body.translation().y).toBeCloseTo(1, 1);

        world.free();
    });
});
