import RAPIER, {init} from "@alexandernanberg/rapier2d/compat";
import {describe, test, expect, beforeAll} from "vitest";

const GRAVITY = {x: 0, y: -9.81};

beforeAll(async () => {
    await init();
});

function groundedCharacter() {
    const world = new RAPIER.World(GRAVITY);
    const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    world.createCollider(RAPIER.ColliderDesc.cuboid(20, 0.5), ground);

    // Held clear of the floor: starting exactly flush with it is a degenerate
    // case the controller resolves differently in 2D and 3D.
    const body = world.createRigidBody(
        RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(0, 1.5),
    );
    const collider = world.createCollider(RAPIER.ColliderDesc.cuboid(0.3, 0.5), body);
    world.step();

    return {world, body, collider};
}

/**
 * The character controller is only ever created and freed elsewhere; none of
 * the movement it computes is checked. Its results all come back through
 * getters on the controller itself rather than a return value.
 */
describe("kinematic character controller", () => {
    test("horizontal movement is preserved and the character is grounded", () => {
        const {world, collider} = groundedCharacter();

        const controller = world.createCharacterController(0.01);
        controller.enableSnapToGround(0.5);
        controller.computeColliderMovement(collider, {x: 1, y: -0.5});

        const movement = controller.computedMovement();
        expect(movement.x).toBeCloseTo(1, 2);
        expect(controller.computedGrounded()).toBe(true);

        world.removeCharacterController(controller);
        world.free();
    });

    test("the floor stops downward movement and is reported as a collision", () => {
        const {world, collider} = groundedCharacter();

        const controller = world.createCharacterController(0.01);
        controller.computeColliderMovement(collider, {x: 0, y: -5});

        // The character stands half a unit above the floor, so only that much of
        // the five-unit descent survives.
        expect(controller.computedMovement().y).toBeCloseTo(-0.5, 1);
        expect(controller.computedGrounded()).toBe(true);
        expect(controller.numComputedCollisions()).toBeGreaterThan(0);
        expect(controller.computedCollision(0)).not.toBeNull();

        world.removeCharacterController(controller);
        world.free();
    });

    test("a reported collision carries a fully populated payload", () => {
        const {world, collider} = groundedCharacter();

        const controller = world.createCharacterController(0.01);
        controller.computeColliderMovement(collider, {x: 0, y: -5});

        const hit = controller.computedCollision(0)!;
        expect(hit).not.toBeNull();

        // Every field arrives in one `getComponents` write, so an off-by-one in
        // that layout (or a scratch buffer sized differently from what Rust
        // writes) shows up as a wrong or missing component rather than a throw.
        expect(hit.toi).toBeGreaterThanOrEqual(0);
        expect(hit.collider).not.toBeNull();

        for (const v of [
            hit.translationDeltaApplied,
            hit.translationDeltaRemaining,
            hit.witness1,
            hit.witness2,
            hit.normal1,
            hit.normal2,
        ]) {
            expect(Number.isFinite(v.x)).toBe(true);
            expect(Number.isFinite(v.y)).toBe(true);
        }

        // The floor was hit from above, so its world-space normal points up and
        // the descent was split between applied and remaining.
        expect(hit.normal1.y).toBeCloseTo(1, 1);
        expect(hit.translationDeltaApplied.y).toBeLessThan(0);
        expect(hit.translationDeltaRemaining.y).toBeLessThan(0);

        world.removeCharacterController(controller);
        world.free();
    });

    test("a wall blocks movement into it", () => {
        const {world, collider} = groundedCharacter();
        const wallBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(1.5, 1));
        world.createCollider(RAPIER.ColliderDesc.cuboid(0.1, 2), wallBody);
        world.step();

        const controller = world.createCharacterController(0.01);
        controller.computeColliderMovement(collider, {x: 5, y: 0});

        // The wall sits ~1.1 units away from the character's edge.
        expect(controller.computedMovement().x).toBeLessThan(1.5);

        world.removeCharacterController(controller);
        world.free();
    });

    test("controller settings round-trip through their getters", () => {
        const world = new RAPIER.World(GRAVITY);
        const controller = world.createCharacterController(0.02);

        expect(controller.offset()).toBeCloseTo(0.02, 6);

        controller.setSlideEnabled(false);
        expect(controller.slideEnabled()).toBe(false);

        controller.enableAutostep(0.5, 0.2, true);
        expect(controller.autostepEnabled()).toBe(true);
        expect(controller.autostepMaxHeight()).toBeCloseTo(0.5, 6);
        expect(controller.autostepMinWidth()).toBeCloseTo(0.2, 6);

        controller.disableAutostep();
        expect(controller.autostepEnabled()).toBe(false);

        controller.enableSnapToGround(0.3);
        expect(controller.snapToGroundEnabled()).toBe(true);
        expect(controller.snapToGroundDistance()).toBeCloseTo(0.3, 6);

        controller.disableSnapToGround();
        expect(controller.snapToGroundEnabled()).toBe(false);

        controller.setMaxSlopeClimbAngle(Math.PI / 4);
        expect(controller.maxSlopeClimbAngle()).toBeCloseTo(Math.PI / 4, 5);

        world.removeCharacterController(controller);
        world.free();
    });

    // `up()` used to hand back `this.raw.up()` directly, so callers got a
    // `RawVector` handle rather than a plain vector — and nothing freed it.
    test("up() returns a plain vector", () => {
        const world = new RAPIER.World(GRAVITY);
        const controller = world.createCharacterController(0.01);

        controller.setUp({x: 0, y: 1});
        expect(controller.up()).toEqual({x: 0, y: 1});

        world.removeCharacterController(controller);
        world.free();
    });
});
