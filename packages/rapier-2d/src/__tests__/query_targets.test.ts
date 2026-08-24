import RAPIER, {init} from "@alexandernanberg/rapier2d/compat";
import {describe, test, expect, beforeAll} from "vitest";

beforeAll(async () => {
    await init();
});

function worldWithBall(y = 0, radius = 1) {
    const world = new RAPIER.World({x: 0, y: 0});
    const body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(0, y));
    const collider = world.createCollider(RAPIER.ColliderDesc.ball(radius), body);
    world.step();
    return {world, body, collider};
}

/**
 * Every getter and scene query writes into a caller-owned target. These check
 * that the target is the object handed back, that the nested vectors are
 * written through rather than replaced, and that a miss leaves it untouched.
 */
describe("required query targets", () => {
    test("world.castRay fills the target", () => {
        const {world, collider} = worldWithBall();
        const ray = new RAPIER.Ray({x: 0, y: 10}, {x: 0, y: -1});

        const target = new RAPIER.RayColliderHit();
        const hit = world.castRay(ray, 100, true, target);

        expect(hit).toBe(target);
        expect(target.collider.handle).toBe(collider.handle);
        expect(target.timeOfImpact).toBeCloseTo(9, 4);

        world.free();
    });

    test("world.castRayAndGetNormal fills the target", () => {
        const {world, collider} = worldWithBall();
        const ray = new RAPIER.Ray({x: 0, y: 10}, {x: 0, y: -1});

        const target = new RAPIER.RayColliderIntersection();
        const normal = target.normal;
        const hit = world.castRayAndGetNormal(ray, 100, true, target);

        expect(hit).toBe(target);
        expect(target.normal).toBe(normal);
        expect(target.collider.handle).toBe(collider.handle);
        expect(target.timeOfImpact).toBeCloseTo(9, 4);
        expect(target.normal.y).toBeCloseTo(1, 4);

        world.free();
    });

    test("a missed cast leaves the target untouched", () => {
        const {world} = worldWithBall();
        const ray = new RAPIER.Ray({x: 50, y: 10}, {x: 0, y: -1});

        const target = new RAPIER.RayColliderIntersection();
        target.timeOfImpact = -1;
        const hit = world.castRayAndGetNormal(ray, 100, true, target);

        expect(hit).toBeNull();
        expect(target.timeOfImpact).toBe(-1);

        world.free();
    });

    test("one target can be reused across many casts", () => {
        const {world} = worldWithBall();
        const target = new RAPIER.RayColliderIntersection();
        const normal = target.normal;

        for (let i = 0; i < 8; i++) {
            const ray = new RAPIER.Ray({x: 0, y: 10 + i}, {x: 0, y: -1});
            const hit = world.castRayAndGetNormal(ray, 100, true, target);
            expect(hit).toBe(target);
            expect(target.normal).toBe(normal);
            expect(target.timeOfImpact).toBeCloseTo(9 + i, 4);
        }

        world.free();
    });

    test("world.projectPoint and projectPointAndGetFeature fill the target", () => {
        const {world, collider} = worldWithBall();
        const point = {x: 0, y: 5};

        const target = new RAPIER.PointColliderProjection();
        const projected = target.point;
        const proj = world.projectPoint(point, true, target);

        expect(proj).toBe(target);
        expect(target.point).toBe(projected);
        expect(target.collider.handle).toBe(collider.handle);
        expect(target.isInside).toBe(false);
        expect(target.point.y).toBeCloseTo(1, 4);

        const withFeature = world.projectPointAndGetFeature(point, target);

        expect(withFeature).toBe(target);
        expect(target.point.y).toBeCloseTo(1, 4);

        world.free();
    });

    test("world.castShape fills the target", () => {
        const {world, collider} = worldWithBall();
        const shape = new RAPIER.Ball(0.5);

        const target = new RAPIER.ColliderShapeCastHit();
        const witness1 = target.witness1;
        const hit = world.castShape({x: 0, y: 10}, 0, {x: 0, y: -1}, shape, 0, 100, true, target);

        expect(hit).toBe(target);
        expect(target.witness1).toBe(witness1);
        expect(target.collider.handle).toBe(collider.handle);
        expect(target.time_of_impact).toBeCloseTo(8.5, 4);

        world.free();
    });

    test("collider queries fill the target", () => {
        const {world, collider} = worldWithBall();

        const projection = new RAPIER.PointProjection();
        expect(collider.projectPoint({x: 0, y: 5}, true, projection)).toBe(projection);
        expect(projection.point.y).toBeCloseTo(1, 4);

        const intersection = new RAPIER.RayIntersection();
        const ray = new RAPIER.Ray({x: 0, y: 10}, {x: 0, y: -1});
        expect(collider.castRayAndGetNormal(ray, 100, true, intersection)).toBe(intersection);
        expect(intersection.timeOfImpact).toBeCloseTo(9, 4);
        expect(intersection.normal.y).toBeCloseTo(1, 4);

        const contact = new RAPIER.ShapeContact();
        const contacted = collider.contactShape(new RAPIER.Ball(1), {x: 0, y: 1.5}, 0, 1, contact);
        expect(contacted).toBe(contact);
        expect(contact.distance).toBeCloseTo(-0.5, 4);

        world.free();
    });

    test("rigid-body getters fill the target", () => {
        const world = new RAPIER.World({x: 0, y: -9.81});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        world.createCollider(RAPIER.ColliderDesc.cuboid(0.5, 0.5), body);
        body.setLinvel({x: 1, y: 2}, true);
        body.setAngvel(1, true);
        body.addForce({x: 4, y: 0}, true);
        world.step();

        const velocity = {x: 0, y: 0};
        expect(body.velocityAtPoint({x: 1, y: 0}, velocity)).toBe(velocity);

        const force = {x: 0, y: 0};
        expect(body.userForce(force)).toBe(force);
        expect(force.x).toBeCloseTo(4, 4);

        const invMass = {x: 0, y: 0};
        expect(body.effectiveInvMass(invMass)).toBe(invMass);
        expect(invMass.x).toBeGreaterThan(0);

        world.free();
    });

    test("joint anchors fill the target", () => {
        const world = new RAPIER.World({x: 0, y: 0});
        const body1 = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        const body2 = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(2, 0));
        const joint = world.createImpulseJoint(
            RAPIER.JointData.fixed({x: 1, y: 0}, 0, {x: -1, y: 0}, 0),
            body1,
            body2,
            true,
        );

        const anchor = {x: 0, y: 0};
        expect(joint.anchor1(anchor)).toBe(anchor);
        expect(anchor.x).toBeCloseTo(1, 4);

        world.free();
    });
});
