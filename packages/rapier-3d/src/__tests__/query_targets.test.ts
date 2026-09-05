import RAPIER, {init} from "@alexandernanberg/rapier3d/compat";
import {describe, test, expect, beforeAll} from "vitest";

beforeAll(async () => {
    await init();
});

function worldWithBall(y = 0, radius = 1) {
    const world = new RAPIER.World({x: 0, y: 0, z: 0});
    const body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(0, y, 0));
    const collider = world.createCollider(RAPIER.ColliderDesc.ball(radius), body);
    world.step();
    return {world, body, collider};
}

function rayIntersectionTarget() {
    return new RAPIER.RayColliderIntersection(
        undefined!,
        0,
        {x: 0, y: 0, z: 0},
        RAPIER.FeatureType.Unknown,
        undefined,
    );
}

function pointProjectionTarget() {
    return new RAPIER.PointColliderProjection(
        undefined!,
        {x: 0, y: 0, z: 0},
        false,
        RAPIER.FeatureType.Unknown,
        undefined,
    );
}

function shapeCastTarget() {
    return new RAPIER.ColliderShapeCastHit(
        undefined!,
        0,
        {x: 0, y: 0, z: 0},
        {x: 0, y: 0, z: 0},
        {x: 0, y: 0, z: 0},
        {x: 0, y: 0, z: 0},
    );
}

/**
 * The scene queries and the getters that return a fresh object can write into a
 * caller-owned one instead. These check that the target is the object handed
 * back, that it holds the same values the allocating call returns, and that the
 * nested vectors are reused rather than replaced.
 */
describe("zero-allocation query targets", () => {
    test("world.castRay fills the target and leaves it alone on a miss", () => {
        const {world, collider} = worldWithBall();
        const ray = new RAPIER.Ray({x: 0, y: 10, z: 0}, {x: 0, y: -1, z: 0});

        const target = new RAPIER.RayColliderHit(null!, -1);
        const hit = world.castRay(
            ray,
            100,
            true,
            undefined,
            undefined,
            undefined,
            undefined,
            undefined,
            target,
        );
        expect(hit).toBe(target);
        expect(target.collider).toBe(collider);
        expect(target.timeOfImpact).toBeCloseTo(9, 3);

        target.timeOfImpact = -1;
        const miss = world.castRay(
            ray,
            1,
            true,
            undefined,
            undefined,
            undefined,
            undefined,
            undefined,
            target,
        );
        expect(miss).toBeNull();
        expect(target.timeOfImpact).toBe(-1);
        world.free();
    });

    test("world.castRayAndGetNormal fills the target", () => {
        const {world, collider} = worldWithBall();
        const ray = new RAPIER.Ray({x: 0, y: 10, z: 0}, {x: 0, y: -1, z: 0});

        const target = rayIntersectionTarget();
        const normal = target.normal;
        const hit = world.castRayAndGetNormal(
            ray,
            100,
            true,
            undefined,
            undefined,
            undefined,
            undefined,
            undefined,
            target,
        );

        expect(hit).toBe(target);
        expect(target.normal).toBe(normal);
        expect(target.collider.handle).toBe(collider.handle);
        expect(target.timeOfImpact).toBeCloseTo(9, 4);
        expect(target.normal.y).toBeCloseTo(1, 4);

        world.free();
    });

    test("a missed cast leaves the target untouched", () => {
        const {world} = worldWithBall();
        const ray = new RAPIER.Ray({x: 50, y: 10, z: 0}, {x: 0, y: -1, z: 0});

        const target = rayIntersectionTarget();
        target.timeOfImpact = -1;
        const hit = world.castRayAndGetNormal(
            ray,
            100,
            true,
            undefined,
            undefined,
            undefined,
            undefined,
            undefined,
            target,
        );

        expect(hit).toBeNull();
        expect(target.timeOfImpact).toBe(-1);

        world.free();
    });

    test("world.projectPoint and projectPointAndGetFeature fill the target", () => {
        const {world, collider} = worldWithBall();
        const point = {x: 0, y: 5, z: 0};

        const target = pointProjectionTarget();
        const projected = target.point;
        const proj = world.projectPoint(
            point,
            true,
            undefined,
            undefined,
            undefined,
            undefined,
            undefined,
            target,
        );

        expect(proj).toBe(target);
        expect(target.point).toBe(projected);
        expect(target.collider.handle).toBe(collider.handle);
        expect(target.isInside).toBe(false);
        expect(target.point.y).toBeCloseTo(1, 4);

        const withFeature = world.projectPointAndGetFeature(
            point,
            undefined,
            undefined,
            undefined,
            undefined,
            undefined,
            target,
        );

        expect(withFeature).toBe(target);
        expect(target.point.y).toBeCloseTo(1, 4);

        world.free();
    });

    test("world.castShape fills the target", () => {
        const {world, collider} = worldWithBall();
        const shape = new RAPIER.Ball(0.5);

        const target = shapeCastTarget();
        const witness1 = target.witness1;
        const hit = world.castShape(
            {x: 0, y: 10, z: 0},
            {x: 0, y: 0, z: 0, w: 1},
            {x: 0, y: -1, z: 0},
            shape,
            0,
            100,
            true,
            undefined,
            undefined,
            undefined,
            undefined,
            undefined,
            target,
        );

        expect(hit).toBe(target);
        expect(target.witness1).toBe(witness1);
        expect(target.collider.handle).toBe(collider.handle);
        expect(target.time_of_impact).toBeCloseTo(8.5, 4);

        world.free();
    });

    test("collider queries fill the target", () => {
        const {world, collider} = worldWithBall();

        const projection = new RAPIER.PointProjection({x: 0, y: 0, z: 0}, false);
        expect(collider.projectPoint({x: 0, y: 5, z: 0}, true, projection)).toBe(projection);
        expect(projection.point.y).toBeCloseTo(1, 4);

        const intersection = new RAPIER.RayIntersection(0, {x: 0, y: 0, z: 0});
        const ray = new RAPIER.Ray({x: 0, y: 10, z: 0}, {x: 0, y: -1, z: 0});
        expect(collider.castRayAndGetNormal(ray, 100, true, intersection)).toBe(intersection);
        expect(intersection.timeOfImpact).toBeCloseTo(9, 4);
        expect(intersection.normal.y).toBeCloseTo(1, 4);

        const contact = new RAPIER.ShapeContact(
            0,
            {x: 0, y: 0, z: 0},
            {x: 0, y: 0, z: 0},
            {x: 0, y: 0, z: 0},
            {x: 0, y: 0, z: 0},
        );
        const contacted = collider.contactShape(
            new RAPIER.Ball(1),
            {x: 0, y: 1.5, z: 0},
            {x: 0, y: 0, z: 0, w: 1},
            1,
            contact,
        );
        expect(contacted).toBe(contact);
        expect(contact.distance).toBeCloseTo(-0.5, 4);

        world.free();
    });

    test("rigid-body getters fill the target", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        world.createCollider(RAPIER.ColliderDesc.cuboid(0.5, 0.5, 0.5), body);
        body.setLinvel({x: 1, y: 2, z: 3}, true);
        body.setAngvel({x: 0, y: 0, z: 1}, true);
        body.addForce({x: 4, y: 0, z: 0}, true);
        world.step();

        const velocity = {x: 0, y: 0, z: 0};
        expect(body.velocityAtPoint({x: 1, y: 0, z: 0}, velocity)).toBe(velocity);
        expect(velocity).toEqual(body.velocityAtPoint({x: 1, y: 0, z: 0}));

        const force = {x: 0, y: 0, z: 0};
        expect(body.userForce(force)).toBe(force);
        expect(force.x).toBeCloseTo(4, 4);

        const invMass = {x: 0, y: 0, z: 0};
        expect(body.effectiveInvMass(invMass)).toBe(invMass);
        expect(invMass).toEqual(body.effectiveInvMass());

        const inertia = {x: 0, y: 0, z: 0};
        expect(body.principalInertia(inertia)).toBe(inertia);
        expect(inertia).toEqual(body.principalInertia());

        const localFrame = {x: 0, y: 0, z: 0, w: 1};
        expect(body.principalInertiaLocalFrame(localFrame)).toBe(localFrame);

        const angularInertia = new RAPIER.SdpMatrix3(new Float32Array(6));
        expect(body.effectiveAngularInertia(angularInertia)).toBe(angularInertia);
        expect(Array.from(angularInertia.elements)).toEqual(
            Array.from(body.effectiveAngularInertia().elements),
        );

        world.free();
    });

    test("joint anchors fill the target", () => {
        const world = new RAPIER.World({x: 0, y: 0, z: 0});
        const body1 = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        const body2 = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(2, 0, 0));
        const joint = world.createImpulseJoint(
            RAPIER.JointData.fixed(
                {x: 1, y: 0, z: 0},
                {x: 0, y: 0, z: 0, w: 1},
                {x: -1, y: 0, z: 0},
                {x: 0, y: 0, z: 0, w: 1},
            ),
            body1,
            body2,
            true,
        );

        const anchor = {x: 0, y: 0, z: 0};
        expect(joint.anchor1(anchor)).toBe(anchor);
        expect(anchor.x).toBeCloseTo(1, 4);

        const frame = {x: 0, y: 0, z: 0, w: 1};
        expect(joint.frameX1(frame)).toBe(frame);

        world.free();
    });
});
