import type {Vector, Rotation} from "../math";
import type {RawColliderSet} from "../raw";
import type {ColliderHandle} from "./collider";
import type {Ray} from "./ray";
import {VectorOps, RotationOps} from "../math";
import {RawShape, RawShapeType} from "../raw";
import {ShapeContact} from "./contact";
import {PointProjection} from "./point";
import {RayIntersection} from "./ray";
import {ShapeCastHit} from "./toi";

/**
 * Unwraps a raw shape constructor that reports invalid input by returning nothing.
 *
 * These constructors take user-supplied buffers, and building a shape parry cannot
 * represent (a ragged vertex array, an index pointing past the last vertex, a
 * degenerate heightfield grid) used to trap inside WASM — which surfaces in JS as
 * `RuntimeError: unreachable`, with no indication of which input was wrong. They
 * now return nothing instead, so the input error can be reported here.
 */
function expectRawShape(rawShape: RawShape | undefined, reason: string): RawShape {
    if (!rawShape) {
        throw new Error(reason);
    }

    return rawShape;
}

export abstract class Shape {
    public abstract intoRaw(): RawShape;

    /**
     * The concrete type of this shape.
     */
    public abstract get type(): ShapeType;

    /**
     * instant mode without cache
     */
    public static fromRaw(rawSet: RawColliderSet, handle: ColliderHandle): Shape {
        return Shape.fromRawShape(rawSet.coShape(handle));
    }

    /**
     * Reconstructs a shape from its raw representation.
     *
     * This takes ownership of `rawShape` and always frees it before returning.
     */
    public static fromRawShape(rawShape: RawShape): Shape {
        const rawType = rawShape.shapeType();

        let extents: Vector;
        let borderRadius: number;
        let vs: Float32Array;
        let indices: Uint32Array;
        let halfHeight: number;
        let radius: number;

        try {
            switch (rawType) {
                case RawShapeType.Ball:
                    return new Ball(rawShape.radius()!);
                case RawShapeType.Cuboid:
                    extents = VectorOps.fromRaw(rawShape.halfExtents()!)!;
                    return new Cuboid(extents.x, extents.y, extents.z);

                case RawShapeType.RoundCuboid:
                    extents = VectorOps.fromRaw(rawShape.halfExtents()!)!;
                    borderRadius = rawShape.roundRadius()!;
                    return new RoundCuboid(extents.x, extents.y, extents.z, borderRadius);

                case RawShapeType.Capsule:
                    halfHeight = rawShape.halfHeight()!;
                    radius = rawShape.radius()!;
                    return new Capsule(halfHeight, radius);

                case RawShapeType.Segment:
                    vs = rawShape.vertices()!;
                    return new Segment(
                        VectorOps.new(vs[0], vs[1], vs[2]),
                        VectorOps.new(vs[3], vs[4], vs[5]),
                    );

                case RawShapeType.Polyline:
                    vs = rawShape.vertices()!;
                    indices = rawShape.indices()!;
                    return new Polyline(vs, indices);

                case RawShapeType.Triangle:
                    vs = rawShape.vertices()!;
                    return new Triangle(
                        VectorOps.new(vs[0], vs[1], vs[2]),
                        VectorOps.new(vs[3], vs[4], vs[5]),
                        VectorOps.new(vs[6], vs[7], vs[8]),
                    );

                case RawShapeType.RoundTriangle:
                    vs = rawShape.vertices()!;
                    borderRadius = rawShape.roundRadius()!;
                    return new RoundTriangle(
                        VectorOps.new(vs[0], vs[1], vs[2]),
                        VectorOps.new(vs[3], vs[4], vs[5]),
                        VectorOps.new(vs[6], vs[7], vs[8]),
                        borderRadius,
                    );

                case RawShapeType.HalfSpace:
                    return new HalfSpace(VectorOps.fromRaw(rawShape.halfspaceNormal()!)!);

                case RawShapeType.Voxels:
                    return new Voxels(
                        rawShape.voxelData()!,
                        VectorOps.fromRaw(rawShape.voxelSize()!)!,
                    );

                case RawShapeType.TriMesh:
                    vs = rawShape.vertices()!;
                    indices = rawShape.indices()!;
                    return new TriMesh(vs, indices, rawShape.triMeshFlags());

                case RawShapeType.HeightField:
                    return new Heightfield(
                        rawShape.heightfieldNRows()!,
                        rawShape.heightfieldNCols()!,
                        rawShape.heightfieldHeights()!,
                        VectorOps.fromRaw(rawShape.heightfieldScale()!)!,
                        rawShape.heightFieldFlags(),
                    );

                case RawShapeType.ConvexPolyhedron: {
                    const mesh = rawShape.convexMeshData();
                    if (!mesh) {
                        throw new Error(
                            "Failed to compute the convex hull of a convex polyhedron shape.",
                        );
                    }
                    vs = mesh.vertices;
                    indices = mesh.indices;
                    mesh.free();
                    return new ConvexPolyhedron(vs, indices);
                }

                case RawShapeType.RoundConvexPolyhedron: {
                    const mesh = rawShape.convexMeshData();
                    if (!mesh) {
                        throw new Error(
                            "Failed to compute the convex hull of a convex polyhedron shape.",
                        );
                    }
                    vs = mesh.vertices;
                    indices = mesh.indices;
                    mesh.free();
                    return new RoundConvexPolyhedron(vs, indices, rawShape.roundRadius()!);
                }

                case RawShapeType.Cylinder:
                    halfHeight = rawShape.halfHeight()!;
                    radius = rawShape.radius()!;
                    return new Cylinder(halfHeight, radius);

                case RawShapeType.RoundCylinder:
                    halfHeight = rawShape.halfHeight()!;
                    radius = rawShape.radius()!;
                    borderRadius = rawShape.roundRadius()!;
                    return new RoundCylinder(halfHeight, radius, borderRadius);

                case RawShapeType.Cone:
                    halfHeight = rawShape.halfHeight()!;
                    radius = rawShape.radius()!;
                    return new Cone(halfHeight, radius);

                case RawShapeType.RoundCone:
                    halfHeight = rawShape.halfHeight()!;
                    radius = rawShape.radius()!;
                    borderRadius = rawShape.roundRadius()!;
                    return new RoundCone(halfHeight, radius, borderRadius);

                case RawShapeType.Compound:
                    // `Compound.fromRawShape` takes ownership of `rawShape`.
                    return Compound.fromRawShape(rawShape);

                default:
                    throw new Error(`unknown shape type: ${RawShapeType[rawType]}`);
            }
        } finally {
            if (rawType !== RawShapeType.Compound) {
                rawShape.free();
            }
        }
    }
    /**
     * Computes the time of impact between two moving shapes.
     * @param shapePos1 - The initial position of this sahpe.
     * @param shapeRot1 - The rotation of this shape.
     * @param shapeVel1 - The velocity of this shape.
     * @param shape2 - The second moving shape.
     * @param shapePos2 - The initial position of the second shape.
     * @param shapeRot2 - The rotation of the second shape.
     * @param shapeVel2 - The velocity of the second shape.
     * @param targetDistance − If the shape moves closer to this distance from a collider, a hit
     *                         will be returned.
     * @param maxToi - The maximum time when the impact can happen.
     * @param stopAtPenetration - If set to `false`, the linear shape-cast won’t immediately stop if
     *   the shape is penetrating another shape at its starting point **and** its trajectory is such
     *   that it’s on a path to exit that penetration state.
     * @param target - Optional target object to write the result to (avoids allocation).
     * @returns If the two moving shapes collider at some point along their trajectories, this returns the
     *  time at which the two shape collider as well as the contact information during the impact. Returns
     *  `null`if the two shapes never collide along their paths.
     */
    public castShape(
        shapePos1: Vector,
        shapeRot1: Rotation,
        shapeVel1: Vector,
        shape2: Shape,
        shapePos2: Vector,
        shapeRot2: Rotation,
        shapeVel2: Vector,
        targetDistance: number,
        maxToi: number,
        stopAtPenetration: boolean,
        target?: ShapeCastHit,
    ): ShapeCastHit | null {
        const rawPos1 = VectorOps.intoRaw(shapePos1);
        const rawRot1 = RotationOps.intoRaw(shapeRot1);
        const rawVel1 = VectorOps.intoRaw(shapeVel1);
        const rawPos2 = VectorOps.intoRaw(shapePos2);
        const rawRot2 = RotationOps.intoRaw(shapeRot2);
        const rawVel2 = VectorOps.intoRaw(shapeVel2);

        const rawShape1 = this.intoRaw();
        const rawShape2 = shape2.intoRaw();

        const result = ShapeCastHit.fromRaw(
            null,
            rawShape1.castShape(
                rawPos1,
                rawRot1,
                rawVel1,
                rawShape2,
                rawPos2,
                rawRot2,
                rawVel2,
                targetDistance,
                maxToi,
                stopAtPenetration,
            )!,
            target,
        );

        rawPos1.free();
        rawRot1.free();
        rawVel1.free();
        rawPos2.free();
        rawRot2.free();
        rawVel2.free();

        rawShape1.free();
        rawShape2.free();

        return result;
    }

    /**
     * Tests if this shape intersects another shape.
     *
     * @param shapePos1 - The position of this shape.
     * @param shapeRot1 - The rotation of this shape.
     * @param shape2  - The second shape to test.
     * @param shapePos2 - The position of the second shape.
     * @param shapeRot2 - The rotation of the second shape.
     * @returns `true` if the two shapes intersect, `false` if they don’t.
     */
    public intersectsShape(
        shapePos1: Vector,
        shapeRot1: Rotation,
        shape2: Shape,
        shapePos2: Vector,
        shapeRot2: Rotation,
    ): boolean {
        const rawPos1 = VectorOps.intoRaw(shapePos1);
        const rawRot1 = RotationOps.intoRaw(shapeRot1);
        const rawPos2 = VectorOps.intoRaw(shapePos2);
        const rawRot2 = RotationOps.intoRaw(shapeRot2);

        const rawShape1 = this.intoRaw();
        const rawShape2 = shape2.intoRaw();

        const result = rawShape1.intersectsShape(rawPos1, rawRot1, rawShape2, rawPos2, rawRot2);

        rawPos1.free();
        rawRot1.free();
        rawPos2.free();
        rawRot2.free();

        rawShape1.free();
        rawShape2.free();

        return result;
    }

    /**
     * Computes one pair of contact points between two shapes.
     *
     * @param shapePos1 - The initial position of this sahpe.
     * @param shapeRot1 - The rotation of this shape.
     * @param shape2 - The second shape.
     * @param shapePos2 - The initial position of the second shape.
     * @param shapeRot2 - The rotation of the second shape.
     * @param prediction - The prediction value, if the shapes are separated by a distance greater than this value, test will fail.
     * @param target - Optional target object to write the result to (avoids allocation).
     * @returns `null` if the shapes are separated by a distance greater than prediction, otherwise contact details. The result is given in world-space.
     */
    contactShape(
        shapePos1: Vector,
        shapeRot1: Rotation,
        shape2: Shape,
        shapePos2: Vector,
        shapeRot2: Rotation,
        prediction: number,
        target?: ShapeContact,
    ): ShapeContact | null {
        const rawPos1 = VectorOps.intoRaw(shapePos1);
        const rawRot1 = RotationOps.intoRaw(shapeRot1);
        const rawPos2 = VectorOps.intoRaw(shapePos2);
        const rawRot2 = RotationOps.intoRaw(shapeRot2);

        const rawShape1 = this.intoRaw();
        const rawShape2 = shape2.intoRaw();

        const result = ShapeContact.fromRaw(
            rawShape1.contactShape(rawPos1, rawRot1, rawShape2, rawPos2, rawRot2, prediction)!,
            target,
        );

        rawPos1.free();
        rawRot1.free();
        rawPos2.free();
        rawRot2.free();

        rawShape1.free();
        rawShape2.free();

        return result;
    }

    containsPoint(shapePos: Vector, shapeRot: Rotation, point: Vector): boolean {
        const rawPos = VectorOps.intoRaw(shapePos);
        const rawRot = RotationOps.intoRaw(shapeRot);
        const rawPoint = VectorOps.intoRaw(point);
        const rawShape = this.intoRaw();

        const result = rawShape.containsPoint(rawPos, rawRot, rawPoint);

        rawPos.free();
        rawRot.free();
        rawPoint.free();
        rawShape.free();

        return result;
    }

    projectPoint(
        shapePos: Vector,
        shapeRot: Rotation,
        point: Vector,
        solid: boolean,
        target?: PointProjection,
    ): PointProjection {
        const rawPos = VectorOps.intoRaw(shapePos);
        const rawRot = RotationOps.intoRaw(shapeRot);
        const rawPoint = VectorOps.intoRaw(point);
        const rawShape = this.intoRaw();

        const result = PointProjection.fromRaw(
            rawShape.projectPoint(rawPos, rawRot, rawPoint, solid),
            target,
        );

        rawPos.free();
        rawRot.free();
        rawPoint.free();
        rawShape.free();

        return result!;
    }

    intersectsRay(ray: Ray, shapePos: Vector, shapeRot: Rotation, maxToi: number): boolean {
        const rawPos = VectorOps.intoRaw(shapePos);
        const rawRot = RotationOps.intoRaw(shapeRot);
        const rawRayOrig = VectorOps.intoRaw(ray.origin);
        const rawRayDir = VectorOps.intoRaw(ray.dir);
        const rawShape = this.intoRaw();

        const result = rawShape.intersectsRay(rawPos, rawRot, rawRayOrig, rawRayDir, maxToi);

        rawPos.free();
        rawRot.free();
        rawRayOrig.free();
        rawRayDir.free();
        rawShape.free();

        return result;
    }

    castRay(
        ray: Ray,
        shapePos: Vector,
        shapeRot: Rotation,
        maxToi: number,
        solid: boolean,
    ): number {
        const rawPos = VectorOps.intoRaw(shapePos);
        const rawRot = RotationOps.intoRaw(shapeRot);
        const rawRayOrig = VectorOps.intoRaw(ray.origin);
        const rawRayDir = VectorOps.intoRaw(ray.dir);
        const rawShape = this.intoRaw();

        const result = rawShape.castRay(rawPos, rawRot, rawRayOrig, rawRayDir, maxToi, solid);

        rawPos.free();
        rawRot.free();
        rawRayOrig.free();
        rawRayDir.free();
        rawShape.free();

        return result;
    }

    castRayAndGetNormal(
        ray: Ray,
        shapePos: Vector,
        shapeRot: Rotation,
        maxToi: number,
        solid: boolean,
        target?: RayIntersection,
    ): RayIntersection {
        const rawPos = VectorOps.intoRaw(shapePos);
        const rawRot = RotationOps.intoRaw(shapeRot);
        const rawRayOrig = VectorOps.intoRaw(ray.origin);
        const rawRayDir = VectorOps.intoRaw(ray.dir);
        const rawShape = this.intoRaw();

        const result = RayIntersection.fromRaw(
            rawShape.castRayAndGetNormal(rawPos, rawRot, rawRayOrig, rawRayDir, maxToi, solid)!,
            target,
        );

        rawPos.free();
        rawRot.free();
        rawRayOrig.free();
        rawRayDir.free();
        rawShape.free();

        return result!;
    }
}

/**
 * An enumeration representing the type of a shape.
 */
export enum ShapeType {
    Ball = 0,
    Cuboid = 1,
    Capsule = 2,
    Segment = 3,
    Polyline = 4,
    Triangle = 5,
    TriMesh = 6,
    HeightField = 7,
    Compound = 8,
    ConvexPolyhedron = 9,
    Cylinder = 10,
    Cone = 11,
    RoundCuboid = 12,
    RoundTriangle = 13,
    RoundCylinder = 14,
    RoundCone = 15,
    RoundConvexPolyhedron = 16,
    HalfSpace = 17,
    Voxels = 18,
}

/**
 * The shapes parry treats as composite, i.e. as a collection of sub-shapes with its
 * own acceleration structure. They cannot be nested inside a {@link Compound}.
 */
const COMPOSITE_SHAPE_TYPES: ReadonlySet<ShapeType> = new Set([
    ShapeType.Compound,
    ShapeType.TriMesh,
    ShapeType.Polyline,
]);

// NOTE: this **must** match the bits in the HeightFieldFlags on the rust side.
/**
 * Flags controlling the behavior of some operations involving heightfields.
 */
export enum HeightFieldFlags {
    /**
     * If set, a special treatment will be applied to contact manifold calculation to eliminate
     * or fix contacts normals that could lead to incorrect bumps in physics simulation (especially
     * on flat surfaces).
     *
     * This is achieved by taking into account adjacent triangle normals when computing contact
     * points for a given triangle.
     */
    FIX_INTERNAL_EDGES = 0b0000_0001,
}

// NOTE: this **must** match the TriMeshFlags on the rust side.
/**
 * Flags controlling the behavior of the triangle mesh creation and of some
 * operations involving triangle meshes.
 */
export enum TriMeshFlags {
    // NOTE: these two flags are not really useful in JS.
    //
    // /**
    //  * If set, the half-edge topology of the trimesh will be computed if possible.
    //  */
    // HALF_EDGE_TOPOLOGY = 0b0000_0001,
    // /** If set, the half-edge topology and connected components of the trimesh will be computed if possible.
    //  *
    //  * Because of the way it is currently implemented, connected components can only be computed on
    //  * a mesh where the half-edge topology computation succeeds. It will no longer be the case in the
    //  * future once we decouple the computations.
    //  */
    // CONNECTED_COMPONENTS = 0b0000_0010,
    /**
     * If set, any triangle that results in a failing half-hedge topology computation will be deleted.
     */
    DELETE_BAD_TOPOLOGY_TRIANGLES = 0b0000_0100,
    /**
     * If set, the trimesh will be assumed to be oriented (with outward normals).
     *
     * The pseudo-normals of its vertices and edges will be computed.
     */
    ORIENTED = 0b0000_1000,
    /**
     * If set, the duplicate vertices of the trimesh will be merged.
     *
     * Two vertices with the exact same coordinates will share the same entry on the
     * vertex buffer and the index buffer is adjusted accordingly.
     */
    MERGE_DUPLICATE_VERTICES = 0b0001_0000,
    /**
     * If set, the triangles sharing two vertices with identical index values will be removed.
     *
     * Because of the way it is currently implemented, this methods implies that duplicate
     * vertices will be merged. It will no longer be the case in the future once we decouple
     * the computations.
     */
    DELETE_DEGENERATE_TRIANGLES = 0b0010_0000,
    /**
     * If set, two triangles sharing three vertices with identical index values (in any order)
     * will be removed.
     *
     * Because of the way it is currently implemented, this methods implies that duplicate
     * vertices will be merged. It will no longer be the case in the future once we decouple
     * the computations.
     */
    DELETE_DUPLICATE_TRIANGLES = 0b0100_0000,
    /**
     * If set, a special treatment will be applied to contact manifold calculation to eliminate
     * or fix contacts normals that could lead to incorrect bumps in physics simulation
     * (especially on flat surfaces).
     *
     * This is achieved by taking into account adjacent triangle normals when computing contact
     * points for a given triangle.
     *
     * /!\ NOT SUPPORTED IN THE 2D VERSION OF RAPIER.
     */
    FIX_INTERNAL_EDGES = 0b1000_0000 | TriMeshFlags.MERGE_DUPLICATE_VERTICES,
}

/**
 * A shape that is a sphere in 3D and a circle in 2D.
 */
export class Ball extends Shape {
    readonly type = ShapeType.Ball;

    /**
     * The balls radius.
     */
    radius: number;

    /**
     * Creates a new ball with the given radius.
     * @param radius - The balls radius.
     */
    constructor(radius: number) {
        super();
        this.radius = radius;
    }

    public intoRaw(): RawShape {
        return RawShape.ball(this.radius);
    }
}

export class HalfSpace extends Shape {
    readonly type = ShapeType.HalfSpace;

    /**
     * The outward normal of the half-space.
     */
    normal: Vector;

    /**
     * Creates a new halfspace delimited by an infinite plane.
     *
     * @param normal - The outward normal of the plane.
     */
    constructor(normal: Vector) {
        super();
        this.normal = normal;
    }

    public intoRaw(): RawShape {
        const n = VectorOps.intoRaw(this.normal);
        const result = RawShape.halfspace(n);
        n.free();
        return result;
    }
}

/**
 * A shape that is a box in 3D and a rectangle in 2D.
 */
export class Cuboid extends Shape {
    readonly type = ShapeType.Cuboid;

    /**
     * The half extent of the cuboid along each coordinate axis.
     */
    halfExtents: Vector;

    /**
     * Creates a new 3D cuboid.
     * @param hx - The half width of the cuboid.
     * @param hy - The half height of the cuboid.
     * @param hz - The half depth of the cuboid.
     */
    constructor(hx: number, hy: number, hz: number) {
        super();
        this.halfExtents = VectorOps.new(hx, hy, hz);
    }

    public intoRaw(): RawShape {
        return RawShape.cuboid(this.halfExtents.x, this.halfExtents.y, this.halfExtents.z);
    }
}

/**
 * A shape that is a box in 3D and a rectangle in 2D, with round corners.
 */
export class RoundCuboid extends Shape {
    readonly type = ShapeType.RoundCuboid;

    /**
     * The half extent of the cuboid along each coordinate axis.
     */
    halfExtents: Vector;

    /**
     * The radius of the cuboid's round border.
     */
    borderRadius: number;

    /**
     * Creates a new 3D cuboid.
     * @param hx - The half width of the cuboid.
     * @param hy - The half height of the cuboid.
     * @param hz - The half depth of the cuboid.
     * @param borderRadius - The radius of the borders of this cuboid. This will
     *   effectively increase the half-extents of the cuboid by this radius.
     */
    constructor(hx: number, hy: number, hz: number, borderRadius: number) {
        super();
        this.halfExtents = VectorOps.new(hx, hy, hz);
        this.borderRadius = borderRadius;
    }

    public intoRaw(): RawShape {
        return RawShape.roundCuboid(
            this.halfExtents.x,
            this.halfExtents.y,
            this.halfExtents.z,
            this.borderRadius,
        );
    }
}

/**
 * A shape that is a capsule.
 */
export class Capsule extends Shape {
    readonly type = ShapeType.Capsule;

    /**
     * The radius of the capsule's basis.
     */
    radius: number;

    /**
     * The capsule's half height, along the `y` axis.
     */
    halfHeight: number;

    /**
     * Creates a new capsule with the given radius and half-height.
     * @param halfHeight - The balls half-height along the `y` axis.
     * @param radius - The balls radius.
     */
    constructor(halfHeight: number, radius: number) {
        super();
        this.halfHeight = halfHeight;
        this.radius = radius;
    }

    public intoRaw(): RawShape {
        return RawShape.capsule(this.halfHeight, this.radius);
    }
}

/**
 * A shape that is a segment.
 */
export class Segment extends Shape {
    readonly type = ShapeType.Segment;

    /**
     * The first point of the segment.
     */
    a: Vector;

    /**
     * The second point of the segment.
     */
    b: Vector;

    /**
     * Creates a new segment shape.
     * @param a - The first point of the segment.
     * @param b - The second point of the segment.
     */
    constructor(a: Vector, b: Vector) {
        super();
        this.a = a;
        this.b = b;
    }

    public intoRaw(): RawShape {
        const ra = VectorOps.intoRaw(this.a);
        const rb = VectorOps.intoRaw(this.b);
        const result = RawShape.segment(ra, rb);
        ra.free();
        rb.free();
        return result;
    }
}

/**
 * A shape that is a segment.
 */
export class Triangle extends Shape {
    readonly type = ShapeType.Triangle;

    /**
     * The first point of the triangle.
     */
    a: Vector;

    /**
     * The second point of the triangle.
     */
    b: Vector;

    /**
     * The second point of the triangle.
     */
    c: Vector;

    /**
     * Creates a new triangle shape.
     *
     * @param a - The first point of the triangle.
     * @param b - The second point of the triangle.
     * @param c - The third point of the triangle.
     */
    constructor(a: Vector, b: Vector, c: Vector) {
        super();
        this.a = a;
        this.b = b;
        this.c = c;
    }

    public intoRaw(): RawShape {
        const ra = VectorOps.intoRaw(this.a);
        const rb = VectorOps.intoRaw(this.b);
        const rc = VectorOps.intoRaw(this.c);
        const result = RawShape.triangle(ra, rb, rc);
        ra.free();
        rb.free();
        rc.free();
        return result;
    }
}

/**
 * A shape that is a triangle with round borders and a non-zero thickness.
 */
export class RoundTriangle extends Shape {
    readonly type = ShapeType.RoundTriangle;

    /**
     * The first point of the triangle.
     */
    a: Vector;

    /**
     * The second point of the triangle.
     */
    b: Vector;

    /**
     * The second point of the triangle.
     */
    c: Vector;

    /**
     * The radius of the triangles's rounded edges and vertices.
     * In 3D, this is also equal to half the thickness of the round triangle.
     */
    borderRadius: number;

    /**
     * Creates a new triangle shape with round corners.
     *
     * @param a - The first point of the triangle.
     * @param b - The second point of the triangle.
     * @param c - The third point of the triangle.
     * @param borderRadius - The radius of the borders of this triangle. In 3D,
     *   this is also equal to half the thickness of the triangle.
     */
    constructor(a: Vector, b: Vector, c: Vector, borderRadius: number) {
        super();
        this.a = a;
        this.b = b;
        this.c = c;
        this.borderRadius = borderRadius;
    }

    public intoRaw(): RawShape {
        const ra = VectorOps.intoRaw(this.a);
        const rb = VectorOps.intoRaw(this.b);
        const rc = VectorOps.intoRaw(this.c);
        const result = RawShape.roundTriangle(ra, rb, rc, this.borderRadius);
        ra.free();
        rb.free();
        rc.free();
        return result;
    }
}

/**
 * A shape that is a triangle mesh.
 */
export class Polyline extends Shape {
    readonly type = ShapeType.Polyline;

    /**
     * The vertices of the polyline.
     */
    vertices: Float32Array;

    /**
     * The indices of the segments.
     */
    indices: Uint32Array;

    /**
     * Creates a new polyline shape.
     *
     * @param vertices - The coordinates of the polyline's vertices.
     * @param indices - The indices of the polyline's segments. If this is `null` or not provided, then
     *    the vertices are assumed to form a line strip.
     */
    constructor(vertices: Float32Array, indices?: Uint32Array) {
        super();
        this.vertices = vertices;
        this.indices = indices ?? new Uint32Array(0);
    }

    public intoRaw(): RawShape {
        return expectRawShape(
            RawShape.polyline(this.vertices, this.indices),
            "invalid polyline: `vertices` must hold 3 coordinates per vertex, and `indices` " +
                "2 in-range vertex indices per segment",
        );
    }
}

/**
 * A shape made of voxels.
 */
export class Voxels extends Shape {
    readonly type = ShapeType.Voxels;

    /**
     * The points or grid coordinates used to initialize the voxels.
     */
    data: Float32Array | Int32Array;

    /**
     * The dimensions of each voxel.
     */
    voxelSize: Vector;

    /**
     * Creates a new shape made of voxels.
     *
     * @param data - Defines the set of voxels. If this is a `Int32Array` then
     *               each voxel is defined from its (signed) grid coordinates,
     *               with 3 (resp 2) contiguous integers per voxel in 3D (resp 2D).
     *               If this is a `Float32Array`, each voxel will be such that
     *               they contain at least one point from this array (where each
     *               point is defined from 3 (resp 2) contiguous numbers per point
     *               in 3D (resp 2D).
     * @param voxelSize - The size of each voxel.
     */
    constructor(data: Float32Array | Int32Array, voxelSize: Vector) {
        super();
        this.data = data;
        this.voxelSize = voxelSize;
    }

    public intoRaw(): RawShape {
        const voxelSize = VectorOps.intoRaw(this.voxelSize);

        let result;
        if (this.data instanceof Int32Array) {
            result = RawShape.voxels(voxelSize, this.data);
        } else {
            result = RawShape.voxelsFromPoints(voxelSize, this.data);
        }

        voxelSize.free();
        return result;
    }
}

/**
 * A compound shape, made of multiple sub-shapes placed at fixed relative poses.
 */
export class Compound extends Shape {
    readonly type = ShapeType.Compound;

    /**
     * The shapes composing this compound shape.
     */
    shapes: Shape[];

    /**
     * The position of each sub-shape, relative to the compound's origin.
     */
    positions: Vector[];

    /**
     * The rotation of each sub-shape, relative to the compound's orientation.
     */
    rotations: Rotation[];

    /**
     * Creates a new compound shape.
     *
     * @param shapes - The shapes composing this compound. Must not be empty, and must not
     *                 contain composite shapes (compounds, triangle meshes and polylines
     *                 cannot be nested inside a compound).
     * @param positions - The position of each sub-shape.
     * @param rotations - The rotation of each sub-shape.
     */
    constructor(shapes: Shape[], positions: Vector[], rotations: Rotation[]) {
        super();

        if (shapes.length !== positions.length || shapes.length !== rotations.length) {
            throw new Error("shapes, positions, and rotations must have the same length");
        }

        if (shapes.length === 0) {
            throw new Error("a compound shape must contain at least one shape");
        }

        // Triangle meshes and polylines are composite shapes too, so they are just as
        // illegal here as a nested compound: all three make the WASM-side builder trap.
        const nested = shapes.find((shape) => COMPOSITE_SHAPE_TYPES.has(shape.type));
        if (nested !== undefined) {
            throw new Error(
                `nested composite shapes are not allowed in a compound (got ${ShapeType[nested.type]})`,
            );
        }

        this.shapes = shapes;
        this.positions = positions;
        this.rotations = rotations;
    }

    /**
     * Reconstructs a compound shape from its raw representation.
     *
     * This takes ownership of `rawShape` and always frees it before returning.
     */
    public static fromRawShape(rawShape: RawShape): Compound {
        try {
            const numShapes = rawShape.compoundLen();
            if (numShapes == null) {
                throw new Error("expected a raw compound shape");
            }

            const shapes: Shape[] = [];
            const positions: Vector[] = [];
            const rotations: Rotation[] = [];

            for (let i = 0; i < numShapes; i++) {
                shapes.push(Shape.fromRawShape(rawShape.compoundShape(i)!));
                positions.push(VectorOps.fromRaw(rawShape.compoundTranslation(i)!)!);
                rotations.push(RotationOps.fromRaw(rawShape.compoundRotation(i)!)!);
            }

            return new Compound(shapes, positions, rotations);
        } finally {
            rawShape.free();
        }
    }

    public intoRaw(): RawShape {
        const rawShapes = this.shapes.map((shape) => shape.intoRaw());

        const positions = new Float32Array(this.positions.length * 3);
        this.positions.forEach((pos, i) => {
            positions[i * 3] = pos.x;
            positions[i * 3 + 1] = pos.y;
            positions[i * 3 + 2] = pos.z;
        });

        const rotations = new Float32Array(this.rotations.length * 4);
        this.rotations.forEach((rot, i) => {
            rotations[i * 4] = rot.x;
            rotations[i * 4 + 1] = rot.y;
            rotations[i * 4 + 2] = rot.z;
            rotations[i * 4 + 3] = rot.w;
        });

        return expectRawShape(
            RawShape.compound(rawShapes, positions, rotations),
            "invalid compound shape: expected at least one non-composite sub-shape, with a " +
                "position and a rotation each",
        );
    }
}

/**
 * A shape that is a triangle mesh.
 */
export class TriMesh extends Shape {
    readonly type = ShapeType.TriMesh;

    /**
     * The vertices of the triangle mesh.
     */
    vertices: Float32Array;

    /**
     * The indices of the triangles.
     */
    indices: Uint32Array;

    /**
     * The triangle mesh flags.
     */
    flags?: TriMeshFlags;

    /**
     * Creates a new triangle mesh shape.
     *
     * @param vertices - The coordinates of the triangle mesh's vertices.
     * @param indices - The indices of the triangle mesh's triangles.
     */
    constructor(vertices: Float32Array, indices: Uint32Array, flags?: TriMeshFlags) {
        super();
        this.vertices = vertices;
        this.indices = indices;
        this.flags = flags;
    }

    public intoRaw(): RawShape {
        return expectRawShape(
            RawShape.trimesh(this.vertices, this.indices, this.flags ?? 0),
            "invalid triangle mesh: `vertices` must hold 3 coordinates per vertex, and " +
                "`indices` 3 in-range vertex indices per triangle",
        );
    }
}

/**
 * A shape that is a convex polygon.
 */
export class ConvexPolyhedron extends Shape {
    readonly type = ShapeType.ConvexPolyhedron;

    /**
     * The vertices of the convex polygon.
     */
    vertices: Float32Array;

    /**
     * The indices of the convex polygon.
     */
    indices?: Uint32Array | null;

    /**
     * Creates a new convex polygon shape.
     *
     * @param vertices - The coordinates of the convex polygon's vertices.
     * @param indices - The index buffer of this convex mesh. If this is `null`
     *   or `undefined`, the convex-hull of the input vertices will be computed
     *   automatically. Otherwise, it will be assumed that the mesh you provide
     *   is already convex.
     */
    constructor(vertices: Float32Array, indices?: Uint32Array | null) {
        super();
        this.vertices = vertices;
        this.indices = indices;
    }

    public intoRaw(): RawShape {
        if (this.indices) {
            return expectRawShape(
                RawShape.convexMesh(this.vertices, this.indices),
                "invalid convex mesh: `vertices` must hold 3 coordinates per vertex, and " +
                    "`indices` 3 in-range vertex indices per face, forming a convex mesh",
            );
        } else {
            return expectRawShape(
                RawShape.convexHull(this.vertices),
                "invalid convex hull: `vertices` must hold 3 coordinates per vertex, and the " +
                    "points must not be degenerate",
            );
        }
    }
}

/**
 * A shape that is a convex polygon.
 */
export class RoundConvexPolyhedron extends Shape {
    readonly type = ShapeType.RoundConvexPolyhedron;

    /**
     * The vertices of the convex polygon.
     */
    vertices: Float32Array;

    /**
     * The indices of the convex polygon.
     */
    indices?: Uint32Array | null;

    /**
     * The radius of the convex polyhedron's rounded edges and vertices.
     */
    borderRadius: number;

    /**
     * Creates a new convex polygon shape.
     *
     * @param vertices - The coordinates of the convex polygon's vertices.
     * @param indices - The index buffer of this convex mesh. If this is `null`
     *   or `undefined`, the convex-hull of the input vertices will be computed
     *   automatically. Otherwise, it will be assumed that the mesh you provide
     *   is already convex.
     * @param borderRadius - The radius of the borders of this convex polyhedron.
     */
    constructor(
        vertices: Float32Array,
        indices: Uint32Array | null | undefined,
        borderRadius: number,
    ) {
        super();
        this.vertices = vertices;
        this.indices = indices;
        this.borderRadius = borderRadius;
    }

    public intoRaw(): RawShape {
        if (this.indices) {
            return expectRawShape(
                RawShape.roundConvexMesh(this.vertices, this.indices, this.borderRadius),
                "invalid round convex mesh: `vertices` must hold 3 coordinates per vertex, and " +
                    "`indices` 3 in-range vertex indices per face, forming a convex mesh",
            );
        } else {
            return expectRawShape(
                RawShape.roundConvexHull(this.vertices, this.borderRadius),
                "invalid round convex hull: `vertices` must hold 3 coordinates per vertex, and " +
                    "the points must not be degenerate",
            );
        }
    }
}

/**
 * A shape that is a heightfield.
 */
export class Heightfield extends Shape {
    readonly type = ShapeType.HeightField;

    /**
     * The number of rows in the heights matrix.
     */
    nrows: number;

    /**
     * The number of columns in the heights matrix.
     */
    ncols: number;

    /**
     * The heights of the heightfield along its local `y` axis,
     * provided as a matrix stored in column-major order.
     */
    heights: Float32Array;

    /**
     * The dimensions of the heightfield's local `x,z` plane.
     */
    scale: Vector;

    /**
     * Flags applied to the heightfield.
     */
    flags?: HeightFieldFlags;

    /**
     * Creates a new heightfield shape.
     *
     * @param nrows − The number of rows in the heights matrix.
     * @param ncols - The number of columns in the heights matrix.
     * @param heights - The heights of the heightfield along its local `y` axis,
     *                  provided as a matrix stored in column-major order.
     * @param scale - The dimensions of the heightfield's local `x,z` plane.
     */
    constructor(
        nrows: number,
        ncols: number,
        heights: Float32Array,
        scale: Vector,
        flags?: HeightFieldFlags,
    ) {
        super();
        this.nrows = nrows;
        this.ncols = ncols;
        this.heights = heights;
        this.scale = scale;
        this.flags = flags;
    }

    public intoRaw(): RawShape {
        const rawScale = VectorOps.intoRaw(this.scale);
        const rawShape = RawShape.heightfield(
            this.nrows,
            this.ncols,
            this.heights,
            rawScale,
            this.flags ?? 0,
        );
        rawScale.free();

        return expectRawShape(
            rawShape,
            "invalid heightfield: `nrows` and `ncols` must be at least 1, and `heights` must " +
                "hold exactly `(nrows + 1) * (ncols + 1)` entries",
        );
    }
}

/**
 * A shape that is a 3D cylinder.
 */
export class Cylinder extends Shape {
    readonly type = ShapeType.Cylinder;

    /**
     * The radius of the cylinder's basis.
     */
    radius: number;

    /**
     * The cylinder's half height, along the `y` axis.
     */
    halfHeight: number;

    /**
     * Creates a new cylinder with the given radius and half-height.
     * @param halfHeight - The balls half-height along the `y` axis.
     * @param radius - The balls radius.
     */
    constructor(halfHeight: number, radius: number) {
        super();
        this.halfHeight = halfHeight;
        this.radius = radius;
    }

    public intoRaw(): RawShape {
        return RawShape.cylinder(this.halfHeight, this.radius);
    }
}

/**
 * A shape that is a 3D cylinder with round corners.
 */
export class RoundCylinder extends Shape {
    readonly type = ShapeType.RoundCylinder;

    /**
     * The radius of the cylinder's basis.
     */
    radius: number;

    /**
     * The cylinder's half height, along the `y` axis.
     */
    halfHeight: number;

    /**
     * The radius of the cylinder's rounded edges and vertices.
     */
    borderRadius: number;

    /**
     * Creates a new cylinder with the given radius and half-height.
     * @param halfHeight - The balls half-height along the `y` axis.
     * @param radius - The balls radius.
     * @param borderRadius - The radius of the borders of this cylinder.
     */
    constructor(halfHeight: number, radius: number, borderRadius: number) {
        super();
        this.borderRadius = borderRadius;
        this.halfHeight = halfHeight;
        this.radius = radius;
    }

    public intoRaw(): RawShape {
        return RawShape.roundCylinder(this.halfHeight, this.radius, this.borderRadius);
    }
}

/**
 * A shape that is a 3D cone.
 */
export class Cone extends Shape {
    readonly type = ShapeType.Cone;

    /**
     * The radius of the cone's basis.
     */
    radius: number;

    /**
     * The cone's half height, along the `y` axis.
     */
    halfHeight: number;

    /**
     * Creates a new cone with the given radius and half-height.
     * @param halfHeight - The balls half-height along the `y` axis.
     * @param radius - The balls radius.
     */
    constructor(halfHeight: number, radius: number) {
        super();
        this.halfHeight = halfHeight;
        this.radius = radius;
    }

    public intoRaw(): RawShape {
        return RawShape.cone(this.halfHeight, this.radius);
    }
}

/**
 * A shape that is a 3D cone with round corners.
 */
export class RoundCone extends Shape {
    readonly type = ShapeType.RoundCone;

    /**
     * The radius of the cone's basis.
     */
    radius: number;

    /**
     * The cone's half height, along the `y` axis.
     */
    halfHeight: number;

    /**
     * The radius of the cylinder's rounded edges and vertices.
     */
    borderRadius: number;

    /**
     * Creates a new cone with the given radius and half-height.
     * @param halfHeight - The balls half-height along the `y` axis.
     * @param radius - The balls radius.
     * @param borderRadius - The radius of the borders of this cone.
     */
    constructor(halfHeight: number, radius: number, borderRadius: number) {
        super();
        this.halfHeight = halfHeight;
        this.radius = radius;
        this.borderRadius = borderRadius;
    }

    public intoRaw(): RawShape {
        return RawShape.roundCone(this.halfHeight, this.radius, this.borderRadius);
    }
}
