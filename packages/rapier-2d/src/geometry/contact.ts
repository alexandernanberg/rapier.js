import type {Vector} from "../math";
import type {RawShapeContact} from "../raw";
import {VectorOps} from "../math";
import {scratch} from "../scratch";

/**
 * The contact info between two shapes.
 */
export class ShapeContact {
    /**
     * Distance between the two contact points.
     * If this is negative, this contact represents a penetration.
     */
    distance: number;

    /**
     * Position of the contact on the first shape.
     */
    point1: Vector;

    /**
     * Position of the contact on the second shape.
     */
    point2: Vector;

    /**
     * Contact normal, pointing towards the exterior of the first shape.
     */
    normal1: Vector;

    /**
     * Contact normal, pointing towards the exterior of the second shape.
     * If these contact data are expressed in world-space, this normal is equal to -normal1.
     */
    normal2: Vector;

    constructor(dist: number, point1: Vector, point2: Vector, normal1: Vector, normal2: Vector) {
        this.distance = dist;
        this.point1 = point1;
        this.point2 = point2;
        this.normal1 = normal1;
        this.normal2 = normal2;
    }

    /**
     * Reads a shape contact from its raw representation.
     *
     * @param raw - The raw contact. It is always freed before returning.
     * @param target - Optional target object to write the result to (avoids allocation).
     */
    public static fromRaw(raw: RawShapeContact, target?: ShapeContact): ShapeContact | null {
        if (!raw) return null;

        raw.getComponents();
        const s = scratch();
        raw.free();

        const result =
            target ??
            new ShapeContact(
                0,
                VectorOps.zeros(),
                VectorOps.zeros(),
                VectorOps.zeros(),
                VectorOps.zeros(),
            );

        result.distance = s[0];
        result.point1 = VectorOps.set(result.point1, s[1], s[2]);
        result.point2 = VectorOps.set(result.point2, s[3], s[4]);
        result.normal1 = VectorOps.set(result.normal1, s[5], s[6]);
        result.normal2 = VectorOps.set(result.normal2, s[7], s[8]);
        return result;
    }
}
