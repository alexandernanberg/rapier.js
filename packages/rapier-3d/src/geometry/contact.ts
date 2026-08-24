import {Vector, VectorOps} from "../math";
import {RawShapeContact} from "../raw";
import {scratch} from "../scratch";

/**
 * The contact info between two shapes.
 */
export class ShapeContact {
    /**
     * Distance between the two contact points.
     * If this is negative, this contact represents a penetration.
     */
    distance = 0;

    /**
     * Position of the contact on the first shape.
     */
    point1: Vector = VectorOps.zeros();

    /**
     * Position of the contact on the second shape.
     */
    point2: Vector = VectorOps.zeros();

    /**
     * Contact normal, pointing towards the exterior of the first shape.
     */
    normal1: Vector = VectorOps.zeros();

    /**
     * Contact normal, pointing towards the exterior of the second shape.
     * If these contact data are expressed in world-space, this normal is equal to -normal1.
     */
    normal2: Vector = VectorOps.zeros();

    /**
     * Reads a shape contact from its raw representation.
     *
     * @param raw - The raw contact. It is always freed before returning.
     * @param target - The object the result is written into.
     */
    public static fromRaw(raw: RawShapeContact, target: ShapeContact): ShapeContact | null {
        if (!raw) return null;

        raw.getComponents();
        const s = scratch();
        raw.free();

        target.distance = s[0];
        target.point1 = VectorOps.set(target.point1, s[1], s[2], s[3]);
        target.point2 = VectorOps.set(target.point2, s[4], s[5], s[6]);
        target.normal1 = VectorOps.set(target.normal1, s[7], s[8], s[9]);
        target.normal2 = VectorOps.set(target.normal2, s[10], s[11], s[12]);
        return target;
    }
}
