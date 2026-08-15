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

    public static fromRaw(raw: RawShapeContact): ShapeContact | null {
        if (!raw) return null;

        raw.getComponents();
        const s = scratch();
        raw.free();

        const result = new ShapeContact(
            s[0],
            VectorOps.zeros(),
            VectorOps.zeros(),
            VectorOps.zeros(),
            VectorOps.zeros(),
        );
        result.point1.x = s[1];
        result.point1.y = s[2];
        result.point1.z = s[3];
        result.point2.x = s[4];
        result.point2.y = s[5];
        result.point2.z = s[6];
        result.normal1.x = s[7];
        result.normal1.y = s[8];
        result.normal1.z = s[9];
        result.normal2.x = s[10];
        result.normal2.y = s[11];
        result.normal2.z = s[12];
        return result;
    }
}
