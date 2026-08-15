import {Vector, VectorOps} from "../math";
import {RawShapeContact} from "../raw";

/**
 * Shared scratch buffer for WASM reads (single-threaded, safe to share).
 *
 * Its length must match exactly what the Rust side writes: `getComponents`
 * hands the whole payload over in one `Float32Array::copy_from`, which asserts
 * that the lengths are equal.
 */
const _scratch = new Float32Array(9);

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

        raw.getComponents(_scratch);
        raw.free();

        const result = new ShapeContact(
            _scratch[0],
            VectorOps.zeros(),
            VectorOps.zeros(),
            VectorOps.zeros(),
            VectorOps.zeros(),
        );
        result.point1.x = _scratch[1];
        result.point1.y = _scratch[2];
        result.point2.x = _scratch[3];
        result.point2.y = _scratch[4];
        result.normal1.x = _scratch[5];
        result.normal1.y = _scratch[6];
        result.normal2.x = _scratch[7];
        result.normal2.y = _scratch[8];
        return result;
    }
}
