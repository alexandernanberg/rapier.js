import {RawVector, RawRotation} from "./raw";

export interface Vector {
    x: number;
    y: number;
}

/**
 * A 2D vector.
 */
export class Vector2 implements Vector {
    x: number;
    y: number;

    constructor(x: number, y: number) {
        this.x = x;
        this.y = y;
    }
}

export class VectorOps {
    public static new(x: number, y: number): Vector {
        return new Vector2(x, y);
    }

    public static zeros(): Vector {
        return VectorOps.new(0.0, 0.0);
    }

    public static fromRaw(raw: RawVector, target?: Vector): Vector | null {
        if (!raw) return null;

        target ??= VectorOps.zeros();
        target.x = raw.x;
        target.y = raw.y;
        raw.free();
        return target;
    }

    public static intoRaw(v: Vector): RawVector {
        return new RawVector(v.x, v.y);
    }

    public static copy(out: Vector, input: Vector) {
        out.x = input.x;
        out.y = input.y;
    }

    public static fromBuffer(buffer: Float32Array, target?: Vector): Vector {
        target ??= VectorOps.zeros();
        target.x = buffer[0];
        target.y = buffer[1];
        return target;
    }
}

/**
 * A rotation angle in radians.
 */
export type Rotation = number;

export class RotationOps {
    public static identity(): number {
        return 0.0;
    }

    public static fromRaw(raw: RawRotation): Rotation | null {
        if (!raw) return null;

        let res = raw.angle;
        raw.free();
        return res;
    }

    public static intoRaw(angle: Rotation): RawRotation {
        return RawRotation.fromAngle(angle);
    }

    public static fromBuffer(buffer: Float32Array): Rotation {
        return buffer[0];
    }
}
