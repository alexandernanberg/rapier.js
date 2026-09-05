/**
 * Geometry helpers shared by the box3d benchmark ports, transcribed from
 * `box3d/src/hull.c` and `box3d/src/mesh.c`.
 */

/**
 * `b3CreateCylinder`: `2 * sides` points forming a cylinder of the given
 * height/radius, its base at `yOffset`, aligned with the Y axis. Returned as a
 * convex point cloud — rapier builds the hull.
 */
export function createCylinder(
    height: number,
    radius: number,
    yOffset: number,
    sides: number,
): Float32Array {
    let points = new Float32Array(2 * sides * 3);
    let deltaAlpha = (2.0 * Math.PI) / sides;
    let alpha = 0.0;
    let i;

    for (i = 0; i < sides; ++i) {
        let x = radius * Math.cos(alpha);
        let z = radius * Math.sin(alpha);

        points[i * 6 + 0] = x;
        points[i * 6 + 1] = yOffset;
        points[i * 6 + 2] = z;
        points[i * 6 + 3] = x;
        points[i * 6 + 4] = yOffset + height;
        points[i * 6 + 5] = z;

        alpha += deltaAlpha;
    }

    return points;
}

/** `b3CreateRock`: 10 points on a Fibonacci lattice on a sphere of `radius`. */
export function createRock(radius: number): Float32Array {
    let pointCount = 10;
    let points = new Float32Array(pointCount * 3);
    let phi = (1.0 + Math.sqrt(5.0)) / 2.0;
    let theta = (2.0 * Math.PI) / phi;
    let deltaSin = Math.sin(theta);
    let deltaCos = Math.cos(theta);
    let cos = 1.0;
    let sin = 0.0;
    let i;

    for (i = 0; i < pointCount; ++i) {
        let z = 1.0 - (2.0 * i + 1.0) / pointCount;
        let radiusXy = Math.sqrt(1.0 - z * z);

        points[i * 3 + 0] = radius * radiusXy * cos;
        points[i * 3 + 1] = radius * radiusXy * sin;
        points[i * 3 + 2] = radius * z;

        let c0 = cos;
        let s0 = sin;
        cos = deltaCos * c0 - deltaSin * s0;
        sin = deltaSin * c0 + deltaCos * s0;
    }

    return points;
}

/**
 * Shared vertex/index grid builder for the box3d grid/wave meshes. `height`
 * yields the Y coordinate at grid cell `(ix, iz)`.
 */
export function gridMesh(
    xCount: number,
    zCount: number,
    cellWidth: number,
    height: (ix: number, iz: number) => number,
) {
    let xWidth = cellWidth * xCount;
    let zWidth = cellWidth * zCount;
    let vertices = new Float32Array((xCount + 1) * (zCount + 1) * 3);
    let indices = new Uint32Array(2 * xCount * zCount * 3);
    let ix, iz;
    let v = 0;

    let x = -0.5 * xWidth;
    for (ix = 0; ix <= xCount; ++ix) {
        let z = -0.5 * zWidth;

        for (iz = 0; iz <= zCount; ++iz) {
            vertices[v++] = x;
            vertices[v++] = height(ix, iz);
            vertices[v++] = z;
            z += cellWidth;
        }

        x += cellWidth;
    }

    let t = 0;
    for (ix = 0; ix < xCount; ++ix) {
        for (iz = 0; iz < zCount; ++iz) {
            let i1 = iz + (zCount + 1) * ix;
            let i2 = i1 + 1;
            let i3 = i2 + (zCount + 1);
            let i4 = i3 - 1;

            indices[t++] = i1;
            indices[t++] = i2;
            indices[t++] = i3;
            indices[t++] = i3;
            indices[t++] = i4;
            indices[t++] = i1;
        }
    }

    return {vertices, indices};
}

/** `b3CreateWaveMesh`: a grid whose height is a product of two sinusoids. */
export function createWaveMesh(
    xCount: number,
    zCount: number,
    cellWidth: number,
    amplitude: number,
    rowFrequency: number,
    columnFrequency: number,
) {
    let omegaZ = 2.0 * Math.PI * rowFrequency * cellWidth;
    let omegaX = 2.0 * Math.PI * columnFrequency * cellWidth;

    return gridMesh(xCount, zCount, cellWidth, (ix, iz) => {
        return amplitude * Math.sin(omegaX * ix) * Math.sin(omegaZ * iz);
    });
}
