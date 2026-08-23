/**
 * The render seam.
 *
 * Deliberately shaped toward the renderer we do *not* have yet: packed instance
 * matrices, explicit camera state, materials as data. A library backend can
 * always adapt down to that; a scene-graph-shaped interface could not adapt up.
 *
 * The contrast with the physics decision is intentional. There the advice was to
 * defer the interface, because it would have had to abstract over *behaviour* —
 * solver semantics, joint types, CCD — which is unknowable with one
 * implementation. This abstracts over *data transport*, whose exact shape is
 * already measured. Transport is worth defining up front; behaviour is not.
 */

export type MeshId = number;
export type MaterialId = number;

/** Engine-owned geometry. Backends upload it; they never own the source data. */
export interface MeshDesc {
    name: string;
    positions: Float32Array;
    normals?: Float32Array;
    uvs?: Float32Array;
    indices?: Uint32Array;
}

/**
 * Material as data, never shader code. A Three backend maps this to
 * MeshStandardMaterial; a WGSL backend maps it to a uniform block. Authoring
 * materials as TSL node graphs would weld us to one backend and to a DSL with
 * far less training data than WGSL.
 */
export interface MaterialDesc {
    name: string;
    /** Linear-space RGB, 0..1. Conversion to sRGB happens at output, once. */
    baseColor: [number, number, number];
    metallic: number;
    roughness: number;
    emissive?: [number, number, number];
    /** Rendered without depth write / with blending. */
    transparent?: boolean;
    opacity?: number;
}

/**
 * One draw call's worth of work. `instances` is a packed array of column-major
 * mat4s produced directly from ECS columns — the measured cost of producing it
 * is 0.014 ms for 2,000 instances, which is why this is the right boundary.
 */
export interface DrawBatch {
    mesh: MeshId;
    material: MaterialId;
    count: number;
    /** count * 16 floats, column-major. May be a view over a larger buffer. */
    instances: Float32Array;
}

export interface CameraState {
    /** Column-major view and projection. */
    view: Float32Array;
    projection: Float32Array;
    position: [number, number, number];
}

export interface RenderBackend {
    createMesh(desc: MeshDesc): MeshId;
    createMaterial(desc: MaterialDesc): MaterialId;
    /** Called once per frame with everything visible. Bulk only. */
    submit(batches: readonly DrawBatch[], camera: CameraState): void;
    resize?(width: number, height: number): void;
    dispose?(): void;
}

/* ------------------------------------------------------------- conventions */

/**
 * Written down because they are invisible until they are wrong, and colour space
 * in particular fails silently — every colour shifts and nothing errors. Asserted
 * in the render tests so a backend swap cannot quietly change them.
 */
export const CONVENTIONS = {
    handedness: "right",
    up: "+Y",
    forward: "-Z",
    matrixLayout: "column-major",
    quaternionOrder: "xyzw",
    /** Working space is linear; sRGB encode happens once, at output. */
    workingColorSpace: "linear",
    outputColorSpace: "srgb",
    /** WebGPU clip space. A GL-era backend must convert rather than assume. */
    depthRange: "0..1",
} as const;
