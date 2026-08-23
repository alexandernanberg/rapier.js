/**
 * ECS columns -> packed instance matrices, grouped into draw batches.
 *
 * This is the only per-entity CPU work any backend requires, and it belongs to
 * the engine rather than to a backend. Buffers are pooled per batch key and
 * reused across frames, so a steady-state frame allocates nothing.
 */

import type {ComponentDef} from "../schema.ts";
import type {World} from "../world.ts";
import type {CameraState, DrawBatch} from "./types.ts";
import {Renderable, Transform} from "../components.ts";
import {Query} from "../query.ts";

interface Bucket {
    mesh: number;
    material: number;
    count: number;
    instances: Float32Array;
}

export class Extractor {
    private readonly query: Query;
    private readonly buckets = new Map<number, Bucket>();
    private readonly out: DrawBatch[] = [];

    constructor(world: World, opts: {without?: ComponentDef[]} = {}) {
        this.query = new Query(world, [Transform, Renderable], opts.without ?? []);
    }

    refresh(): void {
        this.query.refresh();
    }

    /** Rebuild this frame's batches. Returns a stable array, valid until the next call. */
    extract(): readonly DrawBatch[] {
        for (const b of this.buckets.values()) b.count = 0;

        const q = this.query;
        for (let a = 0; a < q.archetypes.length; a++) {
            const arch = q.archetypes[a];
            const c = q.bindings[a];
            // binding order follows Transform's field order, then Renderable's
            const tx = c[0] as Float32Array,
                ty = c[1] as Float32Array,
                tz = c[2] as Float32Array,
                rx = c[3] as Float32Array,
                ry = c[4] as Float32Array,
                rz = c[5] as Float32Array,
                rw = c[6] as Float32Array,
                sx = c[7] as Float32Array,
                sy = c[8] as Float32Array,
                sz = c[9] as Float32Array,
                mesh = c[10] as Uint32Array,
                mat = c[11] as Uint32Array;

            const n = arch.count;
            // Cache the last bucket so runs of same-material entities skip the
            // Map lookup. Measured: it does nothing when materials interleave
            // (the lookup was never the bottleneck) but pays when they group.
            let lastKey = -1;
            let bucket: Bucket | undefined;
            for (let i = 0; i < n; i++) {
                const key = (mesh[i] << 16) | mat[i];
                if (key !== lastKey) {
                    lastKey = key;
                    bucket = this.buckets.get(key);
                    if (bucket === undefined) {
                        bucket = {
                            mesh: mesh[i],
                            material: mat[i],
                            count: 0,
                            instances: new Float32Array(256 * 16),
                        };
                        this.buckets.set(key, bucket);
                    }
                }
                bucket = bucket!;
                if ((bucket.count + 1) * 16 > bucket.instances.length) {
                    const grown = new Float32Array(bucket.instances.length * 2);
                    grown.set(bucket.instances);
                    bucket.instances = grown;
                }

                // compose TRS into a column-major mat4, straight into the batch
                const x = rx[i],
                    y = ry[i],
                    z = rz[i],
                    w = rw[i];
                const x2 = x + x,
                    y2 = y + y,
                    z2 = z + z;
                const xx = x * x2,
                    xy = x * y2,
                    xz = x * z2;
                const yy = y * y2,
                    yz = y * z2,
                    zz = z * z2;
                const wx = w * x2,
                    wy = w * y2,
                    wz = w * z2;
                const a0 = sx[i],
                    a1 = sy[i],
                    a2 = sz[i];

                const m = bucket.instances;
                const o = bucket.count * 16;
                m[o] = (1 - (yy + zz)) * a0;
                m[o + 1] = (xy + wz) * a0;
                m[o + 2] = (xz - wy) * a0;
                m[o + 3] = 0;
                m[o + 4] = (xy - wz) * a1;
                m[o + 5] = (1 - (xx + zz)) * a1;
                m[o + 6] = (yz + wx) * a1;
                m[o + 7] = 0;
                m[o + 8] = (xz + wy) * a2;
                m[o + 9] = (yz - wx) * a2;
                m[o + 10] = (1 - (xx + yy)) * a2;
                m[o + 11] = 0;
                m[o + 12] = tx[i];
                m[o + 13] = ty[i];
                m[o + 14] = tz[i];
                m[o + 15] = 1;
                bucket.count++;
            }
        }

        this.out.length = 0;
        for (const b of this.buckets.values()) {
            if (b.count === 0) continue;
            this.out.push({
                mesh: b.mesh,
                material: b.material,
                count: b.count,
                instances: b.instances.subarray(0, b.count * 16),
            });
        }
        // stable order, so frames are comparable between runs
        this.out.sort((p, r) => p.mesh - r.mesh || p.material - r.material);
        return this.out;
    }
}

/* ------------------------------------------------------------------ camera */

/** Right-handed look-at, column-major, matching CONVENTIONS. */
export function lookAt(
    eye: [number, number, number],
    target: [number, number, number],
    up: [number, number, number] = [0, 1, 0],
    out = new Float32Array(16),
): Float32Array {
    let zx = eye[0] - target[0],
        zy = eye[1] - target[1],
        zz = eye[2] - target[2];
    let l = Math.hypot(zx, zy, zz) || 1;
    zx /= l;
    zy /= l;
    zz /= l;
    let xx = up[1] * zz - up[2] * zy,
        xy = up[2] * zx - up[0] * zz,
        xz = up[0] * zy - up[1] * zx;
    l = Math.hypot(xx, xy, xz) || 1;
    xx /= l;
    xy /= l;
    xz /= l;
    const yx = zy * xz - zz * xy,
        yy = zz * xx - zx * xz,
        yz = zx * xy - zy * xx;

    out[0] = xx;
    out[1] = yx;
    out[2] = zx;
    out[3] = 0;
    out[4] = xy;
    out[5] = yy;
    out[6] = zy;
    out[7] = 0;
    out[8] = xz;
    out[9] = yz;
    out[10] = zz;
    out[11] = 0;
    out[12] = -(xx * eye[0] + xy * eye[1] + xz * eye[2]);
    out[13] = -(yx * eye[0] + yy * eye[1] + yz * eye[2]);
    out[14] = -(zx * eye[0] + zy * eye[1] + zz * eye[2]);
    out[15] = 1;
    return out;
}

/** Perspective with a 0..1 depth range, per CONVENTIONS. */
export function perspective(
    fovY: number,
    aspect: number,
    near: number,
    far: number,
    out = new Float32Array(16),
): Float32Array {
    const f = 1 / Math.tan(fovY / 2);
    out.fill(0);
    out[0] = f / aspect;
    out[5] = f;
    out[10] = far / (near - far);
    out[11] = -1;
    out[14] = (far * near) / (near - far);
    return out;
}

export function makeCamera(
    eye: [number, number, number],
    target: [number, number, number],
    aspect = 16 / 9,
): CameraState {
    return {
        view: lookAt(eye, target),
        projection: perspective(Math.PI / 3, aspect, 0.1, 1000),
        position: eye,
    };
}
