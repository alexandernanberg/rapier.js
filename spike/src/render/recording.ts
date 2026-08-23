/**
 * A backend that draws nothing and records everything.
 *
 * Three jobs, and only one of them is about swapping renderers:
 *
 *   1. It proves the seam is real. If anything starts holding render state
 *      outside the ECS, this backend's output stops matching and you find out
 *      immediately rather than when rewind quietly breaks.
 *   2. It makes rendering assertable in the headless harness — the one subsystem
 *      that otherwise needs a human looking at pixels.
 *   3. It is the null backend for server-side simulation.
 */

import type {
    CameraState,
    DrawBatch,
    MaterialDesc,
    MaterialId,
    MeshDesc,
    MeshId,
    RenderBackend,
} from "./types.ts";

export interface RecordedBatch {
    mesh: MeshId;
    material: MaterialId;
    count: number;
    /** Owned copy, so later frames cannot mutate a recorded one. */
    instances: Float32Array;
}

export interface RecordedFrame {
    index: number;
    batches: RecordedBatch[];
    camera: {position: [number, number, number]; view: Float32Array};
    get instanceCount(): number;
}

export class RecordingBackend implements RenderBackend {
    readonly meshes: MeshDesc[] = [];
    readonly materials: MaterialDesc[] = [];
    readonly frames: RecordedFrame[] = [];
    /** Cap history so a long run does not grow without bound. */
    keep = 16;

    createMesh(desc: MeshDesc): MeshId {
        this.meshes.push(desc);
        return this.meshes.length - 1;
    }

    createMaterial(desc: MaterialDesc): MaterialId {
        this.materials.push(desc);
        return this.materials.length - 1;
    }

    submit(batches: readonly DrawBatch[], camera: CameraState): void {
        const frame: RecordedFrame = {
            index: this.frames.length,
            batches: batches.map((b) => ({
                mesh: b.mesh,
                material: b.material,
                count: b.count,
                instances: b.instances.slice(),
            })),
            camera: {position: camera.position, view: camera.view.slice()},
            get instanceCount() {
                let n = 0;
                for (const b of this.batches) n += b.count;
                return n;
            },
        };
        this.frames.push(frame);
        if (this.frames.length > this.keep) this.frames.shift();
    }

    get lastFrame(): RecordedFrame {
        const f = this.frames[this.frames.length - 1];
        if (!f) throw new Error("no frame has been submitted yet");
        return f;
    }

    /** World-space translation of one instance, for assertions. */
    translationOf(frame: RecordedFrame, batch: number, instance: number): [number, number, number] {
        const b = frame.batches[batch];
        if (!b) throw new Error(`frame ${frame.index} has no batch ${batch}`);
        if (instance >= b.count) {
            throw new Error(`batch ${batch} has ${b.count} instances, asked for ${instance}`);
        }
        const o = instance * 16;
        return [b.instances[o + 12], b.instances[o + 13], b.instances[o + 14]];
    }

    reset(): void {
        this.frames.length = 0;
    }
}
