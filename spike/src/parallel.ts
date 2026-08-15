/**
 * Main-thread side of the parallel executor.
 *
 * The main thread deliberately does not `Atomics.wait` — a browser main thread
 * is forbidden from blocking that way — so it processes its own chunk and then
 * spins on the countdown. Spinning is only defensible because the waits here are
 * sub-millisecond; anything longer needs `waitAsync` and a different frame shape.
 */

import {Worker} from "node:worker_threads";
import {JOB, runJob, slice} from "./kernels.ts";

/* control block:
 *   [0] generation   — bumped to release the workers
 *   [1] job id
 *   [2] countdown    — remaining worker chunks
 *   [3] seat counter — hands out completion-order seats
 */
const GEN = 0,
    JOBID = 1,
    COUNT = 2,
    SEAT = 3;

export class Pool {
    readonly threads: number;
    private readonly workers: Worker[] = [];
    private readonly ctrl: Int32Array;
    readonly results: Float64Array;
    readonly order: Int32Array;
    readonly out: Int32Array;
    private readonly cols: Float32Array[];
    private readonly length: number;
    readonly outStride: number;

    private constructor(
        threads: number,
        ctrl: Int32Array,
        results: Float64Array,
        order: Int32Array,
        out: Int32Array,
        cols: Float32Array[],
        length: number,
    ) {
        this.threads = threads;
        this.ctrl = ctrl;
        this.results = results;
        this.order = order;
        this.out = out;
        this.cols = cols;
        this.length = length;
        this.outStride = Math.ceil(length / threads) + 8;
    }

    static async create(
        buffer: SharedArrayBuffer,
        offsets: number[],
        length: number,
        threads: number,
    ): Promise<Pool> {
        const ctrlBuf = new SharedArrayBuffer(16);
        const resultBuf = new SharedArrayBuffer(8 * threads);
        const orderBuf = new SharedArrayBuffer(4 * threads);
        const outStride = Math.ceil(length / threads) + 8;
        const outBuf = new SharedArrayBuffer(4 * outStride * threads);

        const pool = new Pool(
            threads,
            new Int32Array(ctrlBuf),
            new Float64Array(resultBuf),
            new Int32Array(orderBuf),
            new Int32Array(outBuf),
            offsets.map((o) => new Float32Array(buffer, o, length)),
            length,
        );

        // worker 0 is the main thread; spawn threads-1 real workers
        const ready: Promise<void>[] = [];
        for (let i = 1; i < threads; i++) {
            const w = new Worker(new URL("./parallel-worker.ts", import.meta.url), {
                workerData: {
                    buffer,
                    ctrlBuf,
                    resultBuf,
                    orderBuf,
                    outBuf,
                    offsets,
                    length,
                    index: i,
                    threads,
                },
            });
            pool.workers.push(w);
            ready.push(new Promise<void>((res) => w.once("message", () => res())));
        }
        await Promise.all(ready);
        return pool;
    }

    /** Run one job across all chunks. Returns when every chunk is done. */
    run(job: number): void {
        const ctrl = this.ctrl;
        const workers = this.threads - 1;
        Atomics.store(ctrl, JOBID, job);
        Atomics.store(ctrl, COUNT, workers);
        Atomics.store(ctrl, SEAT, 0);
        Atomics.add(ctrl, GEN, 1);
        Atomics.notify(ctrl, GEN);

        // main thread takes chunk 0
        const {start, end} = slice(this.length, 0, this.threads);
        this.results[0] = runJob(job, this.cols, start, end, this.out, 0);
        const seat = Atomics.add(ctrl, SEAT, 1);
        this.order[seat] = 0;

        while (Atomics.load(ctrl, COUNT) > 0) {
            /* spin — sub-millisecond by construction */
        }
    }

    async destroy(): Promise<void> {
        Atomics.store(this.ctrl, JOBID, JOB.shutdown);
        Atomics.add(this.ctrl, GEN, 1);
        Atomics.notify(this.ctrl, GEN);
        await Promise.all(this.workers.map((w) => w.terminate()));
    }
}

/** Run the same job serially, on this thread, in chunk-index order. */
export function runSerial(
    cols: Float32Array[],
    length: number,
    chunks: number,
    job: number,
    results: Float64Array,
    out: Int32Array,
    outStride: number,
): void {
    for (let i = 0; i < chunks; i++) {
        const {start, end} = slice(length, i, chunks);
        results[i] = runJob(job, cols, start, end, out, i * outStride);
    }
}
