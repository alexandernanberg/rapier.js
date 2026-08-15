/**
 * Worker side of the executor. Sleeps on the control block, wakes per job,
 * processes its own contiguous slice, then signals completion.
 */

import {parentPort, workerData} from "node:worker_threads";
import {JOB, runJob, slice} from "./kernels.ts";

const {buffer, ctrlBuf, resultBuf, orderBuf, outBuf, offsets, length, index, threads} = workerData;

const ctrl = new Int32Array(ctrlBuf);
const results = new Float64Array(resultBuf);
const order = new Int32Array(orderBuf);
const out = new Int32Array(outBuf);
const cols: Float32Array[] = offsets.map((o: number) => new Float32Array(buffer, o, length));

const OUT_STRIDE = Math.ceil(length / threads) + 8;

parentPort!.postMessage("ready");

let seen = 0;
for (;;) {
    Atomics.wait(ctrl, 0, seen);
    const gen = Atomics.load(ctrl, 0);
    if (gen === seen) continue;
    seen = gen;

    const job = Atomics.load(ctrl, 1);
    if (job === JOB.shutdown) break;

    const {start, end} = slice(length, index, threads);
    results[index] = runJob(job, cols, start, end, out, index * OUT_STRIDE);

    // record completion order, so the bench can show what merging by
    // finish-time rather than by chunk index would produce
    const seat = Atomics.add(ctrl, 3, 1);
    order[seat] = index;

    Atomics.sub(ctrl, 2, 1);
    Atomics.notify(ctrl, 2);
}
