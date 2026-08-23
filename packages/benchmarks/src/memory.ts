import {PerformanceObserver} from "node:perf_hooks";
import * as v8 from "node:v8";
import * as vm from "node:vm";

export interface MemoryResult {
    name: string;
    /** Bytes of JS heap allocated per operation. */
    bytesPerOp: number;
    /** Minor + major collections observed per million operations. */
    gcPerMillionOps: number;
    /** Total GC pause time in milliseconds per million operations. */
    gcPauseMsPerMillionOps: number;
    /** True if a collection ran during the allocation pass, making `bytesPerOp` a lower bound. */
    collected: boolean;
}

export interface MemoryBench {
    name: string;
    /** One call performs this many operations (e.g. 100 ray casts). */
    opsPerCall: number;
    /** Upper bound on calls per pass, for benchmarks that are slow per call. */
    maxCalls?: number;
    fn: () => void;
}

/**
 * `global.gc`, exposed at runtime so the suite doesn't need `--expose-gc` on the
 * command line. Returns null if the flag can't be set (then measurements fall
 * back to whatever the heap happens to look like, and are reported as such).
 */
function resolveGc(): (() => void) | null {
    const existing = (globalThis as {gc?: () => void}).gc;
    if (typeof existing === "function") return existing;

    try {
        v8.setFlagsFromString("--expose-gc");
        const fn = vm.runInNewContext("gc") as unknown;
        v8.setFlagsFromString("--no-expose-gc");
        return typeof fn === "function" ? (fn as () => void) : null;
    } catch {
        return null;
    }
}

const gc = resolveGc();

/** Settles the heap so a measurement starts from a known-quiet state. */
function collect() {
    if (!gc) return;
    // Two passes: the first promotes what survives, the second sweeps it.
    gc();
    gc();
}

interface GcTally {
    count: number;
    pauseMs: number;
}

/**
 * Starts counting GC events (and their pause time) until `stop()` is awaited.
 *
 * GC entries reach the observer on a timer turn rather than synchronously —
 * `takeRecords()` straight after a workload comes back empty — so `stop()` has to
 * yield before the tally is complete. Keeping that yield out of the caller's
 * measurement window matters: a collection during the yield would shrink the
 * heap reading that the window is there to take.
 */
function startGcObserver(): {stop: () => Promise<GcTally>} {
    const tally: GcTally = {count: 0, pauseMs: 0};
    const observer = new PerformanceObserver((list) => {
        for (const entry of list.getEntries()) {
            tally.count++;
            tally.pauseMs += entry.duration;
        }
    });

    observer.observe({entryTypes: ["gc"]});

    return {
        stop: async () => {
            await new Promise((resolve) => setTimeout(resolve, 1));
            observer.disconnect();
            return tally;
        },
    };
}

function runCalls(bench: MemoryBench, calls: number) {
    for (let i = 0; i < calls; i++) bench.fn();
}

/** Allocation per operation, measured over a window short enough to avoid a collection. */
async function measureBytesPerOp(bench: MemoryBench): Promise<{
    bytesPerOp: number;
    collected: boolean;
}> {
    // `heapUsed` only reports what is still on the heap, so the window has to be
    // small enough that no collection recycles part of it — but large enough to
    // clear the few kb of sampling noise. Grow it until the delta is legible.
    const noiseFloorBytes = 64 * 1024;
    const targetBytes = 2 * 1024 * 1024;
    const maxCalls = bench.maxCalls ?? 100_000;

    let calls = 1;
    let delta = 0;
    for (let i = 0; i < 20; i++) {
        collect();
        const before = process.memoryUsage().heapUsed;
        runCalls(bench, calls);
        delta = process.memoryUsage().heapUsed - before;

        if (delta >= noiseFloorBytes || calls >= maxCalls) break;
        calls = Math.min(maxCalls, calls * 2);
    }

    // Scale back to roughly `targetBytes` so a collection can't cut the window short.
    if (delta > targetBytes) {
        calls = Math.max(1, Math.round((calls * targetBytes) / delta));
    }

    // Whatever else runs on this heap (timers, the observer, the harness itself)
    // only ever adds to the delta, while a collection inside the window only ever
    // subtracts from it. So: discard the windows where a collection ran, and take
    // the smallest of what's left as the closest estimate of the true rate.
    const clean: number[] = [];
    const all: number[] = [];

    for (let i = 0; i < 7; i++) {
        collect();
        const observer = startGcObserver();
        const before = process.memoryUsage().heapUsed;
        runCalls(bench, calls);
        const windowDelta = process.memoryUsage().heapUsed - before;
        // Sequential on purpose: each window has to be measured on its own, with
        // the observer flushed before the next one starts.
        // eslint-disable-next-line no-await-in-loop
        const tally = await observer.stop();
        const perOp = Math.max(0, windowDelta) / (calls * bench.opsPerCall);

        all.push(perOp);
        if (tally.count === 0) clean.push(perOp);
    }

    return {
        bytesPerOp: Math.min(...(clean.length > 0 ? clean : all)),
        collected: clean.length === 0,
    };
}

/** GC frequency and pause time over a run long enough to force collections. */
async function measureGcPressure(
    bench: MemoryBench,
    bytesPerOp: number,
    quick: boolean,
): Promise<GcTally & {ops: number}> {
    // Enough calls to churn several nursery fills, or a flat count when the
    // benchmark allocates nothing and no amount of churn would trigger a GC.
    const budgetBytes = (quick ? 32 : 128) * 1024 * 1024;
    const perCall = bytesPerOp * bench.opsPerCall;
    const cap = Math.min(bench.maxCalls ?? Infinity, quick ? 20_000 : 100_000);
    const calls =
        perCall > 1
            ? Math.min(cap, Math.max(1, Math.round(budgetBytes / perCall)))
            : Math.min(cap, quick ? 2_000 : 10_000);

    collect();
    const observer = startGcObserver();
    runCalls(bench, calls);
    const tally = await observer.stop();

    return {...tally, ops: calls * bench.opsPerCall};
}

export async function measureMemory(
    benches: MemoryBench[],
    quick: boolean,
): Promise<MemoryResult[]> {
    const results: MemoryResult[] = [];

    for (const bench of benches) {
        // Warm up so JIT compilation and hidden-class churn land outside the window.
        runCalls(bench, 16);

        // Sequential on purpose: overlapping runs would share a heap and the
        // allocation rates would measure each other.
        // eslint-disable-next-line no-await-in-loop
        const {bytesPerOp, collected} = await measureBytesPerOp(bench);
        // eslint-disable-next-line no-await-in-loop
        const pressure = await measureGcPressure(bench, bytesPerOp, quick);
        const perMillion = 1e6 / pressure.ops;

        results.push({
            name: bench.name,
            bytesPerOp,
            gcPerMillionOps: pressure.count * perMillion,
            gcPauseMsPerMillionOps: pressure.pauseMs * perMillion,
            collected,
        });
    }

    return results;
}

export function gcAvailable(): boolean {
    return gc !== null;
}

export function formatBytes(bytes: number): string {
    if (bytes < 0.5) return "0 b";
    if (bytes < 1024) return `${bytes.toFixed(bytes < 10 ? 1 : 0)} b`;
    return `${(bytes / 1024).toFixed(1)} kb`;
}

export function printMemoryTable(results: MemoryResult[]): void {
    console.log("\nAllocations (JS heap; lower is better):");
    console.log("┌─────────────────────────────────┬──────────┬──────────┬────────────┐");
    console.log("│ Benchmark                       │ Bytes/op │ GC / 1M  │ Pause / 1M │");
    console.log("├─────────────────────────────────┼──────────┼──────────┼────────────┤");
    for (const r of results) {
        const name = r.name.slice(0, 31).padEnd(31);
        const bytes = `${formatBytes(r.bytesPerOp)}${r.collected ? "+" : ""}`.padStart(8);
        const gcs = r.gcPerMillionOps.toFixed(0).padStart(8);
        const pause = `${r.gcPauseMsPerMillionOps.toFixed(1)}ms`.padStart(10);
        console.log(`│ ${name} │ ${bytes} │ ${gcs} │ ${pause} │`);
    }
    console.log("└─────────────────────────────────┴──────────┴──────────┴────────────┘");
    if (results.some((r) => r.collected)) {
        console.log("  + a collection ran mid-measurement, so that figure is a lower bound");
    }
}
