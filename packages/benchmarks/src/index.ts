import {run} from "mitata";
import * as fs from "node:fs";
import {createRequire} from "node:module";
import * as path from "node:path";
import {
    loadBaseline,
    saveBaseline,
    compareToBaseline,
    compareMemoryToBaseline,
    hasRegression,
    hostMismatch,
    summarizeComparison,
    printComparisonTable,
    printMemoryComparisonTable,
} from "./baseline.js";
import {gcAvailable, measureMemory, printMemoryTable, type MemoryResult} from "./memory.js";
import {describeHost, type BenchResult, type ResultsFile} from "./results.js";
import {allocationBenches} from "./scenarios/allocations.js";
import {benchGetters} from "./scenarios/getters.js";
import {benchLifecycle} from "./scenarios/lifecycle.js";
import {benchQueries} from "./scenarios/queries.js";
import {benchSetters} from "./scenarios/setters.js";
import {benchSimulation} from "./scenarios/simulation.js";

function packageName(dim: "2d" | "3d", simd: boolean, official: boolean): string {
    if (official) return `@dimforge/rapier${dim}${simd ? "-simd" : ""}-compat`;
    return `@alexandernanberg/rapier${dim}`;
}

/** The installed version of `name`, found by walking up from its entry point. */
function packageVersion(name: string): string | undefined {
    try {
        const require = createRequire(import.meta.url);
        let dir = path.dirname(require.resolve(name));
        while (true) {
            const candidate = path.join(dir, "package.json");
            if (fs.existsSync(candidate)) {
                const pkg = JSON.parse(fs.readFileSync(candidate, "utf-8"));
                if (pkg.name === name) return pkg.version;
            }
            const parent = path.dirname(dir);
            if (parent === dir) return undefined;
            dir = parent;
        }
    } catch {
        return undefined;
    }
}

async function importRapier(dim: "2d" | "3d", simd: boolean, official: boolean) {
    if (official) {
        if (dim === "2d") {
            return simd
                ? await import("@dimforge/rapier2d-simd-compat")
                : await import("@dimforge/rapier2d-compat");
        } else {
            return simd
                ? await import("@dimforge/rapier3d-simd-compat")
                : await import("@dimforge/rapier3d-compat");
        }
    }
    // Our fork — always SIMD, so `--simd` only selects the official variant above.
    if (dim === "2d") {
        return await import("@alexandernanberg/rapier2d/compat");
    } else {
        return await import("@alexandernanberg/rapier3d/compat");
    }
}

const args = process.argv.slice(2);

function parseArgs() {
    let dim: "2d" | "3d" = "3d";
    let quick = false;
    let saveBaselineFlag = false;
    let noCompare = false;
    let simd = false;
    let official = false;
    let noMemory = false;
    let out: string | null = null;

    for (const arg of args) {
        if (arg === "--dim=2d") dim = "2d";
        else if (arg === "--dim=3d") dim = "3d";
        else if (arg === "--quick") quick = true;
        else if (arg === "--save-baseline") saveBaselineFlag = true;
        else if (arg === "--no-compare") noCompare = true;
        else if (arg === "--simd") simd = true;
        else if (arg === "--official") official = true;
        else if (arg === "--no-memory") noMemory = true;
        else if (arg.startsWith("--out=")) out = arg.slice("--out=".length);
        else if (arg === "--help" || arg === "-h") {
            console.log(`
Rapier.js Benchmark Suite

Usage: pnpm bench [options]

Options:
  --dim=2d          Run 2D benchmarks
  --dim=3d          Run 3D benchmarks (default)
  --simd            Use the SIMD build of the official packages (--official only;
                    this fork is always SIMD)
  --official        Use official @dimforge/rapier packages instead of fork
  --quick           Run with fewer bodies (faster setup, same measurement precision)
  --no-memory       Skip the allocation/GC measurements
  --out=<file>      Write the results JSON here instead of results/<dim>-<pkg>-<time>.json
  --save-baseline   Save current results as this machine's baseline (baseline.json,
                    local only — timings don't transfer between machines)
  --no-compare      Run without baseline comparison
  --help, -h        Show this help message

Examples:
  pnpm bench --save-baseline    # On master: record this machine's baseline
  pnpm bench                    # On a branch: compare against it
  pnpm bench --official --simd  # Run official @dimforge SIMD build
  pnpm bench --official         # Run with official @dimforge packages
  pnpm bench --no-compare       # Run without baseline comparison
  pnpm bench:2d                 # Full 2D benchmark
  pnpm bench --quick            # Quick 3D benchmark
  pnpm bench:report a.json b.json  # Fork-vs-upstream markdown from two results files
`);
            process.exit(0);
        }
    }

    return {dim, quick, saveBaselineFlag, noCompare, simd, official, noMemory, out};
}

async function main() {
    const {dim, quick, saveBaselineFlag, noCompare, simd, official, noMemory, out} = parseArgs();
    const is3D = dim === "3d";

    const modifiers = [
        quick ? "quick mode" : null,
        simd && official ? "SIMD" : null,
        official ? "official @dimforge" : null,
    ].filter(Boolean);
    const modifierStr = modifiers.length > 0 ? ` (${modifiers.join(", ")})` : "";

    console.log(`\nRapier ${dim.toUpperCase()} Benchmarks${modifierStr}\n`);

    // Import the appropriate package
    const RAPIER = await importRapier(dim, simd, official);

    await RAPIER.init();

    // Register all benchmarks (mitata collects them globally)
    benchSimulation(RAPIER, is3D, quick);
    benchLifecycle(RAPIER, is3D, quick);
    benchQueries(RAPIER, is3D, quick);
    benchGetters(RAPIER, is3D, quick);
    benchSetters(RAPIER, is3D, quick);

    // Run all registered benchmarks
    const {benchmarks} = await run();

    // Allocation and GC pressure, measured separately: these need a quiet heap
    // and forced collections, which the timing harness deliberately avoids.
    let memory: MemoryResult[] = [];
    if (!noMemory) {
        if (!gcAvailable()) {
            console.log(
                "\nSkipping allocation measurements: could not expose `gc` " +
                    "(run node with --expose-gc).",
            );
        } else {
            memory = await measureMemory(allocationBenches(RAPIER, is3D, quick), quick);
            printMemoryTable(memory);
        }
    }

    // Timings are compared on the median: the mean drags along GC pauses and
    // scheduler hiccups that have nothing to do with the code under test, and
    // the interquartile spread says how much a difference has to be to mean anything.
    const results: BenchResult[] = benchmarks.flatMap((trial) =>
        trial.runs
            .filter((r) => r.stats != null)
            .map((r) => {
                const stats = r.stats!;
                return {
                    name: r.name,
                    mean: stats.avg / 1e6,
                    p50: stats.p50 / 1e6,
                    min: stats.min / 1e6,
                    max: stats.max / 1e6,
                    p25: stats.p25 / 1e6,
                    p75: stats.p75 / 1e6,
                    p99: stats.p99 / 1e6,
                    spread: stats.p50 > 0 ? (stats.p75 - stats.p25) / stats.p50 : 0,
                    samples: stats.samples.length,
                };
            }),
    );

    const resultsFile: ResultsFile = {
        version: 2,
        timestamp: new Date().toISOString(),
        dim,
        package: official ? "official" : "fork",
        packageVersion: packageVersion(packageName(dim, simd, official)),
        simd: official ? simd : true,
        quick,
        host: describeHost(),
        results,
        memory: memory.map((m) => ({
            name: m.name,
            bytesPerOp: m.bytesPerOp,
            gcPerMillionOps: m.gcPerMillionOps,
            gcPauseMsPerMillionOps: m.gcPauseMsPerMillionOps,
        })),
    };

    let outPath: string;
    if (out) {
        outPath = path.resolve(out);
    } else {
        const resultsDir = path.join(new URL("..", import.meta.url).pathname, "results");
        outPath = path.join(resultsDir, `${dim}-${resultsFile.package}-${Date.now()}.json`);
    }
    fs.mkdirSync(path.dirname(outPath), {recursive: true});
    fs.writeFileSync(outPath, JSON.stringify(resultsFile, null, 2));
    console.log(`\nResults saved to ${outPath}`);

    // Handle baseline operations
    if (saveBaselineFlag) {
        saveBaseline(dim, results, memory, resultsFile.host);
    } else if (!noCompare) {
        const baseline = loadBaseline();

        if (baseline && Object.keys(baseline[dim]).length > 0) {
            const mismatch = hostMismatch(baseline, resultsFile.host);
            if (mismatch) {
                console.log(
                    `\n\u26a0\ufe0f  Baseline was recorded on a different machine (${mismatch}); the timing comparison below is not meaningful. Re-record with --save-baseline.`,
                );
            }
            const comparisons = compareToBaseline(dim, results, baseline);
            printComparisonTable(comparisons);

            if (memory.length > 0) {
                const memoryComparisons = compareMemoryToBaseline(dim, memory, baseline);
                printMemoryComparisonTable(memoryComparisons);
                comparisons.push(...memoryComparisons);
            }

            const summaryResult = summarizeComparison(comparisons);
            const parts: string[] = [];

            if (summaryResult.newBenchmarks > 0) {
                parts.push(`${summaryResult.newBenchmarks} new`);
            }
            if (summaryResult.warnings > 0) {
                parts.push(
                    `\u26a0\ufe0f ${summaryResult.warnings} warning${summaryResult.warnings > 1 ? "s" : ""}`,
                );
            }
            if (summaryResult.regressions > 0) {
                parts.push(
                    `\u274c ${summaryResult.regressions} regression${summaryResult.regressions > 1 ? "s" : ""}`,
                );
            }

            if (parts.length > 0) {
                console.log(`\n${parts.join(", ")}`);
            } else {
                console.log("\n\u2713 All benchmarks within tolerance");
            }

            if (hasRegression(comparisons)) {
                process.exit(1);
            }
        } else {
            console.log(
                "\nNo baseline for this machine. Run `pnpm bench --save-baseline` on master to record one.",
            );
        }
    }
}

main().catch((err) => {
    console.error("Benchmark failed:", err);
    process.exit(1);
});
