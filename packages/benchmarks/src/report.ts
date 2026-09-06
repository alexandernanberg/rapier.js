import * as fs from "node:fs";
import {formatBytes} from "./memory.js";
import {formatPercent, formatTime, type BenchResult, type ResultsFile} from "./results.js";

/**
 * Turns results files into the markdown that CI posts on a pull request.
 *
 * Given a fork run and an official `@dimforge` run of the same dimension, it
 * reports both side by side with their ratio. The ratio is the number worth
 * watching: two runs on the same machine, back to back, cancel out how fast that
 * machine happened to be, which a timing compared against a run from last week
 * (or from a different runner) never does.
 *
 * Usage: pnpm bench:report [--out=<file>] <results.json>...
 */

/** Above this, a fork row is flagged as slower than upstream. */
const SLOWER_RATIO = 1.1;

function parseArgs(argv: string[]) {
    let out: string | null = null;
    const files: string[] = [];
    for (const arg of argv) {
        if (arg.startsWith("--out=")) out = arg.slice("--out=".length);
        else if (arg === "--help" || arg === "-h") {
            console.log("Usage: pnpm bench:report [--out=<file>] <results.json>...");
            process.exit(0);
        } else files.push(arg);
    }
    if (files.length === 0) {
        console.error("bench:report: no results files given");
        process.exit(1);
    }
    return {out, files};
}

function load(file: string): ResultsFile {
    const data = JSON.parse(fs.readFileSync(file, "utf-8")) as ResultsFile;
    if (data.version !== 2) {
        throw new Error(`${file}: results version ${data.version}, expected 2`);
    }
    return data;
}

function formatRatio(fork: number, upstream: number): string {
    if (upstream <= 0) return "—";
    const ratio = fork / upstream;
    const text = `${ratio.toFixed(2)}×`;
    return ratio > SLOWER_RATIO ? `${text} ⚠️` : text;
}

function timingTable(fork: ResultsFile, upstream: ResultsFile | null): string {
    const byName = new Map<string, BenchResult>();
    for (const r of upstream?.results ?? []) byName.set(r.name, r);

    let md = "";
    if (upstream) {
        md += "| Benchmark | Fork (p50) | ±IQR | Upstream (p50) | Ratio |\n";
        md += "|-----------|-----------:|-----:|---------------:|------:|\n";
        for (const r of fork.results) {
            const u = byName.get(r.name);
            md += `| ${r.name} | ${formatTime(r.p50)} | ${formatPercent(r.spread, false)} | `;
            md += u ? `${formatTime(u.p50)} | ${formatRatio(r.p50, u.p50)} |\n` : "— | — |\n";
        }
    } else {
        md += "| Benchmark | p50 | ±IQR | Min | Max |\n";
        md += "|-----------|----:|-----:|----:|----:|\n";
        for (const r of fork.results) {
            md += `| ${r.name} | ${formatTime(r.p50)} | ${formatPercent(r.spread, false)} | `;
            md += `${formatTime(r.min)} | ${formatTime(r.max)} |\n`;
        }
    }
    return md;
}

function memoryTable(fork: ResultsFile, upstream: ResultsFile | null): string {
    if (fork.memory.length === 0) return "";

    const byName = new Map<string, number>();
    for (const m of upstream?.memory ?? []) byName.set(m.name, m.bytesPerOp);

    let md = "\n#### Allocations\n\n";
    if (upstream) {
        md +=
            "| Benchmark | Fork bytes/op | Upstream bytes/op | GC / 1M ops | GC pause / 1M ops |\n";
        md +=
            "|-----------|--------------:|------------------:|------------:|------------------:|\n";
    } else {
        md += "| Benchmark | Bytes/op | GC / 1M ops | GC pause / 1M ops |\n";
        md += "|-----------|---------:|------------:|------------------:|\n";
    }
    for (const m of fork.memory) {
        md += `| ${m.name} | ${formatBytes(m.bytesPerOp)} | `;
        if (upstream) {
            const u = byName.get(m.name);
            md += `${u !== undefined ? formatBytes(u) : "—"} | `;
        }
        md += `${m.gcPerMillionOps.toFixed(0)} | ${m.gcPauseMsPerMillionOps.toFixed(1)}ms |\n`;
    }
    md +=
        "\nBytes/op repeats to within a few percent; the GC columns depend on nursery sizing and are indicative only.\n";
    return md;
}

function describePackage(run: ResultsFile): string {
    if (run.package === "fork") return `\`@alexandernanberg/rapier${run.dim}\``;
    const name = run.simd ? `rapier${run.dim}-simd-compat` : `rapier${run.dim}-compat`;
    return run.packageVersion
        ? `\`@dimforge/${name}@${run.packageVersion}\``
        : `\`@dimforge/${name}\``;
}

export function renderReport(runs: ResultsFile[]): string {
    const dims: Array<"2d" | "3d"> = ["3d", "2d"];
    let md = "## ⚡ Benchmark Results\n\n";

    const anyUpstream = runs.some((r) => r.package === "official");
    const anyQuick = runs.some((r) => r.quick);
    const notes: string[] = [];
    if (anyUpstream) {
        notes.push(
            "Ratio is fork ÷ upstream on the median, measured back to back on the same runner; below 1× is faster. Rows without an upstream number are fork-only APIs.",
        );
    }
    notes.push(
        "±IQR is the interquartile range of the fork's samples relative to its median; a difference inside it is noise.",
    );
    if (anyQuick) notes.push("Quick mode: reduced scene sizes, not comparable to a full run.");
    md += notes.join(" ") + "\n";

    let environment: ResultsFile | null = null;
    for (const dim of dims) {
        const fork = runs.find((r) => r.dim === dim && r.package === "fork");
        const upstream = runs.find((r) => r.dim === dim && r.package === "official") ?? null;
        if (!fork) continue;
        environment ??= fork;

        md += `\n### ${dim.toUpperCase()}\n\n`;
        if (upstream) md += `Upstream: ${describePackage(upstream)}\n\n`;
        md += timingTable(fork, upstream);
        md += memoryTable(fork, upstream);
    }

    if (environment) {
        md += "\n<details>\n<summary>Environment</summary>\n\n";
        md += `- **CPU:** ${environment.host.cpu} (${environment.host.cores} cores)\n`;
        md += `- **Node:** ${environment.host.node}\n`;
        md += `- **Platform:** ${environment.host.platform}/${environment.host.arch}\n`;
        md += `- **Timestamp:** ${environment.timestamp}\n`;
        md += "</details>\n";
    }

    return md;
}

const {out, files} = parseArgs(process.argv.slice(2));
const report = renderReport(files.map(load));
if (out) {
    fs.writeFileSync(out, report);
    console.log(`Report written to ${out}`);
} else {
    process.stdout.write(report);
}
