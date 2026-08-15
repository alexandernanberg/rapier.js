import {readFileSync, statSync} from "node:fs";
const p = process.argv[2];
const buf = readFileSync(p);
console.log(`  ${p.split("/").pop()}  ${(statSync(p).size / 1e6).toFixed(1)} MB`);
// compile only (the expensive part), repeated
for (let r = 0; r < 3; r++) {
    const t0 = process.hrtime.bigint();
    const mod = await WebAssembly.compile(buf);
    const t1 = process.hrtime.bigint();
    console.log(
        `    compile #${r + 1}: ${(Number(t1 - t0) / 1e6).toFixed(0)} ms  (${WebAssembly.Module.imports(mod).length} imports)`,
    );
}
