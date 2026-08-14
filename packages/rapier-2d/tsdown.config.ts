import {copyFileSync} from "node:fs";
import {defineConfig} from "tsdown";

const common = {
    format: ["esm"] as ["esm"],
    dts: true,
    sourcemap: true,
    loader: {
        ".wasm": "base64" as const,
    },
};

export default defineConfig([
    {
        ...common,
        entry: {rapier: "./src/rapier.ts"},
        clean: true,
        onSuccess: () => {
            copyFileSync("./wasm/release/rapier_wasm_2d_bg.wasm", "./dist/rapier_wasm_2d_bg.wasm");
        },
    },
    {
        ...common,
        entry: {compat: "./src/rapier-compat.ts"},
    },
]);
