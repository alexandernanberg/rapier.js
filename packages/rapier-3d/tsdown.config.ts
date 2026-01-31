import {copyFileSync} from "node:fs";
import {resolve} from "node:path";
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
            copyFileSync("./wasm/release/rapier_wasm_3d_bg.wasm", "./dist/rapier_wasm_3d_bg.wasm");
        },
    },
    {
        ...common,
        entry: {compat: "./src/rapier-compat.ts"},
    },
    {
        ...common,
        entry: {simd: "./src/rapier-simd.ts"},
        alias: {
            [resolve("./src/raw")]: resolve("./src/raw-simd"),
        },
        onSuccess: () => {
            copyFileSync(
                "./wasm/release-simd/rapier_wasm_3d_bg.wasm",
                "./dist/rapier_wasm_3d_simd_bg.wasm",
            );
        },
    },
    {
        ...common,
        entry: {"compat-simd": "./src/rapier-compat-simd.ts"},
        alias: {
            [resolve("./src/raw")]: resolve("./src/raw-simd"),
        },
    },
]);
