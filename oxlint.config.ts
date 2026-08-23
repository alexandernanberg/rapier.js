import {defineConfig} from "oxlint";
import config from "oxlint-config-alexandernanberg/oxlint/base";

export default defineConfig({
    extends: [config],

    rules: {
        // `rapier.ts` / `rapier-compat.ts` deliberately ship the whole module as a
        // namespace default export, so `RAPIER.World` is the documented way to reach
        // the API and is not a mistaken named import.
        "import/no-named-as-default-member": "off",

        // tsgolint's type model disagrees with the TypeScript this repo builds with:
        // it reports the `x as number as SomeEnum` casts that bridge a wasm-bindgen
        // enum to its TS twin, and a number of non-null assertions, as redundant.
        // Removing them the way its fixer does makes `pnpm typecheck` fail.
        "typescript/no-unnecessary-type-assertion": "off",

        // wasm-bindgen's generated `.d.ts` declares every raw handle non-nullable, so
        // the type checker thinks the guards around them are dead. They are not: `raw`
        // is `undefined` after `free()`, and raw getters return null on a miss.
        "typescript/no-unnecessary-condition": "off",
    },

    overrides: [
        {
            // The benchmark harness drives several engine builds (this repo's 2D and
            // 3D packages plus the upstream `@dimforge` ones) through one untyped
            // facade, so `any` is the point rather than an oversight.
            files: ["packages/benchmarks/**"],
            rules: {
                "typescript/no-explicit-any": "off",
                "typescript/no-unsafe-argument": "off",
                "typescript/no-unsafe-assignment": "off",
                "typescript/no-unsafe-call": "off",
                "typescript/no-unsafe-member-access": "off",
                "typescript/no-unsafe-return": "off",
            },
        },
    ],

    ignorePatterns: ["packages/*/wasm/**"],
});
