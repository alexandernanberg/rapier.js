import {RawIslandManager} from "../raw";
import {handleFromParts, WasmBuffer} from "../wasm_buffer";
import {RigidBodyHandle} from "./rigid_body";

/**
 * The island manager, which tracks which rigid-bodies are awake.
 *
 * To avoid leaking WASM resources, this MUST be freed manually with `islandManager.free()`
 * once you are done using it.
 */
export class IslandManager {
    raw: RawIslandManager;

    private _active = new WasmBuffer();

    /**
     * Release the WASM memory occupied by this island manager.
     */
    public free() {
        if (!!this.raw) {
            this.raw.free();
        }
        this.raw = undefined!;
        // The view points into the buffer that was just freed.
        this._active.release();
    }

    constructor(raw?: RawIslandManager) {
        this.raw = raw || new RawIslandManager();
    }

    /**
     * Applies the given closure to the handle of each active rigid-bodies contained by this set.
     *
     * A rigid-body is active if it is not sleeping, i.e., if it moved recently.
     *
     * @param f - The closure to apply.
     */
    public forEachActiveRigidBodyHandle(f: (handle: RigidBodyHandle) => void) {
        // One call publishes every active handle into WASM-side storage, and the
        // walk below stays in JS: iterating used to cost a boundary crossing per
        // active body, every frame.
        this._active.reset(this.raw.activeBodyHandles());

        // The callback runs between reads and may allocate in WASM (any
        // `intoRaw`, a query, a created entity), which can grow the memory and
        // detach the view: a detached view reads as empty, so holding one across
        // the loop would silently end the walk early. Re-attaching is a cheap
        // check when nothing happened.
        const len = this._active.length;
        for (let i = 0; i < len; i += 2) {
            const u32 = this._active.u32();
            f(handleFromParts(u32[i], u32[i + 1]));
        }
    }
}
