const _fconv = new Float64Array(1);
const _uconv = new Uint32Array(_fconv.buffer);

/**
 * Extracts the arena index (lower 32 bits) from a handle (f64).
 * Handles are f64-encoded arena indices where lower 32 bits = index, upper 32 = generation.
 */
export function handleToIndex(handle: number): number {
    _fconv[0] = handle;
    return _uconv[0];
}

/**
 * Maps handles to the JS objects wrapping the entities behind them.
 *
 * Rapier recycles the arena index of a removed entity for the next one it
 * inserts (with a bumped generation), so the index alone does not identify an
 * entity: a handle kept past its entity's removal would otherwise resolve to
 * whatever took the slot over. Every lookup therefore also compares the stored
 * object's own handle, which carries the generation, against the one asked for.
 */
export class Coarena<T extends {readonly handle: number}> {
    data: Array<T | null>;
    size: number;

    public constructor() {
        this.data = new Array<T | null>();
        this.size = 0;
    }

    public set(handle: number, data: T) {
        let i = handleToIndex(handle);
        while (this.data.length <= i) {
            this.data.push(null);
        }

        if (this.data[i] == null) this.size += 1;
        this.data[i] = data;
    }

    public len(): number {
        return this.size;
    }

    public delete(handle: number) {
        let i = handleToIndex(handle);
        if (i < this.data.length) {
            const elt = this.data[i];
            // A stale handle must not evict the entity that recycled its index.
            if (elt != null && elt.handle === handle) {
                this.size -= 1;
                this.data[i] = null;
            }
        }
    }

    public clear() {
        this.data = new Array<T | null>();
        this.size = 0;
    }

    public get(handle: number): T | null {
        // `undefined`/`NaN` would otherwise reinterpret as index 0 and return an
        // unrelated entity; a missing handle (an `Option` that came back empty
        // from WASM) has to read as "no entity".
        if (handle !== handle || handle === undefined) return null;
        let i = handleToIndex(handle);
        if (i < this.data.length) {
            const elt = this.data[i];
            // Same index, different generation: the entity behind `handle` is gone
            // and another one recycled its slot.
            return elt != null && elt.handle === handle ? elt : null;
        } else {
            return null;
        }
    }

    public forEach(f: (elt: T) => void) {
        for (const elt of this.data) {
            if (elt != null) f(elt);
        }
    }

    public getAll(): Array<T> {
        return this.data.filter((elt) => elt != null);
    }
}
