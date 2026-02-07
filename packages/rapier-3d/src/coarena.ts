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

export class Coarena<T> {
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
            if (this.data[i] != null) this.size -= 1;
            this.data[i] = null;
        }
    }

    public clear() {
        this.data = new Array<T | null>();
    }

    public get(handle: number): T | null {
        let i = handleToIndex(handle);
        if (i < this.data.length) {
            return this.data[i];
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
