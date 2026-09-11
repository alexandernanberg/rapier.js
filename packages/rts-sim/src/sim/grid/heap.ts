/**
 * Binary min-heap with a total ordering.
 *
 * Extracted because the ordering is the determinism-critical part of both A*
 * and the flow-field Dijkstra, and a subtly different comparator in each would
 * be a desync waiting to happen. Ranking by cost alone leaves ties to heap
 * layout — which depends on insertion history — so two clients pick different
 * but equally cheap routes. Every tie here falls through to `id`, which callers
 * pass as the tile index and is unique, making the pop order total.
 *
 * Keys are integers, so comparisons are exact. Backed by preallocated typed
 * arrays: a search allocates nothing.
 */
export class TieBrokenHeap {
    private readonly primary: Int32Array;
    private readonly secondary: Int32Array;
    private readonly ids: Int32Array;
    private size = 0;

    /**
     * `capacity` must allow for a node being queued more than once: neither
     * search decreases keys, they push again and skip the stale pop.
     */
    constructor(capacity: number) {
        this.primary = new Int32Array(capacity);
        this.secondary = new Int32Array(capacity);
        this.ids = new Int32Array(capacity);
    }

    get length(): number {
        return this.size;
    }

    clear(): void {
        this.size = 0;
    }

    /** `secondary` breaks ties before `id` does; pass 0 when unused. */
    push(id: number, primary: number, secondary: number): void {
        let i = this.size++;
        this.primary[i] = primary;
        this.secondary[i] = secondary;
        this.ids[i] = id;

        while (i > 0) {
            const parent = (i - 1) >> 1;
            if (!this.less(i, parent)) break;
            this.swap(i, parent);
            i = parent;
        }
    }

    pop(): number {
        const top = this.ids[0];
        const last = --this.size;

        if (last > 0) {
            this.move(last, 0);
            let i = 0;
            for (;;) {
                const left = 2 * i + 1;
                if (left >= last) break;
                const right = left + 1;
                const child = right < last && this.less(right, left) ? right : left;
                if (!this.less(child, i)) break;
                this.swap(i, child);
                i = child;
            }
        }

        return top;
    }

    private less(a: number, b: number): boolean {
        const {primary, secondary, ids} = this;
        if (primary[a] !== primary[b]) return primary[a] < primary[b];
        if (secondary[a] !== secondary[b]) return secondary[a] < secondary[b];
        return ids[a] < ids[b];
    }

    private swap(a: number, b: number): void {
        const {primary, secondary, ids} = this;
        const p = primary[a];
        const s = secondary[a];
        const id = ids[a];
        primary[a] = primary[b];
        secondary[a] = secondary[b];
        ids[a] = ids[b];
        primary[b] = p;
        secondary[b] = s;
        ids[b] = id;
    }

    private move(from: number, to: number): void {
        const {primary, secondary, ids} = this;
        primary[to] = primary[from];
        secondary[to] = secondary[from];
        ids[to] = ids[from];
    }
}
