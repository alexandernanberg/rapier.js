/**
 * Input as per-tick data rather than as events.
 *
 * The point is to make the simulation a pure function of state and input, so
 * that replay, rollback, networking and agent-driven tests are all the same
 * mechanism. Everything here is fixed-layout and lives in a ring buffer, so a
 * frame is 16 bytes whether it came from a keyboard, a packet, or a test.
 */

/** Digital actions — a bitfield, level state rather than edges. */
export const Actions = {
    moveForward: 1 << 0,
    moveBack: 1 << 1,
    moveLeft: 1 << 2,
    moveRight: 1 << 3,
    jump: 1 << 4,
    fire: 1 << 5,
} as const;

export type Action = (typeof Actions)[keyof typeof Actions];

/** Words per frame: tick, actions, packed aim, packed move. 16 bytes. */
const STRIDE = 4;
const W_TICK = 0;
const W_ACTIONS = 1;
const W_AIM = 2;
const W_MOVE = 3;

/** Quantize [-1,1] to a signed byte. Done at capture, so the sim and the wire
 *  always agree — simulating a raw float and sending a rounded one diverges. */
export function quantizeAxis(v: number): number {
    const c = v < -1 ? -1 : v > 1 ? 1 : v;
    return Math.round(c * 127);
}

export function dequantizeAxis(q: number): number {
    return q / 127;
}

/** Reused view over one latched frame. Never allocated per read. */
export class InputView {
    private buf!: Int32Array;
    private off = 0;
    private prevOff = 0;

    /** @internal */
    _point(buf: Int32Array, off: number, prevOff: number): this {
        this.buf = buf;
        this.off = off;
        this.prevOff = prevOff;
        return this;
    }

    get tick(): number {
        return this.buf[this.off + W_TICK];
    }

    held(a: number): boolean {
        return (this.buf[this.off + W_ACTIONS] & a) !== 0;
    }

    pressed(a: number): boolean {
        const cur = this.buf[this.off + W_ACTIONS];
        const prev = this.buf[this.prevOff + W_ACTIONS];
        return (cur & ~prev & a) !== 0;
    }

    released(a: number): boolean {
        const cur = this.buf[this.off + W_ACTIONS];
        const prev = this.buf[this.prevOff + W_ACTIONS];
        return (~cur & prev & a) !== 0;
    }

    /** Dequantized to [-1,1]. */
    get moveX(): number {
        return dequantizeAxis((this.buf[this.off + W_MOVE] << 24) >> 24);
    }

    get moveY(): number {
        return dequantizeAxis((this.buf[this.off + W_MOVE] << 16) >> 24);
    }

    get aimX(): number {
        return ((this.buf[this.off + W_AIM] << 16) >> 16) / 10430;
    }

    get aimY(): number {
        return (this.buf[this.off + W_AIM] >> 16) / 10430;
    }
}

/**
 * Staging buffer plus a ring of latched frames.
 *
 * Platform events accumulate into staging. Each fixed tick latches staging into
 * the ring and clears the sticky bits — so a key pressed and released between
 * two latches still registers for exactly one tick instead of vanishing.
 */
export class InputBuffer {
    readonly players: number;
    readonly history: number;
    private readonly ring: Int32Array;
    private readonly held: Int32Array;
    private readonly sticky: Int32Array;
    private readonly aim: Int32Array;
    private readonly move: Int32Array;
    private readonly view = new InputView();

    constructor(players = 1, history = 128) {
        this.players = players;
        this.history = history;
        this.ring = new Int32Array(players * history * STRIDE);
        this.held = new Int32Array(players);
        this.sticky = new Int32Array(players);
        this.aim = new Int32Array(players);
        this.move = new Int32Array(players);
    }

    /* ---- staging, driven by the platform layer or by a test ---- */

    press(player: number, a: number): void {
        this.held[player] |= a;
        this.sticky[player] |= a;
    }

    release(player: number, a: number): void {
        this.held[player] &= ~a;
    }

    /** Alias for `press`, for readability when the intent is a sustained hold. */
    hold(player: number, a: number): void {
        this.press(player, a);
    }

    /**
     * Down and up inside a single tick. The sticky bit makes it register for
     * exactly one tick and then clear, which is both what a fast human tap does
     * and what a test almost always means.
     *
     * Without this, `press` alone leaves the action held forever, so a second
     * `press` later produces no rising edge and the jump silently never fires.
     */
    tap(player: number, a: number): void {
        this.sticky[player] |= a;
        this.held[player] &= ~a;
    }

    setMove(player: number, x: number, y: number): void {
        this.move[player] = (quantizeAxis(x) & 0xff) | ((quantizeAxis(y) & 0xff) << 8);
    }

    setAim(player: number, x: number, y: number): void {
        const qx = Math.max(-32768, Math.min(32767, Math.round(x * 10430)));
        const qy = Math.max(-32768, Math.min(32767, Math.round(y * 10430)));
        this.aim[player] = (qx & 0xffff) | (qy << 16);
    }

    /* ---- latching ---- */

    private slot(player: number, tick: number): number {
        return (
            (player * this.history + (((tick % this.history) + this.history) % this.history)) *
            STRIDE
        );
    }

    /** Copy staging into the ring for `tick`, then clear sticky presses. */
    latch(tick: number): void {
        for (let p = 0; p < this.players; p++) {
            const o = this.slot(p, tick);
            this.ring[o + W_TICK] = tick;
            this.ring[o + W_ACTIONS] = this.held[p] | this.sticky[p];
            this.ring[o + W_AIM] = this.aim[p];
            this.ring[o + W_MOVE] = this.move[p];
            this.sticky[p] = 0;
        }
    }

    /** Overwrite a latched frame directly — how replay and remote input arrive. */
    inject(player: number, tick: number, words: ArrayLike<number>): void {
        const o = this.slot(player, tick);
        for (let w = 0; w < STRIDE; w++) this.ring[o + w] = words[w];
    }

    frame(player: number, tick: number): InputView {
        return this.view._point(this.ring, this.slot(player, tick), this.slot(player, tick - 1));
    }

    /** The 16 bytes that would go on the wire for this tick. */
    readFrame(player: number, tick: number, out: Int32Array): void {
        const o = this.slot(player, tick);
        for (let w = 0; w < STRIDE; w++) out[w] = this.ring[o + w];
    }

    static readonly STRIDE = STRIDE;
}
