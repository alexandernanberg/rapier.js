/**
 * Deferred structural changes.
 *
 * Spawning, despawning and adding/removing components while a query is being
 * iterated is the classic source of silent ECS bugs: the entity can be skipped
 * or visited twice depending on the library's internal storage. Rather than
 * memorise one library's rules, systems never mutate structure directly — they
 * push a command here, and the tick applies the whole buffer at one fixed
 * point.
 *
 * That makes the semantics ours rather than bitECS's, which also means this
 * layer survives an ECS swap unchanged.
 *
 * Commands are small objects rather than a packed array: structural changes are
 * rare compared to reads, so clarity wins until a profile says otherwise.
 */

export const CommandKind = {
    Spawn: 0,
    Despawn: 1,
    SetMoveTarget: 2,
    ClearMoveTarget: 3,
    Damage: 4,
} as const;

export type CommandKindValue = (typeof CommandKind)[keyof typeof CommandKind];

export interface Command {
    readonly kind: CommandKindValue;
    readonly a: number;
    readonly b: number;
    readonly c: number;
    readonly d: number;
}

export class CommandBuffer {
    private commands: Command[] = [];

    push(kind: CommandKindValue, a = 0, b = 0, c = 0, d = 0): void {
        this.commands.push({kind, a, b, c, d});
    }

    spawn(kind: number, player: number, x: number, y: number): void {
        this.push(CommandKind.Spawn, kind, player, x, y);
    }

    despawn(eid: number): void {
        this.push(CommandKind.Despawn, eid);
    }

    setMoveTarget(eid: number, x: number, y: number): void {
        this.push(CommandKind.SetMoveTarget, eid, x, y);
    }

    clearMoveTarget(eid: number): void {
        this.push(CommandKind.ClearMoveTarget, eid);
    }

    damage(eid: number, amount: number): void {
        this.push(CommandKind.Damage, eid, amount);
    }

    /**
     * Hands over the queued commands and clears the buffer. Commands queued
     * *during* a flush land in the next tick's buffer rather than extending the
     * current pass, which keeps a flush finite and its ordering obvious.
     */
    drain(): readonly Command[] {
        if (this.commands.length === 0) return EMPTY;
        const drained = this.commands;
        this.commands = [];
        return drained;
    }

    get size(): number {
        return this.commands.length;
    }
}

const EMPTY: readonly Command[] = [];
