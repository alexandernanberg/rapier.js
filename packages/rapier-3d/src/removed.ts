/**
 * @internal Builds the stand-in a removed entity's JS object points its raw set
 * at.
 *
 * Every accessor on a `RigidBody`, `Collider` or joint goes straight to a
 * binding on its raw set, and on the Rust side a handle that no longer exists
 * hits an `expect`. With `panic = "abort"` that is not an exception but a WASM
 * trap, and one that leaves the set's borrow flag stuck, after which the world
 * can neither be stepped nor freed. Swapping the raw set out for this proxy on
 * removal turns any later call into an ordinary JS error instead, at no cost to
 * the entities that are still alive.
 *
 * Symbol lookups are answered with `undefined` rather than an error: they come
 * from the runtime (console inspection, coercion, `then` probing), never from a
 * binding call.
 */
export function removedRef<T extends object>(message: string): T {
    return new Proxy(Object.create(null), {
        get(_target, prop) {
            if (typeof prop === "symbol") return undefined;
            throw new Error(message);
        },
        has() {
            return false;
        },
    }) as T;
}
