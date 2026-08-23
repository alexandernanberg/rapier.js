import type {
    IntegrationParameters,
    IslandManager,
    ImpulseJointSet,
    MultibodyJointSet,
    RigidBodySet,
} from "../dynamics";
import type {BroadPhase, ColliderSet, NarrowPhase} from "../geometry";
import type {Vector} from "../math";
import {VectorOps} from "../math";
import {RawSerializationPipeline} from "../raw";
// `World` builds a `SerializationPipeline` and this deserializes back into a
// `World`. Both references are resolved at call time, never while the modules
// evaluate, so the cycle is inert.
// eslint-disable-next-line import/no-cycle
import {World} from "./world";

/**
 * A pipeline for serializing the physics scene.
 *
 * To avoid leaking WASM resources, this MUST be freed manually with `serializationPipeline.free()`
 * once you are done using it (and all the rigid-bodies it created).
 */
export class SerializationPipeline {
    raw: RawSerializationPipeline;

    /**
     * Release the WASM memory occupied by this serialization pipeline.
     */
    free() {
        if (this.raw) {
            this.raw.free();
        }
        this.raw = undefined!;
    }

    constructor(raw?: RawSerializationPipeline) {
        this.raw = raw || new RawSerializationPipeline();
    }

    /**
     * Serialize a complete physics state into a single byte array.
     * @param gravity - The current gravity affecting the simulation.
     * @param integrationParameters - The integration parameters of the simulation.
     * @param broadPhase - The broad-phase of the simulation.
     * @param narrowPhase - The narrow-phase of the simulation.
     * @param bodies - The rigid-bodies taking part into the simulation.
     * @param colliders - The colliders taking part into the simulation.
     * @param impulseJoints - The impulse joints taking part into the simulation.
     * @param multibodyJoints - The multibody joints taking part into the simulation.
     */
    public serializeAll(
        gravity: Vector,
        integrationParameters: IntegrationParameters,
        islands: IslandManager,
        broadPhase: BroadPhase,
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        impulseJoints: ImpulseJointSet,
        multibodyJoints: MultibodyJointSet,
    ): Uint8Array {
        const rawGra = VectorOps.intoRaw(gravity);

        const res = this.raw.serializeAll(
            rawGra,
            integrationParameters.raw,
            islands.raw,
            broadPhase.raw,
            narrowPhase.raw,
            bodies.raw,
            colliders.raw,
            impulseJoints.raw,
            multibodyJoints.raw,
        );
        rawGra.free();

        return res!;
    }

    /**
     * Deserialize the complete physics state from a single byte array.
     *
     * Returns `null` if the snapshot could not be decoded, which happens when it
     * was taken by a different version of the engine.
     *
     * @param data - The byte array to deserialize.
     */
    public deserializeAll(data: Uint8Array): World | null {
        return World.fromRaw(this.raw.deserializeAll(data)!);
    }
}
