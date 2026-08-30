import * as THREE from "three";
import type {OrbitControls} from "three/addons/controls/OrbitControls.js";

/**
 * A chase camera that keeps its own smoothed state.
 *
 * Three things matter here, and getting any of them wrong shows up as the car
 * appearing to wander around the screen even when it is driving perfectly
 * straight:
 *
 *  - **One rate for everything.** Smoothing the eye position and the look-at
 *    point at different speeds tilts the view back and forth on its own.
 *  - **Frame-rate independence.** A fixed `lerp` factor per frame means the
 *    smoothing changes whenever the frame time does, so ordinary frame-time
 *    jitter turns into camera jitter.
 *  - **A smoothed heading.** The camera sits several metres behind the car, so
 *    it multiplies yaw: a twitch of a fraction of a degree at the car becomes a
 *    visible sideways swing of the view. Following a smoothed heading rather
 *    than the instantaneous one keeps small yaw movement off the screen while
 *    still turning with the car.
 */
export class ChaseCamera {
    private readonly position = new THREE.Vector3();
    private readonly target = new THREE.Vector3();
    private readonly heading = new THREE.Vector3(0, 0, 1);
    private started = false;
    private lastTime = 0;

    private readonly desiredEye = new THREE.Vector3();
    private readonly desiredTarget = new THREE.Vector3();
    private readonly forward = new THREE.Vector3();

    constructor(
        private readonly distance = 9.5,
        private readonly height = 4.2,
        private readonly lookHeight = 1.0,
        /** How fast the camera closes on its ideal spot, per second. */
        private readonly followRate = 9,
        /** How fast it adopts the car's heading, per second. */
        private readonly headingRate = 4,
    ) {}

    /** Reposition immediately, e.g. after the car is teleported. */
    reset() {
        this.started = false;
    }

    update(
        camera: THREE.PerspectiveCamera,
        controls: OrbitControls,
        carPosition: THREE.Vector3,
        carRotation: THREE.Quaternion,
    ) {
        const now = performance.now() / 1000;
        // Clamped so a stall or a background tab does not teleport the camera.
        const dt = this.started ? Math.min(0.1, Math.max(1e-4, now - this.lastTime)) : 1 / 60;
        this.lastTime = now;

        this.forward.set(0, 0, 1).applyQuaternion(carRotation);
        this.forward.y = 0;
        if (this.forward.lengthSq() > 1e-6) this.forward.normalize();
        else this.forward.copy(this.heading);

        if (!this.started) this.heading.copy(this.forward);
        this.heading.lerp(this.forward, 1 - Math.exp(-this.headingRate * dt)).normalize();

        this.desiredEye
            .copy(carPosition)
            .addScaledVector(this.heading, -this.distance)
            .setY(carPosition.y + this.height);
        this.desiredTarget.copy(carPosition).setY(carPosition.y + this.lookHeight);

        if (!this.started) {
            this.position.copy(this.desiredEye);
            this.target.copy(this.desiredTarget);
            this.started = true;
        }

        const k = 1 - Math.exp(-this.followRate * dt);
        this.position.lerp(this.desiredEye, k);
        this.target.lerp(this.desiredTarget, k);

        camera.position.copy(this.position);
        // Keep OrbitControls' target in step, so the `controls.update()` that
        // the renderer runs afterwards agrees with us instead of correcting us.
        controls.target.copy(this.target);
        camera.lookAt(this.target);
    }
}
