import type * as RAPIER from "@alexandernanberg/rapier3d";
import {xxhash128} from "hash-wasm";
import type {DebugInfos} from "./Gui";
import {Graphics} from "./Graphics";
import {Gui} from "./Gui";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

type Builders = Map<string, (RAPIER: RAPIER_API, testbed: Testbed) => void>;

class SimulationParameters {
    backend: string;
    prevBackend: string;
    demo: string;
    numSolverIters: number;
    running: boolean;
    stepping: boolean;
    debugInfos: boolean;
    debugRender: boolean;
    step: () => void;
    restart: () => void;
    takeSnapshot: () => void;
    restoreSnapshot: () => void;
    backends: Array<string>;
    builders: Builders;

    constructor(backends: Array<string>, builders: Builders) {
        this.backend = "rapier";
        this.prevBackend = "rapier";
        this.demo = "collision groups";
        this.numSolverIters = 4;
        this.running = true;
        this.stepping = false;
        this.debugRender = false;
        this.step = () => {};
        this.restart = () => {};
        this.takeSnapshot = () => {};
        this.restoreSnapshot = () => {};
        this.backends = backends;
        this.builders = builders;
        this.debugInfos = false;
    }
}

export class Testbed {
    RAPIER: RAPIER_API;
    gui: Gui;
    graphics: Graphics;
    inhibitLookAt: boolean;
    parameters: SimulationParameters;
    demoToken: number;
    mouse: {x: number; y: number};
    events: RAPIER.EventQueue;
    // Assigned by `setWorld()`, which every demo builder calls.
    world!: RAPIER.World;
    preTimestepAction?: (gfx: Graphics) => void;
    renderAction?: (gfx: Graphics, alpha: number) => void;
    stepId: number;
    prevDemo?: string;
    lastMessageTime: number;
    snap?: Uint8Array;
    snapStepId: number;
    // Fixed timestep state
    lastFrameTime: number;
    accumulator: number;
    maxSubsteps: number;

    constructor(RAPIER: RAPIER_API, builders: Builders) {
        let backends = ["rapier"];
        this.RAPIER = RAPIER;
        let parameters = new SimulationParameters(backends, builders);
        this.gui = new Gui(this, parameters);
        this.graphics = new Graphics();
        this.inhibitLookAt = false;
        this.parameters = parameters;
        this.demoToken = 0;
        this.mouse = {x: 0, y: 0};
        this.events = new RAPIER.EventQueue(true);
        this.stepId = 0;
        this.snapStepId = 0;
        this.lastMessageTime = new Date().getTime();

        // Fixed timestep initialization
        this.lastFrameTime = 0;
        this.accumulator = 0;
        this.maxSubsteps = 6;

        const firstDemo = builders.keys().next().value;
        if (firstDemo === undefined) {
            throw new Error("Testbed needs at least one demo builder.");
        }
        this.switchToDemo(firstDemo);

        window.addEventListener("mousemove", (event) => {
            this.mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
            this.mouse.y = 1 - (event.clientY / window.innerHeight) * 2;
        });
    }

    setpreTimestepAction(action: (gfx: Graphics) => void) {
        this.preTimestepAction = action;
    }

    /**
     * Runs once per rendered frame, just before the scene is drawn, with the
     * interpolation factor between the last two physics steps.
     *
     * Anything visual belongs here rather than in `setpreTimestepAction`: that
     * one runs inside the fixed-step loop, so it fires zero, one or two times
     * per frame, and driving meshes or the camera from it makes them stutter
     * against the colliders, which the renderer interpolates.
     */
    setRenderAction(action: (gfx: Graphics, alpha: number) => void) {
        this.renderAction = action;
    }

    /**
     * Hand the camera to the demo.
     *
     * `Graphics.render` calls `controls.update()` every frame, which re-derives
     * the camera position from OrbitControls' own damped state. A demo that
     * also writes `camera.position` each frame ends up fighting it, and the
     * result reads as a jittery camera. Turning input and damping off leaves
     * the demo in sole control.
     */
    useChaseCamera() {
        this.graphics.controls.enabled = false;
        this.graphics.controls.enableDamping = false;
    }

    setWorld(world: RAPIER.World) {
        document.onkeydown = null; // Reset key events.
        document.onkeyup = null; // Reset key events.

        // Drop any DOM a previous demo added on top of the canvas, so its HUD
        // does not linger over the next one.
        document.querySelectorAll(".demo-overlay").forEach((node) => node.remove());

        this.preTimestepAction = undefined;
        this.renderAction = undefined;
        // A demo may take the camera over (see `useChaseCamera`); give the next
        // one the orbit camera back.
        this.graphics.controls.enabled = true;
        this.graphics.controls.enableDamping = true;
        this.world = world;
        this.world.numSolverIterations = this.parameters.numSolverIters;
        this.demoToken += 1;
        this.stepId = 0;
        this.gui.resetTiming();

        world.forEachCollider((coll) => {
            this.graphics.addCollider(this.RAPIER, world, coll);
        });

        this.lastMessageTime = new Date().getTime();
    }

    lookAt(pos: Parameters<Graphics["lookAt"]>[0]) {
        if (!this.inhibitLookAt) {
            this.graphics.lookAt(pos);
        }

        this.inhibitLookAt = false;
    }

    switchToDemo(demo: string) {
        if (demo == this.prevDemo) {
            this.inhibitLookAt = true;
        }

        this.prevDemo = demo;
        this.graphics.reset();

        this.parameters.prevBackend = this.parameters.backend;

        const builder = this.parameters.builders.get(demo);
        if (builder === undefined) {
            throw new Error(`Unknown demo: ${demo}`);
        }
        builder(this.RAPIER, this);
    }

    switchToBackend(_backend: string) {
        this.switchToDemo(this.parameters.demo);
    }

    takeSnapshot() {
        this.snap = this.world.takeSnapshot();
        this.snapStepId = this.stepId;
    }

    restoreSnapshot() {
        if (!!this.snap) {
            const restored = this.RAPIER.World.restoreSnapshot(this.snap);

            if (restored !== null) {
                this.world.free();
                this.world = restored;
                this.stepId = this.snapStepId;
            }
        }
    }

    run() {
        const currentTime = performance.now() / 1000; // Convert to seconds
        let deltaTime = currentTime - this.lastFrameTime;
        this.lastFrameTime = currentTime;

        // Clamp delta time to prevent spiral of death on long frames
        const maxDeltaTime = this.world.timestep * this.maxSubsteps;
        if (deltaTime > maxDeltaTime) {
            deltaTime = maxDeltaTime;
        }

        let alpha = 1; // Interpolation factor for rendering

        if (this.parameters.running || this.parameters.stepping) {
            this.world.numSolverIterations = this.parameters.numSolverIters;

            const fixedStep = this.world.timestep;
            this.accumulator += deltaTime;

            let totalStepTime = 0;
            let stepCount = 0;

            // Run physics in fixed timestep increments
            while (this.accumulator >= fixedStep) {
                if (!!this.preTimestepAction) {
                    this.preTimestepAction(this.graphics);
                }

                let t0 = performance.now();
                this.world.step(this.events);
                totalStepTime += performance.now() - t0;
                stepCount += 1;

                this.stepId += 1;
                this.accumulator -= fixedStep;
            }

            // Report average step time if any steps were taken
            if (stepCount > 0) {
                this.gui.setTiming(totalStepTime / stepCount);
            }

            // Calculate interpolation factor for smooth rendering
            alpha = this.accumulator / fixedStep;

            if (!!this.parameters.debugInfos) {
                let t0 = performance.now();
                let snapshot = this.world.takeSnapshot();
                let t1 = performance.now();
                let snapshotTime = t1 - t0;

                let debugInfos: DebugInfos = {
                    token: this.demoToken,
                    stepId: this.stepId,
                    worldHash: "",
                    worldHashTime: 0,
                    snapshotTime: 0,
                    timingStep: this.world.timingStep(),
                    timingCollisionDetection: this.world.timingCollisionDetection(),
                    timingBroadPhase: this.world.timingBroadPhase(),
                    timingNarrowPhase: this.world.timingNarrowPhase(),
                    timingSolver: this.world.timingSolver(),
                    timingVelocityAssembly: this.world.timingVelocityAssembly(),
                    timingVelocityResolution: this.world.timingVelocityResolution(),
                    timingVelocityUpdate: this.world.timingVelocityUpdate(),
                    timingVelocityWriteback: this.world.timingVelocityWriteback(),
                    timingCcd: this.world.timingCcd(),
                    timingCcdToiComputation: this.world.timingCcdToiComputation(),
                    timingCcdBroadPhase: this.world.timingCcdBroadPhase(),
                    timingCcdNarrowPhase: this.world.timingCcdNarrowPhase(),
                    timingCcdSolver: this.world.timingCcdSolver(),
                    timingIslandConstruction: this.world.timingIslandConstruction(),
                    timingUserChanges: this.world.timingUserChanges(),
                };
                t0 = performance.now();
                xxhash128(snapshot).then((hash) => {
                    debugInfos.worldHash = hash;
                    t1 = performance.now();
                    let worldHashTime = t1 - t0;
                    debugInfos.worldHashTime = worldHashTime;
                    debugInfos.snapshotTime = snapshotTime;
                    this.gui.setDebugInfos(debugInfos);
                });
            }
        }

        if (this.parameters.stepping) {
            this.parameters.running = false;
            this.parameters.stepping = false;
        }

        if (!!this.renderAction) {
            this.renderAction(this.graphics, alpha);
        }

        this.gui.stats.begin();
        this.graphics.render(this.world, this.parameters.debugRender, alpha);
        this.gui.stats.end();

        requestAnimationFrame(() => this.run());
    }
}
