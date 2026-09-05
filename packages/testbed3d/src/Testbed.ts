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
    postTimestepAction?: (gfx: Graphics) => void;
    physicsHooks?: RAPIER.PhysicsHooks;
    stepId: number;
    prevDemo?: string;
    lastMessageTime: number;
    snap?: Uint8Array;
    snapStepId: number;
    // Fixed timestep state
    lastFrameTime: number;
    accumulator: number;
    maxSubsteps: number;
    /** Bound once, so the frame loop doesn't allocate a closure per frame. */
    private readonly loop: () => void = () => this.run();

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

    /** Runs right after each `world.step()`, e.g. to drain the event queue. */
    setpostTimestepAction(action: (gfx: Graphics) => void) {
        this.postTimestepAction = action;
    }

    setPhysicsHooks(hooks: RAPIER.PhysicsHooks) {
        this.physicsHooks = hooks;
    }

    /** Shows a demo's own readings on screen, the way upstream's example settings do. */
    setDemoText(text: string) {
        this.gui.setDemoText(text);
    }

    setWorld(world: RAPIER.World) {
        document.onkeydown = null; // Reset key events.
        document.onkeyup = null; // Reset key events.

        this.preTimestepAction = undefined;
        this.postTimestepAction = undefined;
        this.physicsHooks = undefined;
        this.gui.setDemoText("");
        this.world = world;
        this.world.numSolverIterations = this.parameters.numSolverIters;
        this.demoToken += 1;
        this.stepId = 0;
        this.gui.resetTiming();

        world.forEachCollider((coll) => {
            this.graphics.addCollider(this.RAPIER, world, coll);
        });

        // The world has not stepped yet, so it reports nothing as active: the first
        // frame has to read every collider rather than only the ones that moved.
        this.graphics.refresh();

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
                // Every transform just changed without a step to report it.
                this.graphics.refresh();
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
                this.world.step(this.events, this.physicsHooks);
                totalStepTime += performance.now() - t0;
                stepCount += 1;

                if (!!this.postTimestepAction) {
                    this.postTimestepAction(this.graphics);
                }

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

        this.gui.stats.begin();
        this.graphics.render(this.world, this.parameters.debugRender, alpha);
        this.gui.stats.end();

        requestAnimationFrame(this.loop);
    }
}
