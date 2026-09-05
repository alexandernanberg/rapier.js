import type * as RAPIER from "@alexandernanberg/rapier2d";
import {Viewport} from "pixi-viewport";
import * as PIXI from "pixi.js";

type RAPIER_API = typeof import("@alexandernanberg/rapier2d");

const BOX_INSTANCE_INDEX = 0;
const BALL_INSTANCE_INDEX = 1;

var kk = 0;

// Scratch object for zero-allocation getters
const _translation = {x: 0, y: 0};

/**
 * One collider and the `Graphics` drawing it, held together so the frame loop reaches
 * a shape without a lookup: the per-collider work in 2D is three writes, and two map
 * lookups to find them cost more than they do.
 */
interface ColliderGfx {
    /**
     * Re-bound whenever the whole scene is re-read, since a `Collider` does not
     * outlive the world that handed it out (a snapshot restore frees that world).
     */
    collider: RAPIER.Collider;
    gfx: PIXI.Graphics;
}

/** Writes one collider's pose into the `Graphics` drawing it. */
function draw(entry: ColliderGfx) {
    entry.collider.translation(_translation);
    entry.gfx.position.x = _translation.x;
    entry.gfx.position.y = -_translation.y;
    // Screen space has y pointing down, so the rotation flips with it.
    entry.gfx.rotation = -entry.collider.rotation();
}

/** The debug lines sharing one color and alpha, drawn by a single stroke. */
interface DebugLineGroup {
    gfx: PIXI.Graphics;
    color: number;
    alpha: number;
    /** Whether the frame being drawn put any lines in this group. */
    used: boolean;
}

export class Graphics {
    coll2gfx: Map<number, ColliderGfx>;
    /**
     * Body handle -> what is drawn for it, so `updatePositions` can go from the bodies
     * that moved to the shapes that have to follow them.
     */
    rb2gfx: Map<number, Array<ColliderGfx>>;
    colorIndex: number;
    colorPalette: Array<number>;
    // Assigned by `init()`, which `create()` awaits before handing out the instance.
    renderer!: PIXI.WebGLRenderer;
    scene!: PIXI.Container;
    viewport!: Viewport;
    /** The colliders, under a container of their own so the debug lines stay above them. */
    shapes!: PIXI.Container;
    instanceGroups!: Array<Array<PIXI.Graphics>>;
    lines!: PIXI.Container;
    /** One `Graphics` per (color, alpha) pair, reused across frames. */
    private lineGroups: Map<number, DebugLineGroup>;
    /** Reused across frames so debug rendering allocates nothing per frame. */
    debugBuffers?: RAPIER.DebugRenderBuffers;
    /**
     * Set when the next frame has to re-read every collider rather than only the ones
     * that moved: a demo was loaded, or a snapshot restored into a world that has not
     * stepped since.
     */
    private fullUpdate: boolean;

    private constructor() {
        this.coll2gfx = new Map();
        this.rb2gfx = new Map();
        this.lineGroups = new Map();
        this.fullUpdate = true;
        this.colorIndex = 0;
        this.colorPalette = [0xf3d9b1, 0x98c1d9, 0x053c5e, 0x1f7a8c];
    }

    static async create(): Promise<Graphics> {
        const graphics = new Graphics();
        await graphics.init();
        return graphics;
    }

    private async init() {
        // High pixel Ratio make the rendering extremely slow, so we cap it.
        // const pixelRatio = window.devicePixelRatio ? Math.min(window.devicePixelRatio, 1.5) : 1;

        this.renderer = new PIXI.WebGLRenderer();
        await this.renderer.init({
            backgroundColor: 0x292929,
            antialias: true,
            // resolution: pixelRatio,
            width: window.innerWidth,
            height: window.innerHeight,
        });

        this.scene = new PIXI.Container();
        document.body.appendChild(this.renderer.canvas);

        this.viewport = new Viewport({
            screenWidth: window.innerWidth,
            screenHeight: window.innerHeight,
            worldWidth: 1000,
            worldHeight: 1000,
            events: this.renderer.events,
        });

        this.scene.addChild(this.viewport);
        this.viewport.drag().pinch().wheel().decelerate();

        // Added once, in the order they draw in: reordering them per frame would mean
        // splicing the viewport's child list, which is as long as the collider count.
        this.shapes = new PIXI.Container();
        this.lines = new PIXI.Container();
        this.viewport.addChild(this.shapes);
        this.viewport.addChild(this.lines);

        let me = this;

        function onWindowResize() {
            me.renderer.resize(window.innerWidth, window.innerHeight);
        }

        function onContextMenu(event: UIEvent) {
            event.preventDefault();
        }

        document.oncontextmenu = onContextMenu;
        document.body.oncontextmenu = onContextMenu;

        window.addEventListener("resize", onWindowResize, false);

        this.initInstances();
    }

    initInstances() {
        this.instanceGroups = [];
        this.instanceGroups.push(
            this.colorPalette.map((color) => {
                let graphics = new PIXI.Graphics();
                graphics.rect(-1.0, -1.0, 2.0, 2.0);
                graphics.fill(color);
                return graphics;
            }),
        );

        this.instanceGroups.push(
            this.colorPalette.map((color) => {
                let graphics = new PIXI.Graphics();
                graphics.circle(0.0, 0.0, 1.0);
                graphics.fill(color);
                return graphics;
            }),
        );
    }

    render(world: RAPIER.World, debugRender: boolean) {
        kk += 1;

        if (debugRender) {
            this.renderDebugLines(world);
        } else {
            this.lines.visible = false;
        }

        this.updatePositions(world);
        this.renderer.render(this.scene);
    }

    /**
     * Redraws the debug lines into the `Graphics` that already hold them.
     *
     * Both halves of this used to be rebuilt every frame — a container and a
     * `Graphics` per color, and a `Color` object per line to read its color out of
     * the buffer. Clearing and refilling what is already there leaves the per-frame
     * cost at the path itself.
     */
    private renderDebugLines(world: RAPIER.World) {
        // Reusing one buffer pair keeps the lines out of the per-frame garbage:
        // `debugRender` copies into it instead of allocating a new pair.
        let buffers = (this.debugBuffers = world.debugRender(
            undefined,
            undefined,
            this.debugBuffers,
        ));
        let vtx = buffers.vertices;
        let cls = buffers.colors;

        this.lines.visible = true;
        this.lineGroups.forEach((group) => {
            group.gfx.clear();
            group.used = false;
        });

        // Lines are grouped by color because `stroke()` re-strokes every path
        // accumulated since the last one, so a stroke per line would be quadratic.
        let count = (vtx.length / 4) | 0;

        for (let i = 0; i < count; i += 1) {
            let c = i * 8;
            // What `Color` packs these components into, without an object per line.
            let color =
                (Math.round(cls[c] * 255) << 16) |
                (Math.round(cls[c + 1] * 255) << 8) |
                Math.round(cls[c + 2] * 255);
            let alpha = cls[c + 3];
            // Color and alpha share a key, since a stroke fixes both.
            let key = color * 1000 + Math.round(alpha * 100);
            let group = this.lineGroups.get(key);

            if (group === undefined) {
                group = {gfx: new PIXI.Graphics(), color, alpha, used: false};
                this.lineGroups.set(key, group);
                this.lines.addChild(group.gfx);
            }

            group.used = true;

            let v = i * 4;
            group.gfx.moveTo(vtx[v], -vtx[v + 1]);
            group.gfx.lineTo(vtx[v + 2], -vtx[v + 3]);
        }

        this.lineGroups.forEach((group) => {
            // A color the scene stopped using keeps its (now empty) `Graphics` around
            // for the frame that needs it again; hiding it costs nothing meanwhile.
            group.gfx.visible = group.used;

            if (group.used) {
                group.gfx.stroke({width: 0.02, color: group.color, alpha: group.alpha});
            }
        });
    }

    lookAt(pos: {zoom: number; target: {x: number; y: number}}) {
        this.viewport.setZoom(pos.zoom);
        this.viewport.moveCenter(pos.target.x, pos.target.y);
    }

    /**
     * Moves the shapes of the bodies that moved.
     *
     * Only the colliders of awake bodies are read: a fixed body never moves, and a
     * settled scene is mostly asleep, so walking every collider every frame re-wrote
     * the transform of thousands of shapes standing still. Nothing is interpolated
     * here, so the pose written on a body's last awake frame is already its resting
     * one and needs no correction once it sleeps.
     */
    updatePositions(world: RAPIER.World) {
        if (this.fullUpdate) {
            // Transforms changed without the world stepping, so there is no active set
            // that describes what moved. This is also where the colliders held above
            // are re-bound, since the world that handed them out may have been freed.
            this.fullUpdate = false;
            world.forEachCollider((collider) => {
                let entry = this.coll2gfx.get(collider.handle);

                if (entry !== undefined) {
                    entry.collider = collider;
                    draw(entry);
                }
            });
            return;
        }

        world.islands.forEachActiveRigidBodyHandle((handle) => {
            let entries = this.rb2gfx.get(handle);

            if (entries === undefined) {
                return;
            }

            for (let i = 0; i < entries.length; ++i) {
                draw(entries[i]);
            }
        });
    }

    /**
     * Forces the next frame to re-read every collider rather than only the ones that
     * moved, for the cases where transforms change without the world stepping: a demo
     * being loaded, or a snapshot restored.
     */
    refresh() {
        this.fullUpdate = true;
    }

    reset() {
        this.coll2gfx.forEach((entry) => {
            this.shapes.removeChild(entry.gfx);
            entry.gfx.destroy();
        });
        this.coll2gfx = new Map();
        this.rb2gfx = new Map();

        // The next demo's debug lines are its own, and a pool keyed by the colors of
        // this one would only keep empty groups around.
        this.lineGroups.forEach((group) => {
            this.lines.removeChild(group.gfx);
            group.gfx.destroy();
        });
        this.lineGroups.clear();

        this.colorIndex = 0;
        this.fullUpdate = true;
    }

    addCollider(RAPIER: RAPIER_API, world: RAPIER.World, collider: RAPIER.Collider) {
        let i;
        let parent = collider.parent();
        let instance;
        let graphics: PIXI.Graphics;
        let vertices;
        // A collider without a parent body is static, so colour it like a fixed one.
        let instanceId = parent === null || parent.isFixed() ? 0 : this.colorIndex + 1;

        switch (collider.shapeType()) {
            case RAPIER.ShapeType.Cuboid:
                let hext = collider.halfExtents()!;
                instance = this.instanceGroups[BOX_INSTANCE_INDEX][instanceId];
                graphics = instance.clone(true);
                graphics.scale.set(hext.x, hext.y);
                this.shapes.addChild(graphics);
                break;
            case RAPIER.ShapeType.Ball:
                let rad = collider.radius();
                instance = this.instanceGroups[BALL_INSTANCE_INDEX][instanceId];
                graphics = instance.clone(true);
                graphics.scale.set(rad, rad);
                this.shapes.addChild(graphics);
                break;
            case RAPIER.ShapeType.Polyline:
                vertices = Array.from(collider.vertices());
                graphics = new PIXI.Graphics();
                graphics.moveTo(vertices[0], -vertices[1]);

                for (i = 2; i < vertices.length; i += 2) {
                    graphics.lineTo(vertices[i], -vertices[i + 1]);
                }

                graphics.stroke({
                    width: 0.2,
                    color: this.colorPalette[instanceId],
                });
                this.shapes.addChild(graphics);
                break;
            case RAPIER.ShapeType.HeightField:
                let heights = Array.from(collider.heightfieldHeights());
                let scale = collider.heightfieldScale()!;
                let step = scale.x / (heights.length - 1);

                graphics = new PIXI.Graphics();
                graphics.moveTo(-scale.x / 2.0, -heights[0] * scale.y);

                for (i = 1; i < heights.length; i += 1) {
                    graphics.lineTo(-scale.x / 2.0 + i * step, -heights[i] * scale.y);
                }

                graphics.stroke({
                    width: 0.2,
                    color: this.colorPalette[instanceId],
                });
                this.shapes.addChild(graphics);
                break;
            case RAPIER.ShapeType.ConvexPolygon:
                vertices = Array.from(collider.vertices());
                graphics = new PIXI.Graphics();
                graphics.moveTo(vertices[0], -vertices[1]);

                for (i = 2; i < vertices.length; i += 2) {
                    graphics.lineTo(vertices[i], -vertices[i + 1]);
                }

                graphics.fill({
                    color: this.colorPalette[instanceId],
                    alpha: 1.0,
                });
                this.shapes.addChild(graphics);
                break;
            case RAPIER.ShapeType.Voxels:
                graphics = new PIXI.Graphics();
                collider.clearShapeCache();
                let shape = collider.shape as RAPIER.Voxels;
                let gridCoords = shape.data;
                let sz = shape.voxelSize;

                for (i = 0; i < gridCoords.length; i += 2) {
                    let minx = gridCoords[i] * sz.x;
                    let miny = gridCoords[i + 1] * sz.y;
                    let maxx = minx + sz.x;
                    let maxy = miny + sz.y;

                    graphics.moveTo(minx, -miny);
                    graphics.lineTo(maxx, -miny);
                    graphics.lineTo(maxx, -maxy);
                    graphics.lineTo(minx, -maxy);
                    graphics.closePath();
                }

                graphics.fill({
                    color: this.colorPalette[instanceId],
                    alpha: 1.0,
                });
                this.shapes.addChild(graphics);
                break;
            default:
                console.log("Unknown shape to render: ", collider.shapeType());
                return;
        }

        let entry: ColliderGfx = {collider, gfx: graphics};

        if (parent !== null) {
            let entries = this.rb2gfx.get(parent.handle);

            if (entries === undefined) {
                this.rb2gfx.set(parent.handle, [entry]);
            } else {
                entries.push(entry);
            }
        }

        // Placed here rather than left to the next frame: a collider on a body that is
        // asleep (or fixed) is never reported active, so nothing would come back to
        // place it. Going through `draw` also settles the rotation sign, which the two
        // used to disagree on.
        draw(entry);

        this.coll2gfx.set(collider.handle, entry);
        this.colorIndex = (this.colorIndex + 1) % (this.colorPalette.length - 1);
    }
}
