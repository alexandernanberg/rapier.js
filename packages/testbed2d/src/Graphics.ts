import type * as RAPIER from "@alexandernanberg/rapier2d";
import {Viewport} from "pixi-viewport";
import * as PIXI from "pixi.js";
import {Color} from "pixi.js";

type RAPIER_API = typeof import("@alexandernanberg/rapier2d");

const BOX_INSTANCE_INDEX = 0;
const BALL_INSTANCE_INDEX = 1;

var kk = 0;

// Scratch object for zero-allocation getters
const _translation = {x: 0, y: 0};

export class Graphics {
    coll2gfx: Map<number, PIXI.Graphics>;
    colorIndex: number;
    colorPalette: Array<number>;
    renderer: PIXI.WebGLRenderer;
    scene: PIXI.Container;
    viewport: Viewport;
    instanceGroups: Array<Array<PIXI.Graphics>>;
    lines: PIXI.Graphics;

    private constructor() {
        this.coll2gfx = new Map();
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

        // Clean up previous debug lines
        if (this.lines) {
            this.viewport.removeChild(this.lines);
            this.lines.destroy();
            this.lines = null;
        }

        if (debugRender) {
            let buffers = world.debugRender();
            let vtx = buffers.vertices;
            let cls = buffers.colors;

            // Group lines by color for efficient batching in pixi.js 8
            // (stroke() re-strokes all accumulated paths, so we batch by color)
            const linesByColor = new Map<number, {vtx: number[]; alpha: number}>();

            for (let i = 0; i < vtx.length / 4; i += 1) {
                const color = new Color([cls[i * 8], cls[i * 8 + 1], cls[i * 8 + 2]]).toNumber();
                const alpha = cls[i * 8 + 3];
                const key = color * 1000 + Math.round(alpha * 100); // Combine color and alpha as key

                if (!linesByColor.has(key)) {
                    linesByColor.set(key, {vtx: [], alpha});
                }
                const group = linesByColor.get(key)!;
                group.vtx.push(vtx[i * 4], -vtx[i * 4 + 1], vtx[i * 4 + 2], -vtx[i * 4 + 3]);
            }

            // Create container for all line groups
            this.lines = new PIXI.Container() as any;
            this.viewport.addChild(this.lines);

            // Draw each color group as a single Graphics with one stroke call
            for (const [key, group] of linesByColor) {
                const color = Math.floor(key / 1000);
                const gfx = new PIXI.Graphics();

                for (let i = 0; i < group.vtx.length; i += 4) {
                    gfx.moveTo(group.vtx[i], group.vtx[i + 1]);
                    gfx.lineTo(group.vtx[i + 2], group.vtx[i + 3]);
                }
                gfx.stroke({width: 0.02, color, alpha: group.alpha});

                this.lines.addChild(gfx);
            }
        }

        this.updatePositions(world);
        this.renderer.render(this.scene);
    }

    lookAt(pos: {zoom: number; target: {x: number; y: number}}) {
        this.viewport.setZoom(pos.zoom);
        this.viewport.moveCenter(pos.target.x, pos.target.y);
    }

    updatePositions(world: RAPIER.World) {
        world.forEachCollider((elt) => {
            let gfx = this.coll2gfx.get(elt.handle);
            elt.translation(_translation);
            let rotation = elt.rotation(); // returns number, no allocation

            if (!!gfx) {
                gfx.position.x = _translation.x;
                gfx.position.y = -_translation.y;
                gfx.rotation = -rotation;
            }
        });
    }

    reset() {
        this.coll2gfx.forEach((gfx) => {
            this.viewport.removeChild(gfx);
            gfx.destroy();
        });
        this.coll2gfx = new Map();
        this.colorIndex = 0;
    }

    addCollider(RAPIER: RAPIER_API, world: RAPIER.World, collider: RAPIER.Collider) {
        let i;
        let parent = collider.parent();
        let instance;
        let graphics: PIXI.Graphics;
        let vertices;
        let instanceId = parent.isFixed() ? 0 : this.colorIndex + 1;

        switch (collider.shapeType()) {
            case RAPIER.ShapeType.Cuboid:
                let hext = collider.halfExtents();
                instance = this.instanceGroups[BOX_INSTANCE_INDEX][instanceId];
                graphics = instance.clone(true);
                graphics.scale.set(hext.x, hext.y);
                this.viewport.addChild(graphics);
                break;
            case RAPIER.ShapeType.Ball:
                let rad = collider.radius();
                instance = this.instanceGroups[BALL_INSTANCE_INDEX][instanceId];
                graphics = instance.clone(true);
                graphics.scale.set(rad, rad);
                this.viewport.addChild(graphics);
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
                this.viewport.addChild(graphics);
                break;
            case RAPIER.ShapeType.HeightField:
                let heights = Array.from(collider.heightfieldHeights());
                let scale = collider.heightfieldScale();
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
                this.viewport.addChild(graphics);
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
                this.viewport.addChild(graphics);
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
                this.viewport.addChild(graphics);
                break;
            default:
                console.log("Unknown shape to render: ", collider.shapeType());
                return;
        }

        collider.translation(_translation);
        let r = collider.rotation();
        graphics.position.x = _translation.x;
        graphics.position.y = -_translation.y;
        graphics.rotation = r;

        this.coll2gfx.set(collider.handle, graphics);
        this.colorIndex = (this.colorIndex + 1) % (this.colorPalette.length - 1);
    }
}
