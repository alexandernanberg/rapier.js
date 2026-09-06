import RAPIER from "@alexandernanberg/rapier3d";
import * as THREE from "three";
import {OrbitControls} from "three/addons/controls/OrbitControls.js";

/** Capacity an instanced mesh starts at, before it doubles on demand. */
const INITIAL_INSTANCES = 256;
/**
 * Vertex count above which a mesh shape gets a mesh of its own instead of being
 * instanced. Hashing the vertex buffer to find its instance group costs more than
 * it saves for a shape that only appears once, and the big ones (heightfields,
 * scenery trimeshes) always do.
 */
const MAX_INSTANCED_VERTICES = 1024;

/** Palette slot used for colliders on a fixed body (or on no body at all). */
const FIXED_COLOR = 0;
/**
 * How many palette slots `addCollider` hands out to dynamic bodies, starting at
 * slot 1. The slots past those are only used when a demo asks for them by index,
 * so adding one doesn't shift the colors of every other demo.
 */
const DYNAMIC_COLORS = 3;
/** Palette slot of the mouse-over highlight. */
const HIGHLIGHT_COLOR = 4;
/**
 * Palette slot left to demos that recolor bodies themselves (e.g. `sensor`
 * flagging the boxes currently inside its sensor).
 */
export const ACCENT_COLOR = 5;
/**
 * Palette slot of sensor colliders, drawn as wireframes so what they overlap
 * stays visible.
 */
const SENSOR_COLOR = 6;

var kk = 0;

// Scratch objects for zero-allocation getters
const _translation = {x: 0, y: 0, z: 0};
const _rotation = {x: 0, y: 0, z: 0, w: 1};

// Scratch objects for interpolation
const _position = new THREE.Vector3();
const _quaternion = new THREE.Quaternion();
const _prevPosition = new THREE.Vector3();
const _prevQuaternion = new THREE.Quaternion();
const _matrix = new THREE.Matrix4();
const _mouse = new THREE.Vector2();

interface InstanceDesc {
    /** Key of the instance group, i.e. of the geometry the collider is drawn with. */
    groupId: string;
    instanceId: number;
    elementId: number;
    highlighted: boolean;
    scale: THREE.Vector3;
    /** Frame whose transform this collider was last drawn at, see `updatePositions`. */
    frameId: number;
    // Interpolation state, populated on the first frame the instance is drawn.
    interpolation?: Interpolation;
}

/** How a collider drawn with a mesh of its own is tracked across frames. */
interface MeshDesc {
    mesh: THREE.Mesh;
    /** Frame whose transform this collider was last drawn at, see `updatePositions`. */
    frameId: number;
    // Interpolation state, populated on the first frame the mesh is drawn.
    interpolation?: Interpolation;
}

/**
 * One geometry and the instanced meshes that draw it, at most one per palette slot.
 *
 * A slot is filled the first time a collider asks to be drawn in that color, so a
 * demo that uses two of them doesn't pay for the other five.
 */
interface InstanceGroup {
    geometry: THREE.BufferGeometry;
    meshes: Array<THREE.InstancedMesh | undefined>;
}

/** How one collider is drawn: which instanced geometry, and the scale to apply to it. */
interface InstancedShape {
    key: string;
    /** Called only the first time `key` is seen, to build the shared geometry. */
    geometry: () => THREE.BufferGeometry;
    scale: THREE.Vector3;
}

interface Interpolation {
    prevPosition: THREE.Vector3;
    prevQuaternion: THREE.Quaternion;
    snapshotPosition: THREE.Vector3;
    snapshotQuaternion: THREE.Quaternion;
}

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

/**
 * Advances one collider's interpolation state towards the pose sitting in
 * `_position` / `_quaternion`, and leaves the pose to draw this frame in
 * `_prevPosition` / `_prevQuaternion`.
 *
 * @param state - The collider's state, or `undefined` the first frame it is drawn.
 * @param alpha - How far into the step being interpolated this frame sits.
 */
function interpolate(state: Interpolation | undefined, alpha: number): Interpolation {
    if (state === undefined) {
        state = {
            prevPosition: _position.clone(),
            prevQuaternion: _quaternion.clone(),
            snapshotPosition: _position.clone(),
            snapshotQuaternion: _quaternion.clone(),
        };
    } else {
        state.prevPosition.copy(state.snapshotPosition);
        state.prevQuaternion.copy(state.snapshotQuaternion);
    }

    _prevPosition.copy(state.prevPosition);
    _prevQuaternion.copy(state.prevQuaternion);
    _prevPosition.lerp(_position, alpha);
    _prevQuaternion.slerp(_quaternion, alpha);

    state.snapshotPosition.copy(_position);
    state.snapshotQuaternion.copy(_quaternion);

    return state;
}

// NOTE: this is a very naive voxels -> mesh conversion. Proper
//       conversions should use something like greedy meshing instead.
function genVoxelsGeometry(collider: RAPIER.Collider) {
    // Clear the cached shape so it gets recomputed from the source of truth,
    // and so we’ll be sure that the data contain grid coordinates even if the
    // voxels were initialized with floating points.
    collider.clearShapeCache();
    let shape = collider.shape as RAPIER.Voxels;
    let gridCoords = shape.data;
    let sz = shape.voxelSize;
    let vertices = [];
    let indices = [];

    let i: number;
    for (i = 0; i < gridCoords.length; i += 3) {
        let minx = gridCoords[i] * sz.x;
        let miny = gridCoords[i + 1] * sz.y;
        let minz = gridCoords[i + 2] * sz.z;
        let maxx = minx + sz.x;
        let maxy = miny + sz.y;
        let maxz = minz + sz.z;

        let k: number = vertices.length / 3;
        vertices.push(minx, miny, maxz);
        vertices.push(minx, miny, minz);
        vertices.push(maxx, miny, minz);
        vertices.push(maxx, miny, maxz);
        vertices.push(minx, maxy, maxz);
        vertices.push(minx, maxy, minz);
        vertices.push(maxx, maxy, minz);
        vertices.push(maxx, maxy, maxz);

        indices.push(k + 4, k + 5, k + 0);
        indices.push(k + 5, k + 1, k + 0);
        indices.push(k + 5, k + 6, k + 1);
        indices.push(k + 6, k + 2, k + 1);
        indices.push(k + 6, k + 7, k + 3);
        indices.push(k + 2, k + 6, k + 3);
        indices.push(k + 7, k + 4, k + 0);
        indices.push(k + 3, k + 7, k + 0);
        indices.push(k + 0, k + 1, k + 2);
        indices.push(k + 3, k + 0, k + 2);
        indices.push(k + 7, k + 6, k + 5);
        indices.push(k + 4, k + 7, k + 5);
    }

    return {
        vertices: new Float32Array(vertices),
        indices: new Uint32Array(indices),
    };
}

/** The geometry of a single sub-shape of a compound, in its own local frame. */
function genSubShapeGeometry(shape: RAPIER.Shape): THREE.BufferGeometry | null {
    switch (shape.type) {
        case RAPIER.ShapeType.Ball: {
            let ball = shape as RAPIER.Ball;
            return new THREE.SphereGeometry(ball.radius);
        }
        case RAPIER.ShapeType.Cuboid:
        case RAPIER.ShapeType.RoundCuboid: {
            let ext = (shape as RAPIER.Cuboid).halfExtents;
            return new THREE.BoxGeometry(ext.x * 2.0, ext.y * 2.0, ext.z * 2.0);
        }
        case RAPIER.ShapeType.Capsule: {
            let capsule = shape as RAPIER.Capsule;
            return new THREE.CapsuleGeometry(capsule.radius, capsule.halfHeight * 2.0);
        }
        case RAPIER.ShapeType.Cylinder:
        case RAPIER.ShapeType.RoundCylinder: {
            let cyl = shape as RAPIER.Cylinder;
            return new THREE.CylinderGeometry(cyl.radius, cyl.radius, cyl.halfHeight * 2.0);
        }
        case RAPIER.ShapeType.Cone:
        case RAPIER.ShapeType.RoundCone: {
            let cone = shape as RAPIER.Cone;
            return new THREE.ConeGeometry(cone.radius, cone.halfHeight * 2.0);
        }
        case RAPIER.ShapeType.TriMesh:
        case RAPIER.ShapeType.ConvexPolyhedron:
        case RAPIER.ShapeType.RoundConvexPolyhedron: {
            let mesh = shape as RAPIER.TriMesh;
            let geometry = new THREE.BufferGeometry();
            geometry.setIndex(new THREE.BufferAttribute(mesh.indices, 1));
            geometry.setAttribute("position", new THREE.BufferAttribute(mesh.vertices, 3));
            return geometry;
        }
        default:
            return null;
    }
}

/**
 * Flattens a compound collider into a single geometry.
 *
 * A compound is rigid, so each sub-shape's local pose can be baked into the
 * vertices here and the whole thing then moves with the collider, the same way
 * the trimesh and heightfield meshes do.
 */
function genCompoundGeometry(collider: RAPIER.Collider) {
    let compound = collider.shape as RAPIER.Compound;
    let vertices: number[] = [];
    let indices: number[] = [];

    compound.shapes.forEach((subShape, i) => {
        let geometry = genSubShapeGeometry(subShape);

        if (!geometry) {
            console.log("Unknown compound sub-shape to render.");
            return;
        }

        let pos = compound.positions[i];
        let rot = compound.rotations[i];
        _matrix.compose(
            new THREE.Vector3(pos.x, pos.y, pos.z),
            new THREE.Quaternion(rot.x, rot.y, rot.z, rot.w),
            new THREE.Vector3(1.0, 1.0, 1.0),
        );
        geometry.applyMatrix4(_matrix);

        // Indices are local to each sub-shape, so they shift by however many
        // vertices are already in the merged buffer.
        let base = vertices.length / 3;
        let position = geometry.getAttribute("position");
        for (let k = 0; k < position.count; ++k) {
            vertices.push(position.getX(k), position.getY(k), position.getZ(k));
        }

        let index = geometry.getIndex();
        if (!!index) {
            for (let k = 0; k < index.count; ++k) {
                indices.push(base + index.getX(k));
            }
        } else {
            // Un-indexed geometry: every three vertices are already a triangle.
            for (let k = 0; k < position.count; ++k) {
                indices.push(base + k);
            }
        }

        geometry.dispose();
    });

    return {
        vertices: new Float32Array(vertices),
        indices: new Uint32Array(indices),
    };
}

function genHeightfieldGeometry(collider: RAPIER.Collider) {
    let heights = collider.heightfieldHeights()!;
    let nrows = collider.heightfieldNRows()!;
    let ncols = collider.heightfieldNCols()!;
    let scale = collider.heightfieldScale()!;

    let vertices = [];
    let indices = [];
    let eltWX = 1.0 / nrows;
    let eltWY = 1.0 / ncols;

    let i: number;
    let j: number;
    for (j = 0; j <= ncols; ++j) {
        for (i = 0; i <= nrows; ++i) {
            let x = (j * eltWX - 0.5) * scale.x;
            let y = heights[j * (nrows + 1) + i] * scale.y;
            let z = (i * eltWY - 0.5) * scale.z;

            vertices.push(x, y, z);
        }
    }

    for (j = 0; j < ncols; ++j) {
        for (i = 0; i < nrows; ++i) {
            let i1 = (i + 0) * (ncols + 1) + (j + 0);
            let i2 = (i + 0) * (ncols + 1) + (j + 1);
            let i3 = (i + 1) * (ncols + 1) + (j + 0);
            let i4 = (i + 1) * (ncols + 1) + (j + 1);

            indices.push(i1, i3, i2);
            indices.push(i3, i4, i2);
        }
    }

    return {
        vertices: new Float32Array(vertices),
        indices: new Uint32Array(indices),
    };
}

/**
 * FNV-1a over the raw bytes, so colliders built from the same vertex buffer land
 * in the same instance group. Scenes that reuse one shape thousands of times (the
 * box3d junkyard, for instance) then draw in a single call instead of one each.
 */
function hashBuffer(buffer: Float32Array): string {
    let bytes = new Uint8Array(buffer.buffer, buffer.byteOffset, buffer.byteLength);
    let hash = 0x811c9dc5;

    for (let i = 0; i < bytes.length; ++i) {
        hash ^= bytes[i];
        hash = Math.imul(hash, 0x01000193);
    }

    return (hash >>> 0).toString(16);
}

/**
 * The instanced geometry a collider is drawn with, or `null` for the shapes that
 * need a mesh of their own (heightfields, voxels, compounds, big meshes).
 *
 * Shapes that are a scaled unit primitive share one geometry; the ones that aren't
 * (a capsule, a convex hull) are keyed by their dimensions or their vertices, so
 * identical ones still share.
 */
function instancedShape(RAPIER: RAPIER_API, collider: RAPIER.Collider): InstancedShape | null {
    switch (collider.shapeType()) {
        case RAPIER.ShapeType.Cuboid:
        case RAPIER.ShapeType.RoundCuboid: {
            let ext = collider.halfExtents()!;
            return {
                key: "cuboid",
                geometry: () => new THREE.BoxGeometry(2.0, 2.0, 2.0),
                scale: new THREE.Vector3(ext.x, ext.y, ext.z),
            };
        }
        case RAPIER.ShapeType.Ball: {
            let rad = collider.radius()!;
            return {
                key: "ball",
                geometry: () => new THREE.SphereGeometry(1.0),
                scale: new THREE.Vector3(rad, rad, rad),
            };
        }
        case RAPIER.ShapeType.Cylinder:
        case RAPIER.ShapeType.RoundCylinder: {
            let rad = collider.radius()!;
            let height = collider.halfHeight()! * 2.0;
            return {
                key: "cylinder",
                geometry: () => new THREE.CylinderGeometry(1.0, 1.0),
                scale: new THREE.Vector3(rad, height, rad),
            };
        }
        case RAPIER.ShapeType.Cone:
        case RAPIER.ShapeType.RoundCone: {
            let rad = collider.radius()!;
            let height = collider.halfHeight()! * 2.0;
            return {
                key: "cone",
                geometry: () => new THREE.ConeGeometry(1.0, 1.0),
                scale: new THREE.Vector3(rad, height, rad),
            };
        }
        case RAPIER.ShapeType.Capsule: {
            // A capsule is not a scaled unit capsule unless its radius and
            // half-height happen to match, so each pair of dimensions is its own
            // geometry. Scenes tend to use one size throughout, so that is still a
            // single group.
            let rad = collider.radius()!;
            let halfHeight = collider.halfHeight()!;
            return {
                key: `capsule:${rad}:${halfHeight}`,
                geometry: () => new THREE.CapsuleGeometry(rad, halfHeight * 2.0),
                scale: new THREE.Vector3(1.0, 1.0, 1.0),
            };
        }
        case RAPIER.ShapeType.TriMesh:
        case RAPIER.ShapeType.ConvexPolyhedron:
        case RAPIER.ShapeType.RoundConvexPolyhedron: {
            let vertices = collider.vertices()!;
            let indices = collider.indices();

            if (indices === undefined || vertices.length > MAX_INSTANCED_VERTICES * 3) {
                return null;
            }

            return {
                key: `mesh:${vertices.length}:${hashBuffer(vertices)}`,
                geometry: () => {
                    let geometry = new THREE.BufferGeometry();
                    geometry.setIndex(new THREE.BufferAttribute(indices, 1));
                    geometry.setAttribute("position", new THREE.BufferAttribute(vertices, 3));
                    return geometry;
                },
                scale: new THREE.Vector3(1.0, 1.0, 1.0),
            };
        }
        default:
            return null;
    }
}

export class Graphics {
    raycaster: THREE.Raycaster;
    highlightedCollider: null | number;
    coll2instance: Map<number, InstanceDesc>;
    coll2mesh: Map<number, MeshDesc>;
    /**
     * Body handle -> handles of the colliders drawn for it.
     *
     * Handles rather than colliders: a `Collider` handed out by one world does not
     * survive that world being freed (a snapshot restore) or the collider being
     * removed, and `removeRigidBody` is called after the body has already left the
     * world.
     */
    rb2colls: Map<number, Array<number>>;
    colorIndex: number;
    colorPalette: Array<number>;
    scene: THREE.Scene;
    camera: THREE.PerspectiveCamera;
    renderer: THREE.WebGLRenderer;
    light: THREE.PointLight;
    lines: THREE.LineSegments;
    controls: OrbitControls;
    /** Reused across frames so debug rendering allocates nothing per frame. */
    debugBuffers?: RAPIER.DebugRenderBuffers;
    /** Geometry key -> the instanced meshes drawing it, one per palette slot in use. */
    instanceGroups: Map<string, InstanceGroup>;
    /**
     * The attributes the debug lines are drawn from, rebuilt only when `debugRender`
     * comes back with a differently sized buffer. Re-uploading into the attributes
     * already on the GPU beats handing three a new one — and so a new GPU buffer —
     * every frame.
     */
    private debugVertices?: THREE.BufferAttribute;
    private debugColors?: THREE.BufferAttribute;
    /** Bumped once per `updatePositions`, to stamp what that frame has drawn. */
    private frameId: number;
    /**
     * The colliders the previous frame drew, and the ones this frame has drawn. A
     * body that fell asleep in between stops being reported active, so the
     * difference is what still needs the one last update that lands it exactly on
     * its resting pose.
     */
    private prevActive: Array<number>;
    private currActive: Array<number>;
    /**
     * Set when the next frame has to re-read every collider rather than just the
     * ones that moved: a demo was loaded, or a snapshot restored into a world that
     * has not stepped since.
     */
    private fullUpdate: boolean;
    /** Instanced meshes written this frame, whose matrices are uploaded at the end of it. */
    private dirtyInstances: Array<THREE.InstancedMesh>;

    constructor() {
        this.raycaster = new THREE.Raycaster();
        this.highlightedCollider = null;
        this.coll2instance = new Map();
        this.coll2mesh = new Map();
        this.rb2colls = new Map();
        this.instanceGroups = new Map();
        this.frameId = 0;
        this.prevActive = [];
        this.currActive = [];
        this.fullUpdate = true;
        this.dirtyInstances = [];
        this.colorIndex = 0;
        this.colorPalette = [0xf3d9b1, 0x98c1d9, 0x053c5e, 0x1f7a8c, 0xff0000, 0xffe066, 0xc8c8c8];
        this.scene = new THREE.Scene();
        this.camera = new THREE.PerspectiveCamera(
            45,
            window.innerWidth / window.innerHeight,
            0.1,
            10000,
        );
        this.renderer = new THREE.WebGLRenderer({antialias: true});
        this.renderer.setSize(window.innerWidth, window.innerHeight);
        this.renderer.setClearColor(0x292929, 1);
        // High pixel Ratio make the rendering extremely slow, so we cap it.
        const pixelRatio = window.devicePixelRatio ? Math.min(window.devicePixelRatio, 1.5) : 1;
        this.renderer.setPixelRatio(pixelRatio);
        document.body.appendChild(this.renderer.domElement);

        let ambientLight = new THREE.AmbientLight(0x606060);
        this.scene.add(ambientLight);
        // In Three.js r155+, decay defaults to 2 for physically correct lighting.
        // Set decay to 0 to restore the old non-physically-correct behavior.
        this.light = new THREE.PointLight(0xffffff, 1, 0, 0);
        this.scene.add(this.light);

        // For the debug-renderer.
        {
            let material = new THREE.LineBasicMaterial({
                color: 0xffffff,
                vertexColors: true,
            });
            let geometry = new THREE.BufferGeometry();
            this.lines = new THREE.LineSegments(geometry, material);
            this.scene.add(this.lines);
        }
        let me = this;

        function onWindowResize() {
            if (!!me.camera) {
                me.camera.aspect = window.innerWidth / window.innerHeight;
                me.camera.updateProjectionMatrix();
                me.renderer.setSize(window.innerWidth, window.innerHeight);
            }
        }

        window.addEventListener("resize", onWindowResize, false);

        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.2;
        this.controls.maxPolarAngle = Math.PI / 2;
    }

    /**
     * The instance group of one geometry.
     *
     * Groups are built the first time a shape needs one and thrown away on
     * `reset()`, so a demo only pays for the shapes it actually uses.
     */
    private groupFor(shape: InstancedShape): InstanceGroup {
        let group = this.instanceGroups.get(shape.key);

        if (group === undefined) {
            group = {
                geometry: shape.geometry(),
                meshes: new Array(this.colorPalette.length).fill(undefined),
            };
            this.instanceGroups.set(shape.key, group);
        }

        return group;
    }

    /**
     * The instanced mesh drawing one geometry in one palette slot, built on first use.
     *
     * Most demos draw a shape in two or three colors, so filling all seven slots up
     * front would leave most of the instance buffers — and the scene nodes holding
     * them — allocated and never drawn.
     */
    private meshFor(group: InstanceGroup, colorIndex: number): THREE.InstancedMesh {
        let mesh = group.meshes[colorIndex];

        if (mesh === undefined) {
            mesh = this.newInstancedMesh(
                group.geometry,
                this.material(colorIndex),
                INITIAL_INSTANCES,
            );
            group.meshes[colorIndex] = mesh;
        }

        return mesh;
    }

    private newInstancedMesh(
        geometry: THREE.BufferGeometry,
        material: THREE.Material,
        capacity: number,
    ) {
        let instance = new THREE.InstancedMesh(geometry, material, capacity);
        instance.userData.elementId2coll = new Map();
        // Stamped by `markDirty`, so an instanced mesh is queued for upload once a
        // frame however many of its instances that frame moves.
        instance.userData.dirtyFrame = -1;
        instance.count = 0;
        instance.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
        // Instances sit wherever their bodies are, which says nothing about where
        // the instanced mesh's own origin is, so the sphere three would cull
        // against is meaningless and can hide the whole batch.
        instance.frustumCulled = false;
        this.scene.add(instance);
        return instance;
    }

    /**
     * Doubles the capacity of one instanced mesh, keeping the instances it holds.
     *
     * `THREE.InstancedMesh` takes its capacity at construction and silently ignores
     * writes past it, so growing means building a bigger one and moving the
     * instance matrices over. The geometry and material are shared, not rebuilt.
     */
    private growInstances(groupId: string, colorIndex: number): THREE.InstancedMesh {
        let group = this.instanceGroups.get(groupId)!;
        let previous = group.meshes[colorIndex]!;
        let grown = this.newInstancedMesh(
            previous.geometry,
            previous.material as THREE.Material,
            previous.instanceMatrix.count * 2,
        );

        grown.userData.elementId2coll = previous.userData.elementId2coll;
        grown.userData.dirtyFrame = previous.userData.dirtyFrame;
        grown.count = previous.count;
        grown.instanceMatrix.array.set(previous.instanceMatrix.array);
        grown.instanceMatrix.needsUpdate = true;

        this.scene.remove(previous);
        // Frees the instance attributes only: the geometry and material live on in
        // the mesh that just replaced this one.
        previous.dispose();

        group.meshes[colorIndex] = grown;

        // `dirtyInstances` may still hold the mesh that was just replaced; its
        // matrices went to `grown`, so the upload has to follow them.
        let queued = this.dirtyInstances.indexOf(previous);
        if (queued !== -1) {
            this.dirtyInstances[queued] = grown;
        }

        return grown;
    }

    /**
     * The material of one palette slot.
     *
     * @param doubleSided - For the shapes that aren't closed surfaces (a heightfield,
     *                      an open trimesh), whose back faces have to be drawn too.
     */
    private material(colorIndex: number, doubleSided: boolean = false) {
        return new THREE.MeshPhongMaterial({
            color: this.colorPalette[colorIndex],
            side: doubleSided ? THREE.DoubleSide : THREE.FrontSide,
            flatShading: true,
            // Sensors don't collide with anything, so drawing them solid would only
            // hide the bodies passing through them.
            wireframe: colorIndex == SENSOR_COLOR,
        });
    }

    render(world: RAPIER.World, debugRender: boolean, alpha: number = 1) {
        kk += 1;
        this.controls.update();
        // if (kk % 100 == 0) {
        //     console.log(this.camera.position);
        //     console.log(this.controls.target);
        // }

        this.light.position.set(
            this.camera.position.x,
            this.camera.position.y,
            this.camera.position.z,
        );

        if (debugRender) {
            // Reusing one buffer pair keeps the lines out of the per-frame garbage:
            // `debugRender` copies into it instead of allocating a new pair.
            let buffers = (this.debugBuffers = world.debugRender(
                undefined,
                undefined,
                this.debugBuffers,
            ));
            this.lines.visible = true;
            this.updateDebugAttribute("position", buffers.vertices, 3);
            this.updateDebugAttribute("color", buffers.colors, 4);
        } else {
            this.lines.visible = false;
        }

        this.updatePositions(world, alpha);
        this.renderer.render(this.scene, this.camera);
    }

    /**
     * Points one of the debug-line attributes at the buffer `debugRender` just filled.
     *
     * That buffer keeps its identity for as long as the line count holds steady, so
     * the usual frame only flags the attribute already on the GPU for re-upload; a
     * new attribute — and with it a new GPU buffer — is built only when the number
     * of lines actually changes.
     */
    private updateDebugAttribute(
        name: "position" | "color",
        array: Float32Array,
        itemSize: number,
    ) {
        let attribute = name === "position" ? this.debugVertices : this.debugColors;

        if (attribute !== undefined && attribute.array === array) {
            attribute.needsUpdate = true;
            return;
        }

        attribute = new THREE.BufferAttribute(array, itemSize);
        attribute.setUsage(THREE.DynamicDrawUsage);
        this.lines.geometry.setAttribute(name, attribute);

        if (name === "position") {
            this.debugVertices = attribute;
        } else {
            this.debugColors = attribute;
        }
    }

    rayAtMousePosition(pos: {x: number; y: number}) {
        this.raycaster.setFromCamera(_mouse.set(pos.x, pos.y), this.camera);
        return this.raycaster.ray;
    }

    lookAt(pos: {
        target: {x: number; y: number; z: number};
        eye: {x: number; y: number; z: number};
    }) {
        this.camera.position.set(pos.eye.x, pos.eye.y, pos.eye.z);
        this.controls.target.set(pos.target.x, pos.target.y, pos.target.z);
        this.controls.update();
    }

    highlightInstanceId() {
        return HIGHLIGHT_COLOR;
    }

    highlightCollider(handle: number) {
        if (handle == this.highlightedCollider)
            // Avoid flickering when moving the mouse on a single collider.
            return;

        if (this.highlightedCollider != null) {
            let desc = this.coll2instance.get(this.highlightedCollider);

            if (!!desc) {
                desc.highlighted = false;
                let highlight = this.instanceGroups.get(desc.groupId)?.meshes[
                    this.highlightInstanceId()
                ];

                if (highlight !== undefined) {
                    highlight.count = 0;
                }
            }
        }
        if (handle != null) {
            let desc = this.coll2instance.get(handle);

            if (!!desc) {
                if (desc.instanceId != 0)
                    // Don't highlight static/kinematic bodies.
                    desc.highlighted = true;
            }
        }
        this.highlightedCollider = handle;
    }

    /**
     * Re-reads the transforms this frame has to draw, and uploads what moved.
     *
     * Only the colliders of awake bodies are read. A fixed body never moves and a
     * settled scene is mostly asleep, so walking every collider every frame spent an
     * interpolation and a matrix write on thousands of shapes standing still — and,
     * by flagging their instanced meshes, re-uploaded every matrix in the scene to
     * the GPU along with them.
     *
     * @param alpha - How far into the step being interpolated this frame sits.
     */
    updatePositions(world: RAPIER.World, alpha: number = 1) {
        this.frameId += 1;
        this.currActive.length = 0;

        if (this.fullUpdate) {
            // Transforms changed without the world stepping, so there is no active
            // set that describes what moved.
            this.fullUpdate = false;
            world.forEachCollider((collider) => {
                this.updateCollider(collider, alpha);
                this.currActive.push(collider.handle);
            });
        } else {
            world.islands.forEachActiveRigidBodyHandle((handle) => {
                let colls = this.rb2colls.get(handle);

                if (colls === undefined) {
                    return;
                }

                for (let i = 0; i < colls.length; ++i) {
                    let collider = world.getCollider(colls[i]);

                    if (collider !== null) {
                        this.updateCollider(collider, alpha);
                        this.currActive.push(colls[i]);
                    }
                }
            });
        }

        // A body that fell asleep since the previous frame is no longer reported, and
        // the last pose it was drawn at is an interpolated one, short of where it
        // actually came to rest. One more update at `alpha = 1` puts it there.
        for (let i = 0; i < this.prevActive.length; ++i) {
            let handle = this.prevActive[i];
            let drawn = this.lastDrawnFrame(handle);

            if (drawn === -1 || drawn === this.frameId) {
                continue;
            }

            let collider = world.getCollider(handle);

            if (collider !== null) {
                this.updateCollider(collider, 1);
            }
        }

        let recycled = this.prevActive;
        this.prevActive = this.currActive;
        this.currActive = recycled;

        for (let i = 0; i < this.dirtyInstances.length; ++i) {
            this.dirtyInstances[i].instanceMatrix.needsUpdate = true;
        }

        this.dirtyInstances.length = 0;
    }

    /**
     * Forces the next frame to re-read every collider rather than only the ones that
     * moved, for the cases where transforms change without the world stepping: a demo
     * being loaded, or a snapshot restored.
     */
    refresh() {
        this.fullUpdate = true;
    }

    /** The frame this collider was last drawn at, or -1 if nothing draws it. */
    private lastDrawnFrame(handle: number): number {
        let gfx = this.coll2instance.get(handle);

        if (gfx !== undefined) {
            return gfx.frameId;
        }

        let meshDesc = this.coll2mesh.get(handle);
        return meshDesc !== undefined ? meshDesc.frameId : -1;
    }

    /** Queues one instanced mesh's matrices for upload, at most once per frame. */
    private markDirty(instance: THREE.InstancedMesh) {
        if (instance.userData.dirtyFrame === this.frameId) {
            return;
        }

        instance.userData.dirtyFrame = this.frameId;
        this.dirtyInstances.push(instance);
    }

    /**
     * Writes one collider's interpolated pose into whatever draws it.
     *
     * @param alpha - How far into the step being interpolated this frame sits.
     */
    private updateCollider(collider: RAPIER.Collider, alpha: number) {
        let gfx = this.coll2instance.get(collider.handle);
        let meshDesc = gfx === undefined ? this.coll2mesh.get(collider.handle) : undefined;

        if (gfx === undefined && meshDesc === undefined) {
            return;
        }

        collider.translation(_translation);
        collider.rotation(_rotation);
        _position.set(_translation.x, _translation.y, _translation.z);
        _quaternion.set(_rotation.x, _rotation.y, _rotation.z, _rotation.w);

        if (gfx !== undefined) {
            gfx.frameId = this.frameId;
            gfx.interpolation = interpolate(gfx.interpolation, alpha);
            _matrix.compose(_prevPosition, _prevQuaternion, gfx.scale);

            let group = this.instanceGroups.get(gfx.groupId)!;
            let instance = group.meshes[gfx.instanceId]!;
            instance.setMatrixAt(gfx.elementId, _matrix);
            this.markDirty(instance);

            if (gfx.highlighted) {
                let highlight = this.meshFor(group, this.highlightInstanceId());
                highlight.count = 1;
                highlight.setMatrixAt(0, _matrix);
                this.markDirty(highlight);
            }

            return;
        }

        let desc = meshDesc!;
        desc.frameId = this.frameId;
        desc.interpolation = interpolate(desc.interpolation, alpha);
        desc.mesh.position.copy(_prevPosition);
        desc.mesh.quaternion.copy(_prevQuaternion);
        desc.mesh.updateMatrix();
    }

    reset() {
        // Groups are keyed by geometry, and a demo's capsule sizes or convex hulls
        // are its own, so they go with it rather than piling up across demos. The
        // next demo rebuilds the handful it needs on its first collider.
        this.instanceGroups.forEach((group) => {
            group.meshes.forEach((instance) => {
                if (instance === undefined) {
                    return;
                }

                this.scene.remove(instance);
                instance.dispose();
                (instance.material as THREE.Material).dispose();
            });

            // One geometry is shared by every color of the group.
            group.geometry.dispose();
        });

        this.instanceGroups = new Map();

        this.coll2mesh.forEach((desc) => {
            this.scene.remove(desc.mesh);
            desc.mesh.geometry.dispose();
            (desc.mesh.material as THREE.Material).dispose();
        });

        this.coll2mesh = new Map();
        this.coll2instance = new Map();
        this.rb2colls = new Map();
        this.colorIndex = 0;
        this.prevActive.length = 0;
        this.currActive.length = 0;
        this.dirtyInstances.length = 0;
        // Nothing has been drawn yet, so the next frame has no active set to work from.
        this.fullUpdate = true;
    }

    // applyModifications(RAPIER: RAPIER_API, world: RAPIER.World, modifications) {
    //     if (!!modifications) {
    //         modifications.addCollider.forEach(coll => {
    //             let collider = world.getCollider(coll.handle);
    //             this.addCollider(RAPIER, world, collider);
    //         });
    //         modifications.removeRigidBody.forEach(body => {
    //             if (!!this.rb2colls.get(body.handle)) {
    //                 this.rb2colls.get(body.handle).forEach(coll => this.removeCollider(coll));
    //                 this.rb2colls.delete(body.handle);
    //             }
    //         });
    //     }
    // }

    removeRigidBody(body: RAPIER.RigidBody) {
        let colls = this.rb2colls.get(body.handle);

        if (colls !== undefined) {
            // The body's own entry goes first: the collider objects it tracked are
            // gone by the time a demo calls this (`world.removeRigidBody` runs first),
            // so only their handles are still usable.
            this.rb2colls.delete(body.handle);
            colls.forEach((handle) => this.removeColliderHandle(handle));
        }
    }

    removeCollider(collider: RAPIER.Collider) {
        this.removeColliderHandle(collider.handle);
    }

    private removeColliderHandle(handle: number) {
        // Shapes drawn as their own mesh (trimesh, heightfield, capsule, …) have
        // no instance, and are removed from the scene instead.
        let meshDesc = this.coll2mesh.get(handle);

        if (meshDesc !== undefined) {
            this.scene.remove(meshDesc.mesh);
            meshDesc.mesh.geometry.dispose();
            (meshDesc.mesh.material as THREE.Material).dispose();
            this.coll2mesh.delete(handle);
            return;
        }

        let gfx = this.coll2instance.get(handle);

        if (gfx === undefined) {
            return;
        }

        let instance = this.instanceGroups.get(gfx.groupId)!.meshes[gfx.instanceId]!;
        let last = instance.count - 1;

        if (last > 0 && last !== gfx.elementId) {
            let coll2 = instance.userData.elementId2coll.get(last);
            instance.userData.elementId2coll.delete(last);
            instance.userData.elementId2coll.set(gfx.elementId, coll2);

            let gfx2 = this.coll2instance.get(coll2.handle);
            if (gfx2 !== undefined) {
                gfx2.elementId = gfx.elementId;
            }

            // The instance moved down into the freed slot carries its pose with it:
            // it is only rewritten while its body is awake, and it may well be asleep.
            instance.getMatrixAt(last, _matrix);
            instance.setMatrixAt(gfx.elementId, _matrix);
            instance.instanceMatrix.needsUpdate = true;
        }

        instance.count -= 1;
        this.coll2instance.delete(handle);
    }

    /**
     * Redraws a collider with another palette slot (see `ACCENT_COLOR`).
     *
     * The color of an instanced shape is the instanced mesh it belongs to, so
     * recoloring means dropping the collider and adding it back to another one.
     */
    setColliderColor(
        RAPIER: RAPIER_API,
        world: RAPIER.World,
        collider: RAPIER.Collider,
        colorIndex: number,
    ) {
        this.removeCollider(collider);
        this.addCollider(RAPIER, world, collider, colorIndex);
    }

    /**
     * @param colorIndex - Palette slot to draw the collider with. Defaults to the
     *                     next one in the rotation used for dynamic bodies.
     */
    addCollider(
        RAPIER: RAPIER_API,
        world: RAPIER.World,
        collider: RAPIER.Collider,
        colorIndex?: number,
    ) {
        let parent = collider.parent();

        if (colorIndex === undefined) {
            this.colorIndex = (this.colorIndex + 1) % DYNAMIC_COLORS;

            if (collider.isSensor()) {
                colorIndex = SENSOR_COLOR;
            } else {
                // A collider without a parent body is static, so color it like a fixed
                // one. The dynamic slots are the `DYNAMIC_COLORS` right after that one.
                colorIndex =
                    parent === null || parent.isFixed() ? FIXED_COLOR : this.colorIndex + 1;
            }
        }

        if (parent !== null) {
            let colls = this.rb2colls.get(parent.handle);

            if (colls === undefined) {
                this.rb2colls.set(parent.handle, [collider.handle]);
            } else if (!colls.includes(collider.handle)) {
                // A recolored collider is added back to a body that already tracks it.
                colls.push(collider.handle);
            }
        }

        let shape = instancedShape(RAPIER, collider);

        if (shape === null) {
            this.addShapeMesh(RAPIER, collider, colorIndex);
            return;
        }

        let group = this.groupFor(shape);
        let instance = this.meshFor(group, colorIndex);

        // An instanced mesh ignores writes past the capacity it was built with, so
        // it has to be replaced by a bigger one before it fills up.
        if (instance.count == instance.instanceMatrix.count) {
            instance = this.growInstances(shape.key, colorIndex);
        }

        let instanceDesc: InstanceDesc = {
            groupId: shape.key,
            instanceId: colorIndex,
            elementId: instance.count,
            highlighted: false,
            scale: shape.scale,
            frameId: -1,
        };

        instance.userData.elementId2coll.set(instance.count, collider);
        instance.count += 1;

        let highlight = group.meshes[this.highlightInstanceId()];

        if (highlight !== undefined) {
            highlight.count = 0;
        }

        // Seeded here rather than left to the next frame: a collider added to a body
        // that is asleep (or fixed) is never reported active, so nothing would come
        // back to place it.
        collider.translation(_translation);
        collider.rotation(_rotation);
        _position.set(_translation.x, _translation.y, _translation.z);
        _quaternion.set(_rotation.x, _rotation.y, _rotation.z, _rotation.w);
        _matrix.compose(_position, _quaternion, instanceDesc.scale);
        instance.setMatrixAt(instanceDesc.elementId, _matrix);
        instance.instanceMatrix.needsUpdate = true;

        this.coll2instance.set(collider.handle, instanceDesc);
    }

    /** Draws the shapes that can't be instanced: heightfields, voxels, compounds, big meshes. */
    private addShapeMesh(RAPIER: RAPIER_API, collider: RAPIER.Collider, colorIndex: number) {
        let vertices;
        let indices;

        switch (collider.shapeType()) {
            case RAPIER.ShapeType.HeightField: {
                let g = genHeightfieldGeometry(collider);
                vertices = g.vertices;
                indices = g.indices;
                break;
            }
            case RAPIER.ShapeType.Voxels: {
                let g = genVoxelsGeometry(collider);
                vertices = g.vertices;
                indices = g.indices;
                break;
            }
            case RAPIER.ShapeType.Compound: {
                // Compounds have no vertex buffer of their own; the mesh is
                // built by flattening their sub-shapes.
                let g = genCompoundGeometry(collider);
                vertices = g.vertices;
                indices = g.indices;
                break;
            }
            case RAPIER.ShapeType.TriMesh:
            case RAPIER.ShapeType.ConvexPolyhedron:
            case RAPIER.ShapeType.RoundConvexPolyhedron: {
                // Only the meshes too big to be worth instancing reach this.
                vertices = collider.vertices()!;
                indices = collider.indices();
                break;
            }
            default:
                console.log("Unknown shape to render: ", collider.shapeType());
                return;
        }

        // `Collider.indices()` is undefined for shapes that aren't indexed.
        if (indices === undefined) {
            console.log("Shape has no index buffer to render: ", collider.shapeType());
            return;
        }

        let geometry = new THREE.BufferGeometry();
        // Straight from the index buffer: `Array.from` on the million-index meshes
        // some scenes build would be far more expensive than the draw itself.
        geometry.setIndex(new THREE.BufferAttribute(indices, 1));
        geometry.setAttribute("position", new THREE.BufferAttribute(vertices, 3));
        this.addMesh(collider, geometry, colorIndex);
    }

    /** Draws a collider with a mesh of its own, for the shapes that aren't instanced. */
    private addMesh(collider: RAPIER.Collider, geometry: THREE.BufferGeometry, colorIndex: number) {
        let mesh = new THREE.Mesh(geometry, this.material(colorIndex, true));
        // As in `addCollider`: place it now, since a collider on a sleeping or fixed
        // body is never reported active for `updatePositions` to pick up.
        collider.translation(_translation);
        collider.rotation(_rotation);
        mesh.position.set(_translation.x, _translation.y, _translation.z);
        mesh.quaternion.set(_rotation.x, _rotation.y, _rotation.z, _rotation.w);
        mesh.updateMatrix();
        this.scene.add(mesh);
        this.coll2mesh.set(collider.handle, {mesh, frameId: -1});
    }
}
