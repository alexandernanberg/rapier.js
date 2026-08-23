import type * as RAPIER_NS from "@alexandernanberg/rapier3d";
import RAPIER from "@alexandernanberg/rapier3d";
import * as THREE from "three";
import {OrbitControls} from "three/addons/controls/OrbitControls.js";

const BOX_INSTANCE_INDEX = 0;
const BALL_INSTANCE_INDEX = 1;
const CYLINDER_INSTANCE_INDEX = 2;
const CONE_INSTANCE_INDEX = 3;

const dummy = new THREE.Object3D();
let kk = 0;

// Scratch objects for zero-allocation getters
const _translation = {x: 0, y: 0, z: 0};
const _rotation = {x: 0, y: 0, z: 0, w: 1};

// Scratch objects for interpolation
const _position = new THREE.Vector3();
const _quaternion = new THREE.Quaternion();
const _prevPosition = new THREE.Vector3();
const _prevQuaternion = new THREE.Quaternion();
const _matrix = new THREE.Matrix4();

interface InstanceDesc {
    groupId: number;
    instanceId: number;
    elementId: number;
    highlighted: boolean;
    scale: THREE.Vector3;
    // Interpolation state, populated on the first frame the instance is drawn.
    interpolation?: Interpolation;
}

interface Interpolation {
    prevPosition: THREE.Vector3;
    prevQuaternion: THREE.Quaternion;
    snapshotPosition: THREE.Vector3;
    snapshotQuaternion: THREE.Quaternion;
}

// three.js types `Object3D.userData` as `Record<string, any>`; these say what we
// actually keep in it.
interface InstanceUserData {
    elementId2coll: Map<number, RAPIER.Collider>;
}

interface MeshUserData {
    // Interpolation state, populated on the first frame the mesh is drawn.
    interpolation?: Interpolation;
}

function instanceData(instance: THREE.InstancedMesh): InstanceUserData {
    return instance.userData as InstanceUserData;
}

type RAPIER_API = typeof RAPIER_NS;

// NOTE: this is a very naive voxels -> mesh conversion. Proper
//       conversions should use something like greedy meshing instead.
function genVoxelsGeometry(collider: RAPIER.Collider) {
    // Clear the cached shape so it gets recomputed from the source of truth,
    // and so we’ll be sure that the data contain grid coordinates even if the
    // voxels were initialized with floating points.
    collider.clearShapeCache();
    const shape = collider.shape as RAPIER.Voxels;
    const gridCoords = shape.data;
    const sz = shape.voxelSize;
    const vertices = [];
    const indices = [];

    let i: number;
    for (i = 0; i < gridCoords.length; i += 3) {
        const minx = gridCoords[i] * sz.x;
        const miny = gridCoords[i + 1] * sz.y;
        const minz = gridCoords[i + 2] * sz.z;
        const maxx = minx + sz.x;
        const maxy = miny + sz.y;
        const maxz = minz + sz.z;

        const k: number = vertices.length / 3;
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
            const ball = shape as RAPIER.Ball;
            return new THREE.SphereGeometry(ball.radius);
        }
        case RAPIER.ShapeType.Cuboid:
        case RAPIER.ShapeType.RoundCuboid: {
            const ext = (shape as RAPIER.Cuboid).halfExtents;
            return new THREE.BoxGeometry(ext.x * 2.0, ext.y * 2.0, ext.z * 2.0);
        }
        case RAPIER.ShapeType.Capsule: {
            const capsule = shape as RAPIER.Capsule;
            return new THREE.CapsuleGeometry(capsule.radius, capsule.halfHeight * 2.0);
        }
        case RAPIER.ShapeType.Cylinder:
        case RAPIER.ShapeType.RoundCylinder: {
            const cyl = shape as RAPIER.Cylinder;
            return new THREE.CylinderGeometry(cyl.radius, cyl.radius, cyl.halfHeight * 2.0);
        }
        case RAPIER.ShapeType.Cone:
        case RAPIER.ShapeType.RoundCone: {
            const cone = shape as RAPIER.Cone;
            return new THREE.ConeGeometry(cone.radius, cone.halfHeight * 2.0);
        }
        case RAPIER.ShapeType.TriMesh:
        case RAPIER.ShapeType.ConvexPolyhedron:
        case RAPIER.ShapeType.RoundConvexPolyhedron: {
            const mesh = shape as RAPIER.TriMesh;
            const geometry = new THREE.BufferGeometry();
            geometry.setIndex(Array.from(mesh.indices));
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
    const compound = collider.shape as RAPIER.Compound;
    const vertices: number[] = [];
    const indices: number[] = [];

    compound.shapes.forEach((subShape, i) => {
        const geometry = genSubShapeGeometry(subShape);

        if (!geometry) {
            console.log("Unknown compound sub-shape to render.");
            return;
        }

        const pos = compound.positions[i];
        const rot = compound.rotations[i];
        _matrix.compose(
            new THREE.Vector3(pos.x, pos.y, pos.z),
            new THREE.Quaternion(rot.x, rot.y, rot.z, rot.w),
            new THREE.Vector3(1.0, 1.0, 1.0),
        );
        geometry.applyMatrix4(_matrix);

        // Indices are local to each sub-shape, so they shift by however many
        // vertices are already in the merged buffer.
        const base = vertices.length / 3;
        const position = geometry.getAttribute("position");
        for (let k = 0; k < position.count; ++k) {
            vertices.push(position.getX(k), position.getY(k), position.getZ(k));
        }

        const index = geometry.getIndex();
        if (index) {
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
    const heights = collider.heightfieldHeights();
    const nrows = collider.heightfieldNRows();
    const ncols = collider.heightfieldNCols();
    const scale = collider.heightfieldScale();

    const vertices = [];
    const indices = [];
    const eltWX = 1.0 / nrows;
    const eltWY = 1.0 / ncols;

    let i: number;
    let j: number;
    for (j = 0; j <= ncols; ++j) {
        for (i = 0; i <= nrows; ++i) {
            const x = (j * eltWX - 0.5) * scale.x;
            const y = heights[j * (nrows + 1) + i] * scale.y;
            const z = (i * eltWY - 0.5) * scale.z;

            vertices.push(x, y, z);
        }
    }

    for (j = 0; j < ncols; ++j) {
        for (i = 0; i < nrows; ++i) {
            const i1 = (i + 0) * (ncols + 1) + (j + 0);
            const i2 = (i + 0) * (ncols + 1) + (j + 1);
            const i3 = (i + 1) * (ncols + 1) + (j + 0);
            const i4 = (i + 1) * (ncols + 1) + (j + 1);

            indices.push(i1, i3, i2);
            indices.push(i3, i4, i2);
        }
    }

    return {
        vertices: new Float32Array(vertices),
        indices: new Uint32Array(indices),
    };
}

export class Graphics {
    raycaster: THREE.Raycaster;
    highlightedCollider: null | number;
    coll2instance: Map<number, InstanceDesc>;
    coll2mesh: Map<number, THREE.Mesh>;
    rb2colls: Map<number, Array<RAPIER.Collider>>;
    colorIndex: number;
    colorPalette: Array<number>;
    scene: THREE.Scene;
    camera: THREE.PerspectiveCamera;
    renderer: THREE.WebGLRenderer;
    light: THREE.PointLight;
    lines: THREE.LineSegments;
    controls: OrbitControls;
    // Assigned by `initInstances()`, called at the end of the constructor.
    instanceGroups!: Array<Array<THREE.InstancedMesh>>;

    constructor() {
        this.raycaster = new THREE.Raycaster();
        this.highlightedCollider = null;
        this.coll2instance = new Map();
        this.coll2mesh = new Map();
        this.rb2colls = new Map();
        this.colorIndex = 0;
        this.colorPalette = [0xf3d9b1, 0x98c1d9, 0x053c5e, 0x1f7a8c, 0xff0000];
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

        const ambientLight = new THREE.AmbientLight(0x606060);
        this.scene.add(ambientLight);
        // In Three.js r155+, decay defaults to 2 for physically correct lighting.
        // Set decay to 0 to restore the old non-physically-correct behavior.
        this.light = new THREE.PointLight(0xffffff, 1, 0, 0);
        this.scene.add(this.light);

        // For the debug-renderer.
        {
            const material = new THREE.LineBasicMaterial({
                color: 0xffffff,
                vertexColors: true,
            });
            const geometry = new THREE.BufferGeometry();
            this.lines = new THREE.LineSegments(geometry, material);
            this.scene.add(this.lines);
        }
        const onWindowResize = () => {
            if (this.camera) {
                this.camera.aspect = window.innerWidth / window.innerHeight;
                this.camera.updateProjectionMatrix();
                this.renderer.setSize(window.innerWidth, window.innerHeight);
            }
        };

        window.addEventListener("resize", onWindowResize, false);

        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.2;
        this.controls.maxPolarAngle = Math.PI / 2;
        this.initInstances();
    }

    initInstances() {
        this.instanceGroups = [];
        this.instanceGroups.push(
            this.colorPalette.map((color) => {
                const box = new THREE.BoxGeometry(2.0, 2.0, 2.0);
                const mat = new THREE.MeshPhongMaterial({
                    color,
                    flatShading: true,
                });
                return new THREE.InstancedMesh(box, mat, 1000);
            }),
        );

        this.instanceGroups.push(
            this.colorPalette.map((color) => {
                const ball = new THREE.SphereGeometry(1.0);
                const mat = new THREE.MeshPhongMaterial({
                    color,
                    flatShading: true,
                });
                return new THREE.InstancedMesh(ball, mat, 1000);
            }),
        );

        this.instanceGroups.push(
            this.colorPalette.map((color) => {
                const cylinder = new THREE.CylinderGeometry(1.0, 1.0);
                const mat = new THREE.MeshPhongMaterial({
                    color,
                    flatShading: true,
                });
                return new THREE.InstancedMesh(cylinder, mat, 100);
            }),
        );

        this.instanceGroups.push(
            this.colorPalette.map((color) => {
                const cone = new THREE.ConeGeometry(1.0, 1.0);
                const mat = new THREE.MeshPhongMaterial({
                    color,
                    flatShading: true,
                });
                return new THREE.InstancedMesh(cone, mat, 100);
            }),
        );

        this.instanceGroups.forEach((groups) => {
            groups.forEach((instance) => {
                instanceData(instance).elementId2coll = new Map();
                instance.count = 0;
                instance.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
                this.scene.add(instance);
            });
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
            const buffers = world.debugRender();
            this.lines.visible = true;
            this.lines.geometry.setAttribute(
                "position",
                new THREE.BufferAttribute(buffers.vertices, 3),
            );
            this.lines.geometry.setAttribute("color", new THREE.BufferAttribute(buffers.colors, 4));
        } else {
            this.lines.visible = false;
        }

        this.updatePositions(world, alpha);
        this.renderer.render(this.scene, this.camera);
    }

    rayAtMousePosition(pos: {x: number; y: number}) {
        this.raycaster.setFromCamera(new THREE.Vector2(pos.x, pos.y), this.camera);
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
        return this.colorPalette.length - 1;
    }

    highlightCollider(handle: number) {
        if (handle == this.highlightedCollider)
            // Avoid flickering when moving the mouse on a single collider.
            return;

        if (this.highlightedCollider != null) {
            const desc = this.coll2instance.get(this.highlightedCollider);

            if (desc) {
                desc.highlighted = false;
                this.instanceGroups[desc.groupId][this.highlightInstanceId()].count = 0;
            }
        }
        if (handle != null) {
            const desc = this.coll2instance.get(handle);

            if (desc) {
                if (desc.instanceId != 0)
                    // Don't highlight static/kinematic bodies.
                    desc.highlighted = true;
            }
        }
        this.highlightedCollider = handle;
    }

    updatePositions(world: RAPIER.World, alpha: number = 1) {
        world.forEachCollider((elt) => {
            const gfx = this.coll2instance.get(elt.handle);
            elt.translation(_translation);
            elt.rotation(_rotation);

            _position.set(_translation.x, _translation.y, _translation.z);
            _quaternion.set(_rotation.x, _rotation.y, _rotation.z, _rotation.w);

            if (gfx) {
                const instance = this.instanceGroups[gfx.groupId][gfx.instanceId];

                let interp = gfx.interpolation;
                if (interp === undefined) {
                    interp = {
                        prevPosition: _position.clone(),
                        prevQuaternion: _quaternion.clone(),
                        snapshotPosition: _position.clone(),
                        snapshotQuaternion: _quaternion.clone(),
                    };
                    gfx.interpolation = interp;
                } else {
                    interp.prevPosition.copy(interp.snapshotPosition);
                    interp.prevQuaternion.copy(interp.snapshotQuaternion);
                }

                _prevPosition.copy(interp.prevPosition);
                _prevQuaternion.copy(interp.prevQuaternion);
                _prevPosition.lerp(_position, alpha);
                _prevQuaternion.slerp(_quaternion, alpha);

                dummy.scale.set(gfx.scale.x, gfx.scale.y, gfx.scale.z);
                dummy.position.copy(_prevPosition);
                dummy.quaternion.copy(_prevQuaternion);
                dummy.updateMatrix();
                instance.setMatrixAt(gfx.elementId, dummy.matrix);

                const highlightInstance =
                    this.instanceGroups[gfx.groupId][this.highlightInstanceId()];
                if (gfx.highlighted) {
                    highlightInstance.count = 1;
                    highlightInstance.setMatrixAt(0, dummy.matrix);
                }

                instance.instanceMatrix.needsUpdate = true;
                highlightInstance.instanceMatrix.needsUpdate = true;

                interp.snapshotPosition.copy(_position);
                interp.snapshotQuaternion.copy(_quaternion);
            }

            const mesh = this.coll2mesh.get(elt.handle);

            if (mesh) {
                const userData = mesh.userData as MeshUserData;

                let interp = userData.interpolation;
                if (interp === undefined) {
                    interp = {
                        prevPosition: _position.clone(),
                        prevQuaternion: _quaternion.clone(),
                        snapshotPosition: _position.clone(),
                        snapshotQuaternion: _quaternion.clone(),
                    };
                    userData.interpolation = interp;
                } else {
                    interp.prevPosition.copy(interp.snapshotPosition);
                    interp.prevQuaternion.copy(interp.snapshotQuaternion);
                }

                _prevPosition.copy(interp.prevPosition);
                _prevQuaternion.copy(interp.prevQuaternion);
                _prevPosition.lerp(_position, alpha);
                _prevQuaternion.slerp(_quaternion, alpha);

                mesh.position.copy(_prevPosition);
                mesh.quaternion.copy(_prevQuaternion);
                mesh.updateMatrix();

                interp.snapshotPosition.copy(_position);
                interp.snapshotQuaternion.copy(_quaternion);
            }
        });
    }

    reset() {
        this.instanceGroups.forEach((groups) => {
            groups.forEach((instance) => {
                instanceData(instance).elementId2coll = new Map();
                instance.count = 0;
            });
        });

        this.coll2mesh.forEach((mesh) => {
            this.scene.remove(mesh);
        });

        this.coll2instance = new Map();
        this.rb2colls = new Map();
        this.colorIndex = 0;
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
        const colls = this.rb2colls.get(body.handle);

        if (colls !== undefined) {
            colls.forEach((coll) => this.removeCollider(coll));
            this.rb2colls.delete(body.handle);
        }
    }

    removeCollider(collider: RAPIER.Collider) {
        const gfx = this.coll2instance.get(collider.handle);

        // Shapes drawn as their own mesh (trimesh, heightfield, …) have no instance.
        if (gfx === undefined) {
            return;
        }

        const instance = this.instanceGroups[gfx.groupId][gfx.instanceId];

        if (instance.count > 1) {
            const elementId2coll = instanceData(instance).elementId2coll;
            const coll2 = elementId2coll.get(instance.count - 1);
            elementId2coll.delete(instance.count - 1);

            if (coll2 !== undefined) {
                elementId2coll.set(gfx.elementId, coll2);

                const gfx2 = this.coll2instance.get(coll2.handle);
                if (gfx2 !== undefined) {
                    gfx2.elementId = gfx.elementId;
                }
            }
        }

        instance.count -= 1;
        this.coll2instance.delete(collider.handle);
    }

    addCollider(rapier: RAPIER_API, world: RAPIER.World, collider: RAPIER.Collider) {
        this.colorIndex = (this.colorIndex + 1) % (this.colorPalette.length - 2);
        const parent = collider.parent();

        if (parent !== null) {
            const colls = this.rb2colls.get(parent.handle);

            if (colls === undefined) {
                this.rb2colls.set(parent.handle, [collider]);
            } else {
                colls.push(collider);
            }
        }

        let instance;
        const instanceDesc: InstanceDesc = {
            groupId: 0,
            // A collider without a parent body is static, so colour it like a fixed one.
            instanceId: parent === null || parent.isFixed() ? 0 : this.colorIndex + 1,
            elementId: 0,
            highlighted: false,
            scale: new THREE.Vector3(1.0, 1.0, 1.0),
        };

        switch (collider.shapeType()) {
            case rapier.ShapeType.Cuboid: {
                const hext = collider.halfExtents();
                instance = this.instanceGroups[BOX_INSTANCE_INDEX][instanceDesc.instanceId];
                instanceDesc.groupId = BOX_INSTANCE_INDEX;
                instanceDesc.scale = new THREE.Vector3(hext.x, hext.y, hext.z);
                break;
            }
            case rapier.ShapeType.Ball: {
                const rad = collider.radius();
                instance = this.instanceGroups[BALL_INSTANCE_INDEX][instanceDesc.instanceId];
                instanceDesc.groupId = BALL_INSTANCE_INDEX;
                instanceDesc.scale = new THREE.Vector3(rad, rad, rad);
                break;
            }
            case rapier.ShapeType.Cylinder:
            case rapier.ShapeType.RoundCylinder: {
                const cyl_rad = collider.radius();
                const cyl_height = collider.halfHeight() * 2.0;
                instance = this.instanceGroups[CYLINDER_INSTANCE_INDEX][instanceDesc.instanceId];
                instanceDesc.groupId = CYLINDER_INSTANCE_INDEX;
                instanceDesc.scale = new THREE.Vector3(cyl_rad, cyl_height, cyl_rad);
                break;
            }
            case rapier.ShapeType.Cone: {
                const cone_rad = collider.radius();
                const cone_height = collider.halfHeight() * 2.0;
                instance = this.instanceGroups[CONE_INSTANCE_INDEX][instanceDesc.instanceId];
                instanceDesc.groupId = CONE_INSTANCE_INDEX;
                instanceDesc.scale = new THREE.Vector3(cone_rad, cone_height, cone_rad);
                break;
            }
            case rapier.ShapeType.TriMesh:
            case rapier.ShapeType.HeightField:
            case rapier.ShapeType.ConvexPolyhedron:
            case rapier.ShapeType.RoundConvexPolyhedron:
            case rapier.ShapeType.Voxels:
            case rapier.ShapeType.Compound: {
                const geometry = new THREE.BufferGeometry();
                let vertices;
                let indices;

                if (collider.shapeType() == rapier.ShapeType.HeightField) {
                    const g = genHeightfieldGeometry(collider);
                    vertices = g.vertices;
                    indices = g.indices;
                } else if (collider.shapeType() == rapier.ShapeType.Voxels) {
                    const g = genVoxelsGeometry(collider);
                    vertices = g.vertices;
                    indices = g.indices;
                } else if (collider.shapeType() == rapier.ShapeType.Compound) {
                    // Compounds have no vertex buffer of their own; the mesh is
                    // built by flattening their sub-shapes.
                    const g = genCompoundGeometry(collider);
                    vertices = g.vertices;
                    indices = g.indices;
                } else {
                    vertices = collider.vertices();
                    indices = collider.indices();
                }

                // `Collider.indices()` is undefined for shapes that aren't indexed.
                if (indices === undefined) {
                    console.log("Shape has no index buffer to render: ", collider.shapeType());
                    return;
                }

                geometry.setIndex(Array.from(indices));
                geometry.setAttribute("position", new THREE.BufferAttribute(vertices, 3));
                const color = parent !== null && !parent.isFixed() ? this.colorIndex + 1 : 0;

                const material = new THREE.MeshPhongMaterial({
                    color: this.colorPalette[color],
                    side: THREE.DoubleSide,
                    flatShading: true,
                });

                const mesh = new THREE.Mesh(geometry, material);
                this.scene.add(mesh);
                this.coll2mesh.set(collider.handle, mesh);
                return;
            }
            default:
                console.log("Unknown shape to render.");
                return;
        }

        if (instance) {
            instanceDesc.elementId = instance.count;
            instanceData(instance).elementId2coll.set(instance.count, collider);
            instance.count += 1;
        }

        const highlightInstance =
            this.instanceGroups[instanceDesc.groupId][this.highlightInstanceId()];
        highlightInstance.count = 0;

        collider.translation(_translation);
        collider.rotation(_rotation);
        dummy.position.set(_translation.x, _translation.y, _translation.z);
        dummy.quaternion.set(_rotation.x, _rotation.y, _rotation.z, _rotation.w);
        dummy.scale.set(instanceDesc.scale.x, instanceDesc.scale.y, instanceDesc.scale.z);
        dummy.updateMatrix();
        instance.setMatrixAt(instanceDesc.elementId, dummy.matrix);
        instance.instanceMatrix.needsUpdate = true;

        this.coll2instance.set(collider.handle, instanceDesc);
    }
}
