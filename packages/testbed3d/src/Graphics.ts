import RAPIER from "@alexandernanberg/rapier3d";
import * as THREE from "three";
import {OrbitControls} from "three/addons/controls/OrbitControls.js";

const BOX_INSTANCE_INDEX = 0;
const BALL_INSTANCE_INDEX = 1;
const CYLINDER_INSTANCE_INDEX = 2;
const CONE_INSTANCE_INDEX = 3;

var dummy = new THREE.Object3D();
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

interface InstanceDesc {
    groupId: number;
    instanceId: number;
    elementId: number;
    highlighted: boolean;
    scale?: THREE.Vector3;
    // Previous frame transform for interpolation
    prevPosition?: THREE.Vector3;
    prevQuaternion?: THREE.Quaternion;
}

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

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

function genHeightfieldGeometry(collider: RAPIER.Collider) {
    let heights = collider.heightfieldHeights();
    let nrows = collider.heightfieldNRows();
    let ncols = collider.heightfieldNCols();
    let scale = collider.heightfieldScale();

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
    instanceGroups: Array<Array<THREE.InstancedMesh>>;

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
        this.initInstances();
    }

    initInstances() {
        this.instanceGroups = [];
        this.instanceGroups.push(
            this.colorPalette.map((color) => {
                let box = new THREE.BoxGeometry(2.0, 2.0, 2.0);
                let mat = new THREE.MeshPhongMaterial({
                    color: color,
                    flatShading: true,
                });
                return new THREE.InstancedMesh(box, mat, 1000);
            }),
        );

        this.instanceGroups.push(
            this.colorPalette.map((color) => {
                let ball = new THREE.SphereGeometry(1.0);
                let mat = new THREE.MeshPhongMaterial({
                    color: color,
                    flatShading: true,
                });
                return new THREE.InstancedMesh(ball, mat, 1000);
            }),
        );

        this.instanceGroups.push(
            this.colorPalette.map((color) => {
                let cylinder = new THREE.CylinderGeometry(1.0, 1.0);
                let mat = new THREE.MeshPhongMaterial({
                    color: color,
                    flatShading: true,
                });
                return new THREE.InstancedMesh(cylinder, mat, 100);
            }),
        );

        this.instanceGroups.push(
            this.colorPalette.map((color) => {
                let cone = new THREE.ConeGeometry(1.0, 1.0);
                let mat = new THREE.MeshPhongMaterial({
                    color: color,
                    flatShading: true,
                });
                return new THREE.InstancedMesh(cone, mat, 100);
            }),
        );

        this.instanceGroups.forEach((groups) => {
            groups.forEach((instance) => {
                instance.userData.elementId2coll = new Map();
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
            let buffers = world.debugRender();
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
            let desc = this.coll2instance.get(this.highlightedCollider);

            if (!!desc) {
                desc.highlighted = false;
                this.instanceGroups[desc.groupId][this.highlightInstanceId()].count = 0;
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

    updatePositions(world: RAPIER.World, alpha: number = 1) {
        world.forEachCollider((elt) => {
            let gfx = this.coll2instance.get(elt.handle);
            elt.translation(_translation);
            elt.rotation(_rotation);

            // Current physics state
            _position.set(_translation.x, _translation.y, _translation.z);
            _quaternion.set(_rotation.x, _rotation.y, _rotation.z, _rotation.w);

            if (!!gfx) {
                let instance = this.instanceGroups[gfx.groupId][gfx.instanceId];

                // Initialize previous state if not set
                if (!gfx.prevPosition) {
                    gfx.prevPosition = _position.clone();
                    gfx.prevQuaternion = _quaternion.clone();
                }

                // Interpolate between previous and current state
                _prevPosition.copy(gfx.prevPosition);
                _prevQuaternion.copy(gfx.prevQuaternion);
                _prevPosition.lerp(_position, alpha);
                _prevQuaternion.slerp(_quaternion, alpha);

                dummy.scale.set(gfx.scale.x, gfx.scale.y, gfx.scale.z);
                dummy.position.copy(_prevPosition);
                dummy.quaternion.copy(_prevQuaternion);
                dummy.updateMatrix();
                instance.setMatrixAt(gfx.elementId, dummy.matrix);

                let highlightInstance =
                    this.instanceGroups[gfx.groupId][this.highlightInstanceId()];
                if (gfx.highlighted) {
                    highlightInstance.count = 1;
                    highlightInstance.setMatrixAt(0, dummy.matrix);
                }

                instance.instanceMatrix.needsUpdate = true;
                highlightInstance.instanceMatrix.needsUpdate = true;

                // Store current state for next frame's interpolation
                gfx.prevPosition.copy(_position);
                gfx.prevQuaternion.copy(_quaternion);
            }

            let mesh = this.coll2mesh.get(elt.handle);

            if (!!mesh) {
                // For meshes, also interpolate
                if (!mesh.userData.prevPosition) {
                    mesh.userData.prevPosition = _position.clone();
                    mesh.userData.prevQuaternion = _quaternion.clone();
                }

                _prevPosition.copy(mesh.userData.prevPosition);
                _prevQuaternion.copy(mesh.userData.prevQuaternion);
                _prevPosition.lerp(_position, alpha);
                _prevQuaternion.slerp(_quaternion, alpha);

                mesh.position.copy(_prevPosition);
                mesh.quaternion.copy(_prevQuaternion);
                mesh.updateMatrix();

                // Store current state for next frame
                mesh.userData.prevPosition.copy(_position);
                mesh.userData.prevQuaternion.copy(_quaternion);
            }
        });
    }

    reset() {
        this.instanceGroups.forEach((groups) => {
            groups.forEach((instance) => {
                instance.userData.elementId2coll = new Map();
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
        if (!!this.rb2colls.get(body.handle)) {
            this.rb2colls.get(body.handle).forEach((coll) => this.removeCollider(coll));
            this.rb2colls.delete(body.handle);
        }
    }

    removeCollider(collider: RAPIER.Collider) {
        let gfx = this.coll2instance.get(collider.handle);
        let instance = this.instanceGroups[gfx.groupId][gfx.instanceId];

        if (instance.count > 1) {
            let coll2 = instance.userData.elementId2coll.get(instance.count - 1);
            instance.userData.elementId2coll.delete(instance.count - 1);
            instance.userData.elementId2coll.set(gfx.elementId, coll2);

            let gfx2 = this.coll2instance.get(coll2.handle);
            gfx2.elementId = gfx.elementId;
        }

        instance.count -= 1;
        this.coll2instance.delete(collider.handle);
    }

    addCollider(RAPIER: RAPIER_API, world: RAPIER.World, collider: RAPIER.Collider) {
        this.colorIndex = (this.colorIndex + 1) % (this.colorPalette.length - 2);
        let parent = collider.parent();
        if (!this.rb2colls.get(parent.handle)) {
            this.rb2colls.set(parent.handle, [collider]);
        } else {
            this.rb2colls.get(parent.handle).push(collider);
        }

        let instance;
        let instanceDesc: InstanceDesc = {
            groupId: 0,
            instanceId: parent.isFixed() ? 0 : this.colorIndex + 1,
            elementId: 0,
            highlighted: false,
        };

        switch (collider.shapeType()) {
            case RAPIER.ShapeType.Cuboid:
                let hext = collider.halfExtents();
                instance = this.instanceGroups[BOX_INSTANCE_INDEX][instanceDesc.instanceId];
                instanceDesc.groupId = BOX_INSTANCE_INDEX;
                instanceDesc.scale = new THREE.Vector3(hext.x, hext.y, hext.z);
                break;
            case RAPIER.ShapeType.Ball:
                let rad = collider.radius();
                instance = this.instanceGroups[BALL_INSTANCE_INDEX][instanceDesc.instanceId];
                instanceDesc.groupId = BALL_INSTANCE_INDEX;
                instanceDesc.scale = new THREE.Vector3(rad, rad, rad);
                break;
            case RAPIER.ShapeType.Cylinder:
            case RAPIER.ShapeType.RoundCylinder:
                let cyl_rad = collider.radius();
                let cyl_height = collider.halfHeight() * 2.0;
                instance = this.instanceGroups[CYLINDER_INSTANCE_INDEX][instanceDesc.instanceId];
                instanceDesc.groupId = CYLINDER_INSTANCE_INDEX;
                instanceDesc.scale = new THREE.Vector3(cyl_rad, cyl_height, cyl_rad);
                break;
            case RAPIER.ShapeType.Cone:
                let cone_rad = collider.radius();
                let cone_height = collider.halfHeight() * 2.0;
                instance = this.instanceGroups[CONE_INSTANCE_INDEX][instanceDesc.instanceId];
                instanceDesc.groupId = CONE_INSTANCE_INDEX;
                instanceDesc.scale = new THREE.Vector3(cone_rad, cone_height, cone_rad);
                break;
            case RAPIER.ShapeType.TriMesh:
            case RAPIER.ShapeType.HeightField:
            case RAPIER.ShapeType.ConvexPolyhedron:
            case RAPIER.ShapeType.RoundConvexPolyhedron:
            case RAPIER.ShapeType.Voxels:
                let geometry = new THREE.BufferGeometry();
                let vertices;
                let indices;

                if (collider.shapeType() == RAPIER.ShapeType.HeightField) {
                    let g = genHeightfieldGeometry(collider);
                    vertices = g.vertices;
                    indices = g.indices;
                } else if (collider.shapeType() == RAPIER.ShapeType.Voxels) {
                    let g = genVoxelsGeometry(collider);
                    vertices = g.vertices;
                    indices = g.indices;
                } else {
                    vertices = collider.vertices();
                    indices = collider.indices();
                }

                geometry.setIndex(Array.from(indices));
                geometry.setAttribute("position", new THREE.BufferAttribute(vertices, 3));
                let color = parent.isFixed() ? 0 : this.colorIndex + 1;

                let material = new THREE.MeshPhongMaterial({
                    color: this.colorPalette[color],
                    side: THREE.DoubleSide,
                    flatShading: true,
                });

                let mesh = new THREE.Mesh(geometry, material);
                // Initialize previous transform for interpolation
                collider.translation(_translation);
                collider.rotation(_rotation);
                mesh.userData.prevPosition = new THREE.Vector3(
                    _translation.x,
                    _translation.y,
                    _translation.z,
                );
                mesh.userData.prevQuaternion = new THREE.Quaternion(
                    _rotation.x,
                    _rotation.y,
                    _rotation.z,
                    _rotation.w,
                );
                this.scene.add(mesh);
                this.coll2mesh.set(collider.handle, mesh);
                return;
            default:
                console.log("Unknown shape to render.");
                return;
        }

        if (!!instance) {
            instanceDesc.elementId = instance.count;
            instance.userData.elementId2coll.set(instance.count, collider);
            instance.count += 1;
        }

        let highlightInstance =
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

        // Initialize previous transform for interpolation
        instanceDesc.prevPosition = new THREE.Vector3(
            _translation.x,
            _translation.y,
            _translation.z,
        );
        instanceDesc.prevQuaternion = new THREE.Quaternion(
            _rotation.x,
            _rotation.y,
            _rotation.z,
            _rotation.w,
        );

        this.coll2instance.set(collider.handle, instanceDesc);
    }
}
