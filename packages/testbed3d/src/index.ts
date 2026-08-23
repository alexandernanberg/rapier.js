import type * as RAPIER_NS from "@alexandernanberg/rapier3d";
import * as CCD from "./demos/ccd";
import * as CharacterController from "./demos/characterController";
import * as CollisionGroups from "./demos/collisionGroups";
import * as CompoundShapes from "./demos/compoundShapes";
import * as ConvexDecomposition from "./demos/convexDecomposition";
import * as ConvexPolyhedron from "./demos/convexPolyhedron";
import * as Damping from "./demos/damping";
import * as Fountain from "./demos/fountain";
import * as glbToConvexHull from "./demos/glbtoConvexHull";
import * as glbToTrimesh from "./demos/glbToTrimesh";
import * as Heightfield from "./demos/heightfield";
import * as Joints from "./demos/joints";
import * as Keva from "./demos/keva";
import * as KinematicBodies from "./demos/kinematicBodies";
import * as LockedRotations from "./demos/lockedRotations";
import * as PidController from "./demos/pidController";
import * as Platform from "./demos/platform";
import * as Pyramid from "./demos/pyramid";
import * as Trimesh from "./demos/trimesh";
import * as Voxels from "./demos/voxels";
import {Testbed} from "./Testbed";

void import("@alexandernanberg/rapier3d/compat").then(async (compat) => {
    // The testbed types against the package root but loads the `compat` build so the
    // WASM comes in inline. tsdown emits an independent declaration file per entry
    // point, so the two describe the same classes as nominally distinct types (their
    // private fields collide). They are the same API built twice.
    const RAPIER = compat as unknown as typeof RAPIER_NS;

    await RAPIER.init();
    const builders = new Map([
        ["collision groups", CollisionGroups.initWorld],
        ["character controller", CharacterController.initWorld],
        ["compound shapes", CompoundShapes.initWorld],
        ["convex decomposition", ConvexDecomposition.initWorld],
        ["convex polyhedron", ConvexPolyhedron.initWorld],
        ["CCD", CCD.initWorld],
        ["damping", Damping.initWorld],
        ["fountain", Fountain.initWorld],
        ["heightfield", Heightfield.initWorld],
        ["joints", Joints.initWorld],
        ["keva tower", Keva.initWorld],
        ["kinematic bodies", KinematicBodies.initWorld],
        ["locked rotations", LockedRotations.initWorld],
        ["pid controller", PidController.initWorld],
        ["platform", Platform.initWorld],
        ["pyramid", Pyramid.initWorld],
        ["triangle mesh", Trimesh.initWorld],
        ["voxels", Voxels.initWorld],
        ["GLTF to convexHull", glbToConvexHull.initWorld],
        ["GLTF to trimesh", glbToTrimesh.initWorld],
    ]);
    const testbed = new Testbed(RAPIER, builders);
    testbed.run();
});
