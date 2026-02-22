import * as CCD from "./demos/ccd";
import * as CharacterController from "./demos/characterController";
import * as CollisionGroups from "./demos/collisionGroups";
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

import("@alexandernanberg/rapier3d/compat-simd").then(async (RAPIER) => {
    await RAPIER.init();
    let builders = new Map([
        ["collision groups", CollisionGroups.initWorld],
        ["character controller", CharacterController.initWorld],
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
    let testbed = new Testbed(RAPIER, builders);
    testbed.run();
});
