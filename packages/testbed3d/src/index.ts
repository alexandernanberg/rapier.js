import * as CCD from "./demos/ccd";
import * as CharacterController from "./demos/characterController";
import * as CollisionGroups from "./demos/collisionGroups";
import * as CompoundShapes from "./demos/compoundShapes";
import * as ConvexDecomposition from "./demos/convexDecomposition";
import * as ConvexPolyhedron from "./demos/convexPolyhedron";
import * as Damping from "./demos/damping";
import * as Domino from "./demos/domino";
import * as Fountain from "./demos/fountain";
import * as glbToConvexHull from "./demos/glbtoConvexHull";
import * as glbToTrimesh from "./demos/glbToTrimesh";
import * as Heightfield from "./demos/heightfield";
import * as JointMotorPosition from "./demos/jointMotorPosition";
import * as Joints from "./demos/joints";
import * as Keva from "./demos/keva";
import * as KinematicBodies from "./demos/kinematicBodies";
import * as LockedRotations from "./demos/lockedRotations";
import * as NewtonCradle from "./demos/newtonCradle";
import * as OneWayPlatforms from "./demos/oneWayPlatforms";
import * as PidController from "./demos/pidController";
import * as Platform from "./demos/platform";
import * as Primitives from "./demos/primitives";
import * as Pyramid from "./demos/pyramid";
import * as Restitution from "./demos/restitution";
import * as RopeJoints from "./demos/ropeJoints";
import * as Sensor from "./demos/sensor";
import * as SpringJoints from "./demos/springJoints";
import * as Trimesh from "./demos/trimesh";
import * as VehicleController from "./demos/vehicleController";
import * as VehicleJoints from "./demos/vehicleJoints";
import * as Voxels from "./demos/voxels";
import {Testbed} from "./Testbed";

import("@alexandernanberg/rapier3d/compat").then(async (compat) => {
    // The testbed types against the package root but loads the `compat` build so the
    // WASM comes in inline. tsdown emits an independent declaration file per entry
    // point, so the two describe the same classes as nominally distinct types (their
    // private fields collide). They are the same API built twice.
    const RAPIER = compat as unknown as typeof import("@alexandernanberg/rapier3d");

    await RAPIER.init();
    let builders = new Map([
        ["collision groups", CollisionGroups.initWorld],
        ["character controller", CharacterController.initWorld],
        ["compound shapes", CompoundShapes.initWorld],
        ["convex decomposition", ConvexDecomposition.initWorld],
        ["convex polyhedron", ConvexPolyhedron.initWorld],
        ["CCD", CCD.initWorld],
        ["damping", Damping.initWorld],
        ["domino", Domino.initWorld],
        ["fountain", Fountain.initWorld],
        ["heightfield", Heightfield.initWorld],
        ["joint motor position", JointMotorPosition.initWorld],
        ["joints", Joints.initWorld],
        ["keva tower", Keva.initWorld],
        ["kinematic bodies", KinematicBodies.initWorld],
        ["locked rotations", LockedRotations.initWorld],
        ["newton cradle", NewtonCradle.initWorld],
        ["one-way platforms", OneWayPlatforms.initWorld],
        ["pid controller", PidController.initWorld],
        ["platform", Platform.initWorld],
        ["primitives", Primitives.initWorld],
        ["pyramid", Pyramid.initWorld],
        ["restitution", Restitution.initWorld],
        ["rope joints", RopeJoints.initWorld],
        ["sensor", Sensor.initWorld],
        ["spring joints", SpringJoints.initWorld],
        ["triangle mesh", Trimesh.initWorld],
        ["vehicle controller", VehicleController.initWorld],
        ["vehicle joints", VehicleJoints.initWorld],
        ["voxels", Voxels.initWorld],
        ["GLTF to convexHull", glbToConvexHull.initWorld],
        ["GLTF to trimesh", glbToTrimesh.initWorld],
    ]);
    let testbed = new Testbed(RAPIER, builders);
    testbed.run();
});
