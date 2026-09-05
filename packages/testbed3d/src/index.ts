import * as B3dJointGrid from "./demos/b3d/jointGrid";
import * as B3dJunkyard from "./demos/b3d/junkyard";
import * as B3dLargePyramid from "./demos/b3d/largePyramid";
import * as B3dLargeWorld from "./demos/b3d/largeWorld";
import * as B3dManyPyramids from "./demos/b3d/manyPyramids";
import * as B3dTrees from "./demos/b3d/trees";
import * as B3dWasher from "./demos/b3d/washer";
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
import * as StressBalls from "./demos/stress/balls";
import * as StressBoxes from "./demos/stress/boxes";
import * as StressCapsules from "./demos/stress/capsules";
import * as StressCcd from "./demos/stress/ccd";
import * as StressCompound from "./demos/stress/compound";
import * as StressConvexPolyhedron from "./demos/stress/convexPolyhedron";
import * as StressHeightfield from "./demos/stress/heightfield";
import * as StressJointBall from "./demos/stress/jointBall";
import * as StressJointFixed from "./demos/stress/jointFixed";
import * as StressJointPrismatic from "./demos/stress/jointPrismatic";
import * as StressJointRevolute from "./demos/stress/jointRevolute";
import * as StressKeva from "./demos/stress/keva";
import * as StressManyKinematics from "./demos/stress/manyKinematics";
import * as StressManyPyramids from "./demos/stress/manyPyramids";
import * as StressManySleep from "./demos/stress/manySleep";
import * as StressManyStatic from "./demos/stress/manyStatic";
import * as StressPyramid from "./demos/stress/pyramid";
import * as StressRagdolls from "./demos/stress/ragdolls";
import * as StressRayCast from "./demos/stress/rayCast";
import * as StressRopes from "./demos/stress/ropes";
import * as StressStacks from "./demos/stress/stacks";
import * as StressTrimesh from "./demos/stress/trimesh";
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

        // Upstream's stress tests, which is why several of them repeat a scene
        // that is already above at a size meant to be measured rather than
        // watched. The biggest ones are scaled down; each says so where it is.
        ["stress: balls", StressBalls.initWorld],
        ["stress: boxes", StressBoxes.initWorld],
        ["stress: capsules", StressCapsules.initWorld],
        ["stress: CCD", StressCcd.initWorld],
        ["stress: compound", StressCompound.initWorld],
        ["stress: convex polyhedron", StressConvexPolyhedron.initWorld],
        ["stress: heightfield", StressHeightfield.initWorld],
        ["stress: joint ball", StressJointBall.initWorld],
        ["stress: joint fixed", StressJointFixed.initWorld],
        ["stress: joint prismatic", StressJointPrismatic.initWorld],
        ["stress: joint revolute", StressJointRevolute.initWorld],
        ["stress: keva tower", StressKeva.initWorld],
        ["stress: many kinematics", StressManyKinematics.initWorld],
        ["stress: many pyramids", StressManyPyramids.initWorld],
        ["stress: many sleep", StressManySleep.initWorld],
        ["stress: many static", StressManyStatic.initWorld],
        ["stress: pyramid", StressPyramid.initWorld],
        ["stress: ragdolls", StressRagdolls.initWorld],
        ["stress: ray cast", StressRayCast.initWorld],
        ["stress: ropes", StressRopes.initWorld],
        ["stress: stacks", StressStacks.initWorld],
        ["stress: triangle mesh", StressTrimesh.initWorld],

        // Ports of the box3d benchmark scenes.
        ["b3d: joint grid", B3dJointGrid.initWorld],
        ["b3d: junkyard", B3dJunkyard.initWorld],
        ["b3d: large pyramid", B3dLargePyramid.initWorld],
        ["b3d: large world", B3dLargeWorld.initWorld],
        ["b3d: many pyramids", B3dManyPyramids.initWorld],
        ["b3d: trees", B3dTrees.initWorld],
        ["b3d: washer", B3dWasher.initWorld],
    ]);
    let testbed = new Testbed(RAPIER, builders);
    testbed.run();
});
