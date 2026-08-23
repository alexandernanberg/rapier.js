import type * as RAPIER_NS from "@alexandernanberg/rapier2d";
import * as CharacterController from "./demos/characterController";
import * as CollisionGroups from "./demos/collisionGroups";
import * as ConvexPolygons from "./demos/convexPolygons";
import * as Cubes from "./demos/cubes";
import * as Heightfield from "./demos/heightfield";
import * as Keva from "./demos/keva";
import * as LockedRotations from "./demos/lockedRotations";
import * as PidController from "./demos/pidController";
import * as Polyline from "./demos/polyline";
import * as RevoluteJoints from "./demos/revoluteJoints";
import * as Voxels from "./demos/voxels";
import {Testbed} from "./Testbed";

void import("@alexandernanberg/rapier2d/compat").then(async (compat) => {
    // The testbed types against the package root but loads the `compat` build so the
    // WASM comes in inline. tsdown emits an independent declaration file per entry
    // point, so the two describe the same classes as nominally distinct types (their
    // private fields collide). They are the same API built twice.
    const RAPIER = compat as unknown as typeof RAPIER_NS;

    await RAPIER.init();
    const builders = new Map([
        ["collision groups", CollisionGroups.initWorld],
        ["character controller", CharacterController.initWorld],
        ["convex polygons", ConvexPolygons.initWorld],
        ["cubes", Cubes.initWorld],
        ["heightfield", Heightfield.initWorld],
        ["joints: revolute", RevoluteJoints.initWorld],
        ["keva tower", Keva.initWorld],
        ["locked rotations", LockedRotations.initWorld],
        ["pid controller", PidController.initWorld],
        ["polyline", Polyline.initWorld],
        ["voxels", Voxels.initWorld],
    ]);
    const testbed = await Testbed.create(RAPIER, builders);
    testbed.run();
});
