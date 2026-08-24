import RAPIER from "@alexandernanberg/rapier3d/compat";

/** Fresh zeroed targets for the required-target getter API. */
export const _v = () => ({x: 0, y: 0, z: 0});
export const _q = () => ({x: 0, y: 0, z: 0, w: 1});
export const _sdp = () => new RAPIER.SdpMatrix3();
