import {bench, summary} from "mitata";

export function benchGetters(RAPIER: any, is3D: boolean, quick: boolean): void {
    const bodyCount = quick ? 1000 : 5000;

    const gravity = is3D ? {x: 0, y: -9.81, z: 0} : {x: 0, y: -9.81};
    const world = new RAPIER.World(gravity);

    const bodies: any[] = [];
    const colliders: any[] = [];

    for (let i = 0; i < bodyCount; i++) {
        const bodyDesc = RAPIER.RigidBodyDesc.dynamic();
        if (is3D) {
            bodyDesc.setTranslation(
                Math.random() * 100 - 50,
                Math.random() * 100,
                Math.random() * 100 - 50,
            );
        } else {
            bodyDesc.setTranslation(Math.random() * 100 - 50, Math.random() * 100);
        }
        const body = world.createRigidBody(bodyDesc);
        const collider = world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
        bodies.push(body);
        colliders.push(collider);
    }

    // Step once to initialize velocities
    world.step();

    // This fork requires a target on every getter; the official packages
    // allocate and take none. Each row is emitted for whichever API is present.
    const supportsTargetParam = (() => {
        try {
            const target: any = is3D ? {x: 0, y: 0, z: 0} : {x: 0, y: 0};
            return bodies[0].translation(target) === target;
        } catch {
            return false;
        }
    })();

    summary(() => {
        if (!supportsTargetParam) {
            bench(`body.translation() x${bodyCount} [alloc]`, () => {
                for (const b of bodies) b.translation();
            });
        }

        if (supportsTargetParam) {
            const translationTarget = is3D ? {x: 0, y: 0, z: 0} : {x: 0, y: 0};
            bench(`body.translation() x${bodyCount} [reuse]`, () => {
                for (const b of bodies) b.translation(translationTarget);
            });
        }
    });

    summary(() => {
        if (!supportsTargetParam) {
            bench(`body.rotation() x${bodyCount} [alloc]`, () => {
                for (const b of bodies) b.rotation();
            });
        }

        if (supportsTargetParam && is3D) {
            const rotationTarget = {x: 0, y: 0, z: 0, w: 1};
            bench(`body.rotation() x${bodyCount} [reuse]`, () => {
                for (const b of bodies) b.rotation(rotationTarget);
            });
        }
    });

    summary(() => {
        if (!supportsTargetParam) {
            bench(`body.linvel() x${bodyCount} [alloc]`, () => {
                for (const b of bodies) b.linvel();
            });
        }

        if (supportsTargetParam) {
            const linvelTarget = is3D ? {x: 0, y: 0, z: 0} : {x: 0, y: 0};
            bench(`body.linvel() x${bodyCount} [reuse]`, () => {
                for (const b of bodies) b.linvel(linvelTarget);
            });
        }
    });

    summary(() => {
        if (!supportsTargetParam) {
            bench(`collider.translation() x${bodyCount} [alloc]`, () => {
                for (const c of colliders) c.translation();
            });
        }

        if (supportsTargetParam) {
            const colliderTransTarget = is3D ? {x: 0, y: 0, z: 0} : {x: 0, y: 0};
            bench(`collider.translation() x${bodyCount} [reuse]`, () => {
                for (const c of colliders) c.translation(colliderTransTarget);
            });
        }
    });
}
