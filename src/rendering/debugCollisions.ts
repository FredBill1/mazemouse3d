import type { PhysicsBody } from "@babylonjs/core/Physics/v2/physicsBody.js";
import type { IPhysicsCollisionEvent } from "@babylonjs/core/Physics/v2/IPhysicsEnginePlugin.js";

export interface CollisionTargetMetadata {
  readonly sensorBeamTarget?: boolean;
  readonly debugCollisionRole?: "floor" | "wall";
}

const COLLISION_STARTED = "COLLISION_STARTED" as IPhysicsCollisionEvent["type"];

export function collisionTargetMetadata(
  debugCollisionRole: CollisionTargetMetadata["debugCollisionRole"],
  sensorBeamTarget = false,
): CollisionTargetMetadata {
  return {
    debugCollisionRole,
    sensorBeamTarget,
  };
}

export function isStartedRobotWallCollision(
  event: Pick<IPhysicsCollisionEvent, "type" | "collider" | "collidedAgainst">,
  robotBodies: ReadonlySet<PhysicsBody>,
): boolean {
  if (event.type !== COLLISION_STARTED) {
    return false;
  }

  const colliderIsRobot = robotBodies.has(event.collider);
  const collidedAgainstIsRobot = robotBodies.has(event.collidedAgainst);

  if (colliderIsRobot === collidedAgainstIsRobot) {
    return false;
  }

  const target = colliderIsRobot ? event.collidedAgainst : event.collider;

  return isCollisionTargetRole(target.transformNode.metadata, "wall");
}

function isCollisionTargetRole(
  metadata: unknown,
  role: CollisionTargetMetadata["debugCollisionRole"],
): boolean {
  return (
    typeof metadata === "object" &&
    metadata !== null &&
    "debugCollisionRole" in metadata &&
    (metadata as CollisionTargetMetadata).debugCollisionRole === role
  );
}
