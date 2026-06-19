import { describe, expect, it } from "vitest";
import type { PhysicsBody } from "@babylonjs/core/Physics/v2/physicsBody.js";
import type { IPhysicsCollisionEvent } from "@babylonjs/core/Physics/v2/IPhysicsEnginePlugin.js";
import { collisionTargetMetadata, isStartedRobotWallCollision } from "../rendering/debugCollisions";

type CollisionStub = Pick<IPhysicsCollisionEvent, "type" | "collider" | "collidedAgainst">;

const COLLISION_STARTED = "COLLISION_STARTED" as IPhysicsCollisionEvent["type"];
const COLLISION_CONTINUED = "COLLISION_CONTINUED" as IPhysicsCollisionEvent["type"];

describe("debug collisions", () => {
  it("counts a wheel body starting contact with a wall", () => {
    const chassis = physicsBody();
    const wheel = physicsBody();
    const wall = physicsBody(collisionTargetMetadata("wall"));

    expect(
      isStartedRobotWallCollision(
        collision(COLLISION_STARTED, wheel, wall),
        new Set([chassis, wheel]),
      ),
    ).toBe(true);
  });

  it("counts wall contacts regardless of collision body order", () => {
    const chassis = physicsBody();
    const wall = physicsBody(collisionTargetMetadata("wall"));

    expect(
      isStartedRobotWallCollision(collision(COLLISION_STARTED, wall, chassis), new Set([chassis])),
    ).toBe(true);
  });

  it("ignores floor contacts", () => {
    const chassis = physicsBody();
    const floor = physicsBody(collisionTargetMetadata("floor"));

    expect(
      isStartedRobotWallCollision(collision(COLLISION_STARTED, chassis, floor), new Set([chassis])),
    ).toBe(false);
  });

  it("ignores continued wall contacts", () => {
    const wheel = physicsBody();
    const wall = physicsBody(collisionTargetMetadata("wall"));

    expect(
      isStartedRobotWallCollision(collision(COLLISION_CONTINUED, wheel, wall), new Set([wheel])),
    ).toBe(false);
  });

  it("ignores collisions that do not include a robot body", () => {
    const wall = physicsBody(collisionTargetMetadata("wall"));
    const otherBody = physicsBody();

    expect(
      isStartedRobotWallCollision(
        collision(COLLISION_STARTED, otherBody, wall),
        new Set([physicsBody()]),
      ),
    ).toBe(false);
  });
});

function collision(
  type: IPhysicsCollisionEvent["type"],
  collider: PhysicsBody,
  collidedAgainst: PhysicsBody,
): CollisionStub {
  return {
    type,
    collider,
    collidedAgainst,
  };
}

function physicsBody(metadata?: unknown): PhysicsBody {
  return {
    transformNode: {
      metadata,
    },
  } as PhysicsBody;
}
