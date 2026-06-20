import { describe, expect, it } from "vitest";
import type { MazeWorkerRequest, MazeWorkerResponse } from "../domain/maze";
import type {
  NavigationControllerDebug,
  NavigationWorkerRequest,
  NavigationWorkerResponse,
} from "../domain/navigation";

describe("maze worker protocol", () => {
  it("uses request ids for generate requests", () => {
    const request: MazeWorkerRequest = {
      type: "generateMaze",
      requestId: 1,
      config: { size: 16, seed: 42 },
    };

    expect(request.type).toBe("generateMaze");
    expect(request.requestId).toBe(1);
  });

  it("can represent worker errors", () => {
    const response: MazeWorkerResponse = {
      type: "error",
      requestId: 1,
      message: "invalid maze config",
    };

    expect(response).toMatchObject({
      type: "error",
      requestId: 1,
    });
  });

  it("can represent navigation worker ticks and commands", () => {
    const request: NavigationWorkerRequest = {
      type: "tick",
      sequence: 2,
      deltaSeconds: 1 / 120,
      pose: {
        x: 0.5,
        z: 0.5,
        yaw: 0,
      },
      velocity: {
        vx: 1,
        omega: 0,
      },
    };
    const response: NavigationWorkerResponse = {
      type: "command",
      sequence: 2,
      command: {
        leftRadPerSec: 8,
        rightRadPerSec: 8,
      },
      twist: {
        linearSpeed: 0.72,
        angularSpeed: 0,
      },
      path: [{ x: 0.5, z: 0.5 }],
      pathVersion: 1,
      targetCell: 3,
      debug: navigationDebug({
        validTrajectories: 10,
        sampledTrajectories: 25,
      }),
    };

    expect(request.type).toBe("tick");
    expect(response.debug.smootherHz).toBeGreaterThan(response.debug.dwbHz);
  });
});

function navigationDebug(overrides: Partial<NavigationControllerDebug>): NavigationControllerDebug {
  return {
    dwbHz: 15,
    smootherHz: 120,
    status: "tracking",
    validTrajectories: 0,
    sampledTrajectories: 0,
    rejectedTrajectories: {
      currentPoseCollision: 0,
      rolloutCollision: 0,
      brakingCollision: 0,
      wheelSpeed: 0,
      trackability: 0,
      lowClearance: 0,
      noProgress: 0,
      noPathProjection: 0,
      nonFiniteScore: 0,
    },
    currentLinearSpeed: 1,
    currentAngularSpeed: 0,
    targetLinearSpeed: 1,
    targetAngularSpeed: 0,
    smoothedLinearSpeed: 0.72,
    smoothedAngularSpeed: 0,
    dynamicWindow: {
      currentV: 1,
      currentW: 0,
      minV: 0,
      maxV: 1.8,
      minW: -2,
      maxW: 2,
    },
    best: null,
    currentClearance: 0.2,
    currentPoseCollides: false,
    pathProgress: 0,
    pathLength: 1,
    remainingDistance: 1,
    pathTrackingError: 0,
    workerComputeMs: 0.2,
    ...overrides,
  };
}
