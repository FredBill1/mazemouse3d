import { afterEach, describe, expect, it, vi } from "vitest";
import { WALL_BITS, type MazeSnapshot } from "../domain/maze";
import {
  DwbMotorDriver,
  defaultNavigationConfig,
  type NavigationWorkerLike,
} from "../rendering/dwbMotorDriver";
import type {
  NavigationControllerDebug,
  NavigationWorkerRequest,
  NavigationWorkerResponse,
} from "../domain/navigation";

class FakeNavigationWorker implements NavigationWorkerLike {
  readonly postMessage = vi.fn((message: NavigationWorkerRequest) => {
    this.messages.push(message);
  });
  readonly terminate = vi.fn();
  readonly addEventListener = vi.fn(
    (_type: "message", listener: (event: MessageEvent<NavigationWorkerResponse>) => void) => {
      this.listener = listener;
    },
  );
  readonly removeEventListener = vi.fn();
  readonly messages: NavigationWorkerRequest[] = [];
  listener: ((event: MessageEvent<NavigationWorkerResponse>) => void) | null = null;

  emit(response: NavigationWorkerResponse): void {
    this.listener?.({ data: response } as MessageEvent<NavigationWorkerResponse>);
  }
}

describe("dwb motor driver", () => {
  afterEach(() => {
    vi.restoreAllMocks();
  });

  it("initializes the navigation worker with a copied maze and DWB config", () => {
    const worker = new FakeNavigationWorker();
    const driver = new DwbMotorDriver(openMaze(), {
      workerFactory: () => worker,
      now: () => 0,
    });

    expect(worker.messages[0]).toMatchObject({
      type: "init",
      size: 2,
      goals: [0, 1, 2, 3],
      seed: 7,
      config: {
        dwbFrequency: 15,
      },
    });

    driver.dispose();
  });

  it("returns stopped until the worker publishes a command", () => {
    const worker = new FakeNavigationWorker();
    const driver = new DwbMotorDriver(openMaze(), {
      workerFactory: () => worker,
      now: () => 0,
    });

    expect(driver.next(1 / 120, sampleAt(0.5, 0.5, 0))).toEqual({
      leftRadPerSec: 0,
      rightRadPerSec: 0,
    });

    driver.dispose();
  });

  it("uses the latest DWB target and clamps generated wheel speed", () => {
    let now = 10;
    const worker = new FakeNavigationWorker();
    const driver = new DwbMotorDriver(openMaze(), {
      workerFactory: () => worker,
      now: () => now,
    });

    driver.next(1 / 120, sampleAt(0.5, 0.5, 0));
    worker.emit({
      type: "command",
      sequence: 1,
      command: {
        leftRadPerSec: 999,
        rightRadPerSec: 4,
      },
      twist: {
        linearSpeed: 1,
        angularSpeed: 0.2,
      },
      path: [{ x: 0.5, z: 0.5 }],
      pathVersion: 2,
      targetCell: 1,
      debug: debugResponse({
        targetLinearSpeed: 99,
        validTrajectories: 12,
        sampledTrajectories: 25,
      }),
    });

    now = 20;
    expect(driver.next(20, sampleAt(0.5, 0.5, 0))).toEqual({
      leftRadPerSec: defaultNavigationConfig().maxWheelRadPerSec,
      rightRadPerSec: defaultNavigationConfig().maxWheelRadPerSec,
    });
    expect(driver.pathVersion).toBe(2);
    expect(driver.pathPoints).toEqual([{ x: 0.5, z: 0.5 }]);
    expect(driver.debugSnapshot).toMatchObject({
      dwbHz: 15,
      status: "tracking",
      targetCell: 1,
    });
    expect(driver.debugSnapshot.smootherHz).toBeGreaterThan(0);

    driver.dispose();
  });

  it("continues smoothing the latest DWB target while waiting for the worker", () => {
    let now = 0;
    const worker = new FakeNavigationWorker();
    const driver = new DwbMotorDriver(openMaze(), {
      workerFactory: () => worker,
      now: () => now,
    });

    driver.next(1 / 120, sampleAt(0.5, 0.5, 0));
    worker.emit({
      type: "command",
      sequence: 1,
      command: {
        leftRadPerSec: 5,
        rightRadPerSec: 5,
      },
      twist: {
        linearSpeed: 0.4,
        angularSpeed: 0,
      },
      path: [],
      pathVersion: 0,
      targetCell: null,
      debug: debugResponse({
        targetLinearSpeed: 0.4,
        validTrajectories: 4,
        sampledTrajectories: 10,
      }),
    });

    now = 2500;

    expect(driver.next(0.25, sampleAt(0.5, 0.5, 0))).toEqual({
      leftRadPerSec: 0.4 / defaultNavigationConfig().wheelRadius,
      rightRadPerSec: 0.4 / defaultNavigationConfig().wheelRadius,
    });

    driver.dispose();
  });
});

function sampleAt(x: number, z: number, yaw: number) {
  return {
    pose: {
      origin: { x, z },
      yaw,
    },
    velocity: {
      vx: 0,
      omega: 0,
    },
  };
}

function debugResponse(
  overrides: Partial<NavigationControllerDebug> = {},
): NavigationControllerDebug {
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
    currentLinearSpeed: 0,
    currentAngularSpeed: 0,
    targetLinearSpeed: 0,
    targetAngularSpeed: 0,
    smoothedLinearSpeed: 0,
    smoothedAngularSpeed: 0,
    dynamicWindow: {
      currentV: 0,
      currentW: 0,
      minV: 0,
      maxV: 1,
      minW: -1,
      maxW: 1,
    },
    best: null,
    currentClearance: Number.POSITIVE_INFINITY,
    currentPoseCollides: false,
    pathProgress: 0,
    pathLength: 0,
    remainingDistance: 0,
    pathTrackingError: 0,
    workerComputeMs: 0.2,
    ...overrides,
  };
}

function openMaze(): MazeSnapshot {
  return {
    size: 2,
    seed: 7,
    iterations: 0,
    initialTemp: 1,
    finalTemp: 1,
    start: 0,
    goals: [0, 1, 2, 3],
    walls: Uint8Array.from([15 & ~WALL_BITS.EAST, 15 & ~WALL_BITS.WEST, 15, 15]),
    solution: [0],
    config: {
      size: 2,
      seed: 7,
      iterations: 0,
      initialTemp: 1,
      finalTemp: 1,
    },
    scoreHistory: [],
    metrics: {
      score: 0,
      shortestPathSteps: 0,
      turnsOnShortestPath: 0,
      longestStraightOnShortestPath: 0,
      longestStraightAnywhere: 0,
      diagonalRunCount: 0,
      longestDiagonalRun: 0,
      deadEnds: 0,
      junctions: 0,
      extraLoops: 0,
      avgDegree: 0,
      pathJunctions: 0,
      sideExitsFromShortestPath: 0,
      bridgeCount: 0,
      pathBridgeCount: 0,
      nonBridgePathEdges: 0,
      pathBridgeRatio: 0,
      full2x2OpenBlocks: 0,
      almost2x2OpenBlocks: 0,
      dense3x3PenaltyUnits: 0,
      degree4Cells: 0,
      adjacentJunctionPairs: 0,
    },
  };
}
