import { describe, expect, it } from "vitest";
import { WALL_BITS, type MazeSnapshot } from "../domain/maze";
import {
  PlannedMotorDriver,
  isPassableHalfGrid,
  nearestPassableHalfGrid,
  quantizeHeading,
  type PathPlanner,
} from "../rendering/plannedMotorDriver";
import type { RobotGroundTruthPose } from "../rendering/motorDriver";

describe("planned motor driver", () => {
  it("matches the half-grid passability rules used by the planner", () => {
    const maze = mazeSnapshot([15 & ~WALL_BITS.EAST, 15 & ~WALL_BITS.WEST, 15, 15]);

    expect(isPassableHalfGrid(maze, 1, 1)).toBe(true);
    expect(isPassableHalfGrid(maze, 2, 1)).toBe(true);
    expect(isPassableHalfGrid(maze, 1, 2)).toBe(false);
    expect(isPassableHalfGrid(maze, 2, 2)).toBe(false);
    expect(nearestPassableHalfGrid(maze, 1.01, 0.51)).toEqual({ x2: 2, z2: 1 });
  });

  it("quantizes Babylon yaw into the planner heading convention", () => {
    expect(quantizeHeading(0)).toBe(0);
    expect(quantizeHeading(Math.PI / 4)).toBe(1);
    expect(quantizeHeading(Math.PI / 2)).toBe(2);
    expect(quantizeHeading(-Math.PI / 2)).toBe(6);
  });

  it("keeps planned wheel speeds within configured bounds", () => {
    const driver = new PlannedMotorDriver(openMaze(), {
      planner: straightPlanner,
      maxWheelRadPerSec: 9,
      seed: 4,
    });
    const command = driver.next(1 / 120, poseAt(0.5, 0.5, 0));

    expect(Math.abs(command.leftRadPerSec)).toBeLessThanOrEqual(9);
    expect(Math.abs(command.rightRadPerSec)).toBeLessThanOrEqual(9);
  });

  it("continues forward when the robot has passed a waypoint without hitting it exactly", () => {
    const driver = new PlannedMotorDriver(openMaze(), {
      planner: cornerPlanner,
      seed: 4,
    });
    const command = driver.next(1 / 120, poseAt(1, 0.8, 0));

    expect(command.leftRadPerSec + command.rightRadPerSec).toBeGreaterThan(0);
  });

  it("selects a new target after reaching the previous target", () => {
    const calls: number[] = [];
    const planner: PathPlanner = (request) => {
      calls.push(request.goalCell);
      const x2 = (request.goalCell % request.size) * 2 + 1;
      const z2 = Math.floor(request.goalCell / request.size) * 2 + 1;

      return {
        cost: 1,
        steps: [{ x2, z2, heading: request.startHeading }],
        waypoints: [{ x2, z2 }],
      };
    };
    const driver = new PlannedMotorDriver(openMaze(), { planner, seed: 1 });

    driver.next(1 / 120, poseAt(0.5, 0.5, 0));
    const firstTarget = calls[0];
    driver.next(1 / 120, poseAt((firstTarget % 2) + 0.5, Math.floor(firstTarget / 2) + 0.5, 0));

    expect(calls).toHaveLength(2);
    expect(calls[1]).not.toBe(firstTarget);
  });

  it("stops safely when the planner cannot produce a path", () => {
    const driver = new PlannedMotorDriver(openMaze(), {
      planner: () => {
        throw new Error("no path");
      },
    });

    expect(driver.next(1 / 120, poseAt(0.5, 0.5, 0))).toEqual({
      leftRadPerSec: 0,
      rightRadPerSec: 0,
    });
  });
});

function straightPlanner(request: Parameters<PathPlanner>[0]): ReturnType<PathPlanner> {
  const x2 = (request.goalCell % request.size) * 2 + 1;
  const z2 = Math.floor(request.goalCell / request.size) * 2 + 1;

  return {
    cost: 1,
    steps: [
      { x2: request.startX2, z2: request.startZ2, heading: request.startHeading },
      { x2, z2, heading: request.startHeading },
    ],
    waypoints: [
      { x2: request.startX2, z2: request.startZ2 },
      { x2, z2 },
    ],
  };
}

function cornerPlanner(request: Parameters<PathPlanner>[0]): ReturnType<PathPlanner> {
  return {
    cost: 1,
    steps: [
      { x2: request.startX2, z2: request.startZ2, heading: request.startHeading },
      { x2: 2, z2: 1, heading: request.startHeading },
      { x2: 2, z2: 2, heading: request.startHeading },
      { x2: 3, z2: 2, heading: request.startHeading },
    ],
    waypoints: [
      { x2: 1, z2: 1 },
      { x2: 2, z2: 1 },
      { x2: 2, z2: 2 },
      { x2: 3, z2: 2 },
    ],
  };
}

function openMaze(): MazeSnapshot {
  return mazeSnapshot([15 & ~WALL_BITS.EAST & ~WALL_BITS.NORTH, 15 & ~WALL_BITS.WEST, 15, 15]);
}

function mazeSnapshot(walls: number[]): MazeSnapshot {
  return {
    size: 2,
    seed: 7,
    iterations: 0,
    initialTemp: 1,
    finalTemp: 1,
    start: 0,
    goals: [0, 1, 2, 3],
    walls: Uint8Array.from(walls),
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

function poseAt(x: number, z: number, yaw: number): RobotGroundTruthPose {
  return {
    origin: { x, y: 0.09, z },
    yaw,
  };
}
