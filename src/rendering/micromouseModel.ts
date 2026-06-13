import type { MazeSnapshot } from "../domain/maze";
import { cellIndexToRowCol } from "./mazeGeometry";

export interface WheelBlueprint {
  readonly id: "front-left" | "front-right" | "rear-left" | "rear-right";
  readonly side: "left" | "right";
  readonly localX: number;
  readonly localZ: number;
}

export interface SensorBlueprint {
  readonly id:
    | "front-left"
    | "front-center-left"
    | "front-center-right"
    | "front-right"
    | "left-side"
    | "right-side";
  readonly localX: number;
  readonly localZ: number;
  readonly yaw: number;
}

export interface MicromouseBlueprint {
  readonly chassis: {
    readonly width: number;
    readonly height: number;
    readonly depth: number;
    readonly centerY: number;
  };
  readonly wheel: {
    readonly radius: number;
    readonly width: number;
    readonly mass: number;
    readonly motorMaxForce: number;
  };
  readonly wheels: readonly WheelBlueprint[];
  readonly sensors: readonly SensorBlueprint[];
  readonly motorCount: number;
  readonly gearCount: number;
  readonly chipCount: number;
  readonly batteryCount: number;
}

export const MICROMOUSE_BLUEPRINT: MicromouseBlueprint = {
  chassis: {
    width: 0.42,
    height: 0.05,
    depth: 0.62,
    centerY: 0.125,
  },
  wheel: {
    radius: 0.075,
    width: 0.05,
    mass: 0.018,
    motorMaxForce: 0.26,
  },
  wheels: [
    { id: "front-left", side: "left", localX: -0.245, localZ: 0.195 },
    { id: "front-right", side: "right", localX: 0.245, localZ: 0.195 },
    { id: "rear-left", side: "left", localX: -0.245, localZ: -0.205 },
    { id: "rear-right", side: "right", localX: 0.245, localZ: -0.205 },
  ],
  sensors: [
    { id: "front-left", localX: -0.17, localZ: 0.335, yaw: -Math.PI / 5 },
    { id: "front-center-left", localX: -0.055, localZ: 0.35, yaw: -Math.PI / 18 },
    { id: "front-center-right", localX: 0.055, localZ: 0.35, yaw: Math.PI / 18 },
    { id: "front-right", localX: 0.17, localZ: 0.335, yaw: Math.PI / 5 },
    { id: "left-side", localX: -0.235, localZ: 0.085, yaw: -Math.PI / 2 },
    { id: "right-side", localX: 0.235, localZ: 0.085, yaw: Math.PI / 2 },
  ],
  motorCount: 2,
  gearCount: 2,
  chipCount: 1,
  batteryCount: 1,
};

export function initialMouseYaw(maze: Pick<MazeSnapshot, "size" | "start" | "solution">): number {
  const nextCell = maze.solution[1];

  if (nextCell === undefined) {
    return 0;
  }

  return yawBetweenCells(maze.start, nextCell, maze.size);
}

export function yawBetweenCells(fromCell: number, toCell: number, size: number): number {
  const from = cellIndexToRowCol(fromCell, size);
  const to = cellIndexToRowCol(toCell, size);
  const deltaCol = to.col - from.col;
  const deltaRow = to.row - from.row;

  if (deltaCol === 1 && deltaRow === 0) {
    return Math.PI / 2;
  }

  if (deltaCol === -1 && deltaRow === 0) {
    return -Math.PI / 2;
  }

  if (deltaCol === 0 && deltaRow === -1) {
    return Math.PI;
  }

  return 0;
}
