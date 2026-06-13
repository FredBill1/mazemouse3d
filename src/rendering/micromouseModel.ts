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
    readonly colliderWidth: number;
    readonly height: number;
    readonly depth: number;
    readonly centerY: number;
    readonly mass: number;
    readonly centerOfMassOffset: {
      readonly x: number;
      readonly y: number;
      readonly z: number;
    };
  };
  readonly pcb: {
    readonly width: number;
    readonly height: number;
    readonly centerY: number;
    readonly rearZ: number;
    readonly frontArcCenterZ: number;
    readonly frontRadius: number;
  };
  readonly wheel: {
    readonly radius: number;
    readonly width: number;
    readonly axleY: number;
    readonly mass: number;
    readonly motorMaxForce: number;
  };
  readonly wheels: readonly WheelBlueprint[];
  readonly sensors: readonly SensorBlueprint[];
  readonly electronics: {
    readonly motorLocalZ: number;
    readonly batteryLocalZ: number;
    readonly connectorLocalZ: number;
  };
  readonly motorCount: number;
  readonly gearCount: number;
  readonly chipCount: number;
  readonly batteryCount: number;
}

export const MICROMOUSE_BLUEPRINT: MicromouseBlueprint = {
  chassis: {
    width: 0.47,
    colliderWidth: 0.31,
    height: 0.11,
    depth: 0.57,
    centerY: 0.105,
    mass: 0.22,
    centerOfMassOffset: {
      x: 0,
      y: -0.025,
      z: -0.16,
    },
  },
  pcb: {
    width: 0.46,
    height: 0.018,
    centerY: 0.048,
    rearZ: -0.31,
    frontArcCenterZ: -0.05,
    frontRadius: 0.26,
  },
  wheel: {
    radius: 0.09,
    width: 0.056,
    axleY: 0.09,
    mass: 0.024,
    motorMaxForce: 0.34,
  },
  wheels: [
    { id: "front-left", side: "left", localX: -0.205, localZ: -0.055 },
    { id: "front-right", side: "right", localX: 0.205, localZ: -0.055 },
    { id: "rear-left", side: "left", localX: -0.205, localZ: -0.25 },
    { id: "rear-right", side: "right", localX: 0.205, localZ: -0.25 },
  ],
  sensors: [
    { id: "front-left", localX: -0.13, localZ: 0.154, yaw: -Math.PI / 4.2 },
    { id: "front-center-left", localX: -0.046, localZ: 0.178, yaw: -Math.PI / 20 },
    { id: "front-center-right", localX: 0.046, localZ: 0.178, yaw: Math.PI / 20 },
    { id: "front-right", localX: 0.13, localZ: 0.154, yaw: Math.PI / 4.2 },
    { id: "left-side", localX: -0.2, localZ: 0.09, yaw: -Math.PI / 2 },
    { id: "right-side", localX: 0.2, localZ: 0.09, yaw: Math.PI / 2 },
  ],
  electronics: {
    motorLocalZ: -0.16,
    batteryLocalZ: -0.205,
    connectorLocalZ: -0.04,
  },
  motorCount: 2,
  gearCount: 4,
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
