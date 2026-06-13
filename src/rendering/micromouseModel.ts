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
    width: 0.49,
    height: 0.11,
    depth: 0.68,
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
    rearZ: -0.32,
    frontArcCenterZ: 0.17,
    frontRadius: 0.23,
  },
  wheel: {
    radius: 0.09,
    width: 0.068,
    axleY: 0.09,
    mass: 0.024,
    motorMaxForce: 0.34,
  },
  wheels: [
    { id: "front-left", side: "left", localX: -0.285, localZ: -0.075 },
    { id: "front-right", side: "right", localX: 0.285, localZ: -0.075 },
    { id: "rear-left", side: "left", localX: -0.285, localZ: -0.255 },
    { id: "rear-right", side: "right", localX: 0.285, localZ: -0.255 },
  ],
  sensors: [
    { id: "front-left", localX: -0.165, localZ: 0.355, yaw: -Math.PI / 4.6 },
    { id: "front-center-left", localX: -0.055, localZ: 0.378, yaw: -Math.PI / 18 },
    { id: "front-center-right", localX: 0.055, localZ: 0.378, yaw: Math.PI / 18 },
    { id: "front-right", localX: 0.165, localZ: 0.355, yaw: Math.PI / 4.6 },
    { id: "left-side", localX: -0.21, localZ: 0.12, yaw: -Math.PI / 2 },
    { id: "right-side", localX: 0.21, localZ: 0.12, yaw: Math.PI / 2 },
  ],
  electronics: {
    motorLocalZ: -0.17,
    batteryLocalZ: -0.19,
    connectorLocalZ: -0.03,
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
