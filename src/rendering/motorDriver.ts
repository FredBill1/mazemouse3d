export interface MotorCommand {
  readonly leftRadPerSec: number;
  readonly rightRadPerSec: number;
}

export interface WorldPoint {
  readonly x: number;
  readonly y: number;
  readonly z: number;
}

export interface RobotGroundTruthPose {
  readonly origin: WorldPoint;
  readonly yaw: number;
}

export const STOPPED_COMMAND: MotorCommand = {
  leftRadPerSec: 0,
  rightRadPerSec: 0,
};

export function commandMagnitude(command: MotorCommand): number {
  return Math.max(Math.abs(command.leftRadPerSec), Math.abs(command.rightRadPerSec));
}

export function clampMotorCommand(command: MotorCommand, maxMagnitude: number): MotorCommand {
  return {
    leftRadPerSec: clamp(command.leftRadPerSec, -maxMagnitude, maxMagnitude),
    rightRadPerSec: clamp(command.rightRadPerSec, -maxMagnitude, maxMagnitude),
  };
}

export function createSeededRandom(seed: number): () => number {
  let state = (seed ^ 0x9e3779b9) >>> 0;

  return () => {
    state = (state + 0x6d2b79f5) >>> 0;
    let value = state;
    value = Math.imul(value ^ (value >>> 15), value | 1);
    value ^= value + Math.imul(value ^ (value >>> 7), value | 61);
    return ((value ^ (value >>> 14)) >>> 0) / 4294967296;
  };
}

export function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}
