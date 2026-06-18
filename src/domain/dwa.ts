import type { WorldPoint } from "../rendering/motorDriver";

export interface DwaOptions {
  readonly maxLinearSpeed?: number;
  readonly maxWheelRadPerSec?: number;
  readonly maxAngularSpeed?: number;
  readonly maxLinearAcceleration?: number;
  readonly maxAngularAcceleration?: number;
  readonly predictionHorizon?: number;
  readonly rolloutStep?: number;
  readonly controlPeriod?: number;
  readonly linearSamples?: number;
  readonly angularSamples?: number;
  readonly pathLookahead?: number;
  readonly safetyMargin?: number;
  readonly arrivalDistance?: number;
  readonly wallThickness?: number;
  readonly wheelRadius?: number;
  readonly trackWidth?: number;
  readonly robotRadius?: number;
  readonly robotHalfWidth?: number;
  readonly robotFrontExtent?: number;
  readonly robotRearExtent?: number;
}

export interface DwaControllerConfig {
  readonly size: number;
  readonly walls: readonly number[];
  readonly seed: number;
  readonly solution: readonly number[];
  readonly options: DwaOptions;
}

export interface DwaTelemetryInput {
  readonly x: number;
  readonly z: number;
  readonly yaw: number;
  readonly velocityX: number;
  readonly velocityZ: number;
  readonly angularVelocityY: number;
}

export interface DwaDebugOutput {
  readonly targetCell: number;
  readonly pathProgress: number;
  readonly pathLength: number;
  readonly targetX: number;
  readonly targetZ: number;
  readonly clearance: number;
  readonly score: number;
  readonly sampledCandidates: number;
  readonly validCandidates: number;
  readonly replanCount: number;
}

export interface DwaCommandOutput {
  readonly leftRadPerSec: number;
  readonly rightRadPerSec: number;
  readonly linearSpeed: number;
  readonly angularSpeed: number;
  readonly debug: DwaDebugOutput;
}

export interface DwaTelemetrySnapshot {
  readonly origin: WorldPoint;
  readonly yaw: number;
  readonly linearVelocity: WorldPoint;
  readonly angularVelocity: WorldPoint;
}

export type DwaWorkerRequest =
  | {
      readonly type: "init";
      readonly requestId: number;
      readonly config: DwaControllerConfig;
    }
  | {
      readonly type: "telemetry";
      readonly requestId: number;
      readonly telemetry: DwaTelemetryInput;
    };

export type DwaWorkerResponse =
  | {
      readonly type: "ready";
      readonly requestId: number;
    }
  | {
      readonly type: "command";
      readonly requestId: number;
      readonly command: DwaCommandOutput;
    }
  | {
      readonly type: "error";
      readonly requestId: number;
      readonly message: string;
    };
