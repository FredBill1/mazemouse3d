import type { MotorCommand } from "../rendering/motorDriver";

export interface NavigationPoint {
  readonly x: number;
  readonly z: number;
}

export interface NavigationPose {
  readonly x: number;
  readonly z: number;
  readonly yaw: number;
}

export interface NavigationVelocity {
  readonly vx: number;
  readonly omega: number;
}

export interface NavigationConfig {
  readonly dwbFrequency: number;
  readonly maxWheelRadPerSec: number;
  readonly maxLinearSpeed: number;
  readonly maxAngularSpeed: number;
  readonly maxLinearAccel: number;
  readonly maxLinearDecel: number;
  readonly maxAngularAccel: number;
  readonly maxAngularDecel: number;
  readonly trackWidth: number;
  readonly wheelRadius: number;
  readonly simTime: number;
  readonly simStep: number;
  readonly vxSamples: number;
  readonly omegaSamples: number;
  readonly waypointTolerance: number;
  readonly arrivalDistance: number;
  readonly robotHalfWidth: number;
  readonly robotFrontLength: number;
  readonly robotRearLength: number;
  readonly robotFootprint: readonly NavigationPoint[];
  readonly robotFootprints: readonly (readonly NavigationPoint[])[];
  readonly safetyMargin: number;
  readonly wallThickness: number;
}

export interface NavigationRejectCounts {
  readonly currentPoseCollision: number;
  readonly rolloutCollision: number;
  readonly brakingCollision: number;
  readonly wheelSpeed: number;
  readonly trackability: number;
  readonly lowClearance: number;
  readonly noProgress: number;
  readonly noPathProjection: number;
  readonly nonFiniteScore: number;
}

export interface NavigationDynamicWindowDebug {
  readonly currentV: number;
  readonly currentW: number;
  readonly minV: number;
  readonly maxV: number;
  readonly minW: number;
  readonly maxW: number;
}

export interface NavigationBestTrajectoryDebug {
  readonly linearSpeed: number;
  readonly angularSpeed: number;
  readonly score: {
    readonly total: number;
    readonly pathDistance: number;
    readonly targetDistance: number;
    readonly headingError: number;
    readonly obstacleCost: number;
    readonly progressReward: number;
    readonly speedReward: number;
    readonly angularCost: number;
    readonly accelerationCost: number;
    readonly lowSpeedTurnCost: number;
    readonly reverseCost: number;
    readonly minClearance: number;
    readonly progress: number;
    readonly endX: number;
    readonly endZ: number;
  };
}

export interface NavigationControllerDebug {
  readonly dwbHz: number;
  readonly smootherHz: number;
  readonly status: string;
  readonly validTrajectories: number;
  readonly sampledTrajectories: number;
  readonly rejectedTrajectories: NavigationRejectCounts;
  readonly currentLinearSpeed: number;
  readonly currentAngularSpeed: number;
  readonly targetLinearSpeed: number;
  readonly targetAngularSpeed: number;
  readonly smoothedLinearSpeed: number;
  readonly smoothedAngularSpeed: number;
  readonly dynamicWindow: NavigationDynamicWindowDebug;
  readonly best: NavigationBestTrajectoryDebug | null;
  readonly currentClearance: number;
  readonly currentPoseCollides: boolean;
  readonly pathProgress: number;
  readonly pathLength: number;
  readonly remainingDistance: number;
  readonly pathTrackingError: number;
  readonly workerComputeMs: number;
}

export interface NavigationInitRequest {
  readonly type: "init";
  readonly requestId: number;
  readonly size: number;
  readonly walls: readonly number[];
  readonly goals: readonly number[];
  readonly seed: number;
  readonly config: NavigationConfig;
}

export interface NavigationTickRequest {
  readonly type: "tick";
  readonly sequence: number;
  readonly deltaSeconds: number;
  readonly pose: NavigationPose;
  readonly velocity: NavigationVelocity;
}

export type NavigationWorkerRequest = NavigationInitRequest | NavigationTickRequest;

export interface NavigationReadyResponse {
  readonly type: "ready";
  readonly requestId: number;
}

export interface NavigationCommandResponse {
  readonly type: "command";
  readonly sequence: number;
  readonly command: MotorCommand;
  readonly twist: {
    readonly linearSpeed: number;
    readonly angularSpeed: number;
  };
  readonly path: readonly NavigationPoint[];
  readonly pathVersion: number;
  readonly targetCell: number | null;
  readonly debug: NavigationControllerDebug;
}

export interface NavigationErrorResponse {
  readonly type: "error";
  readonly requestId?: number;
  readonly sequence?: number;
  readonly message: string;
}

export type NavigationWorkerResponse =
  | NavigationReadyResponse
  | NavigationCommandResponse
  | NavigationErrorResponse;
