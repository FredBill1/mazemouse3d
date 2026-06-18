import type { DwaDebugOutput } from "../domain/dwa";
import type { DwaWorkerStatus } from "../rendering/dwaMotorDriver";
import type { MotorCommand, RobotGroundTruthTelemetry, WorldPoint } from "../rendering/motorDriver";

export interface SimulationDebugSnapshot {
  readonly elapsedSeconds: number;
  readonly fps: number;
  readonly dwaHz: number;
  readonly workerStatus: DwaWorkerStatus;
  readonly workerRestartCount: number;
  readonly lastWorkerLatencyMs: number | null;
  readonly lastWorkerError: string | null;
  readonly pose: WorldPoint;
  readonly yaw: number;
  readonly eulerAngles: WorldPoint;
  readonly linearVelocity: WorldPoint;
  readonly angularVelocity: WorldPoint;
  readonly horizontalSpeed: number;
  readonly totalDistance: number;
  readonly averageSpeed: number;
  readonly wallCollisionCount: number;
  readonly lastCommand: MotorCommand;
  readonly lastDwaDebug: DwaDebugOutput | null;
}

type SimulationDebugListener = (snapshot: SimulationDebugSnapshot) => void;

const ZERO_POINT: WorldPoint = { x: 0, y: 0, z: 0 };
const ZERO_COMMAND: MotorCommand = { leftRadPerSec: 0, rightRadPerSec: 0 };

export const EMPTY_SIMULATION_DEBUG: SimulationDebugSnapshot = {
  elapsedSeconds: 0,
  fps: 0,
  dwaHz: 0,
  workerStatus: "starting",
  workerRestartCount: 0,
  lastWorkerLatencyMs: null,
  lastWorkerError: null,
  pose: ZERO_POINT,
  yaw: 0,
  eulerAngles: ZERO_POINT,
  linearVelocity: ZERO_POINT,
  angularVelocity: ZERO_POINT,
  horizontalSpeed: 0,
  totalDistance: 0,
  averageSpeed: 0,
  wallCollisionCount: 0,
  lastCommand: ZERO_COMMAND,
  lastDwaDebug: null,
};

declare global {
  interface Window {
    __MAZEMOUSE3D_DEBUG__?: SimulationDebugSnapshot;
  }
}

export class SimulationDebugStore {
  #snapshot = EMPTY_SIMULATION_DEBUG;
  #listeners = new Set<SimulationDebugListener>();

  get snapshot(): SimulationDebugSnapshot {
    return this.#snapshot;
  }

  subscribe(listener: SimulationDebugListener): () => void {
    this.#listeners.add(listener);
    listener(this.#snapshot);

    return () => {
      this.#listeners.delete(listener);
    };
  }

  setSnapshot(snapshot: SimulationDebugSnapshot): void {
    this.#snapshot = snapshot;

    if (typeof window !== "undefined") {
      window.__MAZEMOUSE3D_DEBUG__ = snapshot;
    }

    for (const listener of this.#listeners) {
      listener(snapshot);
    }
  }

  reset(): void {
    this.setSnapshot(EMPTY_SIMULATION_DEBUG);
  }
}

export function telemetryPosePoint(telemetry: RobotGroundTruthTelemetry): WorldPoint {
  return {
    x: telemetry.origin.x,
    y: telemetry.origin.y,
    z: telemetry.origin.z,
  };
}
