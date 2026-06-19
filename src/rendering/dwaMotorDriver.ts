import type { MazeSnapshot } from "../domain/maze";
import {
  type DwaCommandOutput,
  type DwaControllerConfig,
  type DwaDebugOutput,
  type DwaOptions,
  type DwaTelemetryInput,
  type DwaTelemetrySnapshot,
  type DwaWorkerRequest,
  type DwaWorkerResponse,
} from "../domain/dwa";
import { MICROMOUSE_BLUEPRINT } from "./micromouseModel";
import { STOPPED_COMMAND, clampMotorCommand, type MotorCommand } from "./motorDriver";

interface WorkerLike {
  postMessage(message: DwaWorkerRequest): void;
  terminate(): void;
  addEventListener(
    type: "message",
    listener: (event: MessageEvent<DwaWorkerResponse>) => void,
  ): void;
  addEventListener(type: "error", listener: (event: ErrorEvent) => void): void;
  removeEventListener(
    type: "message",
    listener: (event: MessageEvent<DwaWorkerResponse>) => void,
  ): void;
  removeEventListener(type: "error", listener: (event: ErrorEvent) => void): void;
}

export type DwaWorkerFactory = () => WorkerLike;
export type DwaWorkerStatus = "starting" | "ready" | "restarting" | "degraded" | "disposed";

export interface DwaMotorDriverOptions extends DwaOptions {
  readonly workerFactory?: DwaWorkerFactory;
  readonly targetControlHz?: number;
  readonly commandTimeoutMs?: number;
  readonly initTimeoutMs?: number;
  readonly degradedAfterMs?: number;
}

interface ResolvedDwaMotorDriverOptions extends Required<DwaOptions> {
  readonly targetControlHz: number;
  readonly commandTimeoutMs: number;
  readonly initTimeoutMs: number;
  readonly degradedAfterMs: number;
}

export interface DwaDriverDebugSnapshot {
  readonly workerStatus: DwaWorkerStatus;
  readonly restartCount: number;
  readonly lastWorkerLatencyMs: number | null;
  readonly lastWorkerError: string | null;
  readonly dwaHz: number;
  readonly targetDwaHz: number;
  readonly inFlight: boolean;
  readonly lastCommand: MotorCommand;
  readonly lastDwaDebug: DwaDebugOutput | null;
}

export const DEFAULT_DWA_OPTIONS: ResolvedDwaMotorDriverOptions = {
  maxLinearSpeed: 5.8,
  maxWheelRadPerSec: 105,
  maxAngularSpeed: 14,
  maxLinearAcceleration: 80,
  maxAngularAcceleration: 92,
  predictionHorizon: 0.45,
  rolloutStep: 1 / 30,
  controlPeriod: 1 / 60,
  linearSamples: 5,
  angularSamples: 11,
  pathLookahead: 0.75,
  safetyMargin: 0.06,
  arrivalDistance: 0.2,
  wallThickness: 0.08,
  wheelRadius: MICROMOUSE_BLUEPRINT.wheel.radius,
  trackWidth: wheelTrackWidth(),
  robotRadius: 0.22,
  robotHalfWidth: MICROMOUSE_BLUEPRINT.chassis.width / 2,
  robotFrontExtent: 0.37,
  robotRearExtent: 0.17,
  targetControlHz: 60,
  commandTimeoutMs: 150,
  initTimeoutMs: 1200,
  degradedAfterMs: 600,
};

export class DwaMotorDriver {
  readonly #maze: MazeSnapshot;
  readonly #options: ResolvedDwaMotorDriverOptions;
  readonly #workerFactory: DwaWorkerFactory;
  #worker: WorkerLike | null = null;
  #requestId = 0;
  #workerReady = false;
  #workerStatus: DwaWorkerStatus = "starting";
  #workerStartedAtMs = 0;
  #restartWindowStartedAtMs = 0;
  #restartCount = 0;
  #consecutiveRestarts = 0;
  #latestTelemetryRequestId: number | null = null;
  #lastAcceptedCommandRequestId = 0;
  #lastTelemetrySentAtMs = 0;
  #lastCommandResponseAtMs = 0;
  #sendAccumulatorSeconds = 0;
  #latestTelemetry: DwaTelemetrySnapshot | null = null;
  #command: MotorCommand = STOPPED_COMMAND;
  #fallbackCommand: MotorCommand = STOPPED_COMMAND;
  #lastDwaDebug: DwaDebugOutput | null = null;
  #lastWorkerLatencyMs: number | null = null;
  #lastWorkerError: string | null = null;
  #commandResponseTimesMs: number[] = [];
  #sendTimer: ReturnType<typeof setInterval> | null = null;
  #disposed = false;

  constructor(maze: MazeSnapshot, options: DwaMotorDriverOptions = {}) {
    this.#maze = maze;
    const { workerFactory, ...dwaOptions } = options;
    this.#options = {
      ...DEFAULT_DWA_OPTIONS,
      ...dwaOptions,
    };
    this.#workerFactory = workerFactory ?? defaultWorkerFactory;
    this.#startWorker("initial");
    this.#startSendTimer();
  }

  next(deltaSeconds: number, telemetry: DwaTelemetrySnapshot): MotorCommand {
    if (this.#disposed) {
      return STOPPED_COMMAND;
    }

    const now = nowMs();
    this.#latestTelemetry = telemetry;
    this.#sendAccumulatorSeconds += Math.max(0, deltaSeconds);
    this.#checkWatchdog(now);
    this.#sendTelemetryIfDue(now, false);

    return this.#commandForState(deltaSeconds, now);
  }

  get command(): MotorCommand {
    return this.#command;
  }

  get debug(): DwaDriverDebugSnapshot {
    const now = nowMs();
    this.#trimResponseTimes(now);

    return {
      workerStatus: this.#workerStatus,
      restartCount: this.#restartCount,
      lastWorkerLatencyMs: this.#lastWorkerLatencyMs,
      lastWorkerError: this.#lastWorkerError,
      dwaHz: this.#commandResponseTimesMs.length,
      targetDwaHz: this.#options.targetControlHz,
      inFlight: false,
      lastCommand: this.#command,
      lastDwaDebug: this.#lastDwaDebug,
    };
  }

  dispose(): void {
    this.#disposed = true;
    this.#workerStatus = "disposed";
    this.#stopSendTimer();
    this.#stopWorker();
  }

  #startSendTimer(): void {
    this.#sendTimer = setInterval(() => {
      if (this.#disposed) {
        return;
      }

      const now = nowMs();
      this.#checkWatchdog(now);
      this.#sendTelemetryIfDue(now, true);
    }, 1000 / this.#options.targetControlHz);
  }

  #stopSendTimer(): void {
    if (this.#sendTimer === null) {
      return;
    }

    clearInterval(this.#sendTimer);
    this.#sendTimer = null;
  }

  #startWorker(reason: "initial" | "watchdog" | "error"): void {
    if (this.#disposed) {
      return;
    }

    this.#stopWorker();
    this.#worker = this.#workerFactory();
    this.#worker.addEventListener("message", this.#handleWorkerMessage);
    this.#worker.addEventListener("error", this.#handleWorkerError);
    this.#workerReady = false;
    this.#workerStartedAtMs = nowMs();
    this.#latestTelemetryRequestId = null;
    this.#lastAcceptedCommandRequestId = 0;
    this.#lastTelemetrySentAtMs = 0;
    this.#lastCommandResponseAtMs = this.#workerStartedAtMs;
    this.#workerStatus = reason === "initial" ? "starting" : "restarting";

    if (reason !== "initial") {
      this.#restartCount += 1;
      this.#consecutiveRestarts += 1;
      this.#restartWindowStartedAtMs ||= this.#workerStartedAtMs;
    }

    this.#postInit();
  }

  #stopWorker(): void {
    if (!this.#worker) {
      return;
    }

    this.#worker.removeEventListener("message", this.#handleWorkerMessage);
    this.#worker.removeEventListener("error", this.#handleWorkerError);
    this.#worker.terminate();
    this.#worker = null;
  }

  #postInit(): void {
    this.#worker?.postMessage({
      type: "init",
      requestId: this.#nextRequestId(),
      config: this.#controllerConfig(),
    });
  }

  #sendTelemetryIfDue(now: number, force: boolean): void {
    if (!this.#worker || !this.#workerReady) {
      return;
    }

    const telemetry = this.#latestTelemetry;

    if (!telemetry) {
      return;
    }

    const intervalSeconds = 1 / this.#options.targetControlHz;

    if (
      !force &&
      this.#sendAccumulatorSeconds < intervalSeconds &&
      this.#commandResponseTimesMs.length > 0
    ) {
      return;
    }

    this.#sendAccumulatorSeconds = 0;
    const requestId = this.#nextRequestId();
    this.#latestTelemetryRequestId = requestId;
    this.#lastTelemetrySentAtMs = now;
    this.#worker.postMessage({
      type: "telemetry",
      requestId,
      telemetry: telemetryToWorkerInput(telemetry),
    });
  }

  #checkWatchdog(now: number): void {
    if (
      this.#workerReady &&
      this.#latestTelemetry !== null &&
      this.#lastTelemetrySentAtMs > 0 &&
      now - this.#lastCommandResponseAtMs > this.#options.commandTimeoutMs &&
      now - this.#lastTelemetrySentAtMs > this.#options.commandTimeoutMs
    ) {
      this.#startWorker("watchdog");
      return;
    }

    if (!this.#workerReady && now - this.#workerStartedAtMs > this.#options.initTimeoutMs) {
      this.#startWorker("watchdog");
      return;
    }

    if (
      this.#restartWindowStartedAtMs > 0 &&
      now - this.#restartWindowStartedAtMs > this.#options.degradedAfterMs
    ) {
      this.#workerStatus = "degraded";
    }
  }

  #commandForState(deltaSeconds: number, now: number): MotorCommand {
    if (this.#workerStatus === "ready") {
      this.#fallbackCommand = this.#command;
      return this.#command;
    }

    const degraded =
      this.#workerStatus === "degraded" ||
      this.#consecutiveRestarts >= 3 ||
      (this.#restartWindowStartedAtMs > 0 &&
        now - this.#restartWindowStartedAtMs > this.#options.degradedAfterMs);
    const target = degraded
      ? STOPPED_COMMAND
      : clampMotorCommand(this.#command, this.#options.maxWheelRadPerSec * 0.45);
    this.#fallbackCommand = approachCommand(
      this.#fallbackCommand,
      target,
      this.#options.maxWheelRadPerSec * 2.2 * Math.max(0, deltaSeconds),
    );

    return this.#fallbackCommand;
  }

  #controllerConfig(): DwaControllerConfig {
    return {
      size: this.#maze.size,
      walls: Array.from(this.#maze.walls),
      seed: this.#maze.seed,
      solution: this.#maze.solution,
      options: {
        maxLinearSpeed: this.#options.maxLinearSpeed,
        maxWheelRadPerSec: this.#options.maxWheelRadPerSec,
        maxAngularSpeed: this.#options.maxAngularSpeed,
        maxLinearAcceleration: this.#options.maxLinearAcceleration,
        maxAngularAcceleration: this.#options.maxAngularAcceleration,
        predictionHorizon: this.#options.predictionHorizon,
        rolloutStep: this.#options.rolloutStep,
        controlPeriod: this.#options.controlPeriod,
        linearSamples: this.#options.linearSamples,
        angularSamples: this.#options.angularSamples,
        pathLookahead: this.#options.pathLookahead,
        safetyMargin: this.#options.safetyMargin,
        arrivalDistance: this.#options.arrivalDistance,
        wallThickness: this.#options.wallThickness,
        wheelRadius: this.#options.wheelRadius,
        trackWidth: this.#options.trackWidth,
        robotRadius: this.#options.robotRadius,
        robotHalfWidth: this.#options.robotHalfWidth,
        robotFrontExtent: this.#options.robotFrontExtent,
        robotRearExtent: this.#options.robotRearExtent,
      },
    };
  }

  #handleWorkerMessage = (event: MessageEvent<DwaWorkerResponse>): void => {
    const response = event.data;
    const now = nowMs();

    if (response.type === "ready") {
      this.#workerReady = true;
      this.#workerStatus = "ready";
      this.#lastCommandResponseAtMs = now;
      return;
    }

    if (response.type === "error") {
      this.#lastWorkerError = response.message;
      this.#startWorker("error");
      return;
    }

    if (
      this.#latestTelemetryRequestId === null ||
      response.requestId > this.#latestTelemetryRequestId ||
      response.requestId < this.#lastAcceptedCommandRequestId
    ) {
      return;
    }

    this.#lastAcceptedCommandRequestId = response.requestId;
    this.#command = sanitizeCommand(response.command, this.#options.maxWheelRadPerSec);
    this.#lastDwaDebug = response.command.debug;
    this.#lastWorkerError = null;
    this.#lastWorkerLatencyMs =
      this.#lastTelemetrySentAtMs > 0 ? now - this.#lastTelemetrySentAtMs : null;
    this.#lastCommandResponseAtMs = now;
    this.#workerStatus = "ready";
    this.#consecutiveRestarts = 0;
    this.#restartWindowStartedAtMs = 0;
    this.#commandResponseTimesMs.push(now);
    this.#trimResponseTimes(now);
  };

  #handleWorkerError = (event: ErrorEvent): void => {
    this.#lastWorkerError = event.message || "DWA worker failed";
    this.#startWorker("error");
  };

  #nextRequestId(): number {
    this.#requestId = (this.#requestId + 1) % Number.MAX_SAFE_INTEGER;
    return this.#requestId;
  }

  #trimResponseTimes(now: number): void {
    while (
      this.#commandResponseTimesMs[0] !== undefined &&
      now - this.#commandResponseTimesMs[0] > 1000
    ) {
      this.#commandResponseTimesMs.shift();
    }
  }
}

function defaultWorkerFactory(): WorkerLike {
  return new Worker(new URL("../workers/dwaWorker.ts", import.meta.url), {
    type: "module",
  });
}

function telemetryToWorkerInput(telemetry: DwaTelemetrySnapshot): DwaTelemetryInput {
  return {
    x: telemetry.origin.x,
    z: telemetry.origin.z,
    yaw: telemetry.yaw,
    velocityX: telemetry.linearVelocity.x,
    velocityZ: telemetry.linearVelocity.z,
    angularVelocityY: telemetry.angularVelocity.y,
  };
}

function sanitizeCommand(command: DwaCommandOutput, maxWheelRadPerSec: number): MotorCommand {
  return clampMotorCommand(
    {
      leftRadPerSec: finiteOrZero(command.leftRadPerSec),
      rightRadPerSec: finiteOrZero(command.rightRadPerSec),
    },
    maxWheelRadPerSec,
  );
}

function finiteOrZero(value: number): number {
  return Number.isFinite(value) ? value : 0;
}

function approachCommand(
  command: MotorCommand,
  target: MotorCommand,
  maxDelta: number,
): MotorCommand {
  return {
    leftRadPerSec: approach(command.leftRadPerSec, target.leftRadPerSec, maxDelta),
    rightRadPerSec: approach(command.rightRadPerSec, target.rightRadPerSec, maxDelta),
  };
}

function approach(value: number, target: number, maxDelta: number): number {
  if (Math.abs(target - value) <= maxDelta) {
    return target;
  }

  return value + Math.sign(target - value) * maxDelta;
}

function nowMs(): number {
  return typeof performance === "undefined" ? Date.now() : performance.now();
}

function wheelTrackWidth(): number {
  const left = MICROMOUSE_BLUEPRINT.wheels.filter((wheel) => wheel.side === "left");
  const right = MICROMOUSE_BLUEPRINT.wheels.filter((wheel) => wheel.side === "right");
  const average = (values: readonly number[]): number =>
    values.reduce((sum, value) => sum + value, 0) / values.length;

  return average(right.map((wheel) => wheel.localX)) - average(left.map((wheel) => wheel.localX));
}
