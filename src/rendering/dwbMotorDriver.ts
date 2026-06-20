import type { MazeSnapshot } from "../domain/maze";
import type {
  NavigationCommandResponse,
  NavigationConfig,
  NavigationControllerDebug,
  NavigationDynamicWindowDebug,
  NavigationPoint,
  NavigationRejectCounts,
  NavigationWorkerRequest,
  NavigationWorkerResponse,
} from "../domain/navigation";
import { MICROMOUSE_BLUEPRINT } from "./micromouseModel";
import { STOPPED_COMMAND, clampMotorCommand, type MotorCommand } from "./motorDriver";

export interface DwbMotorDriverSample {
  readonly pose: {
    readonly origin: {
      readonly x: number;
      readonly z: number;
    };
    readonly yaw: number;
  };
  readonly velocity: {
    readonly vx: number;
    readonly omega: number;
  };
}

interface TwistState {
  readonly linearSpeed: number;
  readonly angularSpeed: number;
}

export interface DwbMotorDriverDebugSnapshot {
  readonly dwbHz: number;
  readonly smootherHz: number;
  readonly workerLatencyMs: number | null;
  readonly targetCell: number | null;
  readonly status: string;
  readonly linearSpeed: number;
  readonly angularSpeed: number;
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
  readonly best: NavigationControllerDebug["best"];
  readonly currentClearance: number;
  readonly currentPoseCollides: boolean;
  readonly pathProgress: number;
  readonly pathLength: number;
  readonly remainingDistance: number;
  readonly pathTrackingError: number;
  readonly workerComputeMs: number;
}

export interface DwbMotorDriverOptions {
  readonly config?: Partial<NavigationConfig>;
  readonly debug?: boolean;
  readonly workerFactory?: () => NavigationWorkerLike;
  readonly now?: () => number;
}

export interface NavigationWorkerLike {
  postMessage(message: NavigationWorkerRequest): void;
  addEventListener(
    type: "message",
    listener: (event: MessageEvent<NavigationWorkerResponse>) => void,
  ): void;
  removeEventListener(
    type: "message",
    listener: (event: MessageEvent<NavigationWorkerResponse>) => void,
  ): void;
  terminate(): void;
}

export const EMPTY_DWB_DEBUG_SNAPSHOT: DwbMotorDriverDebugSnapshot = {
  dwbHz: 0,
  smootherHz: 0,
  workerLatencyMs: null,
  targetCell: null,
  status: "idle",
  linearSpeed: 0,
  angularSpeed: 0,
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
    maxV: 0,
    minW: 0,
    maxW: 0,
  },
  best: null,
  currentClearance: Number.POSITIVE_INFINITY,
  currentPoseCollides: false,
  pathProgress: 0,
  pathLength: 0,
  remainingDistance: 0,
  pathTrackingError: 0,
  workerComputeMs: 0,
};

const WALL_THICKNESS = 0.04;
const BOARD_ARC_SEGMENTS = 18;
const WORKER_SMOOTHER_FREQUENCY_HZ = 60;
const WORKER_SMOOTHER_PERIOD_SECONDS = 1 / WORKER_SMOOTHER_FREQUENCY_HZ;
const MAX_WORKER_TICKS_IN_FLIGHT = 1;

export class DwbMotorDriver {
  readonly #worker: NavigationWorkerLike;
  readonly #now: () => number;
  readonly #debug: boolean;
  readonly #config: NavigationConfig;
  readonly #sentAt = new Map<number, number>();
  #sequence = 0;
  #latestCommand: MotorCommand = STOPPED_COMMAND;
  #latestCommandAt: number | null = null;
  #targetTwist: TwistState = {
    linearSpeed: 0,
    angularSpeed: 0,
  };
  #smoothedTwist: TwistState = {
    linearSpeed: 0,
    angularSpeed: 0,
  };
  #latestPath: readonly NavigationPoint[] = [];
  #pathVersion = 0;
  #debugSnapshot: DwbMotorDriverDebugSnapshot = EMPTY_DWB_DEBUG_SNAPSHOT;
  #workerTickAccumulator = Number.POSITIVE_INFINITY;
  #pendingWorkerDeltaSeconds = 0;
  #controlElapsedSeconds = 0;
  #smootherTicks = 0;
  #disposed = false;

  constructor(maze: MazeSnapshot, options: DwbMotorDriverOptions = {}) {
    this.#config = {
      ...defaultNavigationConfig(),
      ...options.config,
    };
    this.#worker =
      options.workerFactory?.() ??
      new Worker(new URL("../workers/navigationWorker.ts", import.meta.url), {
        type: "module",
      });
    this.#now = options.now ?? (() => performance.now());
    this.#debug = options.debug ?? navigationDebugEnabled();
    this.#worker.addEventListener("message", this.#handleMessage);
    this.#post({
      type: "init",
      requestId: 1,
      size: maze.size,
      walls: Array.from(maze.walls),
      goals: Array.from(maze.goals),
      seed: maze.seed,
      config: this.#config,
    });
  }

  next(deltaSeconds: number, sample: DwbMotorDriverSample): MotorCommand {
    if (this.#disposed) {
      return STOPPED_COMMAND;
    }

    this.#controlElapsedSeconds += deltaSeconds;
    this.#smootherTicks += 1;
    this.#pendingWorkerDeltaSeconds += deltaSeconds;
    this.#workerTickAccumulator += deltaSeconds;

    if (
      this.#sentAt.size < MAX_WORKER_TICKS_IN_FLIGHT &&
      (this.#latestCommandAt === null ||
        this.#workerTickAccumulator >= WORKER_SMOOTHER_PERIOD_SECONDS)
    ) {
      const sequence = (this.#sequence += 1);
      this.#sentAt.set(sequence, this.#now());
      this.#post({
        type: "tick",
        sequence,
        deltaSeconds: this.#pendingWorkerDeltaSeconds,
        pose: {
          x: sample.pose.origin.x,
          z: sample.pose.origin.z,
          yaw: sample.pose.yaw,
        },
        velocity: sample.velocity,
      });
      this.#pendingWorkerDeltaSeconds = 0;
      this.#workerTickAccumulator = 0;
    }

    if (this.#latestCommandAt === null) {
      return STOPPED_COMMAND;
    }

    this.#smoothedTwist = smoothTwist(
      this.#smoothedTwist,
      this.#targetTwist,
      deltaSeconds,
      this.#config,
    );
    this.#latestCommand = twistToMotorCommand(this.#smoothedTwist, this.#config);
    this.#debugSnapshot = {
      ...this.#debugSnapshot,
      smootherHz:
        this.#controlElapsedSeconds > 0 ? this.#smootherTicks / this.#controlElapsedSeconds : 0,
      linearSpeed: this.#smoothedTwist.linearSpeed,
      angularSpeed: this.#smoothedTwist.angularSpeed,
      currentLinearSpeed: sample.velocity.vx,
      currentAngularSpeed: sample.velocity.omega,
      targetLinearSpeed: this.#targetTwist.linearSpeed,
      targetAngularSpeed: this.#targetTwist.angularSpeed,
      smoothedLinearSpeed: this.#smoothedTwist.linearSpeed,
      smoothedAngularSpeed: this.#smoothedTwist.angularSpeed,
    };
    return this.#latestCommand;
  }

  get command(): MotorCommand {
    return this.#latestCommand;
  }

  get debugSnapshot(): DwbMotorDriverDebugSnapshot {
    return this.#debugSnapshot;
  }

  get pathPoints(): readonly NavigationPoint[] {
    return this.#latestPath;
  }

  get pathVersion(): number {
    return this.#pathVersion;
  }

  dispose(): void {
    if (this.#disposed) {
      return;
    }

    this.#disposed = true;
    this.#worker.removeEventListener("message", this.#handleMessage);
    this.#worker.terminate();
  }

  #handleMessage = (event: MessageEvent<NavigationWorkerResponse>): void => {
    const response = event.data;

    if (response.type === "ready") {
      this.#setStatus("ready");
      return;
    }

    if (response.type === "error") {
      this.#setStatus(`error: ${response.message}`);
      if (response.sequence !== undefined) {
        this.#sentAt.delete(response.sequence);
      }
      return;
    }

    this.#acceptCommand(response);
  };

  #acceptCommand(response: NavigationCommandResponse): void {
    const sentAt = this.#sentAt.get(response.sequence);
    const now = this.#now();
    const workerLatencyMs = sentAt === undefined ? null : now - sentAt;

    this.#sentAt.delete(response.sequence);

    if (this.#sentAt.size > 240) {
      const staleSequences = [...this.#sentAt.keys()].slice(0, this.#sentAt.size - 240);

      for (const sequence of staleSequences) {
        this.#sentAt.delete(sequence);
      }
    }

    this.#targetTwist = {
      linearSpeed: response.debug.targetLinearSpeed,
      angularSpeed: response.debug.targetAngularSpeed,
    };
    this.#latestCommandAt = now;
    this.#latestPath = response.path;
    this.#pathVersion = response.pathVersion;
    this.#debugSnapshot = {
      dwbHz: response.debug.dwbHz,
      smootherHz: response.debug.smootherHz,
      workerLatencyMs,
      targetCell: response.targetCell,
      status: response.debug.status,
      linearSpeed: response.twist.linearSpeed,
      angularSpeed: response.twist.angularSpeed,
      validTrajectories: response.debug.validTrajectories,
      sampledTrajectories: response.debug.sampledTrajectories,
      rejectedTrajectories: response.debug.rejectedTrajectories,
      currentLinearSpeed: response.debug.currentLinearSpeed,
      currentAngularSpeed: response.debug.currentAngularSpeed,
      targetLinearSpeed: response.debug.targetLinearSpeed,
      targetAngularSpeed: response.debug.targetAngularSpeed,
      smoothedLinearSpeed: response.debug.smoothedLinearSpeed,
      smoothedAngularSpeed: response.debug.smoothedAngularSpeed,
      dynamicWindow: response.debug.dynamicWindow,
      best: response.debug.best,
      currentClearance: response.debug.currentClearance,
      currentPoseCollides: response.debug.currentPoseCollides,
      pathProgress: response.debug.pathProgress,
      pathLength: response.debug.pathLength,
      remainingDistance: response.debug.remainingDistance,
      pathTrackingError: response.debug.pathTrackingError,
      workerComputeMs: response.debug.workerComputeMs,
    };

    if (this.#debug && response.sequence % 120 === 0) {
      emitNavigationDebug({
        event: "navigation-command",
        sequence: response.sequence,
        command: this.#latestCommand,
        debug: this.#debugSnapshot,
      });
    }
  }

  #setStatus(status: string): void {
    this.#debugSnapshot = {
      ...this.#debugSnapshot,
      status,
    };
  }

  #post(message: NavigationWorkerRequest): void {
    try {
      this.#worker.postMessage(message);
    } catch (error) {
      this.#setStatus(error instanceof Error ? error.message : String(error));
    }
  }
}

export function defaultNavigationConfig(): NavigationConfig {
  const wheelCenterZ =
    MICROMOUSE_BLUEPRINT.wheels.reduce((sum, wheel) => sum + wheel.localZ, 0) /
    MICROMOUSE_BLUEPRINT.wheels.length;
  const wheelOuterHalfWidth =
    Math.max(...MICROMOUSE_BLUEPRINT.wheels.map((wheel) => Math.abs(wheel.localX))) +
    MICROMOUSE_BLUEPRINT.wheel.width / 2;
  const wheelRear =
    wheelCenterZ -
    (Math.min(...MICROMOUSE_BLUEPRINT.wheels.map((wheel) => wheel.localZ)) -
      MICROMOUSE_BLUEPRINT.wheel.radius);
  const front =
    MICROMOUSE_BLUEPRINT.pcb.frontArcCenterZ + MICROMOUSE_BLUEPRINT.pcb.frontRadius - wheelCenterZ;
  const rear = Math.max(wheelCenterZ - MICROMOUSE_BLUEPRINT.pcb.rearZ, wheelRear);

  return {
    dwbFrequency: 15,
    maxWheelRadPerSec: 42,
    maxLinearSpeed: 3,
    maxAngularSpeed: 8,
    maxLinearAccel: 16,
    maxLinearDecel: 20,
    maxAngularAccel: 20,
    maxAngularDecel: 24,
    trackWidth: wheelTrackWidth(),
    wheelRadius: MICROMOUSE_BLUEPRINT.wheel.radius,
    simTime: 0.85,
    simStep: 0.025,
    vxSamples: 9,
    omegaSamples: 21,
    waypointTolerance: 0.13,
    arrivalDistance: 0.22,
    robotHalfWidth: Math.max(MICROMOUSE_BLUEPRINT.pcb.width / 2, wheelOuterHalfWidth),
    robotFrontLength: front,
    robotRearLength: rear,
    robotFootprint: navigationFootprint(wheelCenterZ),
    robotFootprints: [navigationFootprint(wheelCenterZ)],
    safetyMargin: 0.08,
    wallThickness: WALL_THICKNESS,
  };
}

export function navigationDebugEnabled(): boolean {
  try {
    return (
      (typeof window !== "undefined" &&
        new URLSearchParams(window.location.search).has("debugPlanner")) ||
      (typeof window !== "undefined" &&
        window.localStorage.getItem("mazemouse3d.debugPlanner") === "1")
    );
  } catch {
    return false;
  }
}

export type NavigationDebugPayload = Record<string, unknown>;

export function emitNavigationDebug(payload: NavigationDebugPayload): void {
  console.info("[dwbMotorDriver]", payload);

  try {
    const global = (typeof window !== "undefined" ? window : globalThis) as typeof globalThis & {
      __MAZEMOUSE3D_NAVIGATION_DEBUG__?: NavigationDebugPayload[];
    };
    const buffer = (global.__MAZEMOUSE3D_NAVIGATION_DEBUG__ ??= []);
    buffer.push(payload);

    if (buffer.length > 240) {
      buffer.splice(0, buffer.length - 240);
    }
  } catch {
    return;
  }
}

function wheelTrackWidth(): number {
  const left = MICROMOUSE_BLUEPRINT.wheels.filter((wheel) => wheel.side === "left");
  const right = MICROMOUSE_BLUEPRINT.wheels.filter((wheel) => wheel.side === "right");
  const average = (values: readonly number[]): number =>
    values.reduce((sum, value) => sum + value, 0) / values.length;

  return average(right.map((wheel) => wheel.localX)) - average(left.map((wheel) => wheel.localX));
}

function smoothTwist(
  current: TwistState,
  target: TwistState,
  deltaSeconds: number,
  config: NavigationConfig,
): TwistState {
  if (deltaSeconds <= 0) {
    return current;
  }

  return {
    linearSpeed: approachAxis(
      current.linearSpeed,
      target.linearSpeed,
      config.maxLinearAccel,
      config.maxLinearDecel,
      deltaSeconds,
    ),
    angularSpeed: approachAxis(
      current.angularSpeed,
      target.angularSpeed,
      config.maxAngularAccel,
      config.maxAngularDecel,
      deltaSeconds,
    ),
  };
}

function twistToMotorCommand(twist: TwistState, config: NavigationConfig): MotorCommand {
  const leftLinear = twist.linearSpeed + (twist.angularSpeed * config.trackWidth) / 2;
  const rightLinear = twist.linearSpeed - (twist.angularSpeed * config.trackWidth) / 2;

  return clampMotorCommand(
    {
      leftRadPerSec: leftLinear / config.wheelRadius,
      rightRadPerSec: rightLinear / config.wheelRadius,
    },
    config.maxWheelRadPerSec,
  );
}

function approachAxis(
  current: number,
  target: number,
  accel: number,
  decel: number,
  deltaSeconds: number,
): number {
  const movingAwayFromZero = Math.abs(target) > Math.abs(current);
  const limit = movingAwayFromZero ? accel : decel;

  return approachScalar(current, target, limit, deltaSeconds);
}

function approachScalar(
  current: number,
  target: number,
  rate: number,
  deltaSeconds: number,
): number {
  const maxDelta = Math.max(0, rate) * Math.max(0, deltaSeconds);

  if (Math.abs(target - current) <= maxDelta) {
    return target;
  }

  return current + Math.sign(target - current) * maxDelta;
}

function navigationFootprint(wheelCenterZ: number): NavigationPoint[] {
  const points = boardFootprintPoints().map((point) => ({
    x: point.x,
    z: point.z - wheelCenterZ,
  }));
  const wheel = MICROMOUSE_BLUEPRINT.wheel;

  for (const layout of MICROMOUSE_BLUEPRINT.wheels) {
    for (const x of [layout.localX - wheel.width / 2, layout.localX + wheel.width / 2]) {
      for (const z of [layout.localZ - wheel.radius, layout.localZ + wheel.radius]) {
        points.push({
          x,
          z: z - wheelCenterZ,
        });
      }
    }
  }

  return convexHull(points);
}

function boardFootprintPoints(): NavigationPoint[] {
  const { pcb } = MICROMOUSE_BLUEPRINT;
  const halfWidth = pcb.width / 2;
  const sideIntersectionZ = circleSideIntersectionZ(
    halfWidth,
    pcb.frontArcCenterZ,
    pcb.frontRadius,
  );
  const wheelNotch = wheelCutout();
  const points: NavigationPoint[] = [
    { x: wheelNotch.innerX, z: pcb.rearZ },
    { x: wheelNotch.innerX, z: wheelNotch.frontZ },
    { x: halfWidth, z: wheelNotch.frontZ },
    { x: halfWidth, z: sideIntersectionZ },
  ];
  const sideAngle = Math.acos(halfWidth / pcb.frontRadius);

  for (let index = 1; index < BOARD_ARC_SEGMENTS; index += 1) {
    const angle = sideAngle + (index / BOARD_ARC_SEGMENTS) * (Math.PI - sideAngle * 2);
    points.push({
      x: Math.cos(angle) * pcb.frontRadius,
      z: pcb.frontArcCenterZ + Math.sin(angle) * pcb.frontRadius,
    });
  }

  points.push(
    { x: -halfWidth, z: sideIntersectionZ },
    { x: -halfWidth, z: wheelNotch.frontZ },
    { x: -wheelNotch.innerX, z: wheelNotch.frontZ },
    { x: -wheelNotch.innerX, z: pcb.rearZ },
  );

  return points;
}

function circleSideIntersectionZ(halfWidth: number, centerZ: number, radius: number): number {
  return centerZ + Math.sqrt(Math.max(0, radius * radius - halfWidth * halfWidth));
}

function wheelCutout(): { innerX: number; frontZ: number } {
  const margin = 0.012;
  const wheel = MICROMOUSE_BLUEPRINT.wheel;
  const innerX =
    Math.min(...MICROMOUSE_BLUEPRINT.wheels.map((layout) => Math.abs(layout.localX))) -
    wheel.width / 2 -
    margin;
  const frontZ =
    Math.max(...MICROMOUSE_BLUEPRINT.wheels.map((layout) => layout.localZ)) + wheel.radius + margin;

  return {
    innerX,
    frontZ,
  };
}

function convexHull(points: readonly NavigationPoint[]): NavigationPoint[] {
  const unique = [...points]
    .sort((a, b) => a.x - b.x || a.z - b.z)
    .filter(
      (point, index, sorted) =>
        index === 0 ||
        Math.hypot(point.x - sorted[index - 1].x, point.z - sorted[index - 1].z) > 0.000001,
    );

  if (unique.length <= 3) {
    return unique;
  }

  const lower: NavigationPoint[] = [];

  for (const point of unique) {
    while (
      lower.length >= 2 &&
      cross(lower[lower.length - 2], lower[lower.length - 1], point) <= 0
    ) {
      lower.pop();
    }

    lower.push(point);
  }

  const upper: NavigationPoint[] = [];

  for (const point of [...unique].reverse()) {
    while (
      upper.length >= 2 &&
      cross(upper[upper.length - 2], upper[upper.length - 1], point) <= 0
    ) {
      upper.pop();
    }

    upper.push(point);
  }

  lower.pop();
  upper.pop();

  return [...lower, ...upper];
}

function cross(a: NavigationPoint, b: NavigationPoint, c: NavigationPoint): number {
  return (b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x);
}
