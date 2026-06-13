import { plan_path as generatedPlanPath } from "../generated/maze-nav/maze_nav.js";
import type { MazeSnapshot } from "../domain/maze";
import { MICROMOUSE_BLUEPRINT } from "./micromouseModel";
import {
  STOPPED_COMMAND,
  clamp,
  clampMotorCommand,
  createSeededRandom,
  type MotorCommand,
  type RobotGroundTruthPose,
} from "./motorDriver";

export interface HalfGridPoint {
  readonly x2: number;
  readonly z2: number;
}

export interface PlanPathRequest {
  readonly size: number;
  readonly walls: readonly number[];
  readonly startX2: number;
  readonly startZ2: number;
  readonly startHeading: number;
  readonly goalCell: number;
}

export interface PlanPathResult {
  readonly cost: number;
  readonly steps: readonly (HalfGridPoint & { readonly heading: number })[];
  readonly waypoints: readonly HalfGridPoint[];
}

export type PathPlanner = (request: PlanPathRequest) => PlanPathResult;

export interface PlannedMotorDriverOptions {
  readonly planner?: PathPlanner;
  readonly seed?: number;
  readonly debug?: boolean;
  readonly debugIntervalSeconds?: number;
  readonly maxWheelRadPerSec?: number;
  readonly maxLinearSpeed?: number;
  readonly maxAngularSpeed?: number;
  readonly angularGain?: number;
  readonly lookaheadDistance?: number;
  readonly waypointTolerance?: number;
  readonly arrivalDistance?: number;
  readonly turnInPlaceAngle?: number;
  readonly turnInPlaceExitAngle?: number;
}

interface ResolvedPlannedMotorDriverOptions {
  readonly planner: PathPlanner;
  readonly seed: number;
  readonly debug: boolean;
  readonly debugIntervalSeconds: number;
  readonly maxWheelRadPerSec: number;
  readonly maxLinearSpeed: number;
  readonly maxAngularSpeed: number;
  readonly angularGain: number;
  readonly lookaheadDistance: number;
  readonly waypointTolerance: number;
  readonly arrivalDistance: number;
  readonly turnInPlaceAngle: number;
  readonly turnInPlaceExitAngle: number;
}

interface WorldPathPoint {
  readonly x: number;
  readonly z: number;
}

interface ControlDebugState {
  readonly targetDistance: number;
  readonly targetYaw: number;
  readonly headingError: number;
  readonly linearSpeed: number;
  readonly angularSpeed: number;
  readonly turningInPlace: boolean;
}

interface PathProjection {
  readonly progress: number;
  readonly segmentIndex: number;
  readonly distanceSquared: number;
}

export type PlannerDebugPayload = Record<string, unknown>;

const DEFAULT_OPTIONS: Omit<ResolvedPlannedMotorDriverOptions, "planner" | "seed"> = {
  debug: plannerDebugEnabled(),
  debugIntervalSeconds: 0.35,
  maxWheelRadPerSec: 18,
  maxLinearSpeed: 1.5,
  maxAngularSpeed: 4.8,
  angularGain: 5.4,
  lookaheadDistance: 0.34,
  waypointTolerance: 0.13,
  arrivalDistance: 0.18,
  turnInPlaceAngle: Math.PI * 0.58,
  turnInPlaceExitAngle: Math.PI / 3,
};

const REPLAN_ATTEMPT_MULTIPLIER = 2;
const MIN_TARGET_DISTANCE = 0.05;

export class PlannedMotorDriver {
  readonly #maze: MazeSnapshot;
  readonly #walls: number[];
  readonly #random: () => number;
  readonly #options: ResolvedPlannedMotorDriverOptions;
  readonly #trackWidth = wheelTrackWidth();
  #targetCell: number | null = null;
  #path: WorldPathPoint[] = [];
  #pathDistances: number[] = [];
  #pathProgress = 0;
  #pathIndex = 0;
  #turningInPlace = false;
  #command: MotorCommand = STOPPED_COMMAND;
  #lastControlDebug: ControlDebugState | null = null;
  #debugElapsed = 0;

  constructor(maze: MazeSnapshot, options: PlannedMotorDriverOptions = {}) {
    this.#maze = maze;
    this.#walls = Array.from(maze.walls);
    this.#random = createSeededRandom(options.seed ?? maze.seed);
    this.#options = {
      seed: options.seed ?? maze.seed,
      planner: options.planner ?? defaultPlanPath,
      debug: options.debug ?? DEFAULT_OPTIONS.debug,
      debugIntervalSeconds: options.debugIntervalSeconds ?? DEFAULT_OPTIONS.debugIntervalSeconds,
      maxWheelRadPerSec: options.maxWheelRadPerSec ?? DEFAULT_OPTIONS.maxWheelRadPerSec,
      maxLinearSpeed: options.maxLinearSpeed ?? DEFAULT_OPTIONS.maxLinearSpeed,
      maxAngularSpeed: options.maxAngularSpeed ?? DEFAULT_OPTIONS.maxAngularSpeed,
      angularGain: options.angularGain ?? DEFAULT_OPTIONS.angularGain,
      lookaheadDistance: options.lookaheadDistance ?? DEFAULT_OPTIONS.lookaheadDistance,
      waypointTolerance: options.waypointTolerance ?? DEFAULT_OPTIONS.waypointTolerance,
      arrivalDistance: options.arrivalDistance ?? DEFAULT_OPTIONS.arrivalDistance,
      turnInPlaceAngle: options.turnInPlaceAngle ?? DEFAULT_OPTIONS.turnInPlaceAngle,
      turnInPlaceExitAngle: options.turnInPlaceExitAngle ?? DEFAULT_OPTIONS.turnInPlaceExitAngle,
    };
  }

  next(deltaSeconds: number, pose: RobotGroundTruthPose): MotorCommand {
    if (
      this.#targetCell !== null &&
      this.#distanceToCellCenter(pose, this.#targetCell) <= this.#options.arrivalDistance
    ) {
      this.#debugPlanEvent("arrived", pose);
      this.#clearPath();
    }

    if (this.#path.length === 0 && !this.#planNewPath(pose)) {
      this.#command = STOPPED_COMMAND;
      return this.#command;
    }

    this.#updatePathProgress(pose.origin);

    const target = this.#lookaheadPoint(pose.origin);

    if (!target) {
      this.#command = STOPPED_COMMAND;
      return this.#command;
    }

    this.#command = this.#commandToward(pose, target);
    this.#debugTick(deltaSeconds, pose, target);
    return this.#command;
  }

  get command(): MotorCommand {
    return this.#command;
  }

  get targetCell(): number | null {
    return this.#targetCell;
  }

  #clearPath(): void {
    this.#path = [];
    this.#pathDistances = [];
    this.#pathProgress = 0;
    this.#pathIndex = 0;
    this.#turningInPlace = false;
  }

  #planNewPath(pose: RobotGroundTruthPose): boolean {
    const start = nearestPassableHalfGrid(this.#maze, pose.origin.x, pose.origin.z);

    if (!start) {
      this.#debugPlanEvent("plan-start-missing", pose);
      return false;
    }

    const currentCell = cellFromWorld(this.#maze, pose.origin.x, pose.origin.z);
    const totalCells = this.#maze.size * this.#maze.size;
    const maxAttempts = Math.max(1, totalCells * REPLAN_ATTEMPT_MULTIPLIER);

    for (let attempt = 0; attempt < maxAttempts; attempt += 1) {
      const targetCell = this.#chooseTargetCell(currentCell);
      const result = this.#tryPlan(start, quantizeHeading(pose.yaw), targetCell);

      if (!result || result.waypoints.length === 0) {
        continue;
      }

      this.#targetCell = targetCell;
      this.#path = result.waypoints.map((point) => ({
        x: point.x2 / 2,
        z: point.z2 / 2,
      }));
      this.#pathDistances = cumulativePathDistances(this.#path);
      this.#pathIndex = 0;
      this.#pathProgress = 0;
      this.#updatePathProgress(pose.origin);
      this.#debugPlanEvent("planned", pose, {
        cost: result.cost,
        start,
        startHeading: quantizeHeading(pose.yaw),
        waypointCount: result.waypoints.length,
      });
      return true;
    }

    this.#debugPlanEvent("plan-failed", pose, {
      start,
      currentCell,
      maxAttempts,
    });
    return false;
  }

  #chooseTargetCell(currentCell: number): number {
    const totalCells = this.#maze.size * this.#maze.size;

    if (totalCells <= 1) {
      return 0;
    }

    for (let attempt = 0; attempt < totalCells * REPLAN_ATTEMPT_MULTIPLIER; attempt += 1) {
      const candidate = Math.floor(this.#random() * totalCells);

      if (candidate !== currentCell && candidate !== this.#targetCell) {
        return candidate;
      }
    }

    for (let candidate = 0; candidate < totalCells; candidate += 1) {
      if (candidate !== currentCell && candidate !== this.#targetCell) {
        return candidate;
      }
    }

    return currentCell;
  }

  #tryPlan(start: HalfGridPoint, startHeading: number, goalCell: number): PlanPathResult | null {
    try {
      return this.#options.planner({
        size: this.#maze.size,
        walls: this.#walls,
        startX2: start.x2,
        startZ2: start.z2,
        startHeading,
        goalCell,
      });
    } catch (error) {
      if (this.#options.debug) {
        emitPlannerDebug({
          event: "plan-error",
          targetCell: goalCell,
          start,
          startHeading,
          error: error instanceof Error ? error.message : String(error),
        });
      }
      return null;
    }
  }

  #updatePathProgress(position: { readonly x: number; readonly z: number }): void {
    if (this.#path.length < 2) {
      this.#pathProgress = 0;
      this.#pathIndex = 0;
      return;
    }

    const projection = projectOntoPath(
      this.#path,
      this.#pathDistances,
      position,
      Math.max(0, this.#pathProgress - this.#options.waypointTolerance),
    );

    if (!projection) {
      return;
    }

    this.#pathProgress = Math.max(this.#pathProgress, projection.progress);
    this.#pathIndex = projection.segmentIndex;
  }

  #lookaheadPoint(_position: { readonly x: number; readonly z: number }): WorldPathPoint | null {
    if (this.#path.length === 0) {
      return null;
    }

    return samplePathAt(
      this.#path,
      this.#pathDistances,
      this.#pathProgress + this.#options.lookaheadDistance,
    );
  }

  #commandToward(pose: RobotGroundTruthPose, target: WorldPathPoint): MotorCommand {
    const dx = target.x - pose.origin.x;
    const dz = target.z - pose.origin.z;
    const distance = Math.hypot(dx, dz);

    if (distance < MIN_TARGET_DISTANCE) {
      return STOPPED_COMMAND;
    }

    const targetYaw = Math.atan2(dx, dz);
    const headingError = normalizeAngle(targetYaw - pose.yaw);
    const absError = Math.abs(headingError);
    const turningInPlace = this.#shouldTurnInPlace(absError);
    const angularSpeed = clamp(
      headingError * this.#options.angularGain,
      -this.#options.maxAngularSpeed,
      this.#options.maxAngularSpeed,
    );
    const distanceScale = clamp(distance / this.#options.lookaheadDistance, 0.25, 1);
    const headingScale = turningInPlace ? 0 : Math.max(0.16, Math.cos(absError));
    const linearSpeed = this.#options.maxLinearSpeed * distanceScale * headingScale;

    this.#lastControlDebug = {
      targetDistance: distance,
      targetYaw,
      headingError,
      linearSpeed,
      angularSpeed,
      turningInPlace,
    };

    return this.#wheelSpeeds(linearSpeed, angularSpeed);
  }

  #shouldTurnInPlace(absError: number): boolean {
    if (this.#turningInPlace) {
      this.#turningInPlace = absError > this.#options.turnInPlaceExitAngle;
    } else {
      this.#turningInPlace = absError > this.#options.turnInPlaceAngle;
    }

    return this.#turningInPlace;
  }

  #wheelSpeeds(linearSpeed: number, angularSpeed: number): MotorCommand {
    const wheelRadius = MICROMOUSE_BLUEPRINT.wheel.radius;
    const leftLinear = linearSpeed + (angularSpeed * this.#trackWidth) / 2;
    const rightLinear = linearSpeed - (angularSpeed * this.#trackWidth) / 2;

    return clampMotorCommand(
      {
        leftRadPerSec: leftLinear / wheelRadius,
        rightRadPerSec: rightLinear / wheelRadius,
      },
      this.#options.maxWheelRadPerSec,
    );
  }

  #distanceToCellCenter(pose: RobotGroundTruthPose, cell: number): number {
    const center = cellCenterWorld(this.#maze, cell);

    return distance2D(pose.origin, center);
  }

  #debugPlanEvent(
    event: string,
    pose: RobotGroundTruthPose,
    details: Record<string, unknown> = {},
  ): void {
    if (!this.#options.debug) {
      return;
    }

    emitPlannerDebug({
      event,
      targetCell: this.#targetCell,
      pose: compactPose(pose),
      ...details,
    });
  }

  #debugTick(deltaSeconds: number, pose: RobotGroundTruthPose, target: WorldPathPoint): void {
    if (!this.#options.debug) {
      return;
    }

    this.#debugElapsed += deltaSeconds;

    if (this.#debugElapsed < this.#options.debugIntervalSeconds) {
      return;
    }

    this.#debugElapsed = 0;
    emitPlannerDebug({
      event: "control",
      targetCell: this.#targetCell,
      pathIndex: this.#pathIndex,
      pathProgress: roundDebug(this.#pathProgress),
      pathLength: roundDebug(this.#pathDistances[this.#pathDistances.length - 1] ?? 0),
      pose: compactPose(pose),
      target: compactPoint(target),
      control: this.#lastControlDebug
        ? {
            targetDistance: roundDebug(this.#lastControlDebug.targetDistance),
            targetYawDeg: roundDebug(radiansToDegrees(this.#lastControlDebug.targetYaw)),
            headingErrorDeg: roundDebug(radiansToDegrees(this.#lastControlDebug.headingError)),
            linearSpeed: roundDebug(this.#lastControlDebug.linearSpeed),
            angularSpeed: roundDebug(this.#lastControlDebug.angularSpeed),
            turningInPlace: this.#lastControlDebug.turningInPlace,
          }
        : null,
      command: {
        leftRadPerSec: roundDebug(this.#command.leftRadPerSec),
        rightRadPerSec: roundDebug(this.#command.rightRadPerSec),
      },
    });
  }
}

function defaultPlanPath(request: PlanPathRequest): PlanPathResult {
  return generatedPlanPath(request) as PlanPathResult;
}

export function nearestPassableHalfGrid(
  maze: Pick<MazeSnapshot, "size" | "walls">,
  x: number,
  z: number,
): HalfGridPoint | null {
  const limit = maze.size * 2;
  const targetX2 = clamp(Math.round(x * 2), 1, limit - 1);
  const targetZ2 = clamp(Math.round(z * 2), 1, limit - 1);
  let best: { point: HalfGridPoint; distanceSquared: number } | null = null;

  for (let z2 = 1; z2 < limit; z2 += 1) {
    for (let x2 = 1; x2 < limit; x2 += 1) {
      if (!isPassableHalfGrid(maze, x2, z2)) {
        continue;
      }

      const distanceSquared = (x2 - targetX2) ** 2 + (z2 - targetZ2) ** 2;

      if (!best || distanceSquared < best.distanceSquared) {
        best = {
          point: { x2, z2 },
          distanceSquared,
        };
      }
    }
  }

  return best?.point ?? null;
}

export function isPassableHalfGrid(
  maze: Pick<MazeSnapshot, "size" | "walls">,
  x2: number,
  z2: number,
): boolean {
  const limit = maze.size * 2;

  if (x2 <= 0 || z2 <= 0 || x2 >= limit || z2 >= limit) {
    return false;
  }

  const oddX = x2 % 2 !== 0;
  const oddZ = z2 % 2 !== 0;

  if (oddX && oddZ) {
    return true;
  }

  if (!oddX && !oddZ) {
    return false;
  }

  if (!oddX) {
    const row = (z2 - 1) / 2;
    const col = x2 / 2 - 1;

    return row >= 0 && row < maze.size && col >= 0 && col + 1 < maze.size
      ? (maze.walls[row * maze.size + col] & 2) === 0
      : false;
  }

  const row = z2 / 2 - 1;
  const col = (x2 - 1) / 2;

  return row >= 0 && row + 1 < maze.size && col >= 0 && col < maze.size
    ? (maze.walls[row * maze.size + col] & 1) === 0
    : false;
}

export function quantizeHeading(yaw: number): number {
  return modulo(Math.round(yaw / (Math.PI / 4)), 8);
}

function cellFromWorld(maze: Pick<MazeSnapshot, "size">, x: number, z: number): number {
  const col = Math.trunc(clamp(Math.floor(x), 0, maze.size - 1));
  const row = Math.trunc(clamp(Math.floor(z), 0, maze.size - 1));

  return row * maze.size + col;
}

function cellCenterWorld(maze: Pick<MazeSnapshot, "size">, cell: number): WorldPathPoint {
  return {
    x: (cell % maze.size) + 0.5,
    z: Math.floor(cell / maze.size) + 0.5,
  };
}

function distance2D(
  from: { readonly x: number; readonly z: number },
  to: { readonly x: number; readonly z: number },
): number {
  return Math.hypot(to.x - from.x, to.z - from.z);
}

function cumulativePathDistances(path: readonly WorldPathPoint[]): number[] {
  const distances = [0];

  for (let index = 1; index < path.length; index += 1) {
    distances.push(distances[index - 1] + distance2D(path[index - 1], path[index]));
  }

  return distances;
}

function projectOntoPath(
  path: readonly WorldPathPoint[],
  distances: readonly number[],
  position: { readonly x: number; readonly z: number },
  minimumProgress: number,
): PathProjection | null {
  let best: PathProjection | null = null;

  for (let index = 0; index + 1 < path.length; index += 1) {
    const start = path[index];
    const end = path[index + 1];
    const dx = end.x - start.x;
    const dz = end.z - start.z;
    const lengthSquared = dx * dx + dz * dz;

    if (lengthSquared <= Number.EPSILON) {
      continue;
    }

    const rawT = ((position.x - start.x) * dx + (position.z - start.z) * dz) / lengthSquared;
    const t = clamp(rawT, 0, 1);
    const progress = distances[index] + Math.sqrt(lengthSquared) * t;

    if (progress < minimumProgress) {
      continue;
    }

    const projectionX = start.x + dx * t;
    const projectionZ = start.z + dz * t;
    const distanceSquared = (position.x - projectionX) ** 2 + (position.z - projectionZ) ** 2;

    if (!best || distanceSquared < best.distanceSquared) {
      best = {
        progress,
        segmentIndex: index,
        distanceSquared,
      };
    }
  }

  return best;
}

function samplePathAt(
  path: readonly WorldPathPoint[],
  distances: readonly number[],
  progress: number,
): WorldPathPoint | null {
  if (path.length === 0) {
    return null;
  }

  if (path.length === 1) {
    return path[0] ?? null;
  }

  const totalLength = distances[distances.length - 1] ?? 0;
  const clampedProgress = clamp(progress, 0, totalLength);

  for (let index = 0; index + 1 < path.length; index += 1) {
    const segmentStart = distances[index];
    const segmentEnd = distances[index + 1];

    if (clampedProgress > segmentEnd && index + 2 < path.length) {
      continue;
    }

    const segmentLength = Math.max(Number.EPSILON, segmentEnd - segmentStart);
    const t = clamp((clampedProgress - segmentStart) / segmentLength, 0, 1);

    return {
      x: path[index].x + (path[index + 1].x - path[index].x) * t,
      z: path[index].z + (path[index + 1].z - path[index].z) * t,
    };
  }

  return path[path.length - 1] ?? null;
}

function normalizeAngle(angle: number): number {
  let normalized = modulo(angle + Math.PI, Math.PI * 2) - Math.PI;

  if (normalized <= -Math.PI) {
    normalized += Math.PI * 2;
  }

  return normalized;
}

function modulo(value: number, divisor: number): number {
  return ((value % divisor) + divisor) % divisor;
}

function wheelTrackWidth(): number {
  const left = MICROMOUSE_BLUEPRINT.wheels.filter((wheel) => wheel.side === "left");
  const right = MICROMOUSE_BLUEPRINT.wheels.filter((wheel) => wheel.side === "right");
  const average = (values: readonly number[]): number =>
    values.reduce((sum, value) => sum + value, 0) / values.length;

  return average(right.map((wheel) => wheel.localX)) - average(left.map((wheel) => wheel.localX));
}

export function plannerDebugEnabled(): boolean {
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

function compactPose(pose: RobotGroundTruthPose): Record<string, number> {
  return {
    x: roundDebug(pose.origin.x),
    z: roundDebug(pose.origin.z),
    yawDeg: roundDebug(radiansToDegrees(pose.yaw)),
  };
}

function compactPoint(point: WorldPathPoint): Record<string, number> {
  return {
    x: roundDebug(point.x),
    z: roundDebug(point.z),
  };
}

function radiansToDegrees(radians: number): number {
  return (radians * 180) / Math.PI;
}

function roundDebug(value: number): number {
  return Math.round(value * 1000) / 1000;
}

export function emitPlannerDebug(payload: PlannerDebugPayload): void {
  console.info("[plannedMotorDriver]", payload);

  try {
    const global = (typeof window !== "undefined" ? window : globalThis) as typeof globalThis & {
      __MAZEMOUSE3D_PLANNER_DEBUG__?: PlannerDebugPayload[];
    };
    const buffer = (global.__MAZEMOUSE3D_PLANNER_DEBUG__ ??= []);
    buffer.push(payload);

    if (buffer.length > 240) {
      buffer.splice(0, buffer.length - 240);
    }
  } catch {
    return;
  }
}
