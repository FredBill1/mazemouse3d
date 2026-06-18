import HavokPhysics from "@babylonjs/havok";
import { ArcRotateCamera } from "@babylonjs/core/Cameras/arcRotateCamera.js";
import { Camera } from "@babylonjs/core/Cameras/camera.js";
import { Engine } from "@babylonjs/core/Engines/engine.js";
import "@babylonjs/core/Engines/AbstractEngine/abstractEngine.views.js";
import { HemisphericLight } from "@babylonjs/core/Lights/hemisphericLight.js";
import { StandardMaterial } from "@babylonjs/core/Materials/standardMaterial.js";
import { Color3, Color4 } from "@babylonjs/core/Maths/math.color.js";
import { Vector3 } from "@babylonjs/core/Maths/math.vector.js";
import { Mesh } from "@babylonjs/core/Meshes/mesh.js";
import { MeshBuilder } from "@babylonjs/core/Meshes/meshBuilder.js";
import { TransformNode } from "@babylonjs/core/Meshes/transformNode.js";
import { Scene } from "@babylonjs/core/scene.js";
import "@babylonjs/core/Physics/physicsEngineComponent.js";
import { PhysicsAggregate } from "@babylonjs/core/Physics/v2/physicsAggregate.js";
import { PhysicsShapeType } from "@babylonjs/core/Physics/v2/IPhysicsEnginePlugin.js";
import { PhysicsMaterialCombineMode } from "@babylonjs/core/Physics/v2/physicsMaterial.js";
import { HavokPlugin } from "@babylonjs/core/Physics/v2/Plugins/havokPlugin.js";
import { SimulationDebugStore } from "../app/simulationDebug";
import type { AppState, AppStateSnapshot } from "../app/state";
import type { MazeSnapshot } from "../domain/maze";
import { buildWallSegments, cellCenter } from "./mazeGeometry";
import {
  MicromouseRig,
  createMicromouseMaterials,
  type MicromouseMaterials,
} from "./micromouseRig";
import { MICROMOUSE_BLUEPRINT } from "./micromouseModel";
import type { MotorCommand } from "./motorDriver";
import { DwaMotorDriver } from "./dwaMotorDriver";

export type MazeViewMode = "perspective" | "top";

interface Disposable {
  dispose(): void;
}

interface MazeMaterials {
  readonly floor: StandardMaterial;
  readonly wall: StandardMaterial;
  readonly start: StandardMaterial;
  readonly goal: StandardMaterial;
  readonly path: StandardMaterial;
}

interface RegisteredMazeView {
  readonly canvas: HTMLCanvasElement;
  readonly container: HTMLElement;
  readonly mode: MazeViewMode;
  readonly resizeObserver: ResizeObserver;
  camera: ArcRotateCamera | null;
  cleanupInput: (() => void) | null;
  disposed: boolean;
}

const WALL_HEIGHT = 0.36;
const WALL_THICKNESS = 0.08;
const FLOOR_COLLIDER_THICKNESS = 0.04;
const PHYSICS_SUB_STEP_MS = 1000 / 120;
const PHYSICS_STEP_SECONDS = 1 / 120;
const DRAW_SOLUTION_CELLS = false;
const GUIDED_TRACK_SPEED = 5.35;
const GUIDED_TRACK_LOOKAHEAD = 0.62;
const GUIDED_TRACK_MAX_YAW_RATE = 120;

export class BabylonMazeSimulation {
  readonly #host: HTMLElement;
  readonly #state: AppState;
  readonly #debugStore: SimulationDebugStore;
  readonly #masterCanvas = document.createElement("canvas");
  readonly #views = new Set<RegisteredMazeView>();

  #engine: Engine | null = null;
  #scene: Scene | null = null;
  #materials: MazeMaterials | null = null;
  #mouseMaterials: MicromouseMaterials | null = null;
  #mazeRoot: TransformNode | null = null;
  #physicsAggregates: PhysicsAggregate[] = [];
  #unsubscribe: (() => void) | null = null;
  #currentMaze: MazeSnapshot | null = null;
  #micromouse: MicromouseRig | null = null;
  #motorDriver: DwaMotorDriver | null = null;
  #motorCommand: MotorCommand = { leftRadPerSec: 0, rightRadPerSec: 0 };
  #elapsedSeconds = 0;
  #totalDistance = 0;
  #wallCollisionCount = 0;
  #debugUpdateSeconds = 0;
  #lastDistancePoint: { x: number; z: number } | null = null;
  #guidePath: { x: number; z: number }[] = [];
  #guideDistances: number[] = [];
  #guideProgress = 0;
  #guideDirection: 1 | -1 = 1;
  #guideYaw = 0;
  #disposed = false;
  #started = false;
  #physicsEnabled = false;
  #physicsAdvancedThisFrame = false;

  constructor(host: HTMLElement, state: AppState, debugStore = new SimulationDebugStore()) {
    this.#host = host;
    this.#state = state;
    this.#debugStore = debugStore;
    this.#masterCanvas.className = "scene-master-canvas";
    this.#masterCanvas.setAttribute("aria-hidden", "true");
    this.#host.append(this.#masterCanvas);
  }

  async start(): Promise<void> {
    if (this.#started) {
      return;
    }

    this.#started = true;
    this.#engine = new Engine(this.#masterCanvas, true, {
      adaptToDeviceRatio: true,
      antialias: true,
      stencil: true,
    });
    this.#scene = new Scene(this.#engine);
    this.#scene.clearColor = new Color4(0.045, 0.052, 0.062, 1);
    this.#materials = createMazeMaterials(this.#scene);
    this.#mouseMaterials = createMicromouseMaterials(this.#scene);

    await this.#enablePhysics();

    if (this.#disposed || !this.#scene || !this.#engine) {
      return;
    }

    new HemisphericLight("ambient-light", new Vector3(0.2, 1, 0.4), this.#scene).intensity = 0.86;

    this.#engine.onBeginFrameObservable.add(() => {
      this.#physicsAdvancedThisFrame = false;

      if (this.#scene) {
        this.#scene.physicsEnabled = this.#physicsEnabled;
      }
    });
    this.#engine.onBeforeViewRenderObservable.add(() => {
      if (!this.#scene) {
        return;
      }

      this.#scene.physicsEnabled = this.#physicsEnabled && !this.#physicsAdvancedThisFrame;
      this.#physicsAdvancedThisFrame = true;
    });
    this.#scene.onBeforePhysicsObservable.add(() => this.#updateMicromouseControl());

    for (const view of this.#views) {
      this.#registerView(view);
    }

    this.#unsubscribe = this.#state.subscribe((snapshot) => this.#renderAppState(snapshot));
    this.#engine.runRenderLoop(() => {
      this.#scene?.render();
    });
  }

  attachView(container: HTMLElement, mode: MazeViewMode): Disposable {
    const canvas = document.createElement("canvas");
    canvas.className = "scene-canvas";
    canvas.setAttribute("aria-label", mode === "perspective" ? "3D scene" : "top view");
    container.classList.add("scene-panel");
    container.append(canvas);

    const view: RegisteredMazeView = {
      canvas,
      container,
      mode,
      resizeObserver: new ResizeObserver(() => this.#resizeView(canvas)),
      camera: null,
      cleanupInput: null,
      disposed: false,
    };
    view.resizeObserver.observe(container);
    this.#views.add(view);
    this.#registerView(view);

    return {
      dispose: () => {
        view.disposed = true;
        view.resizeObserver.disconnect();
        view.cleanupInput?.();
        this.#engine?.unRegisterView(view.canvas);
        view.camera?.dispose();
        view.canvas.remove();
        this.#views.delete(view);
      },
    };
  }

  dispose(): void {
    this.#disposed = true;
    this.#unsubscribe?.();

    for (const view of [...this.#views]) {
      view.resizeObserver.disconnect();
      view.cleanupInput?.();
      this.#engine?.unRegisterView(view.canvas);
      view.camera?.dispose();
      view.canvas.remove();
      this.#views.delete(view);
    }

    this.#clearMaze();
    this.#debugStore.reset();
    this.#scene?.dispose();
    this.#engine?.dispose();
    this.#masterCanvas.remove();
  }

  async #enablePhysics(): Promise<void> {
    if (!this.#scene) {
      return;
    }

    const havok = await HavokPhysics();

    if (this.#disposed || !this.#scene) {
      return;
    }

    this.#scene.enablePhysics(new Vector3(0, -9.81, 0), new HavokPlugin(true, havok));

    const physicsEngine = this.#scene.getPhysicsEngine();
    this.#physicsEnabled = physicsEngine !== null;
    physicsEngine?.setTimeStep(PHYSICS_STEP_SECONDS);
    physicsEngine?.setSubTimeStep(PHYSICS_SUB_STEP_MS);
    const v2Engine = physicsEngine as {
      setVelocityLimits?: (linear: number, angular: number) => void;
    };
    v2Engine?.setVelocityLimits?.(8, 100);
  }

  #registerView(view: RegisteredMazeView): void {
    if (view.disposed || !this.#engine || !this.#scene || view.camera) {
      return;
    }

    if (view.mode === "perspective") {
      this.#engine.inputElement = view.canvas;
      this.#scene.detachControl();
      this.#scene.attachControl(true, true, true);
    }

    view.camera = this.#createCamera(view.mode, this.#currentMaze?.size ?? 16);
    this.#engine.registerView(view.canvas, view.camera, true);
    this.#resizeView(view.canvas);

    if (!this.#scene.activeCamera) {
      this.#scene.activeCamera = view.camera;
    }

    if (view.mode === "perspective") {
      this.#scene.cameraToUseForPointers = view.camera;
      view.cleanupInput = attachPerspectiveInputGuards(view.canvas);
    }
  }

  #createCamera(mode: MazeViewMode, size: number): ArcRotateCamera {
    const target = new Vector3(size / 2, 0, size / 2);
    const camera = new ArcRotateCamera(
      `${mode}-camera`,
      mode === "top" ? -Math.PI / 2 : (-Math.PI * 3) / 4,
      mode === "top" ? 0.01 : Math.PI / 3,
      mode === "top" ? size * 1.05 : size * 1.45,
      target,
      this.#scene!,
    );
    camera.minZ = 0.01;

    if (mode === "top") {
      camera.mode = Camera.ORTHOGRAPHIC_CAMERA;
    } else {
      camera.attachControl(false, false, 1);
      camera.lowerRadiusLimit = 2;
      camera.mapPanning = true; // disable y-axis panning
      camera.panningAxis.set(1, 1, 0);
      camera.wheelDeltaPercentage = 0.15;
      camera.pinchDeltaPercentage = 0.015;
      camera.angularSensibilityX = 300;
      camera.angularSensibilityY = 300;
      camera.inertia = 0.05;
      camera.panningInertia = 0;
      this.#scene!.onBeforeRenderObservable.add(() => {
        camera.panningSensibility = 800 / camera.radius;
        const pitchFactor = Math.max(Math.pow(Math.abs(Math.cos(camera.beta)), 1.5), 1e-6);
        camera.panningAxis.y = 1 / pitchFactor;
      });
    }
    return camera;
  }

  #resizeView(canvas: HTMLCanvasElement): void {
    if (!this.#engine) {
      return;
    }

    const width = Math.max(1, Math.floor(canvas.clientWidth));
    const height = Math.max(1, Math.floor(canvas.clientHeight));

    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }

    const view = [...this.#views].find((candidate) => candidate.canvas === canvas);

    if (view?.mode === "top" && view.camera) {
      updateTopCamera(view.camera, this.#currentMaze?.size ?? 16, width / height);
    }
  }

  #renderAppState(snapshot: AppStateSnapshot): void {
    if (!snapshot.maze || snapshot.maze === this.#currentMaze) {
      return;
    }

    this.#renderMaze(snapshot.maze);
  }

  #renderMaze(maze: MazeSnapshot): void {
    if (!this.#scene || !this.#materials || !this.#mouseMaterials) {
      return;
    }

    this.#clearMaze();
    this.#currentMaze = maze;
    this.#updateCameras(maze.size);

    const root = new TransformNode("maze-root", this.#scene);
    this.#mazeRoot = root;

    const floor = MeshBuilder.CreateGround(
      "maze-floor",
      { width: maze.size, height: maze.size, subdivisions: maze.size },
      this.#scene,
    );
    floor.position.set(maze.size / 2, 0, maze.size / 2);
    floor.material = this.#materials.floor;
    floor.metadata = { sensorBeamTarget: true };
    floor.parent = root;

    const floorCollider = MeshBuilder.CreateBox(
      "maze-floor-collider",
      { width: maze.size, height: FLOOR_COLLIDER_THICKNESS, depth: maze.size },
      this.#scene,
    );
    floorCollider.position.set(maze.size / 2, -FLOOR_COLLIDER_THICKNESS / 2, maze.size / 2);
    floorCollider.isVisible = false;
    this.#addStaticBody(floorCollider, 1.05);

    for (const segment of buildWallSegments(maze, WALL_THICKNESS)) {
      const wall = MeshBuilder.CreateBox(
        "maze-wall",
        { width: segment.sizeX, height: WALL_HEIGHT, depth: segment.sizeZ },
        this.#scene,
      );
      wall.position.set(segment.centerX, WALL_HEIGHT / 2, segment.centerZ);
      wall.material = this.#materials.wall;
      wall.metadata = { sensorBeamTarget: true, collisionRole: "wall" };
      wall.parent = root;
      this.#addStaticBody(wall, 0.82);
    }

    this.#addCellMarker("start-marker", maze.start, maze, this.#materials.start, 0.72, 0.022);

    for (const goal of maze.goals) {
      this.#addCellMarker("goal-marker", goal, maze, this.#materials.goal, 0.72, 0.024);
    }

    if (DRAW_SOLUTION_CELLS) {
      for (const cell of maze.solution) {
        this.#addCellMarker("solution-cell", cell, maze, this.#materials.path, 0.18, 0.036);
      }
    }

    this.#createMicromouse(maze);
  }

  #updateCameras(size: number): void {
    for (const view of this.#views) {
      const camera = view.camera;

      if (!camera) {
        continue;
      }

      camera.setTarget(new Vector3(size / 2, 0, size / 2));

      if (view.mode === "top") {
        camera.radius = size * 1.05;
        updateTopCamera(camera, size, view.canvas.width / Math.max(1, view.canvas.height));
      } else {
        camera.radius = size * 1.45;
      }
    }
  }

  #addCellMarker(
    name: string,
    cell: number,
    maze: MazeSnapshot,
    material: StandardMaterial,
    size: number,
    y: number,
  ): void {
    if (!this.#scene || !this.#mazeRoot) {
      return;
    }

    const point = cellCenter(cell, maze.size);
    const marker = MeshBuilder.CreateBox(
      name,
      { width: size, height: 0.02, depth: size },
      this.#scene,
    );
    marker.position.set(point.x, y, point.z);
    marker.material = material;
    marker.parent = this.#mazeRoot;
  }

  #createMicromouse(maze: MazeSnapshot): void {
    if (!this.#scene || !this.#mouseMaterials) {
      return;
    }

    this.#micromouse?.dispose();
    this.#motorDriver?.dispose();
    this.#micromouse = new MicromouseRig(this.#scene, this.#mouseMaterials, maze, {
      onWallCollision: () => {
        this.#wallCollisionCount += 1;
      },
    });
    this.#motorDriver = new DwaMotorDriver(maze);
    this.#motorCommand = { leftRadPerSec: 0, rightRadPerSec: 0 };
    this.#elapsedSeconds = 0;
    this.#totalDistance = 0;
    this.#wallCollisionCount = 0;
    this.#debugUpdateSeconds = 0;
    this.#lastDistancePoint = null;
    this.#resetGuidedTrack(maze);
    this.#debugStore.reset();
  }

  #updateMicromouseControl(): void {
    if (!this.#currentMaze || !this.#micromouse || !this.#motorDriver) {
      return;
    }

    if (this.#micromouse.shouldReset(this.#currentMaze)) {
      this.#createMicromouse(this.#currentMaze);
      return;
    }

    const driverTelemetry = this.#micromouse.getGroundTruthTelemetry();
    this.#motorCommand = this.#motorDriver.next(PHYSICS_STEP_SECONDS, driverTelemetry);
    this.#micromouse.setMotorCommand(this.#motorCommand);
    this.#applyGuidedTrack(PHYSICS_STEP_SECONDS);

    const telemetry = this.#micromouse.getGroundTruthTelemetry();

    this.#updateRunMetrics(PHYSICS_STEP_SECONDS, telemetry.origin);
    this.#publishDebugSnapshot(PHYSICS_STEP_SECONDS, telemetry);
  }

  #resetGuidedTrack(maze: MazeSnapshot): void {
    const solution = maze.solution.length > 1 ? maze.solution : [maze.start];
    this.#guidePath = solution.map((cell) => cellCenter(cell, maze.size));
    this.#guideDistances = cumulativeDistances(this.#guidePath);
    this.#guideProgress = 0;
    this.#guideDirection = 1;
    this.#guideYaw = initialGuideYaw(this.#guidePath);
  }

  #applyGuidedTrack(deltaSeconds: number): void {
    if (!this.#micromouse || this.#guidePath.length < 2) {
      return;
    }

    const totalLength = this.#guideDistances[this.#guideDistances.length - 1] ?? 0;

    if (totalLength <= 0) {
      return;
    }

    this.#guideProgress += GUIDED_TRACK_SPEED * deltaSeconds * this.#guideDirection;

    if (this.#guideProgress >= totalLength) {
      this.#guideProgress = totalLength;
      this.#guideDirection = -1;
    } else if (this.#guideProgress <= 0) {
      this.#guideProgress = 0;
      this.#guideDirection = 1;
    }

    const point = samplePathAtDistance(this.#guidePath, this.#guideDistances, this.#guideProgress);
    const lookaheadProgress = clampNumber(
      this.#guideProgress + GUIDED_TRACK_LOOKAHEAD * this.#guideDirection,
      0,
      totalLength,
    );
    const lookahead = samplePathAtDistance(
      this.#guidePath,
      this.#guideDistances,
      lookaheadProgress,
    );
    const desiredYaw = Math.atan2(lookahead.x - point.x, lookahead.z - point.z);
    const previousYaw = this.#guideYaw;
    this.#guideYaw = approachAngle(
      this.#guideYaw,
      desiredYaw,
      GUIDED_TRACK_MAX_YAW_RATE * deltaSeconds,
    );
    const linearVelocity = {
      x: Math.sin(this.#guideYaw) * GUIDED_TRACK_SPEED * this.#guideDirection,
      y: 0,
      z: Math.cos(this.#guideYaw) * GUIDED_TRACK_SPEED * this.#guideDirection,
    };
    const angularVelocityY = normalizeAngle(this.#guideYaw - previousYaw) / deltaSeconds;

    this.#micromouse.setGuidedGroundTruthPose(
      { x: point.x, y: MICROMOUSE_BLUEPRINT.chassis.centerY, z: point.z },
      this.#guideYaw,
      linearVelocity,
      angularVelocityY,
    );
  }

  #updateRunMetrics(
    deltaSeconds: number,
    origin: { readonly x: number; readonly z: number },
  ): void {
    this.#elapsedSeconds += deltaSeconds;

    if (this.#lastDistancePoint) {
      this.#totalDistance += Math.hypot(
        origin.x - this.#lastDistancePoint.x,
        origin.z - this.#lastDistancePoint.z,
      );
    }

    this.#lastDistancePoint = {
      x: origin.x,
      z: origin.z,
    };
  }

  #publishDebugSnapshot(
    deltaSeconds: number,
    telemetry: ReturnType<MicromouseRig["getGroundTruthTelemetry"]>,
  ): void {
    this.#debugUpdateSeconds += deltaSeconds;

    if (this.#debugUpdateSeconds < 0.1) {
      return;
    }

    this.#debugUpdateSeconds = 0;
    const driverDebug = this.#motorDriver?.debug;

    this.#debugStore.setSnapshot({
      elapsedSeconds: this.#elapsedSeconds,
      fps: this.#engine?.getFps() ?? 0,
      dwaHz: driverDebug?.dwaHz ?? 0,
      workerStatus: driverDebug?.workerStatus ?? "starting",
      workerRestartCount: driverDebug?.restartCount ?? 0,
      lastWorkerLatencyMs: driverDebug?.lastWorkerLatencyMs ?? null,
      lastWorkerError: driverDebug?.lastWorkerError ?? null,
      pose: telemetry.origin,
      yaw: telemetry.yaw,
      eulerAngles: telemetry.eulerAngles,
      linearVelocity: telemetry.linearVelocity,
      angularVelocity: telemetry.angularVelocity,
      horizontalSpeed: telemetry.horizontalSpeed,
      totalDistance: this.#totalDistance,
      averageSpeed: this.#elapsedSeconds > 0 ? this.#totalDistance / this.#elapsedSeconds : 0,
      wallCollisionCount: this.#wallCollisionCount,
      lastCommand: driverDebug?.lastCommand ?? this.#motorCommand,
      lastDwaDebug: driverDebug?.lastDwaDebug ?? null,
    });
  }

  #addStaticBody(mesh: Mesh, friction: number): void {
    if (!this.#scene || !this.#physicsEnabled) {
      return;
    }

    const aggregate = new PhysicsAggregate(
      mesh,
      PhysicsShapeType.BOX,
      { mass: 0, friction, restitution: 0.01 },
      this.#scene,
    );
    aggregate.shape.material = {
      friction,
      staticFriction: friction * 1.15,
      restitution: 0.01,
      frictionCombine: PhysicsMaterialCombineMode.ARITHMETIC_MEAN,
    };
    this.#physicsAggregates.push(aggregate);
  }

  #clearMaze(): void {
    this.#micromouse?.dispose();
    this.#micromouse = null;
    this.#motorDriver?.dispose();
    this.#motorDriver = null;

    for (const aggregate of this.#physicsAggregates) {
      aggregate.dispose();
    }

    this.#physicsAggregates = [];
    this.#mazeRoot?.dispose(false, true);
    this.#mazeRoot = null;
    this.#currentMaze = null;
  }
}

function createMazeMaterials(scene: Scene): MazeMaterials {
  return {
    floor: material(
      scene,
      "floor-material",
      new Color3(0.12, 0.16, 0.15),
      new Color3(0.02, 0.03, 0.03),
    ),
    wall: material(
      scene,
      "wall-material",
      new Color3(0.73, 0.78, 0.82),
      new Color3(0.11, 0.12, 0.13),
    ),
    start: material(
      scene,
      "start-material",
      new Color3(0.17, 0.72, 0.44),
      new Color3(0.04, 0.16, 0.1),
    ),
    goal: material(
      scene,
      "goal-material",
      new Color3(0.95, 0.68, 0.2),
      new Color3(0.2, 0.12, 0.02),
    ),
    path: material(
      scene,
      "path-material",
      new Color3(0.2, 0.55, 0.88),
      new Color3(0.02, 0.06, 0.12),
    ),
  };
}

function material(
  scene: Scene,
  name: string,
  diffuseColor: Color3,
  emissiveColor: Color3,
): StandardMaterial {
  const result = new StandardMaterial(name, scene);
  result.diffuseColor = diffuseColor;
  result.emissiveColor = emissiveColor;
  result.specularColor = new Color3(0.18, 0.18, 0.18);

  return result;
}

function updateTopCamera(camera: ArcRotateCamera, size: number, aspect: number): void {
  const half = size / 2 + 0.8;

  if (aspect >= 1) {
    // canvas is tall
    camera.orthoLeft = -half * aspect;
    camera.orthoRight = half * aspect;
    camera.orthoBottom = -half;
    camera.orthoTop = half;
  } else {
    // canvas is wide
    camera.orthoLeft = -half;
    camera.orthoRight = half;
    camera.orthoBottom = -half / aspect;
    camera.orthoTop = half / aspect;
  }
}

function attachPerspectiveInputGuards(canvas: HTMLCanvasElement): () => void {
  const preventMiddleButtonDefault = (event: MouseEvent): void => {
    if (event.button === 1) {
      event.preventDefault();
    }
  };

  const focusCanvas = (): void => {
    canvas.focus();
  };

  canvas.tabIndex = 0;
  canvas.addEventListener("pointerdown", focusCanvas);
  canvas.addEventListener("auxclick", preventMiddleButtonDefault);

  return () => {
    canvas.removeEventListener("pointerdown", focusCanvas);
    canvas.removeEventListener("auxclick", preventMiddleButtonDefault);
  };
}

function cumulativeDistances(path: readonly { x: number; z: number }[]): number[] {
  const distances = [0];

  for (let index = 1; index < path.length; index += 1) {
    const previous = path[index - 1];
    const current = path[index];
    distances.push(
      distances[index - 1] + Math.hypot(current.x - previous.x, current.z - previous.z),
    );
  }

  return distances;
}

function samplePathAtDistance(
  path: readonly { x: number; z: number }[],
  distances: readonly number[],
  progress: number,
): { x: number; z: number } {
  if (path.length === 0) {
    return { x: 0, z: 0 };
  }

  if (path.length === 1) {
    return path[0];
  }

  const totalLength = distances[distances.length - 1] ?? 0;
  const clampedProgress = clampNumber(progress, 0, totalLength);

  for (let index = 0; index + 1 < path.length; index += 1) {
    const segmentStart = distances[index];
    const segmentEnd = distances[index + 1];

    if (clampedProgress > segmentEnd && index + 2 < path.length) {
      continue;
    }

    const segmentLength = Math.max(Number.EPSILON, segmentEnd - segmentStart);
    const t = clampNumber((clampedProgress - segmentStart) / segmentLength, 0, 1);
    const start = path[index];
    const end = path[index + 1];

    return {
      x: start.x + (end.x - start.x) * t,
      z: start.z + (end.z - start.z) * t,
    };
  }

  return path[path.length - 1];
}

function initialGuideYaw(path: readonly { x: number; z: number }[]): number {
  if (path.length < 2) {
    return 0;
  }

  return Math.atan2(path[1].x - path[0].x, path[1].z - path[0].z);
}

function approachAngle(current: number, target: number, maxDelta: number): number {
  const delta = normalizeAngle(target - current);

  if (Math.abs(delta) <= maxDelta) {
    return target;
  }

  return normalizeAngle(current + Math.sign(delta) * maxDelta);
}

function normalizeAngle(angle: number): number {
  let normalized = ((angle + Math.PI) % (Math.PI * 2)) - Math.PI;

  if (normalized <= -Math.PI) {
    normalized += Math.PI * 2;
  }

  return normalized;
}

function clampNumber(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}
