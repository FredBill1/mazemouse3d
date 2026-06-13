import HavokPhysics from "@babylonjs/havok";
import { Engine } from "@babylonjs/core/Engines/engine.js";
import { Scene } from "@babylonjs/core/scene.js";
import { ArcRotateCamera } from "@babylonjs/core/Cameras/arcRotateCamera.js";
import { Camera } from "@babylonjs/core/Cameras/camera.js";
import { HemisphericLight } from "@babylonjs/core/Lights/hemisphericLight.js";
import { StandardMaterial } from "@babylonjs/core/Materials/standardMaterial.js";
import { Color3, Color4 } from "@babylonjs/core/Maths/math.color.js";
import { Vector3 } from "@babylonjs/core/Maths/math.vector.js";
import { Mesh } from "@babylonjs/core/Meshes/mesh.js";
import { MeshBuilder } from "@babylonjs/core/Meshes/meshBuilder.js";
import { TransformNode } from "@babylonjs/core/Meshes/transformNode.js";
import "@babylonjs/core/Physics/physicsEngineComponent.js";
import { PhysicsAggregate } from "@babylonjs/core/Physics/v2/physicsAggregate.js";
import { PhysicsShapeType } from "@babylonjs/core/Physics/v2/IPhysicsEnginePlugin.js";
import { HavokPlugin } from "@babylonjs/core/Physics/v2/Plugins/havokPlugin.js";
import type { AppState, AppStateSnapshot } from "../app/state";
import type { MazeSnapshot } from "../domain/maze";
import { buildWallSegments, cellCenter } from "./mazeGeometry";

export type MazeViewMode = "perspective" | "top";

const WALL_HEIGHT = 0.36;
const WALL_THICKNESS = 0.08;

interface MazeMaterials {
  readonly floor: StandardMaterial;
  readonly wall: StandardMaterial;
  readonly start: StandardMaterial;
  readonly goal: StandardMaterial;
  readonly path: StandardMaterial;
  readonly mouse: StandardMaterial;
}

export class BabylonMazeViewport {
  readonly #canvas = document.createElement("canvas");
  readonly #resizeObserver: ResizeObserver;
  readonly #mode: MazeViewMode;
  readonly #state: AppState;

  #engine: Engine | null = null;
  #scene: Scene | null = null;
  #camera: ArcRotateCamera | null = null;
  #materials: MazeMaterials | null = null;
  #mazeRoot: TransformNode | null = null;
  #physicsAggregates: PhysicsAggregate[] = [];
  #unsubscribe: (() => void) | null = null;
  #disposed = false;
  #physicsEnabled = false;

  constructor(container: HTMLElement, state: AppState, mode: MazeViewMode) {
    this.#mode = mode;
    this.#state = state;
    this.#canvas.className = "scene-canvas";
    this.#canvas.setAttribute("aria-label", mode === "perspective" ? "3D scene" : "top view");
    container.classList.add("scene-panel");
    container.append(this.#canvas);

    this.#resizeObserver = new ResizeObserver(() => {
      this.#engine?.resize();
    });
    this.#resizeObserver.observe(container);
  }

  async start(): Promise<void> {
    this.#engine = new Engine(this.#canvas, true, {
      adaptToDeviceRatio: true,
      antialias: true,
      stencil: true,
    });
    this.#scene = new Scene(this.#engine);
    this.#scene.clearColor = new Color4(0.045, 0.052, 0.062, 1);
    this.#materials = createMaterials(this.#scene);

    if (this.#mode === "perspective") {
      await this.#enablePhysics();
    }

    if (this.#disposed || !this.#scene || !this.#engine) {
      return;
    }

    new HemisphericLight("ambient-light", new Vector3(0.2, 1, 0.4), this.#scene).intensity = 0.82;
    this.#camera = this.#createCamera(16);
    this.#unsubscribe = this.#state.subscribe((snapshot) => this.#renderAppState(snapshot));
    this.#engine.runRenderLoop(() => {
      this.#scene?.render();
    });
  }

  dispose(): void {
    this.#disposed = true;
    this.#unsubscribe?.();
    this.#resizeObserver.disconnect();
    this.#clearMaze();
    this.#scene?.dispose();
    this.#engine?.dispose();
    this.#canvas.remove();
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
    this.#physicsEnabled = this.#scene.getPhysicsEngine() !== null;
  }

  #createCamera(size: number): ArcRotateCamera {
    const target = new Vector3(size / 2, 0, size / 2);
    const camera = new ArcRotateCamera(
      `${this.#mode}-camera`,
      this.#mode === "top" ? -Math.PI / 2 : -Math.PI / 4,
      this.#mode === "top" ? 0.01 : Math.PI / 3,
      this.#mode === "top" ? size * 1.05 : size * 1.45,
      target,
      this.#scene!,
    );

    camera.attachControl(this.#canvas, true);
    camera.minZ = 0.01;
    camera.wheelPrecision = 45;

    if (this.#mode === "top") {
      camera.mode = Camera.ORTHOGRAPHIC_CAMERA;
      camera.inputs.removeByType("ArcRotateCameraPointersInput");
      camera.inputs.removeByType("ArcRotateCameraKeyboardMoveInput");
      updateTopCamera(camera, size);
    }

    return camera;
  }

  #renderAppState(snapshot: AppStateSnapshot): void {
    if (!snapshot.maze) {
      return;
    }

    this.#renderMaze(snapshot.maze);
  }

  #renderMaze(maze: MazeSnapshot): void {
    if (!this.#scene || !this.#materials) {
      return;
    }

    this.#clearMaze();
    this.#updateCamera(maze.size);

    const root = new TransformNode("maze-root", this.#scene);
    this.#mazeRoot = root;

    const floor = MeshBuilder.CreateGround(
      "maze-floor",
      { width: maze.size, height: maze.size, subdivisions: maze.size },
      this.#scene,
    );
    floor.position.set(maze.size / 2, 0, maze.size / 2);
    floor.material = this.#materials.floor;
    floor.parent = root;
    this.#addStaticBody(floor);

    for (const segment of buildWallSegments(maze, WALL_THICKNESS)) {
      const wall = MeshBuilder.CreateBox(
        "maze-wall",
        { width: segment.sizeX, height: WALL_HEIGHT, depth: segment.sizeZ },
        this.#scene,
      );
      wall.position.set(segment.centerX, WALL_HEIGHT / 2, segment.centerZ);
      wall.material = this.#materials.wall;
      wall.parent = root;
      this.#addStaticBody(wall);
    }

    this.#addCellMarker("start-marker", maze.start, maze, this.#materials.start, 0.72, 0.022);

    for (const goal of maze.goals) {
      this.#addCellMarker("goal-marker", goal, maze, this.#materials.goal, 0.72, 0.024);
    }

    for (const cell of maze.solution) {
      this.#addCellMarker("solution-cell", cell, maze, this.#materials.path, 0.18, 0.036);
    }

    this.#addMouseBody(maze);
  }

  #updateCamera(size: number): void {
    if (!this.#camera) {
      return;
    }

    this.#camera.setTarget(new Vector3(size / 2, 0, size / 2));

    if (this.#mode === "top") {
      this.#camera.radius = size * 1.05;
      updateTopCamera(this.#camera, size);
      return;
    }

    this.#camera.radius = size * 1.45;
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

  #addMouseBody(maze: MazeSnapshot): void {
    if (!this.#scene || !this.#mazeRoot || !this.#materials) {
      return;
    }

    const start = cellCenter(maze.start, maze.size);
    const mouse = MeshBuilder.CreateCylinder(
      "mouse-placeholder",
      { diameter: 0.48, height: 0.18, tessellation: 28 },
      this.#scene,
    );
    mouse.position.set(start.x, 0.14, start.z);
    mouse.material = this.#materials.mouse;
    mouse.parent = this.#mazeRoot;

    if (this.#physicsEnabled) {
      this.#physicsAggregates.push(
        new PhysicsAggregate(
          mouse,
          PhysicsShapeType.CYLINDER,
          { mass: 0.18, friction: 0.8, restitution: 0.05 },
          this.#scene,
        ),
      );
    }
  }

  #addStaticBody(mesh: Mesh): void {
    if (!this.#scene || !this.#physicsEnabled) {
      return;
    }

    this.#physicsAggregates.push(
      new PhysicsAggregate(mesh, PhysicsShapeType.BOX, { mass: 0, friction: 0.75 }, this.#scene),
    );
  }

  #clearMaze(): void {
    for (const aggregate of this.#physicsAggregates) {
      aggregate.dispose();
    }

    this.#physicsAggregates = [];
    this.#mazeRoot?.dispose(false, true);
    this.#mazeRoot = null;
  }
}

function createMaterials(scene: Scene): MazeMaterials {
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
    mouse: material(
      scene,
      "mouse-material",
      new Color3(0.88, 0.24, 0.24),
      new Color3(0.18, 0.02, 0.02),
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

function updateTopCamera(camera: ArcRotateCamera, size: number): void {
  const half = size / 2 + 0.8;
  camera.orthoLeft = -half;
  camera.orthoRight = half;
  camera.orthoBottom = -half;
  camera.orthoTop = half;
}
