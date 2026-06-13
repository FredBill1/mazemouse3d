import { ArcRotateCamera } from "@babylonjs/core/Cameras/arcRotateCamera.js";
import { Camera } from "@babylonjs/core/Cameras/camera.js";
import { Engine } from "@babylonjs/core/Engines/engine.js";
import "@babylonjs/core/Engines/AbstractEngine/abstractEngine.views.js";
import { HemisphericLight } from "@babylonjs/core/Lights/hemisphericLight.js";
import { StandardMaterial } from "@babylonjs/core/Materials/standardMaterial.js";
import { Color3, Color4 } from "@babylonjs/core/Maths/math.color.js";
import { Vector3 } from "@babylonjs/core/Maths/math.vector.js";
import { MeshBuilder } from "@babylonjs/core/Meshes/meshBuilder.js";
import { Scene } from "@babylonjs/core/scene.js";
import {
  createMicromouseDisplayModel,
  createMicromouseMaterials,
} from "../rendering/micromouseRig";

interface Disposable {
  dispose(): void;
}

interface PreviewViewConfig {
  readonly id: string;
  readonly label: string;
  readonly position: Vector3;
  readonly frameWidth: number;
  readonly frameHeight: number;
}

interface PreviewView extends PreviewViewConfig {
  readonly canvas: HTMLCanvasElement;
  readonly resizeObserver: ResizeObserver;
  readonly camera: ArcRotateCamera;
}

const VIEW_TARGET = new Vector3(0, 0.105, 0.02);
const VIEW_CONFIGS: readonly PreviewViewConfig[] = [
  {
    id: "top",
    label: "Top",
    position: new Vector3(0, 1.24, 0.02),
    frameWidth: 0.82,
    frameHeight: 0.84,
  },
  {
    id: "side",
    label: "Side",
    position: new Vector3(1.15, 0.11, 0.02),
    frameWidth: 0.86,
    frameHeight: 0.36,
  },
  {
    id: "front",
    label: "Front",
    position: new Vector3(0, 0.11, 1.15),
    frameWidth: 0.72,
    frameHeight: 0.36,
  },
  {
    id: "iso",
    label: "45 Deg",
    position: new Vector3(0.78, 0.58, 0.78),
    frameWidth: 0.9,
    frameHeight: 0.68,
  },
];

export function mountMicromousePreview(root: HTMLElement): Disposable {
  root.replaceChildren();
  root.className = "mouse-preview-shell";

  const header = document.createElement("header");
  header.className = "mouse-preview-topbar";
  header.innerHTML = `
    <a class="mouse-preview-back" href="#">Maze</a>
    <div class="mouse-preview-title">Micromouse Model</div>
  `;

  const grid = document.createElement("main");
  grid.className = "mouse-preview-grid";

  const masterCanvas = document.createElement("canvas");
  masterCanvas.className = "scene-master-canvas";
  masterCanvas.setAttribute("aria-hidden", "true");

  root.append(header, grid, masterCanvas);

  const engine = new Engine(masterCanvas, true, {
    adaptToDeviceRatio: true,
    antialias: true,
    stencil: true,
  });
  const scene = new Scene(engine);
  scene.clearColor = new Color4(0.045, 0.052, 0.062, 1);

  new HemisphericLight("preview-ambient-light", new Vector3(0.25, 1, 0.38), scene).intensity = 0.92;
  addPreviewGround(scene);
  createMicromouseDisplayModel(scene, createMicromouseMaterials(scene), {
    includeWheels: true,
    namePrefix: "preview-micromouse",
  });

  const views = VIEW_CONFIGS.map((config) => createView(config, scene, engine, grid));

  engine.runRenderLoop(() => {
    scene.render();
  });

  return {
    dispose() {
      for (const view of views) {
        view.resizeObserver.disconnect();
        engine.unRegisterView(view.canvas);
        view.camera.dispose();
      }

      engine.stopRenderLoop();
      scene.dispose();
      engine.dispose();
      root.replaceChildren();
    },
  };
}

function createView(
  config: PreviewViewConfig,
  scene: Scene,
  engine: Engine,
  grid: HTMLElement,
): PreviewView {
  const cell = document.createElement("section");
  cell.className = "mouse-preview-cell";

  const label = document.createElement("div");
  label.className = "mouse-preview-label";
  label.textContent = config.label;

  const canvas = document.createElement("canvas");
  canvas.className = "mouse-preview-canvas";
  canvas.setAttribute("aria-label", `${config.label} micromouse view`);

  cell.append(canvas, label);
  grid.append(cell);

  const camera = new ArcRotateCamera(
    `mouse-preview-${config.id}-camera`,
    0,
    Math.PI / 2,
    1,
    VIEW_TARGET,
    scene,
  );
  camera.setPosition(config.position);
  camera.mode = Camera.ORTHOGRAPHIC_CAMERA;
  camera.minZ = 0.01;
  camera.maxZ = 10;
  camera.detachControl();

  const resizeObserver = new ResizeObserver(() => {
    resizeCanvas(canvas);
    fitOrthographicCamera(camera, canvas, config.frameWidth, config.frameHeight);
  });
  resizeObserver.observe(cell);

  resizeCanvas(canvas);
  fitOrthographicCamera(camera, canvas, config.frameWidth, config.frameHeight);
  engine.registerView(canvas, camera, true);

  return {
    ...config,
    canvas,
    resizeObserver,
    camera,
  };
}

function addPreviewGround(scene: Scene): void {
  const material = new StandardMaterial("preview-ground-material", scene);
  material.diffuseColor = new Color3(0.11, 0.13, 0.14);
  material.emissiveColor = new Color3(0.018, 0.02, 0.022);
  material.specularColor = new Color3(0.06, 0.06, 0.06);

  const ground = MeshBuilder.CreateGround(
    "preview-ground",
    { width: 1.12, height: 0.98, subdivisions: 2 },
    scene,
  );
  ground.position.y = -0.001;
  ground.material = material;
}

function resizeCanvas(canvas: HTMLCanvasElement): void {
  const width = Math.max(1, Math.floor(canvas.clientWidth));
  const height = Math.max(1, Math.floor(canvas.clientHeight));

  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
}

function fitOrthographicCamera(
  camera: ArcRotateCamera,
  canvas: HTMLCanvasElement,
  frameWidth: number,
  frameHeight: number,
): void {
  const aspect = Math.max(0.1, canvas.clientWidth / Math.max(1, canvas.clientHeight));
  const frameAspect = frameWidth / frameHeight;
  const halfWidth = aspect >= frameAspect ? (frameHeight * aspect) / 2 : frameWidth / 2;
  const halfHeight = aspect >= frameAspect ? frameHeight / 2 : frameWidth / aspect / 2;

  camera.orthoLeft = -halfWidth;
  camera.orthoRight = halfWidth;
  camera.orthoBottom = -halfHeight;
  camera.orthoTop = halfHeight;
}
