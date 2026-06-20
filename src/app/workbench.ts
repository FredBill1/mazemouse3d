import { DockviewComponent, type IContentRenderer } from "dockview-core";
import "dockview-core/dist/styles/dockview.css";
import type { AppState, AppStateSnapshot } from "./state";
import type { MazeWorkerClient } from "./mazeWorkerClient";
import {
  BabylonMazeSimulation,
  type DebugVector3,
  type MazeSimulationDebugSnapshot,
  type MazeViewMode,
} from "../rendering/babylonMazeSimulation";

interface Disposable {
  dispose(): void;
}

export function mountWorkbench(
  root: HTMLElement,
  state: AppState,
  mazeWorker: MazeWorkerClient,
): Disposable {
  root.replaceChildren();
  root.className = "app-shell";

  const topbar = document.createElement("header");
  topbar.className = "topbar";
  topbar.innerHTML = `
    <div class="brand">
      <span class="brand-mark" aria-hidden="true"></span>
      <div>
        <h1>mazemouse3d</h1>
        <p>Micromouse simulator</p>
      </div>
    </div>
    <div class="run-state" data-run-state>Idle</div>
  `;

  const dockHost = document.createElement("main");
  dockHost.className = "dock-host dockview-theme-dark";

  root.append(topbar, dockHost);
  const simulation = new BabylonMazeSimulation(root, state);
  void simulation.start();

  const dockview = new DockviewComponent(dockHost, {
    createComponent: ({ name }) => {
      switch (name) {
        case "scene-3d":
          return new BabylonPanel(simulation, "perspective");
        case "scene-top":
          return new BabylonPanel(simulation, "top");
        case "metrics":
          return new MetricsPanel(state, mazeWorker);
        case "debug":
          return new DebugPanel(simulation);
        default:
          return new EmptyPanel(`Unknown panel: ${name}`);
      }
    },
    disableFloatingGroups: true,
  });

  dockview.layout(dockHost.clientWidth, dockHost.clientHeight);
  const isVertical = dockHost.clientWidth < dockHost.clientHeight;
  const topPanelInitialSize = Math.min(
    420,
    Math.min(dockHost.clientHeight, dockHost.clientWidth) / 2,
  );

  const scenePanel = dockview.addPanel({
    id: "scene-3d",
    title: "3D Scene",
    component: "scene-3d",
  });

  const topPanel = dockview.addPanel({
    id: "scene-top",
    title: "Top View",
    component: "scene-top",
    position: { referencePanel: scenePanel, direction: isVertical ? "below" : "right" },
    initialWidth: topPanelInitialSize,
    initialHeight: topPanelInitialSize,
  });

  const metricsPanel = dockview.addPanel({
    id: "metrics",
    title: "Metrics",
    component: "metrics",
    position: { referencePanel: topPanel, direction: isVertical ? "right" : "below" },
  });

  dockview.addPanel({
    id: "debug",
    title: "Debug",
    component: "debug",
    position: { referencePanel: metricsPanel, direction: "below" },
  });

  const runState = topbar.querySelector<HTMLElement>("[data-run-state]");
  const unsubscribe = state.subscribe((snapshot) => {
    if (runState) {
      runState.textContent = snapshot.status;
      runState.dataset.status = snapshot.status;
    }
  });

  return {
    dispose() {
      unsubscribe();
      dockview.dispose();
      simulation.dispose();
      root.replaceChildren();
    },
  };
}

class BabylonPanel implements IContentRenderer {
  readonly element = document.createElement("div");
  readonly #mode: MazeViewMode;
  readonly #simulation: BabylonMazeSimulation;
  #view: Disposable | null = null;

  constructor(simulation: BabylonMazeSimulation, mode: MazeViewMode) {
    this.#simulation = simulation;
    this.#mode = mode;
    this.element.className = "panel-fill";
  }

  init(): void {
    this.#view = this.#simulation.attachView(this.element, this.#mode);
  }

  layout(): void {
    this.element.querySelector("canvas")?.dispatchEvent(new Event("resize"));
  }

  dispose(): void {
    this.#view?.dispose();
  }
}

class MetricsPanel implements IContentRenderer {
  readonly element = document.createElement("div");
  readonly #mazeWorker: MazeWorkerClient;
  readonly #state: AppState;
  #unsubscribe: (() => void) | null = null;

  constructor(state: AppState, mazeWorker: MazeWorkerClient) {
    this.#state = state;
    this.#mazeWorker = mazeWorker;
    this.element.className = "metrics-panel";
  }

  init(): void {
    this.#unsubscribe = this.#state.subscribe((snapshot) => this.#render(snapshot));
  }

  dispose(): void {
    this.#unsubscribe?.();
  }

  #render(snapshot: AppStateSnapshot): void {
    const header = document.createElement("div");
    header.className = "metrics-header";

    const title = document.createElement("div");
    title.className = "metrics-title";
    title.textContent = "Run";

    const generate = document.createElement("button");
    generate.type = "button";
    generate.className = "primary-action";
    generate.textContent = snapshot.status === "generating" ? "Generating" : "Generate";
    generate.disabled = snapshot.status === "generating";
    generate.addEventListener("click", () => {
      this.#mazeWorker.generate({
        ...snapshot.config,
        seed: snapshot.config.seed + 1,
      });
    });

    header.append(title, generate);

    const config = metricGrid([
      ["Status", snapshot.status],
      ["Size", `${snapshot.config.size} x ${snapshot.config.size}`],
      ["Seed", String(snapshot.config.seed)],
      ["Iterations", String(snapshot.config.iterations)],
    ]);

    const details = snapshot.maze
      ? metricGrid([
          ["Score", snapshot.maze.metrics.score.toFixed(2)],
          ["Shortest path", String(snapshot.maze.metrics.shortestPathSteps)],
          ["Turns", String(snapshot.maze.metrics.turnsOnShortestPath)],
          ["Dead ends", String(snapshot.maze.metrics.deadEnds)],
          ["Junctions", String(snapshot.maze.metrics.junctions)],
          ["Loops", String(snapshot.maze.metrics.extraLoops)],
          ["Diagonal runs", String(snapshot.maze.metrics.diagonalRunCount)],
          ["Bridge ratio", snapshot.maze.metrics.pathBridgeRatio.toFixed(3)],
        ])
      : statusBlock(snapshot.error ?? "Waiting for maze");

    this.element.replaceChildren(header, config, details);
  }
}

class DebugPanel implements IContentRenderer {
  readonly element = document.createElement("div");
  readonly #simulation: BabylonMazeSimulation;
  readonly #slots = new Map<string, HTMLElement>();
  #unsubscribe: (() => void) | null = null;

  constructor(simulation: BabylonMazeSimulation) {
    this.#simulation = simulation;
    this.element.className = "debug-panel";
  }

  init(): void {
    this.element.replaceChildren(
      debugSection(
        "Run",
        [
          ["elapsed", "Elapsed"],
          ["fps", "FPS"],
          ["distance", "Distance"],
          ["average-speed", "Average speed"],
          ["wall-collisions", "Wall hits"],
        ],
        this.#slots,
      ),
      debugSection(
        "Pose",
        [
          ["position-x", "Position X"],
          ["position-y", "Position Y"],
          ["position-z", "Position Z"],
          ["rotation-x", "Rotation X"],
          ["rotation-y", "Rotation Y"],
          ["rotation-z", "Rotation Z"],
        ],
        this.#slots,
      ),
      debugSection(
        "Velocity",
        [
          ["linear-velocity-x", "Linear X"],
          ["linear-velocity-y", "Linear Y"],
          ["linear-velocity-z", "Linear Z"],
          ["angular-velocity-x", "Angular X"],
          ["angular-velocity-y", "Angular Y"],
          ["angular-velocity-z", "Angular Z"],
        ],
        this.#slots,
      ),
      debugSection(
        "Controller",
        [
          ["controller-status", "Status"],
          ["dwb-frequency", "DWB"],
          ["smoother-frequency", "Smoother"],
          ["worker-latency", "Worker latency"],
          ["worker-compute", "Worker compute"],
          ["target-cell", "Target cell"],
          ["command-linear", "Command linear"],
          ["command-angular", "Command angular"],
          ["valid-trajectories", "Valid trajectories"],
        ],
        this.#slots,
      ),
      debugSection(
        "DWB Diagnostics",
        [
          ["current-twist", "Current v/w"],
          ["target-twist", "Target v/w"],
          ["dynamic-window", "Dynamic window"],
          ["best-trajectory", "Best trajectory"],
          ["best-score", "Best score"],
          ["clearance", "Clearance"],
          ["path-progress", "Path progress"],
          ["rejects", "Rejects"],
        ],
        this.#slots,
      ),
    );
    this.#unsubscribe = this.#simulation.subscribeDebug((snapshot) => this.#render(snapshot));
  }

  dispose(): void {
    this.#unsubscribe?.();
  }

  #render(snapshot: MazeSimulationDebugSnapshot): void {
    this.#set("elapsed", `${snapshot.elapsedSeconds.toFixed(2)} s`);
    this.#set("fps", snapshot.fps > 0 ? snapshot.fps.toFixed(1) : "--");
    this.#set("distance", snapshot.totalDistance.toFixed(3));
    this.#set("average-speed", `${snapshot.averageSpeed.toFixed(3)} u/s`);
    this.#set("wall-collisions", String(snapshot.wallCollisions));

    this.#setVector("position", snapshot.position, formatUnit);
    this.#setVector("rotation", snapshot.rotationDegrees, formatDegrees);
    this.#setVector("linear-velocity", snapshot.linearVelocity, formatUnitSpeed);
    this.#setVector("angular-velocity", snapshot.angularVelocityDegrees, formatDegreesPerSecond);
    this.#set("controller-status", snapshot.controller.status);
    this.#set("dwb-frequency", formatHz(snapshot.controller.dwbHz));
    this.#set("smoother-frequency", formatHz(snapshot.controller.smootherHz));
    this.#set(
      "worker-latency",
      snapshot.controller.workerLatencyMs === null
        ? "--"
        : `${snapshot.controller.workerLatencyMs.toFixed(1)} ms`,
    );
    this.#set("worker-compute", `${snapshot.controller.workerComputeMs.toFixed(2)} ms`);
    this.#set(
      "target-cell",
      snapshot.controller.targetCell === null ? "--" : String(snapshot.controller.targetCell),
    );
    this.#set("command-linear", formatUnitSpeed(snapshot.controller.linearSpeed));
    this.#set("command-angular", formatRadiansPerSecond(snapshot.controller.angularSpeed));
    this.#set(
      "valid-trajectories",
      `${snapshot.controller.validTrajectories}/${snapshot.controller.sampledTrajectories}`,
    );
    this.#set(
      "current-twist",
      `${snapshot.controller.currentLinearSpeed.toFixed(3)} / ${snapshot.controller.currentAngularSpeed.toFixed(3)}`,
    );
    this.#set(
      "target-twist",
      `${snapshot.controller.targetLinearSpeed.toFixed(3)} / ${snapshot.controller.targetAngularSpeed.toFixed(3)}`,
    );
    this.#set(
      "dynamic-window",
      `v ${snapshot.controller.dynamicWindow.minV.toFixed(2)}..${snapshot.controller.dynamicWindow.maxV.toFixed(2)}, w ${snapshot.controller.dynamicWindow.minW.toFixed(2)}..${snapshot.controller.dynamicWindow.maxW.toFixed(2)}`,
    );
    this.#set(
      "best-trajectory",
      snapshot.controller.best
        ? `${snapshot.controller.best.linearSpeed.toFixed(3)} / ${snapshot.controller.best.angularSpeed.toFixed(3)}`
        : "--",
    );
    this.#set(
      "best-score",
      snapshot.controller.best
        ? `${snapshot.controller.best.score.total.toFixed(2)} p=${snapshot.controller.best.score.progress.toFixed(2)} c=${snapshot.controller.best.score.minClearance.toFixed(3)}`
        : "--",
    );
    this.#set(
      "clearance",
      `${formatFinite(snapshot.controller.currentClearance)} collides=${snapshot.controller.currentPoseCollides ? "yes" : "no"}`,
    );
    this.#set(
      "path-progress",
      `${snapshot.controller.pathProgress.toFixed(2)} / ${snapshot.controller.pathLength.toFixed(2)} (${snapshot.controller.remainingDistance.toFixed(2)} left, err ${formatFinite(snapshot.controller.pathTrackingError)})`,
    );
    this.#set("rejects", formatRejects(snapshot.controller.rejectedTrajectories));
  }

  #setVector(
    prefix: string,
    vector: DebugVector3 | null,
    formatter: (value: number) => string,
  ): void {
    for (const axis of DEBUG_AXES) {
      this.#set(`${prefix}-${axis}`, vector ? formatter(vector[axis]) : "--");
    }
  }

  #set(key: string, value: string): void {
    const slot = this.#slots.get(key);

    if (slot) {
      slot.textContent = value;
    }
  }
}

class EmptyPanel implements IContentRenderer {
  readonly element = document.createElement("div");

  constructor(message: string) {
    this.element.className = "empty-panel";
    this.element.textContent = message;
  }

  init(): void {
    return;
  }
}

function metricGrid(entries: ReadonlyArray<readonly [label: string, value: string]>): HTMLElement {
  const grid = document.createElement("dl");
  grid.className = "metric-grid";

  for (const [label, value] of entries) {
    const term = document.createElement("dt");
    term.textContent = label;

    const description = document.createElement("dd");
    description.textContent = value;

    grid.append(term, description);
  }

  return grid;
}

function debugSection(
  title: string,
  entries: ReadonlyArray<readonly [key: string, label: string]>,
  slots: Map<string, HTMLElement>,
): HTMLElement {
  const section = document.createElement("section");
  section.className = "debug-section";

  const heading = document.createElement("div");
  heading.className = "debug-section-title";
  heading.textContent = title;

  section.append(heading, debugMetricGrid(entries, slots));

  return section;
}

function debugMetricGrid(
  entries: ReadonlyArray<readonly [key: string, label: string]>,
  slots: Map<string, HTMLElement>,
): HTMLElement {
  const grid = document.createElement("dl");
  grid.className = "metric-grid debug-metric-grid";

  for (const [key, label] of entries) {
    const term = document.createElement("dt");
    term.textContent = label;

    const description = document.createElement("dd");
    description.textContent = "--";
    slots.set(key, description);

    grid.append(term, description);
  }

  return grid;
}

function statusBlock(message: string): HTMLElement {
  const block = document.createElement("div");
  block.className = "status-block";
  block.textContent = message;

  return block;
}

const DEBUG_AXES = ["x", "y", "z"] as const;

function formatUnit(value: number): string {
  return value.toFixed(3);
}

function formatUnitSpeed(value: number): string {
  return `${value.toFixed(3)} u/s`;
}

function formatDegrees(value: number): string {
  return `${value.toFixed(1)} deg`;
}

function formatDegreesPerSecond(value: number): string {
  return `${value.toFixed(1)} deg/s`;
}

function formatHz(value: number): string {
  return value > 0 ? `${value.toFixed(1)} Hz` : "--";
}

function formatRadiansPerSecond(value: number): string {
  return `${((value * 180) / Math.PI).toFixed(1)} deg/s`;
}

function formatFinite(value: number): string {
  return Number.isFinite(value) ? value.toFixed(3) : "--";
}

function formatRejects(
  rejects: MazeSimulationDebugSnapshot["controller"]["rejectedTrajectories"],
): string {
  const entries: Array<[string, number]> = [
    ["cur", rejects.currentPoseCollision],
    ["roll", rejects.rolloutCollision],
    ["brake", rejects.brakingCollision],
    ["wheel", rejects.wheelSpeed],
    ["track", rejects.trackability],
    ["clear", rejects.lowClearance],
    ["prog", rejects.noProgress],
    ["path", rejects.noPathProjection],
    ["nan", rejects.nonFiniteScore],
  ];
  const parts = entries
    .filter(([, count]) => count > 0)
    .map(([label, count]) => `${label}:${count}`);

  return parts.length > 0 ? parts.join(" ") : "none";
}
