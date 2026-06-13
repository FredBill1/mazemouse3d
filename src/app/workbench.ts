import { DockviewComponent, type IContentRenderer } from "dockview-core";
import "dockview-core/dist/styles/dockview.css";
import type { AppState, AppStateSnapshot } from "./state";
import type { MazeWorkerClient } from "./mazeWorkerClient";
import { BabylonMazeSimulation, type MazeViewMode } from "../rendering/babylonMazeSimulation";

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
        default:
          return new EmptyPanel(`Unknown panel: ${name}`);
      }
    },
    disableFloatingGroups: true,
  });

  const scenePanel = dockview.addPanel({
    id: "scene-3d",
    title: "3D Scene",
    component: "scene-3d",
  });

  const topPanel = dockview.addPanel({
    id: "scene-top",
    title: "Top View",
    component: "scene-top",
    position: { referencePanel: scenePanel, direction: "right" },
    initialWidth: 420,
    minimumWidth: 320,
  });

  dockview.addPanel({
    id: "metrics",
    title: "Metrics",
    component: "metrics",
    position: { referencePanel: topPanel, direction: "below" },
    initialHeight: 260,
    minimumWidth: 320,
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

function statusBlock(message: string): HTMLElement {
  const block = document.createElement("div");
  block.className = "status-block";
  block.textContent = message;

  return block;
}
