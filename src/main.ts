import "./style.css";
import { AppState } from "./app/state";
import { MazeWorkerClient } from "./app/mazeWorkerClient";
import { mountMicromousePreview } from "./app/micromousePreview";
import { mountWorkbench } from "./app/workbench";
import { DEFAULT_MAZE_CONFIG } from "./domain/maze";

interface Disposable {
  dispose(): void;
}

const root = document.querySelector<HTMLElement>("#app");

if (!root) {
  throw new Error("missing #app root");
}

const appRoot = root;
let mounted: Disposable | null = null;

mountCurrentRoute();

window.addEventListener("hashchange", () => {
  mountCurrentRoute();
});

window.addEventListener("beforeunload", () => {
  mounted?.dispose();
});

function mountCurrentRoute(): void {
  mounted?.dispose();
  mounted =
    location.hash === "#/mouse-preview" ? mountMicromousePreview(appRoot) : mountWorkbenchRoute();
}

function mountWorkbenchRoute(): Disposable {
  const appState = new AppState();
  const mazeWorker = new MazeWorkerClient(appState);
  const workbench = mountWorkbench(appRoot, appState, mazeWorker);

  mazeWorker.generate(DEFAULT_MAZE_CONFIG);

  return {
    dispose() {
      workbench.dispose();
      mazeWorker.dispose();
    },
  };
}
