import "./style.css";
import { AppState } from "./app/state";
import { MazeWorkerClient } from "./app/mazeWorkerClient";
import { mountWorkbench } from "./app/workbench";
import { DEFAULT_MAZE_CONFIG } from "./domain/maze";

const root = document.querySelector<HTMLElement>("#app");

if (!root) {
  throw new Error("missing #app root");
}

const appState = new AppState();
const mazeWorker = new MazeWorkerClient(appState);
const workbench = mountWorkbench(root, appState, mazeWorker);

mazeWorker.generate(DEFAULT_MAZE_CONFIG);

window.addEventListener("beforeunload", () => {
  workbench.dispose();
  mazeWorker.dispose();
});
