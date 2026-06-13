import type { AppState } from "./state";
import { type MazeConfig, type MazeWorkerRequest, type MazeWorkerResponse } from "../domain/maze";
import { resolveMazeConfig } from "../domain/mazeAdapter";

export class MazeWorkerClient {
  #requestId = 0;
  #worker: Worker;
  readonly #state: AppState;

  constructor(state: AppState) {
    this.#state = state;
    this.#worker = new Worker(new URL("../workers/mazeWorker.ts", import.meta.url), {
      type: "module",
    });
    this.#worker.addEventListener("message", this.#handleMessage);
    this.#worker.addEventListener("error", this.#handleWorkerError);
  }

  generate(config: MazeConfig = {}): void {
    const requestId = (this.#requestId += 1);
    const resolvedConfig = resolveMazeConfig(config);
    const request: MazeWorkerRequest = {
      type: "generateMaze",
      requestId,
      config: resolvedConfig,
    };

    this.#state.setGenerating(resolvedConfig);
    this.#worker.postMessage(request);
  }

  dispose(): void {
    this.#worker.removeEventListener("message", this.#handleMessage);
    this.#worker.removeEventListener("error", this.#handleWorkerError);
    this.#worker.terminate();
  }

  #handleMessage = (event: MessageEvent<MazeWorkerResponse>): void => {
    const response = event.data;

    if (response.requestId !== this.#requestId) {
      return;
    }

    if (response.type === "mazeGenerated") {
      this.#state.setMaze(response.snapshot);
      return;
    }

    this.#state.setError(response.message);
  };

  #handleWorkerError = (event: ErrorEvent): void => {
    this.#state.setError(event.message || "maze worker failed");
  };
}
