import type { AppState } from "./state";
import { type MazeConfig, type MazeWorkerRequest, type MazeWorkerResponse } from "../domain/maze";
import { resolveMazeConfig } from "../domain/mazeAdapter";
import {
  createDefaultMazeSnapshot,
  isPrecomputedDefaultMazeConfig,
} from "../domain/defaultMazeData";

export class MazeWorkerClient {
  #requestId = 0;
  #worker: Worker | null = null;
  readonly #state: AppState;

  constructor(state: AppState) {
    this.#state = state;
  }

  generate(config: MazeConfig = {}): void {
    const requestId = (this.#requestId += 1);
    const resolvedConfig = resolveMazeConfig(config);

    if (!config.includeScoreHistory && isPrecomputedDefaultMazeConfig(resolvedConfig)) {
      this.#state.setMaze(createDefaultMazeSnapshot());
      return;
    }

    const request: MazeWorkerRequest = {
      type: "generateMaze",
      requestId,
      config: {
        ...resolvedConfig,
        includeScoreHistory: config.includeScoreHistory ?? false,
      },
    };

    this.#state.setGenerating(resolvedConfig);
    this.#getWorker().postMessage(request);
  }

  dispose(): void {
    this.#worker?.removeEventListener("message", this.#handleMessage);
    this.#worker?.removeEventListener("error", this.#handleWorkerError);
    this.#worker?.terminate();
    this.#worker = null;
  }

  #getWorker(): Worker {
    if (!this.#worker) {
      this.#worker = new Worker(new URL("../workers/mazeWorker.ts", import.meta.url), {
        type: "module",
      });
      this.#worker.addEventListener("message", this.#handleMessage);
      this.#worker.addEventListener("error", this.#handleWorkerError);
    }

    return this.#worker;
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
