import { DEFAULT_MAZE_CONFIG, type MazeSnapshot, type ResolvedMazeConfig } from "../domain/maze";

export type AppStatus = "idle" | "generating" | "ready" | "error";

export interface AppStateSnapshot {
  readonly status: AppStatus;
  readonly config: ResolvedMazeConfig;
  readonly maze: MazeSnapshot | null;
  readonly error: string | null;
}

type AppStateListener = (snapshot: AppStateSnapshot) => void;

export class AppState {
  #snapshot: AppStateSnapshot = {
    status: "idle",
    config: DEFAULT_MAZE_CONFIG,
    maze: null,
    error: null,
  };

  #listeners = new Set<AppStateListener>();

  get snapshot(): AppStateSnapshot {
    return this.#snapshot;
  }

  subscribe(listener: AppStateListener): () => void {
    this.#listeners.add(listener);
    listener(this.#snapshot);

    return () => {
      this.#listeners.delete(listener);
    };
  }

  setGenerating(config: ResolvedMazeConfig): void {
    this.#setSnapshot({
      ...this.#snapshot,
      status: "generating",
      config,
      error: null,
    });
  }

  setMaze(maze: MazeSnapshot): void {
    this.#setSnapshot({
      status: "ready",
      config: maze.config,
      maze,
      error: null,
    });
  }

  setError(message: string): void {
    this.#setSnapshot({
      ...this.#snapshot,
      status: "error",
      error: message,
    });
  }

  #setSnapshot(snapshot: AppStateSnapshot): void {
    this.#snapshot = snapshot;

    for (const listener of this.#listeners) {
      listener(this.#snapshot);
    }
  }
}
