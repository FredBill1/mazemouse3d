import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { MazeWorkerClient } from "../app/mazeWorkerClient";
import { AppState } from "../app/state";
import { DEFAULT_MAZE_CONFIG, type MazeWorkerRequest } from "../domain/maze";

class FakeWorker {
  static instances: FakeWorker[] = [];

  readonly postMessage = vi.fn();
  readonly terminate = vi.fn();
  readonly addEventListener = vi.fn();
  readonly removeEventListener = vi.fn();

  constructor(..._args: unknown[]) {
    FakeWorker.instances.push(this);
  }
}

describe("maze worker client", () => {
  beforeEach(() => {
    FakeWorker.instances = [];
    vi.stubGlobal("Worker", FakeWorker);
  });

  afterEach(() => {
    vi.unstubAllGlobals();
  });

  it("uses the precomputed maze for the default config", () => {
    const state = new AppState();
    const client = new MazeWorkerClient(state);

    client.generate(DEFAULT_MAZE_CONFIG);

    expect(FakeWorker.instances).toHaveLength(0);
    expect(state.snapshot.status).toBe("ready");
    expect(state.snapshot.maze?.config).toEqual(DEFAULT_MAZE_CONFIG);

    client.dispose();
  });

  it("uses the worker when score history is requested", () => {
    const state = new AppState();
    const client = new MazeWorkerClient(state);

    client.generate({ ...DEFAULT_MAZE_CONFIG, includeScoreHistory: true });

    expect(FakeWorker.instances).toHaveLength(1);
    expect(state.snapshot.status).toBe("generating");
    expect(workerRequest().config).toMatchObject({
      ...DEFAULT_MAZE_CONFIG,
      includeScoreHistory: true,
    });

    client.dispose();
  });

  it("uses the worker for non-default configs", () => {
    const state = new AppState();
    const client = new MazeWorkerClient(state);

    client.generate({ ...DEFAULT_MAZE_CONFIG, seed: DEFAULT_MAZE_CONFIG.seed + 1 });

    expect(FakeWorker.instances).toHaveLength(1);
    expect(state.snapshot.status).toBe("generating");
    expect(workerRequest().config).toMatchObject({
      ...DEFAULT_MAZE_CONFIG,
      seed: DEFAULT_MAZE_CONFIG.seed + 1,
      includeScoreHistory: false,
    });

    client.dispose();
  });
});

function workerRequest(): MazeWorkerRequest {
  const worker = FakeWorker.instances[0];
  const request = worker?.postMessage.mock.calls[0]?.[0];

  if (!request) {
    throw new Error("expected worker request");
  }

  return request as MazeWorkerRequest;
}
